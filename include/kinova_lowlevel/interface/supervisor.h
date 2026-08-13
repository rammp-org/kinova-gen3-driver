#pragma once
#include <atomic>
#include <deque>
#include <mutex>
#include <optional>
#include <thread>
#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/feedback_tap.h"
#include "kinova_lowlevel/joint_impedance_mode.h"
#include "kinova_lowlevel/joint_position_mode.h"
#include "kinova_lowlevel/rt_executor.h"
#include "kinova_lowlevel/interface/ports.h"
#include "kinova_lowlevel/interface/trajectory_executor.h"
namespace kinova::interface {

struct SupervisorConfig { double sampler_hz = 250.0; double pump_hz = 100.0; double mode_settle_s = 0.25; };

class Supervisor : public CommandSink {
 public:
  Supervisor(JointPositionMode& pos, JointImpedanceMode& imp, RtExecutor& exec,
             Seqlock<JointFeedback>& snap, Dynamics& pump_dyn,
             StreamPort& stream, ActionServerPort& action, SupervisorConfig cfg = {});
  ~Supervisor();
  void start();   // request initial (position) mode; spawn sampler + pump threads
  void stop();    // signal + join both threads

  // CommandSink (called on the backend thread):
  GoalResponse   on_trajectory_goal(const TrajectoryGoal&) override;
  void           on_trajectory_accepted(const GoalId&, const TrajectoryGoal&) override;
  CancelResponse on_trajectory_cancel(const GoalId&) override;
  GainsResult    on_set_gains(const GainsRequest&) override;
  ArmState       on_query_state() override;

 private:
  struct Inbound { GoalId id; TrajectoryGoal goal; bool cancel=false; };
  void sampler_loop();
  void pump_loop();

  JointPositionMode& pos_;  JointImpedanceMode& imp_;  RtExecutor& exec_;
  Seqlock<JointFeedback>& snap_;  Dynamics& pump_dyn_;
  StreamPort& stream_;  ActionServerPort& action_;  SupervisorConfig cfg_;

  std::optional<TrajectoryExecutor> traj_;            // rebuilt on mode switch (Task 8)
  ControlModeKind active_mode_kind_ = ControlModeKind::kPosition;
  std::atomic<bool> in_flight_{false};                // read by on_trajectory_goal
  std::atomic<uint8_t> atomic_mode_{0};               // 0=pos 1=imp; read by on_trajectory_goal

  std::mutex q_mtx_;  std::deque<Inbound> inbox_;     // backend -> sampler handoff
  Seqlock<ArmState> state_snap_;                      // pump -> query_state

  std::atomic<bool> running_{false};
  std::thread sampler_, pump_;
};
}  // namespace kinova::interface
