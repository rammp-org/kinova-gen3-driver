#pragma once
#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>
#include <optional>
#include <thread>
#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/feedback_tap.h"
#include "kinova_lowlevel/joint_impedance_mode.h"
#include "kinova_lowlevel/joint_position_mode.h"
#include "kinova_lowlevel/joint_target_sink.h"
#include "kinova_lowlevel/joint_torque_mode.h"
#include "kinova_lowlevel/rt_executor.h"
#include "kinova_lowlevel/interface/ports.h"
#include "kinova_lowlevel/interface/streaming_session.h"
#include "kinova_lowlevel/interface/trajectory_executor.h"
namespace kinova::interface {

// Reuse the last-good measured q when a lock-free feedback-snapshot read fails
// (Seqlock::load == false — e.g. the RT writer preempted mid-store). A failed read
// must NEVER inject a bogus q into the divergence guard: on the real arm that
// produced a false PATH_TOLERANCE_VIOLATED abort mid-motion. Mirrors the proven
// last-good-q pattern in apps/trajectory_run.cpp.
inline kinova::JointVec sampled_q(bool loaded, const kinova::JointVec& fresh,
                                  const kinova::JointVec& last) {
  return loaded ? fresh : last;
}

struct SupervisorConfig { double sampler_hz = 250.0; double pump_hz = 100.0; double mode_settle_s = 0.25; };

class Supervisor : public CommandSink, public StreamSink {
 public:
  Supervisor(JointPositionMode& pos, JointImpedanceMode& imp, JointTorqueMode& tau, RtExecutor& exec,
             Seqlock<JointFeedback>& snap, Dynamics& pump_dyn,
             StreamPort& stream, ActionServerPort& action, SupervisorConfig cfg = {});
  ~Supervisor();
  void start();   // request initial (position) mode; spawn sampler + pump threads
  void stop();    // signal + join both threads

  // CommandSink (called on the backend thread):
  GoalResponse   on_trajectory_goal(const TrajectoryGoal&) override;
  void           on_trajectory_accepted(const GoalId&, const TrajectoryGoal&) override;
  CancelResponse on_trajectory_cancel(const CancelRequest&) override;
  GainsResult    on_set_gains(const GainsRequest&) override;
  ArmState       on_query_state() override;
  void           on_halt(HaltReason) override;

  // StreamSink (called on the backend thread):
  StreamOpenResult on_stream_open(const StreamOpenRequest&) override;
  void             on_stream_close(const StreamCloseRequest&) override;
  void             on_setpoint_joint_position(const JointSetpoint&) override;
  void             on_setpoint_joint_velocity(const JointSetpoint&) override;
  void             on_setpoint_joint_torque(const JointSetpoint&) override;
  void             on_setpoint_pose(const PoseSetpoint&) override;
  void             on_setpoint_twist(const TwistSetpoint&) override;

 private:
  struct Inbound { GoalId id; TrajectoryGoal goal; bool cancel=false; };
  void sampler_loop();
  void pump_loop();
  kinova::JointTargetSink& active_sink();            // pos_ or imp_, per active_mode_kind_
  // The sink a streaming session writes to, chosen by its declared control mode.
  kinova::JointTargetSink& stream_joint_sink();      // pos_ or imp_
  // One teardown, three callers: graceful close, deadline expiry, and on_halt.
  void close_stream();

  JointPositionMode& pos_;  JointImpedanceMode& imp_;  JointTorqueMode& tau_;  RtExecutor& exec_;
  Seqlock<JointFeedback>& snap_;  Dynamics& pump_dyn_;
  StreamPort& stream_;  ActionServerPort& action_;  SupervisorConfig cfg_;

  std::optional<TrajectoryExecutor> traj_;            // rebuilt on mode switch
  // Which mode the EXECUTOR is running. ATOMIC because it is written by the
  // sampler on a goal-driven mode change AND by the backend thread when a
  // streaming session opens in a different mode, while both threads read it.
  // It deliberately does NOT say what traj_ is bound to -- see traj_bound_kind_.
  // Merging the two is what let a stream's mode change leave traj_ writing into
  // a mode nobody is running. It is also the ONLY record of the running mode:
  // on_trajectory_goal reads it directly, so there is no second (binary) copy to
  // fall out of step -- a kTorque stream is reported as kTorque, not as position.
  std::atomic<ControlModeKind> active_mode_kind_{ControlModeKind::kPosition};
  // Which mode's sink traj_ is actually bound to. SAMPLER-OWNED: set at every
  // traj_.emplace() and read only by the sampler, so no backend-thread mode
  // change can desynchronise it from traj_. Both must agree with the goal before
  // the rebind may be skipped -- either one alone leaves a silent desync.
  ControlModeKind traj_bound_kind_ = ControlModeKind::kPosition;
  std::atomic<bool> in_flight_{false};                // read by on_trajectory_goal

  StreamingSession  session_;                         // streaming-tier lifecycle
  std::atomic<bool> stream_open_{false};              // mirrors session_, read by the sampler + goal pre-check
  // Serialises the streaming WRITES (admit + set_target) against the teardown's
  // hold latch. The mark-closed-first ordering decides WHETHER a setpoint is
  // admitted; this makes admit-and-write atomic with respect to close_stream(),
  // so a setpoint that has already passed admit() can never interleave with the
  // teardown's write into the same single-writer double buffer. Backend/sampler
  // threads only -- never taken on the RT path.
  std::mutex stream_mtx_;
  JointVec   stream_hold_q_ = JointVec::Zero();       // last-good measured q for the teardown hold
  bool       have_hold_q_   = false;                  // false until the first successful snapshot read
  std::chrono::steady_clock::time_point t0_{};        // time origin for session stamps; set in start()

  std::mutex q_mtx_;  std::deque<Inbound> inbox_;     // backend -> sampler handoff
  bool       halt_pending_ = false;                  // guarded by q_mtx_
  HaltReason halt_reason_  = HaltReason::kOwnershipRevoked;   // guarded by q_mtx_
  Seqlock<ArmState> state_snap_;                      // pump -> query_state

  std::atomic<bool> running_{false};
  std::thread sampler_, pump_;
};
}  // namespace kinova::interface
