#pragma once
#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/feedback_tap.h"
#include "kinova_lowlevel/joint_impedance_mode.h"
#include "kinova_lowlevel/joint_position_mode.h"
#include "kinova_lowlevel/joint_target_sink.h"
#include "kinova_lowlevel/joint_torque_mode.h"
#include "kinova_lowlevel/joint_velocity_mode.h"
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

// Everything the Supervisor needs, in one named place.
//
// POINTERS here, references everywhere else -- which is this library's existing rule, not
// an exception to it. Every stored dependency in include/kinova_lowlevel/ is a reference;
// the one raw pointer is RtExecutor's requested_ mode, and it is a pointer because a mode
// may be absent or swapped. A reference means "always there"; a pointer means "may be
// absent."
//
// The pointers exist ONLY at this construction boundary, so fields can be named and
// defaulted -- a struct of references would force positional initialisation and would
// still fail to compile at every site when a member is added, which is the breakage being
// replaced. require() converts each one to a reference immediately, so what the Supervisor
// STORES matches every other unit and the pointer-ness never leaks past the constructor.
//
// The cost is a null check, paid once in a constructor that throws naming the field. This
// repo fails loud at startup rather than degrading at 1 kHz.
//
// Assign by name at the call site:
//     SupervisorDeps d;
//     d.pos = &pos; d.imp = &imp; ... d.stream = &backend; d.action = &router;
//     Supervisor sup{d};
// which is legible in a way that ten positional arguments -- two of which were the same
// object passed as different port types -- was not.
struct SupervisorDeps {
  JointPositionMode*      pos      = nullptr;
  JointImpedanceMode*     imp      = nullptr;
  JointTorqueMode*        tau      = nullptr;
  JointVelocityMode*      vel      = nullptr;
  RtExecutor*             exec     = nullptr;
  Seqlock<JointFeedback>* snap     = nullptr;
  Dynamics*               pump_dyn = nullptr;
  StreamPort*             stream   = nullptr;
  ActionServerPort*       action   = nullptr;
  SupervisorConfig        cfg{};
};

class Supervisor : public CommandSink, public StreamSink {
 public:
  explicit Supervisor(const SupervisorDeps& deps);
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

  // Test/diagnostic: is a streaming session currently admitting setpoints?
  bool stream_is_open() const { return stream_open_.load(); }
  // Why the last session ended. Distinct causes because a client cannot otherwise
  // tell a lapsed deadline (its own fault, retry) from an IK fault (the pose path
  // stopped converging, retrying the same stream reproduces it).
  StreamCloseCause stream_close_cause() const { return close_cause_.load(); }

 private:
  struct Inbound { GoalId id; TrajectoryGoal goal; bool cancel=false; };
  void sampler_loop();
  void pump_loop();
  // The joint-target sink a control mode kind owns, EXPLICIT over all four kinds.
  // nullptr for the kinds that have no joint target (kTorque, kVelocity) so a
  // caller must decide what to do rather than silently writing into pos_.
  kinova::JointTargetSink* sink_for(ControlModeKind);
  // The pose-target sink a control mode kind owns, EXPLICIT for the same reason
  // sink_for is: an inline "impedance or else" ternary would recreate the exact
  // binary mapping sink_for's own comment records as having been wrong before.
  kinova::PoseTargetSink* pose_sink_for(ControlModeKind);
  // One teardown, four callers: graceful close, deadline expiry, IK fault, on_halt.
  void close_stream(StreamCloseCause);

  JointPositionMode& pos_;  JointImpedanceMode& imp_;  JointTorqueMode& tau_;  JointVelocityMode& vel_;
  RtExecutor& exec_;
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
  // Why the last session ended. Written by whichever thread ran the teardown,
  // read by anyone asking after the fact.
  std::atomic<StreamCloseCause> close_cause_{StreamCloseCause::kNone};
  // Serialises the streaming WRITES (admit + set_target) against the teardown's
  // hold latch. The mark-closed-first ordering decides WHETHER a setpoint is
  // admitted; this makes admit-and-write atomic with respect to close_stream(),
  // so a setpoint that has already passed admit() can never interleave with the
  // teardown's write into the same single-writer double buffer. It ALSO serialises
  // close_stream() against on_stream_open()'s tail, so a close still in flight
  // cannot disarm the watchdog a re-open has just armed. Backend/sampler threads
  // only -- never taken on the RT path, and never held across the mode settle.
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
