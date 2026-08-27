#pragma once
#include <atomic>
#include <mutex>
#include <random>
#include <string>
#include "kinova_lowlevel/interface/ports.h"
namespace kinova::interface {

// Decides WHO may command the arm, and nothing else.
//
// Decorates a CommandSink: every inbound command carries a Token; the Arbiter
// compares it against the live grant and either delegates downstream or rejects.
// It includes nothing from controls -- no ControlMode, no RtExecutor, no Dynamics,
// no Transport -- so the safety-critical admission logic is unit-testable against a
// fake sink with no robot, no URDF and no threads. Same idiom as FeedbackTap
// decorating Transport.
//
// Lock discipline (spec: "Thread safety"): the mutex IS held across command
// delegation, so admit-and-deliver is atomic against revoke()/estop() -- otherwise a
// command admitted a moment before a revoke could reach the Supervisor AFTER the halt
// and restart a stopped arm. It is NEVER held across on_halt(). The one thing that
// does NOT wait for it is estop(): a delegated call may block for hundreds of
// milliseconds (the streaming tier's mode settle), so the e-stop latch and its halt
// both run outside m_ -- see estopped_ below.
class Arbiter : public CommandSink, public StreamSink, public ArbitrationSink {
 public:
  // seed == 0 -> seed the token RNG from std::random_device.
  Arbiter(CommandSink& downstream, StreamSink& downstream_stream, ArbitrationMode mode,
          uint64_t seed = 0);

  // ArbitrationSink
  GrantResult       grant(const std::string& owner_id) override;
  void              revoke() override;
  void              estop() override;
  void              estop_clear() override;
  ArbitrationStatus status() const override;

  // CommandSink
  GoalResponse   on_trajectory_goal(const TrajectoryGoal&) override;
  void           on_trajectory_accepted(const GoalId&, const TrajectoryGoal&) override;
  CancelResponse on_trajectory_cancel(const CancelRequest&) override;
  GainsResult    on_set_gains(const GainsRequest&) override;
  ArmState       on_query_state() override;       // never gated -- reads are always open
  void           on_halt(HaltReason) override;    // pass-through

  // StreamSink
  StreamOpenResult on_stream_open(const StreamOpenRequest&) override;
  void             on_stream_close(const StreamCloseRequest&) override;
  void             on_setpoint_joint_position(const JointSetpoint&) override;
  void             on_setpoint_joint_velocity(const JointSetpoint&) override;
  void             on_setpoint_joint_torque(const JointSetpoint&) override;
  void             on_setpoint_pose(const PoseSetpoint&) override;
  void             on_setpoint_twist(const TwistSetpoint&) override;

 private:
  bool  admit(const Token&) const;   // caller holds m_
  Token mint();                      // caller holds m_

  CommandSink&    down_;
  StreamSink&     down_stream_;
  ArbitrationMode mode_;
  mutable std::mutex m_;
  std::mt19937_64 rng_;
  bool        owned_ = false;
  // ATOMIC and deliberately readable WITHOUT m_. Delegated calls run under m_, and
  // Supervisor::on_stream_open now sleeps mode_settle_s (250 ms) inside one of
  // them -- an e-stop that has to queue behind that is not an e-stop. estop()
  // latches this BEFORE it contends for m_, so admission is refused from that
  // instant regardless of who holds the lock.
  std::atomic<bool> estopped_{false};
  Token       token_{};
  std::string owner_id_;
  uint64_t    generation_ = 0;
  uint64_t    rejected_ = 0;
};
}  // namespace kinova::interface
