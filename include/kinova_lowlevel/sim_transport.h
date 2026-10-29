#pragma once
#include <atomic>
#include <functional>

#include "kinova_lowlevel/feedback_tap.h"  // Seqlock
#include "kinova_lowlevel/transport.h"
namespace kinova {
// Fake robot for tests/benchmarks with no hardware. Echoes state, advances a
// frame counter, optionally busy-waits `latency_us` to mimic the round-trip.
class SimTransport : public Transport {
 public:
  explicit SimTransport(const JointFeedback& initial, int latency_us = 0);
  void connect() override;
  void set_servoing_low_level() override;
  void set_actuator_modes(const ActuatorModes&) override;
  void exchange(const JointCommand&, JointFeedback&) override;
  void send(const JointCommand&) override;
  void receive(JointFeedback&) override;
  void safe_shutdown() override;
  void release_low_level() override;
  bool arm_in_fault() override;
  void clear_faults() override;
  // Test/bench observability: the last command the loop wrote. Read it only after the
  // RT thread is joined -- it is not synchronised.
  const JointCommand& last_command() const { return last_cmd_; }
  // Test knobs for the gripper. The sim's gripper closes a FRACTION of the remaining
  // gap each cycle rather than teleporting, because "moving" and "stalled" are the two
  // states the grasp lifecycle needs to distinguish and an instant echo has neither.
  void set_gripper_lag(float per_cycle_fraction) { gripper_lag_ = per_cycle_fraction; }
  // Simulate an object: the fingers cannot close past this position, and effort rises
  // to the commanded force cap once they are stopped by it. Negative disables.
  // THIS IS A MODEL, not measured hardware behaviour: the arm was measured to settle
  // to a low holding current (effort ~0.05) on a sustained grasp, not the commanded
  // cap (see GripperFeedback::effort). Do not calibrate a holding-detector against
  // this sim's effort value.
  void set_gripper_blocked_at(float position) { gripper_block_ = position; }

  // Test knobs for the link lifecycle (ArmLink). Atomic, so a test may flip them
  // while the link runs on another thread (throw_on_exchange excepted: set it
  // before run(), and have the hook read its own atomics).
  // The next n connect() calls throw std::runtime_error("sim: arm unreachable").
  void fail_connects(int n) { fail_connects_ = n; }
  // An arm fault: reported in feedback and by arm_in_fault(); clear_faults() ends
  // it unless it is sticky.
  void set_arm_fault(bool f) { arm_fault_ = f; }
  void set_fault_sticky(bool s) { fault_sticky_ = s; }
  // A push on the arm: feedback tau reports (what it would otherwise) - tau_ext,
  // the command-convention sign, so the contact observer reads +tau_ext. The arm
  // does not move: the sim has no dynamics. Lock-free (a Seqlock); one test thread
  // may call it while the loop runs.
  void set_external_torque(const JointVec& tau_ext) { tau_ext_.store(tau_ext); }
  // After a clear_faults() that ends the fault, arm_in_fault() still reports it n
  // more times: the real arm takes a moment to leave IN_FAULT.
  void set_clear_lag(int n) { clear_lag_ = n; }
  // What feedback reports as JointFeedback::low_level.
  void set_low_level_feedback(bool l) { low_level_fb_ = l; }
  // Called at the top of every exchange(); may throw.
  void throw_on_exchange(std::function<void()> f) { on_exchange_ = std::move(f); }
  // The next n arm_in_fault() calls throw TransportTimeout.
  void fail_health(int n) { fail_health_ = n; }
  // Teleport the arm (a move made while the driver was not commanding). Call only
  // while nothing is running the transport.
  void set_joint_positions(const JointVec& q) { state_.q = q; }
  // Call counts.
  std::atomic<int> connects{0}, low_level_entries{0}, releases{0}, shutdowns{0}, clears{0};

 private:
  void step_torque(const JointCommand&);
  void step_gripper(const GripperCommand&);
  void report_wrapped(JointFeedback& fb);
  JointFeedback state_;
  JointCommand last_cmd_;
  int latency_us_ = 0;
  uint64_t frame_ = 0;
  float gripper_lag_ = 1.0f;     // default: reach the target in one cycle (old behaviour)
  float gripper_block_ = -1.0f;  // no object
  std::atomic<int> fail_connects_{0}, fail_health_{0}, clear_lag_{0}, lag_left_{0};
  std::atomic<bool> arm_fault_{false}, fault_sticky_{false}, low_level_fb_{true};
  std::function<void()> on_exchange_;
  Seqlock<JointVec> tau_ext_;
  JointVec tau_ext_seen_ = JointVec::Zero();  // last whole read: a torn one keeps it
};
}  // namespace kinova
