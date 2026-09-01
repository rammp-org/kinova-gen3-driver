#pragma once
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
  // Test/bench observability: the last command the loop wrote. Read it only after the
  // RT thread is joined -- it is not synchronised.
  const JointCommand& last_command() const { return last_cmd_; }
  // Test knobs for the gripper. The sim's gripper closes a FRACTION of the remaining
  // gap each cycle rather than teleporting, because "moving" and "stalled" are the two
  // states the grasp lifecycle needs to distinguish and an instant echo has neither.
  void set_gripper_lag(float per_cycle_fraction) { gripper_lag_ = per_cycle_fraction; }
  // Simulate an object: the fingers cannot close past this position, and effort rises
  // to the commanded force cap once they are stopped by it. Negative disables.
  void set_gripper_blocked_at(float position) { gripper_block_ = position; }
 private:
  void step_gripper(const GripperCommand&);
  JointFeedback state_;
  JointCommand last_cmd_;
  int latency_us_ = 0;
  uint64_t frame_ = 0;
  float gripper_lag_   = 1.0f;    // default: reach the target in one cycle (old behaviour)
  float gripper_block_ = -1.0f;   // no object
};
}  // namespace kinova
