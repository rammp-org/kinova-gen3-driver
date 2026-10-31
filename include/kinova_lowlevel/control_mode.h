#pragma once
#include "kinova_lowlevel/joint_types.h"
#include "kinova_lowlevel/transport.h"
namespace kinova {
class ControlMode {
 public:
  virtual ~ControlMode() = default;
  virtual ActuatorModes required_modes() const = 0;
  virtual void on_enter(const JointFeedback&) = 0;
  // RT-safe. dt_s is the measured wall time since this mode's previous compute():
  // nominal 1/rate_hz on the first call, varying cycle to cycle, up to
  // RtExecutor's cap after misses or a stall. The first call runs right after
  // on_enter, BEFORE the actuators switch to required_modes(), on the same
  // feedback with qd zeroed: its command holds the arm open-loop through the
  // switch (~165 ms on KORTEX), so it must be safe as a fixed hold. Zeroing qd
  // leaves damper terms out of it; both impedance modes also ramp their spring
  // in from zero, so theirs is gravity.
  virtual void compute(const JointFeedback& fb, double dt_s, JointCommand& out) = 0;
  virtual void on_exit() = 0;
};
}  // namespace kinova
