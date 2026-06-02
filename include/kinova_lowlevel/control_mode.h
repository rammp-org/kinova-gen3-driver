#pragma once
#include "kinova_lowlevel/joint_types.h"
#include "kinova_lowlevel/transport.h"
namespace kinova {
class ControlMode {
 public:
  virtual ~ControlMode() = default;
  virtual ActuatorModes required_modes() const = 0;
  virtual void on_enter(const JointFeedback&) = 0;
  virtual void compute(const JointFeedback& fb, double dt_s, JointCommand& out) = 0;  // RT-safe
  virtual void on_exit() = 0;
};
}  // namespace kinova
