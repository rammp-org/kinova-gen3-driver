#pragma once
#include "kinova_lowlevel/control_mode.h"
#include "kinova_lowlevel/dynamics.h"
namespace kinova {
struct GravityCompParams { double scale = 1.0; double damping = 0.0; double torque_limit = 39.0; };
class GravityCompTorqueMode : public ControlMode {
 public:
  GravityCompTorqueMode(Dynamics& dyn, GravityCompParams p = {});
  ActuatorModes required_modes() const override;
  void on_enter(const JointFeedback&) override;
  void compute(const JointFeedback& fb, double dt_s, JointCommand& out) override;
  void on_exit() override {}
 private:
  Dynamics& dyn_;
  GravityCompParams p_;
  JointVec tau_;
};
}  // namespace kinova
