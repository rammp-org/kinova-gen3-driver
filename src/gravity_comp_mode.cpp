#include "kinova_lowlevel/gravity_comp_mode.h"

#include <algorithm>

namespace kinova {

GravityCompTorqueMode::GravityCompTorqueMode(Dynamics& dyn, GravityCompParams p)
    : dyn_(dyn), p_(p) {
  tau_.setZero();
}

ActuatorModes GravityCompTorqueMode::required_modes() const {
  ActuatorModes modes;
  modes.fill(ActuatorMode::kTorque);
  return modes;
}

void GravityCompTorqueMode::on_enter(const JointFeedback&) {}

void GravityCompTorqueMode::compute(const JointFeedback& fb, double /*dt_s*/,
                                    JointCommand& out) {
  // RT-safe: tau_ is a preallocated member; gravity() does no allocation.
  dyn_.gravity(fb.q, tau_);
  tau_ = p_.scale * tau_ - p_.damping * fb.qd;
  for (int i = 0; i < kNumJoints; ++i) {
    tau_[i] = std::clamp(tau_[i], -p_.torque_limit, p_.torque_limit);
  }
  out.mode = ActuatorMode::kTorque;
  out.torque = tau_;
  out.position = fb.q;
}

}  // namespace kinova
