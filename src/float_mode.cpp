#include "kinova_lowlevel/float_mode.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "kinova_lowlevel/units.h"

namespace kinova {

namespace {
void require_nonneg_finite(const JointVec& v, const char* name) {
  for (int i = 0; i < kNumJoints; ++i)
    if (!std::isfinite(v[i]) || v[i] < 0.0)
      throw std::invalid_argument(std::string("FloatMode: ") + name +
                                  " must be finite and non-negative");
}
}  // namespace

FloatMode::FloatMode(Dynamics& dyn, FloatParams p) : dyn_(dyn), p_(p) {
  require_nonneg_finite(p_.damping, "damping");
  require_nonneg_finite(p_.spring, "spring");
  if (!std::isfinite(p_.drag_band_rad) || p_.drag_band_rad <= 0.0)
    throw std::invalid_argument("FloatMode: drag_band_rad must be finite and > 0");
  for (int i = 0; i < kNumJoints; ++i)
    if (!std::isfinite(p_.torque_limit[i]) || p_.torque_limit[i] <= 0.0)
      throw std::invalid_argument("FloatMode: torque_limit must be finite and > 0");
  JointVec lo, hi;
  dyn_.joint_limits(lo, hi);
  for (int i = 0; i < kNumJoints; ++i)
    continuous_[i] = !std::isfinite(lo[i]) && !std::isfinite(hi[i]);
  g_.setZero();
  err_.setZero();
  tau_.setZero();
}

ActuatorModes FloatMode::required_modes() const {
  ActuatorModes modes;
  modes.fill(ActuatorMode::kTorque);
  return modes;
}

void FloatMode::on_enter(const JointFeedback& fb) { anchor_ = fb.q; }

void FloatMode::compute(const JointFeedback& fb, double, JointCommand& out) {
  const double band = p_.drag_band_rad;
  // q is wrapped to (-pi, pi]: on a continuous joint wrap q - q_a too, or a joint at
  // +-pi (home's j3) flips the anchor to the far side each time it crosses the wrap.
  for (int i = 0; i < kNumJoints; ++i) {
    const double raw = fb.q[i] - anchor_[i];
    err_[i] = continuous_[i] ? wrap_to_pi(raw) : raw;
    if (err_[i] > band) {
      anchor_[i] = fb.q[i] - band;
      err_[i] = band;
    } else if (err_[i] < -band) {
      anchor_[i] = fb.q[i] + band;
      err_[i] = -band;
    }
    if (continuous_[i]) anchor_[i] = wrap_to_pi(anchor_[i]);
  }
  dyn_.gravity(fb.q, g_);  // RT-safe: no alloc
  for (int i = 0; i < kNumJoints; ++i) {
    const double t = g_[i] - p_.damping[i] * fb.qd[i] - p_.spring[i] * err_[i];
    tau_[i] = std::clamp(t, -p_.torque_limit[i], p_.torque_limit[i]);
  }
  out.mode = ActuatorMode::kTorque;
  out.torque = tau_;
  out.position = fb.q;
}

}  // namespace kinova
