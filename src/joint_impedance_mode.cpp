#include "kinova_lowlevel/joint_impedance_mode.h"
#include <algorithm>
#include <cmath>
namespace kinova {

JointImpedanceMode::JointImpedanceMode(Dynamics& dyn, JointImpedanceParams p)
    : dyn_(dyn), ik_(dyn, p.ik) {
  // Cache the URDF limits once. set_gains runs on a non-RT thread and must never
  // touch Dynamics -- it is not thread-safe against the RT loop's fk/jacobian.
  dyn.joint_limits(q_lower_urdf_, q_upper_urdf_);
  seed_limits(p);
  ik_.set_params(p.ik);
  gains_[0] = p;
  gains_[1] = p;
}

void JointImpedanceMode::seed_limits(JointImpedanceParams& p) const noexcept {
  for (int i = 0; i < kNumJoints; ++i) {
    if (!std::isfinite(p.ik.q_lower[i])) p.ik.q_lower[i] = q_lower_urdf_[i];
    if (!std::isfinite(p.ik.q_upper[i])) p.ik.q_upper[i] = q_upper_urdf_[i];
  }
}

ActuatorModes JointImpedanceMode::required_modes() const {
  ActuatorModes modes; modes.fill(ActuatorMode::kTorque); return modes;
}

JointImpedanceParams JointImpedanceMode::params() const noexcept {
  return gains_[gains_active_.load(std::memory_order_acquire)];  // copy active slot
}

void JointImpedanceMode::set_gains(const JointImpedanceParams& p) noexcept {
  const int next = 1 - gains_active_.load(std::memory_order_relaxed);
  gains_[next] = p;
  seed_limits(gains_[next]);
  gains_active_.store(next, std::memory_order_release);
}

void JointImpedanceMode::set_target(const Pose& x_d) noexcept {
  const int next = 1 - ext_active_.load(std::memory_order_relaxed);
  ext_target_[next] = x_d;
  ext_active_.store(next, std::memory_order_release);
  has_ext_target_.store(true, std::memory_order_release);
}

void JointImpedanceMode::on_enter(const JointFeedback& fb) {
  entry_pose_ = dyn_.fk(fb.q);                 // hold where we are
  // The reference starts exactly at the measured configuration, then integrates
  // OPEN-LOOP. Re-seeding from fb.q every cycle would collapse the spring to zero
  // error and degenerate this into rigid tracking, losing all compliance.
  q_d_ = fb.q;
  has_ext_target_.store(false, std::memory_order_release);
  ramp_elapsed_ = 0.0;
  last_ik_ = IkResult{};
}

void JointImpedanceMode::compute(const JointFeedback& fb, double dt_s,
                                 JointCommand& out) {
  const JointImpedanceParams p = params();   // own a snapshot for the whole cycle
  const Pose target = has_ext_target_.load(std::memory_order_acquire)
                          ? ext_target_[ext_active_.load(std::memory_order_acquire)]
                          : entry_pose_;

  ik_.set_params(p.ik);                      // fixed-size copy, no alloc
  const JointVec q_prev = q_d_;
  last_ik_ = ik_.solve(target, q_d_);        // warm-started from last cycle

  // Bound reference speed so a teleported target ramps in instead of slamming.
  const double max_step = p.max_ref_speed * dt_s;
  for (int i = 0; i < kNumJoints; ++i)
    q_d_[i] = std::clamp(q_d_[i], q_prev[i] - max_step, q_prev[i] + max_step);

  dyn_.gravity(fb.q, g_);

  // Leash the SPRING only. Gravity is never scaled or leashed, so the arm cannot
  // sag when the spring saturates. Applied to the torque, not to q_d_ -- pushing
  // the arm away must not corrupt the IK reference.
  for (int i = 0; i < kNumJoints; ++i) {
    const double e = std::clamp(q_d_[i] - fb.q[i], -p.max_tracking_error,
                                p.max_tracking_error);
    tau_[i] = p.Kq[i] * e - p.Dq[i] * fb.qd[i];
  }

  // Ramp scales the spring 0->1 over gain_ramp_s on entry; gravity is ALWAYS
  // applied in full so the arm never sags while the spring fades in. ramp uses
  // elapsed-at-start-of-cycle, then advances.
  const double ramp = (p.gain_ramp_s <= 0.0)
                          ? 1.0
                          : std::min(1.0, ramp_elapsed_ / p.gain_ramp_s);
  tau_ = g_ + ramp * tau_;
  ramp_elapsed_ += dt_s;

  for (int i = 0; i < kNumJoints; ++i)
    tau_[i] = std::clamp(tau_[i], -p.torque_limit[i], p.torque_limit[i]);

  out.mode = ActuatorMode::kTorque;
  out.torque = tau_;
  out.position = fb.q;                       // passthrough for following-error hold
}

}  // namespace kinova
