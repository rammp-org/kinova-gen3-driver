#include "kinova_lowlevel/cartesian_impedance_mode.h"
#include <algorithm>

namespace kinova {

CartesianImpedanceMode::CartesianImpedanceMode(Dynamics& dyn, CartesianImpedanceParams p)
    : dyn_(dyn) {
  gains_[0] = p;
  gains_[1] = p;
  J_.setZero();
  g_.setZero();
  tau_.setZero();
}

ActuatorModes CartesianImpedanceMode::required_modes() const {
  ActuatorModes modes; modes.fill(ActuatorMode::kTorque); return modes;
}

CartesianImpedanceParams CartesianImpedanceMode::params() const noexcept {
  return gains_[gains_active_.load(std::memory_order_acquire)];  // copy the active slot
}

void CartesianImpedanceMode::set_gains(const CartesianImpedanceParams& p) noexcept {
  int next = 1 - gains_active_.load(std::memory_order_relaxed);
  gains_[next] = p;
  gains_active_.store(next, std::memory_order_release);
}

void CartesianImpedanceMode::set_target(const Pose& x_d) noexcept {
  int next = 1 - ext_active_.load(std::memory_order_relaxed);
  ext_target_[next] = x_d;
  ext_active_.store(next, std::memory_order_release);
  has_ext_target_.store(true, std::memory_order_release);
}

void CartesianImpedanceMode::on_enter(const JointFeedback& fb) {
  entry_pose_ = dyn_.fk(fb.q);                       // hold where we are
  q_rest_ = fb.q;
  has_ext_target_.store(false, std::memory_order_release);
  ramp_elapsed_ = 0.0;
}

void CartesianImpedanceMode::compute(const JointFeedback& fb, double dt_s,
                                     JointCommand& out) {
  const CartesianImpedanceParams p = params();   // own a snapshot for the whole cycle
  const Pose target = has_ext_target_.load(std::memory_order_acquire)
                          ? ext_target_[ext_active_.load(std::memory_order_acquire)]
                          : entry_pose_;

  Pose x = dyn_.fk(fb.q);
  dyn_.jacobian(fb.q, J_);
  Vector6 xd = J_ * fb.qd;
  Vector6 e  = pose_error(target, x);
  Vector6 F  = p.Kx.cwiseProduct(e) - p.Dx.cwiseProduct(xd);

  dyn_.gravity(fb.q, g_);

  JointVec tau_active = J_.transpose() * F;
  if (p.nullspace_on) {
    // Damped pseudo-inverse of Jᵀ:  (Jᵀ)⁺ = (J Jᵀ + λ²I)⁻¹ J   (6x7).
    // N = I - Jᵀ (Jᵀ)⁺ projects secondary joint torques into null(J), so they
    // produce no task-space wrench. Fixed-size LDLT — no heap allocation.
    Eigen::Matrix<double, 6, 6> JJt = J_ * J_.transpose();
    JJt.diagonal().array() += p.pinv_damping * p.pinv_damping;
    Eigen::Matrix<double, 6, kNumJoints> JtPinv = JJt.ldlt().solve(J_);
    Eigen::Matrix<double, kNumJoints, kNumJoints> N =
        Eigen::Matrix<double, kNumJoints, kNumJoints>::Identity()
        - J_.transpose() * JtPinv;
    JointVec tau0 = p.nullspace_kp * (q_rest_ - fb.q) - p.nullspace_kd * fb.qd;
    tau_active += N * tau0;
  }

  // Ramp scales the ACTIVE wrench (task + nullspace) 0->1 over gain_ramp_s on
  // entry; gravity is ALWAYS applied in full so the arm never sags while the
  // compliance fades in. ramp uses elapsed-at-start-of-cycle, then advances.
  const double ramp = (p.gain_ramp_s <= 0.0)
                          ? 1.0
                          : std::min(1.0, ramp_elapsed_ / p.gain_ramp_s);
  tau_ = g_ + ramp * tau_active;
  ramp_elapsed_ += dt_s;

  // Per-joint torque limit. Note: saturation can break the exact nullspace
  // orthogonality — safety takes precedence over the projection.
  for (int i = 0; i < kNumJoints; ++i)
    tau_[i] = std::clamp(tau_[i], -p.torque_limit, p.torque_limit);

  out.mode = ActuatorMode::kTorque;
  out.torque = tau_;
  out.position = fb.q;                               // passthrough for following-error hold
}

}  // namespace kinova
