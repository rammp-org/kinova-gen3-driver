#pragma once
#include "kinova_lowlevel/cartesian_types.h"
namespace kinova {
// Decoupled geometric SE(3) error: [ p_d - p ; rotvec(R_d * R^{-1}) ].
// Singularity-free for orientation errors below pi. Pairs with diagonal
// world-frame stiffness. Eigen-only — no Pinocchio.
Vector6 pose_error(const Pose& desired, const Pose& current);

// A = J·Jᵀ + λ²I — the damped Gram matrix behind every damped-pseudoinverse
// solve in the repo (the wrench estimator below, the impedance null-space
// projector, DiffIk). ONE place to change the regularization if it ever
// evolves (e.g. scaling λ with ‖J‖). velocity_reference.cpp alone cannot use
// it: it factors the UNDAMPED Gram matrix first to read manipulability off
// the LDLT for free, then adds its adaptive λ² to the matrix it already
// holds. Fixed-size, alloc-free.
inline void damped_jjt(const Jacobian6& J, double damping,
                       Eigen::Matrix<double, 6, 6>& A_out) {
  A_out.noalias() = J * J.transpose();
  A_out.diagonal().array() += damping * damping;
}

// Map an external joint-torque residual to the EE wrench that explains it:
// F = (J·Jᵀ + λ²I)⁻¹ J · tau_ext — the damped pseudoinverse of Jᵀ, solving
// tau_ext = Jᵀ·F in the least-squares sense. Per direction the gain is
// σ/(σ²+λ²), peaking at 1/(2λ) at σ=λ, so the estimate is HARD-BOUNDED by
// ‖F‖ ≤ ‖tau_ext‖/(2λ). λ = 0.05 caps amplification at 10×: the ~2.5 N·m
// proximal-joint residual noise floor maps to ≲25 N near a singular pose
// (where an undamped inverse would explode), while biasing the estimate only
// a few percent at working poses. Do NOT shrink λ toward DiffIk's 1e-3: DiffIk
// clamps its input before the solve, this estimator feeds raw sensor noise in.
// Frame and sign follow the Jacobian and residual the caller supplies — see
// ArmState::ee_wrench for the driver's convention. Fixed-size, alloc-free.
Vector6 ee_wrench_from_residual(const Jacobian6& J, const JointVec& tau_ext, double damping = 0.05);
}  // namespace kinova
