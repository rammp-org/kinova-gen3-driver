#pragma once
#include "kinova_lowlevel/cartesian_types.h"
namespace kinova {
// Decoupled geometric SE(3) error: [ p_d - p ; rotvec(R_d * R^{-1}) ].
// Singularity-free for orientation errors below pi. Pairs with diagonal
// world-frame stiffness. Eigen-only — no Pinocchio.
Vector6 pose_error(const Pose& desired, const Pose& current);

// Map an external joint-torque residual to the EE wrench that explains it:
// F = (J·Jᵀ + λ²I)⁻¹ J · tau_ext — the damped pseudoinverse of Jᵀ, solving
// tau_ext = Jᵀ·F in the least-squares sense. The damping bounds the estimate at
// singular poses, where an undamped inverse would turn a residual's null-space
// component into an enormous phantom wrench; λ matches DiffIkParams::damping.
// Frame and sign follow the Jacobian and residual the caller supplies — see
// ArmState::ee_wrench for the driver's convention. Fixed-size, alloc-free.
Vector6 ee_wrench_from_residual(const Jacobian6& J, const JointVec& tau_ext, double damping = 1e-3);
}  // namespace kinova
