#pragma once
#include "kinova_lowlevel/cartesian_types.h"
namespace kinova {
// Decoupled geometric SE(3) error: [ p_d - p ; rotvec(R_d * R^{-1}) ].
// Singularity-free for orientation errors below pi. Pairs with diagonal
// world-frame stiffness. Eigen-only — no Pinocchio.
Vector6 pose_error(const Pose& desired, const Pose& current);
}  // namespace kinova
