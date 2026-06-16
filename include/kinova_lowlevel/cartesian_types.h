#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "kinova_lowlevel/joint_types.h"

namespace kinova {

// Spatial 6-vector: [vx vy vz | wx wy wz] (linear over angular).
using Vector6 = Eigen::Matrix<double, 6, 1>;

// 6xN frame Jacobian (maps joint velocity -> spatial velocity of the EE frame).
using Jacobian6 = Eigen::Matrix<double, 6, kNumJoints>;

// A rigid-body pose (SE(3)). Fixed-size, no heap — RT-safe to copy.
struct Pose {
  Eigen::Vector3d    p = Eigen::Vector3d::Zero();          // position [m]
  Eigen::Quaterniond R = Eigen::Quaterniond::Identity();   // orientation
};

}  // namespace kinova
