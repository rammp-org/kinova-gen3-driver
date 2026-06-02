#pragma once
#include <cmath>
#include "kinova_lowlevel/joint_types.h"
namespace kinova {
inline JointVec deg_to_rad(const JointVec& d) { return d * kDeg2Rad; }
inline JointVec rad_to_deg(const JointVec& r) { return r * kRad2Deg; }
inline double wrap_to_pi(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a <= -M_PI) a += 2.0 * M_PI;
  return a;
}
}  // namespace kinova
