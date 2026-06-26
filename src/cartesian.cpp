#include "kinova_lowlevel/cartesian.h"
namespace kinova {
Vector6 pose_error(const Pose& desired, const Pose& current) {
  Vector6 e;
  e.head<3>() = desired.p - current.p;
  Eigen::Quaterniond qe = desired.R * current.R.inverse();
  if (qe.w() < 0) qe.coeffs() *= -1.0;          // shortest geodesic
  Eigen::AngleAxisd aa(qe.normalized());
  e.tail<3>() = aa.angle() * aa.axis();
  return e;
}
}  // namespace kinova
