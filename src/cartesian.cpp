#include "kinova_lowlevel/cartesian.h"

#include <Eigen/Cholesky>  // LDLT for the damped 6x6 solve
namespace kinova {
Vector6 pose_error(const Pose& desired, const Pose& current) {
  Vector6 e;
  e.head<3>() = desired.p - current.p;
  Eigen::Quaterniond qe = desired.R * current.R.inverse();
  if (qe.w() < 0) qe.coeffs() *= -1.0;  // shortest geodesic
  Eigen::AngleAxisd aa(qe.normalized());
  e.tail<3>() = aa.angle() * aa.axis();
  return e;
}

Vector6 ee_wrench_from_residual(const Jacobian6& J, const JointVec& tau_ext, double damping) {
  Eigen::Matrix<double, 6, 6> A;
  damped_jjt(J, damping, A);
  return A.ldlt().solve(J * tau_ext);
}
}  // namespace kinova
