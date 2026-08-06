#include "kinova_lowlevel/diff_ik.h"
#include <algorithm>
#include <cmath>
#include <Eigen/Cholesky>
namespace kinova {
namespace {

// Push away from a hard stop: 0 in the interior, ramping linearly to +/-1 at the
// stop. Continuous joints (infinite limits) contribute nothing.
double limit_gradient(double q, double lo, double hi, double margin) {
  if (margin <= 0.0) return 0.0;
  if (std::isfinite(lo) && q < lo + margin) return (lo + margin - q) / margin;
  if (std::isfinite(hi) && q > hi - margin) return (hi - margin - q) / margin;
  return 0.0;
}

// Posture error, wrapped to [-pi, pi] for continuous joints. Without this, a rest
// angle of +3.0 against a measured -3.0 reads as a 6.0 rad error and the bias
// drives the joint the long way round instead of the 0.28 rad short way.
double posture_error(double q_rest, double q, bool continuous) {
  const double d = q_rest - q;
  return continuous ? std::remainder(d, 2.0 * M_PI) : d;
}

}  // namespace

DiffIkSolver::DiffIkSolver(Dynamics& dyn, DiffIkParams p) : dyn_(dyn), p_(p) {
  J_.setZero();
}

IkResult DiffIkSolver::solve(const Pose& target, JointVec& q) {
  // With a posture bias or limit avoidance active, reaching the commanded pose is
  // NOT a reason to stop: the null-space motion that resolves the redundant DOF
  // happens precisely when the task error is already zero (operator holding
  // still). Only the pure-task case may take the fast path.
  const bool secondary_active = (p_.posture_gain != 0.0 || p_.limit_gain != 0.0);
  IkResult r;
  for (int k = 0; k <= p_.max_iters; ++k) {
    Vector6 e = pose_error(target, dyn_.fk(q));
    r.pos_err = e.head<3>().norm();
    r.rot_err = e.tail<3>().norm();
    r.iters = k;
    // `converged` always reports the TASK, so callers can trust it as a tracking
    // metric regardless of what the secondary objectives are doing.
    r.converged = (r.pos_err < p_.pos_tol && r.rot_err < p_.rot_tol);
    if (k == p_.max_iters || (r.converged && !secondary_active)) return r;

    // Bound the error fed to the solve so a far/unreachable target produces a
    // bounded step instead of one enormous jump.
    if (r.pos_err > p_.max_pos_err) e.head<3>() *= p_.max_pos_err / r.pos_err;
    if (r.rot_err > p_.max_rot_err) e.tail<3>() *= p_.max_rot_err / r.rot_err;

    dyn_.jacobian(q, J_);
    Eigen::Matrix<double, 6, 6> A = J_ * J_.transpose();
    A.diagonal().array() += p_.damping * p_.damping;
    // Fixed-size LDLT: no heap. Same pattern as the null-space projector in
    // cartesian_impedance_mode.cpp. Explicit type rather than `auto` -- an Eigen
    // decomposition bound by `auto` can dangle on its operand.
    const Eigen::LDLT<Eigen::Matrix<double, 6, 6>> A_ldlt(A);

    JointVec dq = J_.transpose() * A_ldlt.solve(e);

    // Secondary objectives, projected into null(J) so they cannot disturb the
    // task. The projector uses the DAMPED inverse, so it is approximate near
    // singularities -- deliberate: robustness beats exactness here.
    JointVec q0;
    for (int i = 0; i < kNumJoints; ++i) {
      const bool continuous =
          !std::isfinite(p_.q_lower[i]) && !std::isfinite(p_.q_upper[i]);
      q0[i] = p_.posture_gain * posture_error(p_.q_rest[i], q[i], continuous) +
              p_.limit_gain * limit_gradient(q[i], p_.q_lower[i], p_.q_upper[i],
                                             p_.limit_margin);
    }
    const Eigen::Matrix<double, 6, kNumJoints> AinvJ = A_ldlt.solve(J_);
    dq += q0 - J_.transpose() * (AinvJ * q0);      // (I - Jt A^-1 J) q0

    // Fixed point: task met and the secondary objectives have nothing left to
    // pull on. Stopping here is what keeps the steady-state cost low when the
    // operator holds a pose, instead of burning max_iters every cycle.
    if (dq.lpNorm<Eigen::Infinity>() < p_.min_step) return r;

    for (int i = 0; i < kNumJoints; ++i) {
      q[i] += std::clamp(dq[i], -p_.max_joint_step, p_.max_joint_step);
      const double lo = p_.q_lower[i] + p_.limit_clamp_margin;
      const double hi = p_.q_upper[i] - p_.limit_clamp_margin;
      if (std::isfinite(lo) && q[i] < lo) q[i] = lo;
      if (std::isfinite(hi) && q[i] > hi) q[i] = hi;
    }
  }
  return r;
}

}  // namespace kinova
