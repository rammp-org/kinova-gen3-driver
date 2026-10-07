#include "kinova_lowlevel/velocity_reference.h"

#include <cmath>
namespace kinova {

void TwistDlsSolver::solve(const Jacobian6& J, const JointVec& q, const Vector6& V,
                           const TwistDlsParams& p, const std::array<bool, kNumJoints>& continuous,
                           JointVec& qd_out) noexcept {
  A_.noalias() = J * J.transpose();  // 6x6, symmetric positive semi-definite

  // Decompose UNDAMPED first, purely to measure conditioning: LDLT hands us
  // det(J J^T) as prod(D) for free, so manipulability costs no extra solve.
  ldlt_.compute(A_);
  const double w2 = ldlt_.vectorD().prod();
  w_last_ = w2 > 0.0 ? std::sqrt(w2) : 0.0;

  // Damping rises as manipulability falls. This is a REQUIRED part of the design,
  // not a refinement: in velocity mode whatever is computed goes to the actuators,
  // so there is no torque clamp standing behind a bad solve.
  double lambda = p.dls_damping;
  if (p.w_threshold > 0.0 && w_last_ < p.w_threshold) {
    const double r = 1.0 - w_last_ / p.w_threshold;  // 0 at threshold, 1 at singular
    lambda = p.dls_damping + (p.dls_damping_max - p.dls_damping) * r * r;
  }
  A_.diagonal().array() += lambda * lambda;
  ldlt_.compute(A_);

  // Task term: qd = J^T (J J^T + lambda^2 I)^-1 V
  y_.noalias() = ldlt_.solve(V);
  qd_out.noalias() = J.transpose() * y_;

  if (p.posture_gain == 0.0) return;

  // Null-space posture bias, projected without ever forming the 7x7 projector:
  //   (I - J^T (JJ^T + lambda^2 I)^-1 J) b  ==  b - J^T ((JJ^T + lambda^2 I)^-1 (J b))
  // Continuous joints must take the SHORT way to the rest posture, or the bias
  // pushes the joint most of a turn the wrong way.
  for (int i = 0; i < kNumJoints; ++i) {
    double d = p.q_rest[i] - q[i];
    if (continuous[i]) d = wrap_to_pi(d);
    bias_[i] = p.posture_gain * d;
  }
  y_.noalias() = ldlt_.solve(J * bias_);
  qd_out.noalias() += bias_ - J.transpose() * y_;
}

}  // namespace kinova
