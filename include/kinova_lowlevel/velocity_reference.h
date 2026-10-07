#pragma once
#include <Eigen/Cholesky>
#include <algorithm>
#include <array>

#include "kinova_lowlevel/cartesian_types.h"
#include "kinova_lowlevel/joint_types.h"
#include "kinova_lowlevel/units.h"
namespace kinova {

// The velocity-command-to-position-reference mechanism proven in
// JointVelocityMode since v1.1.1 (#34), factored out so compliant streaming
// (Supervisor sampler -> JointImpedanceMode) runs the SAME math instead of a
// divergent copy. Two pieces: the integrate-and-leash step, and the
// twist -> qd damped-least-squares resolution. Both are RT-safe (no
// allocation, no locks); JointVelocityMode calls them from compute(), the
// Supervisor from its non-RT sampler thread.

// How far the reference may lead the MEASURED position, rad. This is the windup
// guard for a blocked joint -- contact, a limit, an arm that cannot keep up:
// without it the reference marches on while q stays put and the arm snaps across
// the whole gap the instant it frees. Deliberately tighter than
// JointPositionMode's 0.35 rad default: a velocity stream has no destination to
// justify a long lead, and under contact this number IS the bound on how hard the
// position servo (or impedance spring) pushes.
constexpr double kVelocityRefMaxLead = 0.1;

// One step of velocity -> position-reference integration: integrate, then the
// same tail JointPositionMode runs -- leash to the measurement, keep continuous
// joints in the transport's (-pi, pi] representation, never command past a limit.
inline void integrate_leashed_reference(JointVec& q_ref, const JointVec& qd, double dt_s,
                                        const JointVec& q_meas, double max_lead,
                                        const std::array<bool, kNumJoints>& continuous,
                                        const JointVec& q_lower, const JointVec& q_upper) noexcept {
  for (int i = 0; i < kNumJoints; ++i) {
    q_ref[i] += qd[i] * dt_s;

    // Leash the reference to the MEASURED position. A no-op whenever the arm is
    // tracking; it only bites when the arm cannot follow. Continuous joints take
    // the short way, or a reference just across the wrap reads as a 2*pi lead.
    double lead = q_ref[i] - q_meas[i];
    if (continuous[i]) lead = wrap_to_pi(lead);
    q_ref[i] = q_meas[i] + std::clamp(lead, -max_lead, max_lead);

    if (continuous[i])
      q_ref[i] = wrap_to_pi(q_ref[i]);
    else
      q_ref[i] = std::clamp(q_ref[i], q_lower[i], q_upper[i]);
  }
}

// Cap a joint-velocity command against the URDF ratings, the same shape as
// JointVelocityMode::limit: scale UNIFORMLY so the fastest joint just reaches
// its cap (a bare per-joint clamp would rotate a commanded EE twist when one
// joint saturates), then hard-clamp as the backstop. The compliant stream
// path must cap like the stiff path does -- a wrong-units qd would otherwise
// integrate silently and drag the arm on the leash (review finding).
inline void limit_joint_velocity(const JointVec& v_max, JointVec& qd) noexcept {
  double s = 1.0;
  for (int i = 0; i < kNumJoints; ++i) {
    const double a = std::abs(qd[i]);
    if (a > v_max[i] && a > 0.0) s = std::min(s, v_max[i] / a);
  }
  qd *= s;
  for (int i = 0; i < kNumJoints; ++i) qd[i] = std::clamp(qd[i], -v_max[i], v_max[i]);
}

// Parameters for the twist resolution. Field semantics and defaults are
// identical to the matching JointVelocityParams fields -- see that struct's
// comments for the damping/manipulability rationale and the measured numbers
// the defaults are pinned to.
struct TwistDlsParams {
  double dls_damping = 1e-3;
  double w_threshold = 0.0033;
  double dls_damping_max = 0.10;
  double posture_gain = 0.15;
  JointVec q_rest =  // elbow-up home; matches DiffIkParams::q_rest
      (JointVec() << 0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57).finished();
};

// EE twist [linear; angular] (base frame) -> qd by damped least squares with a
// null-space posture bias. The caller supplies the Jacobian: JointVelocityMode
// computes it on the RT thread, the Supervisor under its Dynamics mutex -- the
// solver itself never touches Dynamics. Preallocated scratch; solve() is
// RT-safe.
class TwistDlsSolver {
 public:
  void solve(const Jacobian6& J, const JointVec& q, const Vector6& V, const TwistDlsParams& p,
             const std::array<bool, kNumJoints>& continuous, JointVec& qd_out) noexcept;

  // Manipulability at the last solve, sqrt(det(J J^T)). 0 until a twist has
  // been solved. Worth watching: it is what drives the damping.
  double last_manipulability() const noexcept { return w_last_; }
  void reset() noexcept { w_last_ = 0.0; }

 private:
  Eigen::Matrix<double, 6, 6> A_ = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt_;
  Vector6 y_ = Vector6::Zero();
  JointVec bias_ = JointVec::Zero();
  double w_last_ = 0.0;
};

}  // namespace kinova
