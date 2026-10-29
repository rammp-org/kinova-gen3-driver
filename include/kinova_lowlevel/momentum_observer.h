#pragma once
#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/joint_types.h"

namespace kinova {
// Generalized-momentum observer for external joint torque (De Luca, Albu-Schaffer,
// Haddadin, Hirzinger, IROS 2006). Ported from alexchunlin's observer on
// feat/momentum-observer-demo (kinova-gen3-ros2).
//
// Convention (same as JointFeedback::tau): tau is the torque the actuator applies, so
// at rest tau == g(q) and the dynamics read
//     M(q) qdd + C(q, qd) qd + g(q) = tau + tau_ext
// i.e. tau_ext = M qdd + C qd + g - tau. The quasi-static g(q) - tau drops the first
// two terms, which is wrong by the arm's own inertia while it moves.
//
// The observer avoids qdd. With p = M(q) qd and Mdot = C + C^T,
//     dp/dt = tau + C^T qd - g + tau_ext
// Everything on the right but tau_ext is known, so integrate it and compare with the
// measured p; the gap is the external torque:
//     r = K ( p - p0 - integral( tau + C^T qd - g + r ) dt )
// r follows tau_ext as a first-order filter with cutoff K rad/s. Fed with the torque
// sensors (not the motor command), so friction and tracking error stay out of r.
// Model error shows up as a slow pose-dependent bias, like the quasi-static residual.
//
// Integration is explicit Euler. step() is RT-safe: no allocation, no locks.
class MomentumObserver {
 public:
  // Throws std::invalid_argument unless gain is finite and > 0.
  MomentumObserver(Dynamics& dyn, double gain);
  // The next step() seeds p0 and returns zeros.
  void reset() noexcept;
  // Returns r, the external joint torque estimate [N·m].
  const JointVec& step(const JointVec& q, const JointVec& qd, const JointVec& tau,
                       double dt_s) noexcept;
  const JointVec& residual() const noexcept { return r_; }
  double gain() const noexcept { return gain_; }
  double time_since_reset() const noexcept { return time_s_; }  // sum of dt since reset

 private:
  Dynamics& dyn_;
  double gain_;
  bool seeded_ = false;
  double time_s_ = 0.0;
  JointMat M_ = JointMat::Zero();
  JointMat C_ = JointMat::Zero();
  JointVec g_ = JointVec::Zero();
  JointVec p_ = JointVec::Zero();
  JointVec p0_ = JointVec::Zero();
  JointVec integral_ = JointVec::Zero();
  JointVec r_ = JointVec::Zero();
};
}  // namespace kinova
