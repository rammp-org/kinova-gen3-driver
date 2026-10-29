#include "kinova_lowlevel/momentum_observer.h"

#include <cmath>
#include <stdexcept>

namespace kinova {
MomentumObserver::MomentumObserver(Dynamics& dyn, double gain) : dyn_(dyn), gain_(gain) {
  if (!std::isfinite(gain) || gain <= 0.0)
    throw std::invalid_argument("MomentumObserver: gain must be finite and > 0");
}

void MomentumObserver::reset() noexcept {
  seeded_ = false;
  time_s_ = 0.0;
  integral_.setZero();
  r_.setZero();
}

const JointVec& MomentumObserver::step(const JointVec& q, const JointVec& qd, const JointVec& tau,
                                       double dt_s) noexcept {
  dyn_.mass_matrix(q, M_);
  p_ = M_ * qd;
  if (!seeded_) {
    p0_ = p_;
    integral_.setZero();
    r_.setZero();
    time_s_ = 0.0;
    seeded_ = true;
    return r_;
  }
  dyn_.coriolis(q, qd, C_);
  dyn_.gravity(q, g_);
  integral_ += (tau + C_.transpose() * qd - g_ + r_) * dt_s;
  r_ = gain_ * (p_ - p0_ - integral_);
  time_s_ += dt_s;
  return r_;
}
}  // namespace kinova
