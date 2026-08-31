#include "kinova_lowlevel/joint_velocity_mode.h"
#include <algorithm>
#include <cmath>
#include "kinova_lowlevel/units.h"
namespace kinova {

JointVelocityMode::JointVelocityMode(Dynamics& dyn, JointVelocityParams p)
    : dyn_(dyn) {
  // Cache the URDF limits once. set_params runs on a non-RT thread and must never
  // touch Dynamics -- it is not thread-safe against the RT loop.
  JointVec lo, hi;
  dyn.joint_limits(lo, hi);
  dyn.velocity_limits(v_max_urdf_);
  for (int i = 0; i < kNumJoints; ++i)
    continuous_[i] = !std::isfinite(lo[i]) && !std::isfinite(hi[i]);
  seed_limits(p);
  params_[0] = p;
  params_[1] = p;
  ext_qd_[0].setZero(); ext_qd_[1].setZero();
  ext_twist_[0].setZero(); ext_twist_[1].setZero();
  wd_.arm(p.cmd_timeout_s);
}

void JointVelocityMode::seed_limits(JointVelocityParams& p) const noexcept {
  for (int i = 0; i < kNumJoints; ++i) {
    const double v = std::isfinite(p.max_qd[i]) ? p.max_qd[i] : v_max_urdf_[i];
    // A caller may ask for less than the hardware can do, never for more. A
    // negative request stops the joint rather than reversing it, which would also
    // violate std::clamp's lo <= hi precondition below.
    p.max_qd[i] = std::clamp(v, 0.0, v_max_urdf_[i]);
  }
}

ActuatorModes JointVelocityMode::required_modes() const {
  ActuatorModes modes; modes.fill(ActuatorMode::kVelocity); return modes;
}

JointVelocityParams JointVelocityMode::params() const noexcept {
  return params_[params_active_.load(std::memory_order_acquire)];
}

void JointVelocityMode::set_params(const JointVelocityParams& p) noexcept {
  const int next = 1 - params_active_.load(std::memory_order_relaxed);
  params_[next] = p;
  seed_limits(params_[next]);
  params_active_.store(next, std::memory_order_release);
}

void JointVelocityMode::set_velocity_target(const JointVec& qd_d) noexcept {
  const int next = 1 - qd_active_.load(std::memory_order_relaxed);
  ext_qd_[next] = qd_d;
  qd_active_.store(next, std::memory_order_release);
  source_.store(Source::kJoint, std::memory_order_release);
  wd_.bump();   // must be LAST: its release publishes everything above it
}

void JointVelocityMode::set_twist_target(const Vector6& V) noexcept {
  const int next = 1 - tw_active_.load(std::memory_order_relaxed);
  ext_twist_[next] = V;
  tw_active_.store(next, std::memory_order_release);
  source_.store(Source::kTwist, std::memory_order_release);
  wd_.bump();   // must be LAST
}

void JointVelocityMode::set_command_timeout(double s) noexcept {
  wd_.arm(s >= 0.0 ? s : params().cmd_timeout_s);
}

void JointVelocityMode::on_enter(const JointFeedback&) {
  // Drop any target from a previous session: re-entering must not resume a motion
  // someone asked for minutes ago.
  source_.store(Source::kNone, std::memory_order_release);
  qd_target_.setZero();
  twist_target_.setZero();
  qd_cmd_.setZero();
  frozen_ = false;
  w_last_ = 0.0;
  wd_.reset();
}

void JointVelocityMode::limit(const JointVelocityParams& p, JointVec& qd) noexcept {
  // Scale UNIFORMLY so the fastest joint just reaches its cap. A bare per-joint
  // clamp would silently ROTATE the commanded EE twist when one joint saturates,
  // which is the one thing a mode named "velocity" must not do.
  double s = 1.0;
  for (int i = 0; i < kNumJoints; ++i) {
    const double a = std::abs(qd[i]);
    if (a > p.max_qd[i] && a > 0.0) s = std::min(s, p.max_qd[i] / a);
  }
  qd *= s;
  // Hard backstop: scaling covers the normal case, this holds even when max_qd
  // contains a zero (scale would be 0/0) or the scale underflows.
  for (int i = 0; i < kNumJoints; ++i)
    qd[i] = std::clamp(qd[i], -p.max_qd[i], p.max_qd[i]);
}

void JointVelocityMode::compute(const JointFeedback& fb, double dt_s,
                                JointCommand& out) {
  const JointVelocityParams p = params();   // own a snapshot for the whole cycle
  out.mode = ActuatorMode::kVelocity;

  // Staleness: the stream stopped, so stop moving. Zero is the only safe command
  // for a stiff velocity mode -- holding the last velocity would keep the arm
  // travelling toward nothing. LATCHED, so disarming cannot un-freeze it.
  const bool stale = wd_.tick(dt_s);
  if (stale) frozen_ = true;
  else if (wd_.fresh()) frozen_ = false;

  const Source src = source_.load(std::memory_order_acquire);
  if (frozen_ || src == Source::kNone) {
    qd_cmd_.setZero();
    out.velocity = qd_cmd_;
    // RtExecutor reuses one JointCommand across mode changes, so a position or
    // torque left by a previous mode would still be sitting in these fields --
    // don't leave a previous mode's setpoint lying around. ECHO the measured
    // position, exactly as every other mode does (joint_torque_mode.cpp,
    // joint_impedance_mode.cpp, cartesian_impedance_mode.cpp,
    // joint_position_mode.cpp), rather than writing zero: zero is not "unset", it
    // is "all joints at 0 rad", a meaningful and wrong command that is harmless
    // only because KortexTransport happens to overwrite the field.
    out.position = fb.q;
    out.torque.setZero();
    return;
  }

  // Adopt the payload exactly when the counter moves, never merely because the
  // stream is not yet stale -- otherwise a target published before on_enter would
  // be picked up after it.
  if (src == Source::kJoint) {
    if (wd_.fresh()) qd_target_ = ext_qd_[qd_active_.load(std::memory_order_acquire)];
    qd_cmd_ = qd_target_;
  } else {
    if (wd_.fresh()) twist_target_ = ext_twist_[tw_active_.load(std::memory_order_acquire)];
    solve_twist(fb.q, twist_target_, p, qd_cmd_);
  }

  limit(p, qd_cmd_);
  out.velocity = qd_cmd_;
  // Same as the frozen path above: echo the measured position rather than zeroing
  // it, so a previous mode's setpoint cannot survive here and the field still
  // carries the value every other mode puts there.
  out.position = fb.q;
  out.torque.setZero();
}

void JointVelocityMode::solve_twist(const JointVec& q, const Vector6& V,
                                    const JointVelocityParams& p,
                                    JointVec& qd_out) noexcept {
  dyn_.jacobian(q, J_);
  A_.noalias() = J_ * J_.transpose();          // 6x6, symmetric positive semi-definite

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
    const double r = 1.0 - w_last_ / p.w_threshold;   // 0 at threshold, 1 at singular
    lambda = p.dls_damping + (p.dls_damping_max - p.dls_damping) * r * r;
  }
  A_.diagonal().array() += lambda * lambda;
  ldlt_.compute(A_);

  // Task term: qd = J^T (J J^T + lambda^2 I)^-1 V
  y_.noalias() = ldlt_.solve(V);
  qd_out.noalias() = J_.transpose() * y_;

  if (p.posture_gain == 0.0) return;

  // Null-space posture bias, projected without ever forming the 7x7 projector:
  //   (I - J^T (JJ^T + lambda^2 I)^-1 J) b  ==  b - J^T ((JJ^T + lambda^2 I)^-1 (J b))
  // Continuous joints must take the SHORT way to the rest posture, or the bias
  // pushes the joint most of a turn the wrong way.
  for (int i = 0; i < kNumJoints; ++i) {
    double d = p.q_rest[i] - q[i];
    if (continuous_[i]) d = wrap_to_pi(d);
    bias_[i] = p.posture_gain * d;
  }
  y_.noalias() = ldlt_.solve(J_ * bias_);
  qd_out.noalias() += bias_ - J_.transpose() * y_;
}

}  // namespace kinova
