#include "kinova_lowlevel/joint_velocity_mode.h"

#include <algorithm>
#include <cmath>

#include "kinova_lowlevel/units.h"
#include "kinova_lowlevel/velocity_reference.h"  // limit_joint_velocity
namespace kinova {

// The leash constant and its rationale live in velocity_reference.h
// (kVelocityRefMaxLead): the Supervisor's compliant velocity/twist path shares
// the same bound, deliberately.

JointVelocityMode::JointVelocityMode(Dynamics& dyn, JointVelocityParams p) : dyn_(dyn) {
  // Cache the URDF limits once. set_params runs on a non-RT thread and must never
  // touch Dynamics -- it is not thread-safe against the RT loop.
  dyn.joint_limits(q_lower_urdf_, q_upper_urdf_);
  dyn.velocity_limits(v_max_urdf_);
  for (int i = 0; i < kNumJoints; ++i)
    continuous_[i] = !std::isfinite(q_lower_urdf_[i]) && !std::isfinite(q_upper_urdf_[i]);
  seed_limits(p);
  params_[0] = p;
  params_[1] = p;
  ext_qd_[0].setZero();
  ext_qd_[1].setZero();
  ext_twist_[0].setZero();
  ext_twist_[1].setZero();
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
  ActuatorModes modes;
  modes.fill(ActuatorMode::kPosition);
  return modes;
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
  wd_.bump();  // must be LAST: its release publishes everything above it
}

void JointVelocityMode::set_twist_target(const Vector6& V) noexcept {
  const int next = 1 - tw_active_.load(std::memory_order_relaxed);
  ext_twist_[next] = V;
  tw_active_.store(next, std::memory_order_release);
  source_.store(Source::kTwist, std::memory_order_release);
  wd_.bump();  // must be LAST
}

void JointVelocityMode::set_command_timeout(double s) noexcept {
  wd_.arm(s >= 0.0 ? s : params().cmd_timeout_s);
}

void JointVelocityMode::on_enter(const JointFeedback& fb) {
  // Drop any target from a previous session: re-entering must not resume a motion
  // someone asked for minutes ago.
  source_.store(Source::kNone, std::memory_order_release);
  qd_target_.setZero();
  twist_target_.setZero();
  qd_cmd_.setZero();
  q_ref_ = fb.q;  // hold where we are until a target arrives
  frozen_ = false;
  twist_dls_.reset();
  wd_.reset();
}

void JointVelocityMode::limit(const JointVelocityParams& p, JointVec& qd) noexcept {
  // One cap for both velocity paths: the shared helper carries the
  // uniform-scale-then-clamp semantics (and its rationale), so the stiff and
  // compliant streams cannot drift apart (review finding). The only
  // difference is the source of the cap: this mode's user-configurable
  // max_qd (URDF-seeded) vs the compliant path's raw URDF ratings.
  kinova::limit_joint_velocity(p.max_qd, qd);
}

void JointVelocityMode::compute(const JointFeedback& fb, double dt_s, JointCommand& out) {
  const JointVelocityParams p = params();  // own a snapshot for the whole cycle

  // Staleness: the stream stopped, so stop moving. Zero velocity is the only safe
  // command for a stiff velocity mode -- holding the last velocity would keep the
  // arm travelling toward nothing. LATCHED, so disarming cannot un-freeze it.
  const bool stale = wd_.tick(dt_s);
  if (stale)
    frozen_ = true;
  else if (wd_.fresh())
    frozen_ = false;

  const Source src = source_.load(std::memory_order_acquire);
  if (frozen_ || src == Source::kNone) {
    qd_cmd_.setZero();
    // Frozen: freeze where the arm actually IS, not at the reference. A dead
    // client must not leave a leash's worth of travel still to finish. Before any
    // target the reference is the entry pose already (on_enter), so it is held.
    if (frozen_) q_ref_ = fb.q;
  } else {
    // Adopt the payload exactly when the counter moves, never merely because the
    // stream is not yet stale -- otherwise a target published before on_enter
    // would be picked up after it.
    if (src == Source::kJoint) {
      if (wd_.fresh()) qd_target_ = ext_qd_[qd_active_.load(std::memory_order_acquire)];
      qd_cmd_ = qd_target_;
    } else {
      if (wd_.fresh()) twist_target_ = ext_twist_[tw_active_.load(std::memory_order_acquire)];
      solve_twist(fb.q, twist_target_, p, qd_cmd_);
    }
    limit(p, qd_cmd_);
  }

  // Integrate the (limited) velocity into the position reference -- the shared
  // integrate/leash/wrap/clamp step (velocity_reference.h).
  integrate_leashed_reference(q_ref_, qd_cmd_, dt_s, fb.q, kVelocityRefMaxLead, continuous_,
                              q_lower_urdf_, q_upper_urdf_);

  out.mode = ActuatorMode::kPosition;
  out.position = q_ref_;
  // RtExecutor reuses one JointCommand across mode changes, so a torque or
  // velocity left by a previous mode would still be sitting in these fields.
  // The transport ignores them in kPosition today; that is not a reason to leave
  // stale setpoints where something later could act on them. The velocity this
  // mode integrates is readable through commanded().
  out.velocity.setZero();
  out.torque.setZero();
}

void JointVelocityMode::solve_twist(const JointVec& q, const Vector6& V,
                                    const JointVelocityParams& p, JointVec& qd_out) noexcept {
  dyn_.jacobian(q, J_);
  // The DLS math (adaptive damping + null-space posture) lives in the shared
  // solver (velocity_reference.h); this mode owns only the Jacobian call.
  TwistDlsParams dp;
  dp.dls_damping = p.dls_damping;
  dp.w_threshold = p.w_threshold;
  dp.dls_damping_max = p.dls_damping_max;
  dp.posture_gain = p.posture_gain;
  dp.q_rest = p.q_rest;
  twist_dls_.solve(J_, q, V, dp, continuous_, qd_out);
}

}  // namespace kinova
