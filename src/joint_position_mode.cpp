#include "kinova_lowlevel/joint_position_mode.h"
#include <algorithm>
#include <cmath>
#include "kinova_lowlevel/units.h"
namespace kinova {

JointPositionMode::JointPositionMode(Dynamics& dyn, JointPositionParams p)
    : dyn_(dyn), ik_(dyn, p.ik) {
  // Cache the URDF limits once. set_params runs on a non-RT thread and must never
  // touch Dynamics -- it is not thread-safe against the RT loop.
  dyn.joint_limits(q_lower_urdf_, q_upper_urdf_);
  dyn.velocity_limits(v_max_urdf_);
  for (int i = 0; i < kNumJoints; ++i) {
    continuous_[i] =
        !std::isfinite(q_lower_urdf_[i]) && !std::isfinite(q_upper_urdf_[i]);
  }
  seed_limits(p);
  params_[0] = p;
  params_[1] = p;
  // ik_(dyn, p.ik) ran in the initialiser list, BEFORE seed_limits() above, so the
  // solver would otherwise hold the unseeded (infinite) joint limits for the life
  // of the mode. Push the seeded copy in now.
  ik_.set_params(params_[0].ik);
  wd_.arm(p.cmd_timeout_s);
}

void JointPositionMode::seed_limits(JointPositionParams& p) const noexcept {
  for (int i = 0; i < kNumJoints; ++i) {
    if (!std::isfinite(p.q_lower[i])) p.q_lower[i] = q_lower_urdf_[i];
    if (!std::isfinite(p.q_upper[i])) p.q_upper[i] = q_upper_urdf_[i];
    // Speed: seed when unset, otherwise clamp into [0, URDF]. A caller may ask
    // for less than the hardware can do, never for more -- and a negative
    // request freezes the reference rather than reversing it, which would make
    // std::clamp's lo > hi precondition fail below.
    const double v =
        std::isfinite(p.max_ref_speed[i]) ? p.max_ref_speed[i] : v_max_urdf_[i];
    p.max_ref_speed[i] = std::clamp(v, 0.0, v_max_urdf_[i]);
    if (!std::isfinite(p.ik.q_lower[i])) p.ik.q_lower[i] = q_lower_urdf_[i];
    if (!std::isfinite(p.ik.q_upper[i])) p.ik.q_upper[i] = q_upper_urdf_[i];
  }
}

ActuatorModes JointPositionMode::required_modes() const {
  ActuatorModes modes; modes.fill(ActuatorMode::kPosition); return modes;
}

JointPositionParams JointPositionMode::params() const noexcept {
  return params_[params_active_.load(std::memory_order_acquire)];  // copy active slot
}

void JointPositionMode::set_params(const JointPositionParams& p) noexcept {
  const int next = 1 - params_active_.load(std::memory_order_relaxed);
  params_[next] = p;
  seed_limits(params_[next]);
  params_active_.store(next, std::memory_order_release);
  ik_.set_params(params_[next].ik);
}

void JointPositionMode::set_target(const JointVec& q_d) noexcept {
  const int next = 1 - ext_active_.load(std::memory_order_relaxed);
  ext_target_[next] = q_d;
  ext_active_.store(next, std::memory_order_release);
  source_.store(TargetSource::kJoint, std::memory_order_release);
  wd_.bump();   // must be LAST: its release publishes everything above it
}

void JointPositionMode::set_target(const Pose& x_d) noexcept {
  const int next = 1 - pose_active_.load(std::memory_order_relaxed);
  pose_target_[next] = x_d;
  pose_active_.store(next, std::memory_order_release);
  source_.store(TargetSource::kPose, std::memory_order_release);
  wd_.bump();   // must be LAST: its release publishes everything above it
}

// s >= 0 arms with s; s < 0 restores this mode's own configured default.
void JointPositionMode::set_command_timeout(double s) noexcept {
  wd_.arm(s >= 0.0 ? s : params().cmd_timeout_s);
}

void JointPositionMode::on_enter(const JointFeedback& fb) {
  entry_q_ = fb.q;    // hold where we are
  q_ref_ = fb.q;
  // Drop any target from a previous session. Re-entering the mode must not yank
  // the arm toward a configuration someone asked for minutes ago.
  source_.store(TargetSource::kEntry, std::memory_order_release);
  ik_bad_s_ = 0.0;
  ik_faulted_.store(false, std::memory_order_release);
  last_ik_ = IkResult{};
  wd_.reset();
}

void JointPositionMode::compute(const JointFeedback& fb, double dt_s,
                                JointCommand& out) {
  const JointPositionParams p = params();   // own a snapshot for the whole cycle

  // Staleness: the stream stopped, so stop chasing it. Freeze where the arm
  // actually IS -- both the reference and the target for this cycle. Parking at
  // the last reference would keep the rate limiter slewing toward a destination
  // nobody is asking for; leaving the target in place would slew straight back
  // out of the freeze. Disarmed (cmd_timeout_s <= 0) this never fires.
  const bool stale = wd_.tick(dt_s);
  if (stale) q_ref_ = fb.q;

  JointVec target;
  if (stale) {
    target = fb.q;
  } else {
    switch (source_.load(std::memory_order_acquire)) {
      case TargetSource::kJoint:
        target = ext_target_[ext_active_.load(std::memory_order_acquire)];
        last_ik_ = IkResult{};   // last_ik() means THIS cycle's solve; without the
                                 // reset a stale result from a previous pose target
                                 // outlives the target itself and reads as an IK
                                 // that never ran having run.
        break;
      case TargetSource::kPose: {
        // Warm-start from the current reference: the solve refines in place, and
        // seeding from q_ref_ keeps the solution on the branch we are already on.
        ik_q_ = q_ref_;
        last_ik_ = ik_.solve(pose_target_[pose_active_.load(std::memory_order_acquire)],
                             ik_q_);
        // Sustained non-convergence is a fault; a single miss is not. Accumulated
        // clock-free from the dt the caller already has, exactly like the watchdog.
        if (last_ik_.converged) {
          ik_bad_s_ = 0.0;
        } else if (p.ik_fault_s > 0.0) {
          ik_bad_s_ += dt_s;
          if (ik_bad_s_ >= p.ik_fault_s)
            ik_faulted_.store(true, std::memory_order_release);
        }
        // Position mode is STIFF. Holding a stale reference while the client
        // believes it is tracking is the silent divergence this driver exists to
        // fail loud on -- so freeze at the measured configuration immediately and
        // let the sampler tear the session down.
        target = ik_faulted_.load(std::memory_order_relaxed) ? fb.q : ik_q_;
        break;
      }
      case TargetSource::kEntry:
      default:
        target = entry_q_;
        last_ik_ = IkResult{};
        break;
    }
  }

  for (int i = 0; i < kNumJoints; ++i) {
    const bool bounded =
        std::isfinite(p.q_lower[i]) && std::isfinite(p.q_upper[i]);

    // Clamp the TARGET into the software limits first, so the reference settles
    // exactly on the stop instead of carrying a permanent standing error against
    // an unreachable setpoint.
    double tgt = target[i];
    if (bounded) tgt = std::clamp(tgt, p.q_lower[i], p.q_upper[i]);

    // Rate limit. Continuous joints must take the SHORT way round: reference and
    // measurement both live in (-pi, pi], so a target on the far side of the wrap
    // reads as a ~2*pi error and the reference would walk most of a full turn in
    // the wrong direction -- which on the arm is the joint spinning.
    double err = tgt - q_ref_[i];
    if (continuous_[i]) err = wrap_to_pi(err);
    const double max_step = p.max_ref_speed[i] * dt_s;
    q_ref_[i] += std::clamp(err, -max_step, max_step);

    // Leash the reference to the MEASURED position. A no-op whenever the arm is
    // actually tracking; it only bites when the arm cannot follow. Without it a
    // blocked arm lets q_ref march all the way to the target while q stays put,
    // and the arm snaps across the whole accumulated gap the instant it frees.
    if (p.max_following_error > 0.0) {
      double lead = q_ref_[i] - fb.q[i];
      if (continuous_[i]) lead = wrap_to_pi(lead);
      q_ref_[i] = fb.q[i] + std::clamp(lead, -p.max_following_error,
                                       p.max_following_error);
    }

    // Keep the command in the SAME representation the transport reports
    // measurements in, (-pi, pi]. Kinematically a no-op.
    if (continuous_[i]) q_ref_[i] = wrap_to_pi(q_ref_[i]);

    // Final guard: whatever the leash did with the measured position, never
    // command past a position limit.
    if (bounded) q_ref_[i] = std::clamp(q_ref_[i], p.q_lower[i], p.q_upper[i]);
  }

  out.mode = ActuatorMode::kPosition;
  out.position = q_ref_;
  // RtExecutor reuses one JointCommand across mode changes, so a torque or
  // velocity left by a previous mode would still be sitting in these fields.
  // The transport ignores them in kPosition today; that is not a reason to leave
  // stale setpoints where something later could act on them.
  out.torque.setZero();
  out.velocity.setZero();
}

}  // namespace kinova
