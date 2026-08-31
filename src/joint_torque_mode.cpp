#include "kinova_lowlevel/joint_torque_mode.h"

#include <algorithm>
#include <cmath>

namespace kinova {

JointTorqueMode::JointTorqueMode(Dynamics& dyn, JointTorqueParams p)
    : dyn_(dyn), p_(p) {
  tau_ff_buf_[0].setZero();
  tau_ff_buf_[1].setZero();
  g_.setZero();
  tau_.setZero();
  wd_.arm(p_.cmd_timeout_s);
}

ActuatorModes JointTorqueMode::required_modes() const {
  ActuatorModes modes;
  modes.fill(ActuatorMode::kTorque);
  return modes;
}

void JointTorqueMode::set_torque(const JointVec& tau_ff) noexcept {
  // Single non-RT writer: fill the inactive buffer, publish the index, then bump
  // the write counter. compute() detects freshness via the counter and reads the
  // active buffer; the release/acquire on the index makes the buffer write
  // visible to the RT reader.
  // relaxed load is safe: this thread is the sole writer of tau_ff_active_; a
  // second writer would corrupt buffer selection, so no concurrent writer exists.
  const int inactive = 1 - tau_ff_active_.load(std::memory_order_relaxed);
  tau_ff_buf_[inactive] = tau_ff;
  tau_ff_active_.store(inactive, std::memory_order_release);
  wd_.bump();
}

// s >= 0 arms with s; s < 0 restores this mode's own configured default.
void JointTorqueMode::set_command_timeout(double s) noexcept {
  wd_.arm(s >= 0.0 ? s : p_.cmd_timeout_s);
}

void JointTorqueMode::on_enter(const JointFeedback&) {
  // Enter as gravity-comp hold: discard any prior command and reset the
  // watchdog. CommandWatchdog::reset() snaps its last-seen count to the current
  // one, so a command sent before entry is NOT treated as fresh.
  //
  // Benign race: set_torque() does buffer store -> release-store the index ->
  // fetch_add the counter (three separate steps). A non-RT set_torque() call
  // that lands between the index store and the counter bump here can still be
  // observed as "already counted" (or not) depending on interleaving, so a
  // command issued concurrently with on_enter may or may not be discarded —
  // it is adopted on the very next cycle either way. This is fine under the
  // documented single-supervisor-thread usage (on_enter and set_torque are not
  // expected to race in practice), but it is weaker than "always discarded."
  tau_ff_target_.setZero();
  tau_ff_applied_.setZero();
  wd_.reset();
}

void JointTorqueMode::compute(const JointFeedback& fb, double dt_s,
                              JointCommand& out) {
  // --- read the published feedforward and advance the staleness watchdog ----
  // The counter GATES the buffer read: adopt only on the cycle the counter
  // moves, never merely because the stream is not yet stale -- otherwise a
  // command published before on_enter would be resurrected after it.
  const bool stale = wd_.tick(dt_s);
  if (wd_.fresh()) {
    const int active = tau_ff_active_.load(std::memory_order_acquire);
    tau_ff_target_ = tau_ff_buf_[active];  // fresh command this cycle
  }

  // Resolve the applied feedforward: hold target until stale, then ramp to 0.
  if (stale) {
    if (p_.cmd_decay_s > 0.0) {
      // Per-cycle decrement proportional to the held target magnitude; reaches
      // exactly zero after cmd_decay_s of staleness. Sign preserved.
      const double frac = dt_s / p_.cmd_decay_s;
      for (int i = 0; i < kNumJoints; ++i) {
        const double step = frac * std::abs(tau_ff_target_[i]);
        double applied = tau_ff_applied_[i];
        if (applied > step) applied -= step;
        else if (applied < -step) applied += step;
        else applied = 0.0;          // within one step of zero: land exactly on it
        tau_ff_applied_[i] = applied;
      }
    } else {
      tau_ff_applied_.setZero();
    }
  } else {
    tau_ff_applied_ = tau_ff_target_;
  }

  // --- compose and clamp ----------------------------------------------------
  dyn_.gravity(fb.q, g_);  // RT-safe: no alloc
  tau_ = p_.scale * g_ - p_.damping * fb.qd + tau_ff_applied_;
  for (int i = 0; i < kNumJoints; ++i) {
    tau_[i] = std::clamp(tau_[i], -p_.torque_limit[i], p_.torque_limit[i]);
  }
  out.mode = ActuatorMode::kTorque;
  out.torque = tau_;
  out.position = fb.q;
}

}  // namespace kinova
