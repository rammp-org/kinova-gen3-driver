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
  write_count_.fetch_add(1, std::memory_order_release);
}

void JointTorqueMode::on_enter(const JointFeedback&) {
  // Enter as gravity-comp hold: discard any prior command and reset the
  // watchdog. Snapping last_seen_write_ to the current count means a command
  // sent before entry is NOT treated as fresh.
  tau_ff_target_.setZero();
  tau_ff_applied_.setZero();
  stale_s_ = 0.0;
  last_seen_write_ = write_count_.load(std::memory_order_acquire);
}

void JointTorqueMode::compute(const JointFeedback& fb, double dt_s,
                              JointCommand& out) {
  // --- read the published feedforward and advance the staleness watchdog ----
  const uint64_t wc = write_count_.load(std::memory_order_acquire);
  if (wc != last_seen_write_) {
    const int active = tau_ff_active_.load(std::memory_order_acquire);
    tau_ff_target_ = tau_ff_buf_[active];  // fresh command this cycle
    last_seen_write_ = wc;
    stale_s_ = 0.0;
  } else {
    stale_s_ += dt_s;
  }

  // Resolve the applied feedforward: hold target until stale, then ramp to 0.
  if (p_.cmd_timeout_s > 0.0 && stale_s_ >= p_.cmd_timeout_s) {
    if (p_.cmd_decay_s > 0.0) {
      // Per-cycle decrement proportional to the held target magnitude; reaches
      // exactly zero after cmd_decay_s of staleness. Sign preserved.
      const double frac = dt_s / p_.cmd_decay_s;
      for (int i = 0; i < kNumJoints; ++i) {
        const double dec = frac * std::abs(tau_ff_target_[i]);
        double a = tau_ff_applied_[i];
        if (a > dec) a -= dec;
        else if (a < -dec) a += dec;
        else a = 0.0;
        tau_ff_applied_[i] = a;
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
