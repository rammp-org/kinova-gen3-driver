#include "kinova_lowlevel/contact_monitor.h"

#include <cmath>
#include <sstream>
#include <stdexcept>

namespace kinova {

ThresholdCheck validate_contact_thresholds(const JointVec& theta, const JointVec& ceiling) {
  for (int i = 0; i < kNumJoints; ++i) {
    // Written as negated accepts so a NaN on either side is refused.
    if (!(std::isfinite(ceiling[i]) && ceiling[i] > 0.0)) {
      std::ostringstream m;
      m << "joint " << i + 1 << " ceiling " << ceiling[i] << " N·m must be finite and > 0";
      return {false, m.str()};
    }
    if (!(std::isfinite(theta[i]) && theta[i] > 0.0 && theta[i] <= ceiling[i])) {
      std::ostringstream m;
      m << "joint " << i + 1 << " threshold " << theta[i]
        << " N·m must be finite, > 0 and <= ceiling " << ceiling[i] << " N·m";
      return {false, m.str()};
    }
  }
  return {true, {}};
}

namespace {
ContactMonitorConfig validated(const ContactMonitorConfig& c) {
  if (!(std::isfinite(c.gain) && c.gain > 0.0))
    throw std::invalid_argument("ContactMonitorConfig.gain must be finite and > 0");
  if (c.debounce_cycles < 1)
    throw std::invalid_argument("ContactMonitorConfig.debounce_cycles must be >= 1");
  // Check the ceiling on its own first, so a bad one is not reported as a bad threshold.
  const ThresholdCheck ceil = validate_contact_thresholds(c.ceiling, c.ceiling);
  if (!ceil.ok) throw std::invalid_argument("ContactMonitorConfig.ceiling: " + ceil.message);
  const ThresholdCheck theta = validate_contact_thresholds(c.initial_theta, c.ceiling);
  if (!theta.ok)
    throw std::invalid_argument("ContactMonitorConfig.initial_theta: " + theta.message);
  return c;
}
}  // namespace

ContactMonitor::ContactMonitor(Dynamics& dyn, ContactMonitorConfig cfg)
    : cfg_(validated(cfg)), settle_s_(5.0 / cfg_.gain), observer_(dyn, cfg_.gain) {
  theta_buf_[0] = cfg_.initial_theta;
  theta_buf_[1] = cfg_.initial_theta;
  theta_ = cfg_.initial_theta;
  status_.detection = cfg_.detection;
  status_.theta = theta_;
  publish();
}

void ContactMonitor::set_thresholds(const JointVec& theta) noexcept {
  // Sole writer of theta_active_, so the relaxed load is safe. The release store makes
  // the buffer write visible to the acquire load in step().
  const int inactive = 1 - theta_active_.load(std::memory_order_relaxed);
  theta_buf_[inactive] = theta;
  theta_active_.store(inactive, std::memory_order_release);
}

void ContactMonitor::request_clear() noexcept {
  clear_requested_.store(true, std::memory_order_release);
}

void ContactMonitor::restart() noexcept {
  observer_.reset();
  for (int& c : count_) c = 0;
  pending_dt_ = 0.0;
  status_.r = observer_.residual();
  status_.phase =
      latched_.load(std::memory_order_relaxed) ? ContactPhase::kLatched : ContactPhase::kSettling;
}

void ContactMonitor::on_run_start() noexcept {
  restart();
  publish();
}

void ContactMonitor::publish() noexcept { status_pub_.store(status_); }

bool ContactMonitor::step(const JointFeedback& fb, double dt_s, bool missed) noexcept {
  const uint64_t cycle = cycle_++;

  // 1. A pending clear.
  if (clear_requested_.exchange(false, std::memory_order_acq_rel)) {
    latched_.store(false, std::memory_order_release);
    restart();
  }
  theta_ = theta_buf_[theta_active_.load(std::memory_order_acquire)];
  status_.theta = theta_;
  // The RT thread is the only writer of latched_.
  const bool latched = latched_.load(std::memory_order_relaxed);

  // 2. Skip a missed or non-finite sample; its dt is owed to the next good one.
  if (missed || !(fb.q.allFinite() && fb.qd.allFinite() && fb.tau.allFinite())) {
    if (!missed) ++status_.bad_samples;
    pending_dt_ += dt_s;
    publish();
    return latched;
  }

  // 3. Step the observer over the true elapsed time.
  status_.r = observer_.step(fb.q, fb.qd, fb.tau, pending_dt_ + dt_s);
  pending_dt_ = 0.0;
  if (latched) {
    status_.phase = ContactPhase::kLatched;
    publish();
    return true;
  }
  if (observer_.time_since_reset() < settle_s_) {
    status_.phase = ContactPhase::kSettling;
    publish();
    return false;
  }
  status_.phase = ContactPhase::kArmed;

  // 4. Debounce. A counter saturates at the count, so it crosses once per stretch over.
  int tripped = -1;
  for (int i = 0; i < kNumJoints; ++i) {
    if (std::abs(status_.r[i]) > theta_[i]) {
      if (count_[i] < cfg_.debounce_cycles && ++count_[i] == cfg_.debounce_cycles && tripped < 0)
        tripped = i;
    } else {
      count_[i] = 0;
    }
  }
  if (tripped >= 0) {
    ++status_.would_trips;
    status_.has_trip = true;
    status_.last.cycle = cycle;
    status_.last.joint = tripped;
    status_.last.r = status_.r;
    status_.last.theta = theta_;
    status_.last.q = fb.q;
    status_.last.qd = fb.qd;
    if (cfg_.detection == ContactDetection::kEnforce) {
      latched_.store(true, std::memory_order_release);
      status_.phase = ContactPhase::kLatched;
    }
  }

  // 5. Publish.
  publish();
  return status_.phase == ContactPhase::kLatched;
}

}  // namespace kinova
