#include "kinova_lowlevel/contact_monitor.h"

#include <algorithm>
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

ThresholdCheck validate_tip_payload(const TipPayload& p, double max_kg) {
  if (!(std::isfinite(p.mass) && p.mass >= 0.0 && p.mass <= max_kg)) {
    std::ostringstream m;
    m << "payload mass " << p.mass << " kg must be finite, >= 0 and <= " << max_kg << " kg";
    return {false, m.str()};
  }
  if (!(p.com.allFinite() && p.com.norm() <= 1.0)) {
    std::ostringstream m;
    m << "payload centre of mass (" << p.com.x() << ", " << p.com.y() << ", " << p.com.z()
      << ") m must be finite and within 1 m of the EE frame";
    return {false, m.str()};
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
  if (c.impulse) {
    const ThresholdCheck imp = validate_contact_thresholds(c.impulse_theta, c.ceiling);
    if (!imp.ok) throw std::invalid_argument("ContactMonitorConfig.impulse_theta: " + imp.message);
  }
  if (!(std::isfinite(c.slow_tau_s) && c.slow_tau_s > 0.0))
    throw std::invalid_argument("ContactMonitorConfig.slow_tau_s must be finite and > 0");
  if (c.weigh_window_cycles < 1)
    throw std::invalid_argument("ContactMonitorConfig.weigh_window_cycles must be >= 1");
  if (!(std::isfinite(c.still_qd) && c.still_qd > 0.0))
    throw std::invalid_argument("ContactMonitorConfig.still_qd must be finite and > 0");
  if (!(std::isfinite(c.max_fit_rms) && c.max_fit_rms > 0.0))
    throw std::invalid_argument("ContactMonitorConfig.max_fit_rms must be finite and > 0");
  if (!(std::isfinite(c.max_payload_kg) && c.max_payload_kg > 0.0))
    throw std::invalid_argument("ContactMonitorConfig.max_payload_kg must be finite and > 0");
  return c;
}
}  // namespace

ContactMonitor::ContactMonitor(Dynamics& dyn, ContactMonitorConfig cfg)
    : cfg_(validated(cfg)), settle_s_(5.0 / cfg_.gain), dyn_(dyn), observer_(dyn, cfg_.gain) {
  theta_buf_[0] = cfg_.initial_theta;
  theta_buf_[1] = cfg_.initial_theta;
  theta_ = cfg_.initial_theta;
  if (cfg_.impulse) {
    impulse_buf_[0] = impulse_buf_[1] = impulse_theta_ = cfg_.impulse_theta;
  }
  status_.detection = cfg_.detection;
  status_.theta = theta_;
  status_.impulse = cfg_.impulse;
  status_.impulse_theta = impulse_theta_;
  publish();
}

void ContactMonitor::set_impulse_thresholds(const JointVec& theta) noexcept {
  const int inactive = 1 - impulse_active_.load(std::memory_order_relaxed);
  impulse_buf_[inactive] = theta;
  impulse_active_.store(inactive, std::memory_order_release);
}

void ContactMonitor::set_payload(const TipPayload& p) noexcept {
  const int inactive = 1 - payload_active_.load(std::memory_order_relaxed);
  payload_buf_[inactive] = p;
  payload_active_.store(inactive, std::memory_order_release);
  payload_seq_.fetch_add(1, std::memory_order_release);
}

void ContactMonitor::request_load_change_begin() noexcept {
  begin_requested_.store(true, std::memory_order_release);
}

void ContactMonitor::request_load_change_end(const Eigen::Vector3d& com) noexcept {
  end_com_buf_ = com;
  end_requested_.store(true, std::memory_order_release);
}

void ContactMonitor::set_outcome(WeighOutcome o, double delta_kg, double fit_rms) noexcept {
  status_.weigh.count += 1;
  status_.weigh.outcome = o;
  status_.weigh.delta_kg = delta_kg;
  status_.weigh.fit_rms = fit_rms;
  status_.weigh.payload = payload_;
}

void ContactMonitor::adopt_payload(const TipPayload& p) noexcept {
  Eigen::Vector4d theta;
  theta << p.mass, p.mass * p.com;
  // r jumps by Y (theta - old) as the new payload is compensated; move the slow
  // average with it so the jump is not an impulse.
  slow_ += Y_ * (theta - payload_theta_);
  payload_ = p;
  payload_theta_ = theta;
}

JointVec ContactMonitor::project_off_load(const JointVec& f) const noexcept {
  // Least squares on the 4 columns of Y, which span only 3 directions (a mass on the
  // gravity line through the EE gives no torque): a small ridge keeps the fixed-size
  // 4x4 solve defined without touching the well-conditioned directions.
  Eigen::Matrix4d A = Y_.transpose() * Y_;
  A.diagonal().array() += 1e-9 * A.trace() + 1e-12;
  const Eigen::Vector4d x = A.ldlt().solve(Y_.transpose() * f);
  return f - Y_ * x;
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
  // The slow average re-seeds while settling. A load change cannot survive the gap.
  if (load_ != LoadChange::kNone) set_outcome(WeighOutcome::kAborted);
  load_ = LoadChange::kNone;
  begin_pending_ = end_pending_ = false;
  status_.load_change = load_;
  status_.r = observer_.residual();
  status_.phase =
      latched_.load(std::memory_order_relaxed) ? ContactPhase::kLatched : ContactPhase::kSettling;
}

void ContactMonitor::on_run_start() noexcept {
  // A clear requested between runs is consumed here, before the executor picks its
  // first mode: the run then adopts the requested mode directly, not float first.
  if (clear_requested_.exchange(false, std::memory_order_acq_rel))
    latched_.store(false, std::memory_order_release);
  restart();
  publish();
}

void ContactMonitor::publish() noexcept { status_pub_.store(status_); }

bool ContactMonitor::step(const JointFeedback& fb, double dt_s, bool missed) noexcept {
  const uint64_t cycle = cycle_++;

  // 1. A pending clear, then the handoffs.
  if (clear_requested_.exchange(false, std::memory_order_acq_rel)) {
    latched_.store(false, std::memory_order_release);
    restart();
  }
  theta_ = theta_buf_[theta_active_.load(std::memory_order_acquire)];
  status_.theta = theta_;
  if (cfg_.impulse) {
    impulse_theta_ = impulse_buf_[impulse_active_.load(std::memory_order_acquire)];
    status_.impulse_theta = impulse_theta_;
  }
  if (begin_requested_.exchange(false, std::memory_order_acq_rel)) begin_pending_ = true;
  if (end_requested_.exchange(false, std::memory_order_acq_rel)) {
    weigh_com_ = end_com_buf_;
    end_pending_ = true;
  }
  const uint32_t seq = payload_seq_.load(std::memory_order_acquire);
  const bool payload_pending = seq != payload_seen_;
  // The RT thread is the only writer of latched_.
  const bool latched = latched_.load(std::memory_order_relaxed);

  // 2. Skip a missed or non-finite sample; its dt is owed to the next good one.
  if (missed || !(fb.q.allFinite() && fb.qd.allFinite() && fb.tau.allFinite())) {
    if (!missed) ++status_.bad_samples;
    pending_dt_ += dt_s;
    publish();
    return latched;
  }

  // 3. Step the observer over the true elapsed time, and compensate the payload.
  const double dt = pending_dt_ + dt_s;
  JointVec r = observer_.step(fb.q, fb.qd, fb.tau, dt);
  pending_dt_ = 0.0;
  const bool need_y = payload_.mass > 0.0 || load_ != LoadChange::kNone || begin_pending_ ||
                      end_pending_ || payload_pending;
  if (need_y) dyn_.tip_payload_regressor(fb.q, Y_);
  if (payload_pending) {
    payload_seen_ = seq;
    adopt_payload(payload_buf_[payload_active_.load(std::memory_order_acquire)]);
    if (load_ != LoadChange::kNone) set_outcome(WeighOutcome::kAborted);
    load_ = LoadChange::kNone;
    begin_pending_ = end_pending_ = false;
  }
  if (payload_.mass > 0.0) r += Y_ * payload_theta_;
  status_.r = r;
  status_.payload = payload_;

  if (latched) {
    status_.phase = ContactPhase::kLatched;
    publish();
    return true;
  }
  if (observer_.time_since_reset() < settle_s_) {
    status_.phase = ContactPhase::kSettling;
    slow_ = r;  // r is not trusted yet; the average starts where it ends up
    status_.fast.setZero();
    publish();
    return false;
  }
  status_.phase = ContactPhase::kArmed;

  // 4. Grasp and release: requests, windows, the weigh.
  load_change_step(fb);
  r = status_.r;  // a weigh that finished this cycle has re-compensated it

  // 5. What the two trips read. During a load change, the part a held load can
  // produce is projected out: of r relative to the slow average at the begin
  // request, and of the fast band.
  JointVec fast = r - slow_;
  JointVec level = r;
  if (load_ != LoadChange::kNone) {
    fast = project_off_load(fast);
    level = load_ref_ + project_off_load(r - load_ref_);
  }
  status_.fast = fast;

  // 6. Debounce. A counter saturates at the count, so it crosses once per stretch over.
  int tripped = -1;
  bool tripped_impulse = false;
  for (int i = 0; i < kNumJoints; ++i) {
    const bool over_level = std::abs(level[i]) > theta_[i];
    const bool over_fast = cfg_.impulse && std::abs(fast[i]) > impulse_theta_[i];
    if (over_level || over_fast) {
      if (count_[i] < cfg_.debounce_cycles && ++count_[i] == cfg_.debounce_cycles && tripped < 0) {
        tripped = i;
        tripped_impulse = !over_level;
      }
    } else {
      count_[i] = 0;
    }
  }
  if (tripped >= 0) {
    ++status_.would_trips;
    status_.has_trip = true;
    status_.last.cycle = cycle;
    status_.last.joint = tripped;
    status_.last.impulse = tripped_impulse;
    status_.last.r = r;
    status_.last.theta = theta_;
    status_.last.q = fb.q;
    status_.last.qd = fb.qd;
    if (cfg_.detection == ContactDetection::kEnforce) {
      latched_.store(true, std::memory_order_release);
      status_.phase = ContactPhase::kLatched;
    }
  }

  // 7. The slow average, after the trips have read the fast band against it.
  slow_ += (dt / (cfg_.slow_tau_s + dt)) * (r - slow_);

  // 8. Publish.
  status_.load_change = load_;
  publish();
  return status_.phase == ContactPhase::kLatched;
}

void ContactMonitor::load_change_step(const JointFeedback& fb) noexcept {
  if (begin_pending_) {
    begin_pending_ = false;
    load_ = LoadChange::kTaring;
    load_ref_ = slow_;
    win_n_ = 0;
  }
  if (end_pending_) {
    if (load_ == LoadChange::kHolding) {
      end_pending_ = false;
      load_ = LoadChange::kWeighing;
      win_n_ = 0;
    } else if (load_ == LoadChange::kNone) {
      end_pending_ = false;
      set_outcome(WeighOutcome::kNoLoadChange);
    }  // kTaring: held until the tare is done; kWeighing: already weighing
  }
  if (load_ != LoadChange::kTaring && load_ != LoadChange::kWeighing) return;

  if (win_n_ == 0) {
    win_r_.setZero();
    win_q_.setZero();
    win_qd_max_ = 0.0;
  }
  win_r_ += status_.r;
  win_q_ += fb.q;
  win_qd_max_ = std::max(win_qd_max_, fb.qd.cwiseAbs().maxCoeff());
  if (++win_n_ < cfg_.weigh_window_cycles) return;

  const bool still = win_qd_max_ < cfg_.still_qd;
  win_n_ = 0;
  if (load_ == LoadChange::kTaring) {
    if (still) {
      tare_r_ = win_r_ / cfg_.weigh_window_cycles;
      load_ = LoadChange::kHolding;
    } else {
      load_ = LoadChange::kNone;
      set_outcome(WeighOutcome::kNotStillAtTare);
    }
    return;
  }
  if (!still) {
    load_ = LoadChange::kHolding;
    set_outcome(WeighOutcome::kNotStill);
    return;
  }
  finish_weigh();
}

void ContactMonitor::finish_weigh() noexcept {
  // The held load moved r by -Y (delta theta). With the centre of mass given, only the
  // mass is fitted: one pose is enough, and the tare at nearly the same pose cancels
  // the model's pose-dependent bias, which is larger than a light object's torque.
  const JointVec r_mean = win_r_ / cfg_.weigh_window_cycles;
  const JointVec q_mean = win_q_ / cfg_.weigh_window_cycles;
  const JointVec dr = r_mean - tare_r_;
  TipRegressor Y;
  dyn_.tip_payload_regressor(q_mean, Y);
  Eigen::Vector4d dir;
  dir << 1.0, weigh_com_;
  const JointVec y = Y * dir;
  const double yy = y.squaredNorm();
  if (!(yy > 1e-12)) {
    load_ = LoadChange::kHolding;
    set_outcome(WeighOutcome::kUnexplained);
    return;
  }
  const double dm = -y.dot(dr) / yy;
  const double fit_rms = std::sqrt((y * dm + dr).squaredNorm() / kNumJoints);
  const Eigen::Vector4d theta = payload_theta_ + dm * dir;
  if (fit_rms > cfg_.max_fit_rms) {
    load_ = LoadChange::kHolding;
    set_outcome(WeighOutcome::kUnexplained, dm, fit_rms);
    return;
  }
  if (theta[0] < -0.05 || theta[0] > cfg_.max_payload_kg) {
    load_ = LoadChange::kHolding;
    set_outcome(WeighOutcome::kOutOfRange, dm, fit_rms);
    return;
  }
  TipPayload p;
  // Under 50 g is empty: below what a tare a few cm from the weigh pose resolves (the
  // bias moves with the pose), and below anything worth compensating.
  if (theta[0] > 0.05) {
    p.mass = theta[0];
    p.com = theta.tail<3>() / theta[0];
  }
  // Y_ is this cycle's regressor (need_y held: a load change was active).
  adopt_payload(p);
  // Re-seat the average on the compensated r: what is left of the grasp step is now
  // explained, and must not read as an impulse once the projection stops.
  status_.r += Y_ * (payload_theta_ - (theta - dm * dir));
  slow_ = status_.r;
  load_ = LoadChange::kNone;
  set_outcome(WeighOutcome::kOk, dm, fit_rms);
}

}  // namespace kinova
