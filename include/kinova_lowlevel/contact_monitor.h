#pragma once
#include <atomic>
#include <cstdint>
#include <limits>
#include <string>

#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/feedback_tap.h"
#include "kinova_lowlevel/joint_types.h"
#include "kinova_lowlevel/momentum_observer.h"

namespace kinova {

// "off" is not a mode here: it means no monitor is attached at all (the node decides).
enum class ContactDetection : uint8_t { kObserve, kEnforce };
enum class ContactPhase : uint8_t { kSettling, kArmed, kLatched };

struct ContactTrip {
  uint64_t cycle = 0;  // index of the step() call that tripped, 0 = the first since construction
  int joint = -1;      // 0-based, the first joint to reach the debounce count
  JointVec r = JointVec::Zero(), theta = JointVec::Zero(), q = JointVec::Zero(),
           qd = JointVec::Zero();
};

// Published every cycle. Fixed-size members only, no heap: the Seqlock copies it
// whole and a torn copy must never hold a bad pointer.
struct ContactStatus {
  ContactDetection detection = ContactDetection::kEnforce;
  ContactPhase phase = ContactPhase::kSettling;
  // The observer is past its 5/K settle since the last (re)init, so r is trusted.
  // Not the same as phase != kSettling: a run that restarts latched re-settles too.
  bool settled = false;
  JointVec r = JointVec::Zero();      // observer residual, external joint torque [N·m]
  JointVec theta = JointVec::Zero();  // active thresholds [N·m]
  uint64_t would_trips = 0;           // crossings that reached the debounce count (both modes)
  uint64_t bad_samples = 0;           // non-finite q/qd/tau skipped
  bool has_trip = false;
  ContactTrip last{};
};

struct ContactMonitorConfig {
  ContactDetection detection = ContactDetection::kEnforce;
  double gain = 25.0;       // observer K [rad/s]
  int debounce_cycles = 5;  // consecutive good cycles over θ to trip
  // No usable default: both start NaN, so a config that never sets them is refused.
  // initial_theta must pass validate_contact_thresholds against ceiling.
  JointVec initial_theta = JointVec::Constant(std::numeric_limits<double>::quiet_NaN());
  JointVec ceiling = JointVec::Constant(std::numeric_limits<double>::quiet_NaN());
};

struct ThresholdCheck {
  bool ok = false;
  std::string message;
};

// Non-RT, pure. Refuses a threshold that is non-finite, <= 0, or above ceiling[i], and
// a ceiling that is non-finite or <= 0. The message names the joint (1-based) and the
// limit, e.g. "joint 5 threshold 7.1 N·m must be finite, > 0 and <= ceiling 7 N·m".
ThresholdCheck validate_contact_thresholds(const JointVec& theta, const JointVec& ceiling);

// Unexpected-contact detection on the momentum-observer residual r (see
// momentum_observer.h). A joint is over when |r_i| > θ_i; a trip is any joint over for
// debounce_cycles consecutive good cycles.
//
// Threads. Two parties, each method belongs to one of them:
//   - The RT thread (the executor): on_run_start() and step(). It is the only writer of
//     the latch, the status and the observer, and the only user of `dyn`.
//   - One non-RT thread (the Supervisor): set_thresholds() and request_clear(). Single
//     writer; a second concurrent caller is not supported.
//   latched(), status(), ceiling() and detection() are safe from any thread.
// Handoffs, all lock-free:
//   - thresholds: two buffers + an atomic index (as JointTorqueMode::set_torque),
//     adopted at the top of the next step(). A reader copy can only tear if two
//     set_thresholds() calls land inside one 56-byte copy; each joint then holds a
//     value from one of two validated vectors.
//   - clear: an atomic flag, consumed (exchange) by the next step() or on_run_start().
//   - latch: an atomic written only by the RT thread.
//   - status: a Seqlock, stored at the end of every step() and by on_run_start().
//
// Phases.
//   - kSettling: for 5/K seconds of observer time after the observer is (re)initialized
//     (construction, on_run_start(), a consumed clear), r is not trusted: no counting,
//     no trip, the debounce counters stay zero.
//   - kArmed: counting. Per joint, the counter climbs while over and drops to 0 on the
//     first cycle that is not. The cycle a counter reaches debounce_cycles is a crossing:
//     would_trips += 1 and `last` records it. Joints reaching the count on the same
//     cycle are one crossing, recorded under the lowest-numbered joint.
//   - kLatched (enforce only): the first crossing latches. The latch holds until a clear
//     and nothing else: on_run_start() re-settles the observer but keeps the latch
//     (unless a clear is pending, which it consumes), and phase stays kLatched. While
//     latched the observer still runs (r stays live in the status) but nothing is
//     counted.
// Observe never latches. A joint counts once per crossing: its counter saturates at
// debounce_cycles, so a sustained push is one would-trip, and it re-arms only after it
// drops back under θ.
//
// Samples. dt_s is the time since the previous step(); the executor measures it between
// feedback reads. A missed cycle (missed = true) or one with any non-finite q/qd/tau does
// not step the observer or touch the counters; its dt is carried into the next good step.
// Non-finite samples are counted in bad_samples; a missed cycle is not (its feedback is
// stale, not bad). A good step integrates the carried dt plus its own, capped at 1/K: at
// K*dt = 1 a still arm reads its external torque exactly, above it the Euler step
// overshoots. A long gap (back-to-back timeouts, a set_actuator_modes handshake) does
// not re-seed the observer: that would blind the monitor for 5/K at every mode switch.
// The settling window counts observer time, i.e. the capped steps.
class ContactMonitor {
 public:
  // Throws std::invalid_argument naming the field: gain not finite and > 0,
  // debounce_cycles < 1, a ceiling or initial_theta that fails validation.
  ContactMonitor(Dynamics& dyn, ContactMonitorConfig cfg);

  // --- non-RT, single writer (the Supervisor) ---
  // Caller has validated theta against ceiling(); takes effect on the next step().
  void set_thresholds(const JointVec& theta) noexcept;
  // Consumed by the next step() or on_run_start(): unlatch, re-initialize the
  // observer, phase kSettling.
  void request_clear() noexcept;

  // --- any thread ---
  bool latched() const noexcept { return latched_.load(std::memory_order_acquire); }
  // Latest published status; false on a torn read (retry later).
  bool status(ContactStatus& out) const noexcept { return status_pub_.load(out); }
  // Advances on every status store. Unchanged across a reader's polls means the RT
  // loop is not stepping (no run, or ArmLink waiting for a clear): the status is frozen.
  uint32_t status_version() const noexcept { return status_pub_.version(); }
  const JointVec& ceiling() const noexcept { return cfg_.ceiling; }
  ContactDetection detection() const noexcept { return cfg_.detection; }

  // --- RT thread only ---
  // At each RtExecutor::run start: consume a pending clear, then re-initialize the
  // observer (kSettling unless latched). Without a clear the latch is kept.
  void on_run_start() noexcept;
  // One cycle on the cycle's feedback, dt_s after the previous one. Returns true while
  // latched (enforce only).
  bool step(const JointFeedback& fb, double dt_s, bool missed) noexcept;

 private:
  void restart() noexcept;  // observer reset, counters zero, no carried dt
  void publish() noexcept;

  const ContactMonitorConfig cfg_;  // validated before observer_ is built
  const double settle_s_;
  const double max_step_s_;  // 1/K: the longest step the observer integrates
  MomentumObserver observer_;

  // Threshold handoff: Supervisor writes the inactive buffer, then flips the index.
  JointVec theta_buf_[2];
  std::atomic<int> theta_active_{0};
  std::atomic<bool> clear_requested_{false};
  std::atomic<bool> latched_{false};

  // RT-thread-only state.
  JointVec theta_ = JointVec::Zero();  // adopted at the top of each step()
  int count_[kNumJoints] = {};
  double pending_dt_ = 0.0;  // dt of skipped cycles, owed to the next good step
  uint64_t cycle_ = 0;
  ContactStatus status_{};
  Seqlock<ContactStatus> status_pub_;
};

}  // namespace kinova
