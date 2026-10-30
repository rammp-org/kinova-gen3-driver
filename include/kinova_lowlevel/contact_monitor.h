#pragma once
#include <Eigen/Core>
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

// A held object as a point mass: kg, and its centre of mass in the EE frame [m].
struct TipPayload {
  double mass = 0.0;
  Eigen::Vector3d com = Eigen::Vector3d::Zero();
};

// Where a grasp or release is (request_load_change_begin / _end):
//   kTaring   averaging r over weigh_window_cycles before the gripper moves
//   kHolding  tared; the gripper may close or open; waiting for the end request
//   kWeighing averaging r over weigh_window_cycles after the lift
enum class LoadChange : uint8_t { kNone, kTaring, kHolding, kWeighing };

enum class WeighOutcome : uint8_t {
  kNone,            // no weigh yet
  kOk,              // weighed; `payload` is now compensated (a fit under 50 g is empty)
  kNotStillAtTare,  // the arm moved during the tare; back to kNone, begin again
  kNotStill,        // the arm moved during the weigh; still kHolding, end again
  kUnexplained,     // the change is not a load in the gripper (fit_rms above max_fit_rms:
                    // still on the table, someone holding the arm); still kHolding
  kOutOfRange,      // fitted mass < -0.05 kg or above max_payload_kg; still kHolding
  kNoLoadChange,    // an end request with no begin
  kAborted,         // a clear, a run restart or set_payload ended the load change
};

struct WeighResult {
  uint32_t count = 0;  // bumps on every outcome, so a reader can tell a new one
  WeighOutcome outcome = WeighOutcome::kNone;
  double delta_kg = 0.0;  // fitted change in mass
  double fit_rms = 0.0;   // N·m RMS over joints that the fitted payload did not explain
  TipPayload payload{};   // what the monitor compensates after this outcome
};

struct ContactTrip {
  uint64_t cycle = 0;    // index of the step() call that tripped, 0 = the first since construction
  int joint = -1;        // 0-based, the first joint to reach the debounce count
  bool impulse = false;  // reached it on the fast band alone, not on |r|
  JointVec r = JointVec::Zero(), theta = JointVec::Zero(), q = JointVec::Zero(),
           qd = JointVec::Zero();
};

// Published every cycle. Fixed-size members only, no heap: the Seqlock copies it
// whole and a torn copy must never hold a bad pointer.
struct ContactStatus {
  ContactDetection detection = ContactDetection::kEnforce;
  ContactPhase phase = ContactPhase::kSettling;
  // External joint torque [N·m]: the observer residual less the payload's gravity torque.
  JointVec r = JointVec::Zero();
  JointVec theta = JointVec::Zero();  // active thresholds [N·m]
  JointVec fast = JointVec::Zero();   // r minus its slow average: what the impulse trip reads
  bool impulse = false;               // the impulse trip is configured
  JointVec impulse_theta = JointVec::Zero();
  TipPayload payload{};
  LoadChange load_change = LoadChange::kNone;
  WeighResult weigh{};
  uint64_t would_trips = 0;  // crossings that reached the debounce count (both modes)
  uint64_t bad_samples = 0;  // non-finite q/qd/tau skipped
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

  // Impulse trip, optional. A joint is also over when |r_i - s_i| > impulse_theta_i,
  // s a first-order average of r with time constant slow_tau_s. A collision is abrupt
  // and shows in r - s; a lean, hand-guiding, a resting contact and the model's
  // pose-dependent bias are slow and do not, so impulse_theta can sit well under θ.
  // With `impulse` set, impulse_theta must pass validate_contact_thresholds.
  bool impulse = false;
  JointVec impulse_theta = JointVec::Constant(std::numeric_limits<double>::quiet_NaN());
  double slow_tau_s = 0.2;

  // Weighing. A window is weigh_window_cycles good armed cycles; it counts as still
  // when max |qd| over it stays under still_qd.
  int weigh_window_cycles = 300;
  double still_qd = 0.02;       // rad/s
  double max_fit_rms = 1.0;     // N·m
  double max_payload_kg = 4.0;  // a heavier fit is a misread, not an object
};

struct ThresholdCheck {
  bool ok = false;
  std::string message;
};

// Non-RT, pure. Refuses a threshold that is non-finite, <= 0, or above ceiling[i], and
// a ceiling that is non-finite or <= 0. The message names the joint (1-based) and the
// limit, e.g. "joint 5 threshold 7.1 N·m must be finite, > 0 and <= ceiling 7 N·m".
ThresholdCheck validate_contact_thresholds(const JointVec& theta, const JointVec& ceiling);

// Non-RT, pure. Refuses a non-finite or negative mass, one above max_kg, and a non-finite
// centre of mass or one more than 1 m from the EE frame.
ThresholdCheck validate_tip_payload(const TipPayload& p, double max_kg);

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
// Payload. A declared or weighed point mass at the EE: its gravity torque Y(q) (m, m c)
// (Dynamics::tip_payload_regressor) is added back to the observer residual, so a held
// object reads as zero external torque and θ does not have to be loosened to carry it.
// Only the monitor's view changes: `dyn` is not modified, so the control modes that
// share it are unaffected. The object's own inertia is not compensated; on a 0.5 kg
// bottle carried at speed that leaves about 0.2 N·m (1.5 kg: 0.6 N·m) on the worst
// joint, against the >= 2 N·m thresholds.
//
// Grasp and release. request_load_change_begin() with the arm still, before the gripper
// moves: r is averaged over one window (kTaring), then kHolding. Close or open the
// gripper, lift, hold still, request_load_change_end(com): r is averaged again
// (kWeighing), the change is fitted as a mass at `com` (EE frame) and, if it explains
// the change, compensated from then on. From the begin request to the outcome, both
// trips read r with the torques a held load can produce projected out (span of Y: a
// vertical force and a horizontal moment at the EE, three of seven directions), so the
// grasp does not trip but a bump from the side, on the forearm or the elbow still does.
// Wait for kHolding before moving the gripper; a tare taken while it moves is wrong.
//
// Observe never latches. A joint counts once per crossing: its counter saturates at
// debounce_cycles, so a sustained push is one would-trip, and it re-arms only after it
// drops back under θ.
//
// Samples. A missed cycle (missed = true) or one with any non-finite q/qd/tau does not
// step the observer or touch the counters; its dt is carried into the next good step so
// the observer integrates over the true elapsed time. Non-finite samples are counted in
// bad_samples; a missed cycle is not (its feedback is stale, not bad).
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
  // Caller has validated theta against ceiling(); takes effect on the next step().
  // Stored but unused unless the config enabled the impulse trip.
  void set_impulse_thresholds(const JointVec& theta) noexcept;
  // Caller has validated with validate_tip_payload. Replaces the compensated payload
  // on the next step() (mass 0 removes it) and ends a load change in progress.
  void set_payload(const TipPayload& p) noexcept;
  // See "Grasp and release". Both are consumed by the next armed step().
  void request_load_change_begin() noexcept;
  void request_load_change_end(const Eigen::Vector3d& com) noexcept;

  // --- any thread ---
  bool latched() const noexcept { return latched_.load(std::memory_order_acquire); }
  // Latest published status; false on a torn read (retry later).
  bool status(ContactStatus& out) const noexcept { return status_pub_.load(out); }
  const JointVec& ceiling() const noexcept { return cfg_.ceiling; }
  ContactDetection detection() const noexcept { return cfg_.detection; }

  // --- RT thread only ---
  // At each RtExecutor::run start: consume a pending clear, then re-initialize the
  // observer (kSettling unless latched). Without a clear the latch is kept.
  void on_run_start() noexcept;
  // One cycle on the cycle's feedback. Returns true while latched (enforce only).
  bool step(const JointFeedback& fb, double dt_s, bool missed) noexcept;

 private:
  void restart() noexcept;  // observer reset, counters zero, no carried dt
  void publish() noexcept;
  JointVec project_off_load(const JointVec& f) const noexcept;  // f less its part in span(Y_)
  void load_change_step(const JointFeedback& fb) noexcept;
  void finish_weigh() noexcept;
  void set_outcome(WeighOutcome o, double delta_kg = 0.0, double fit_rms = 0.0) noexcept;
  void adopt_payload(const TipPayload& p) noexcept;  // shifts the slow band with it

  const ContactMonitorConfig cfg_;  // validated before observer_ is built
  const double settle_s_;
  Dynamics& dyn_;
  MomentumObserver observer_;

  // Threshold handoff: Supervisor writes the inactive buffer, then flips the index.
  JointVec theta_buf_[2];
  std::atomic<int> theta_active_{0};
  std::atomic<bool> clear_requested_{false};
  std::atomic<bool> latched_{false};
  JointVec impulse_buf_[2];
  std::atomic<int> impulse_active_{0};
  // Payload: write the inactive buffer, flip the index, bump the sequence.
  TipPayload payload_buf_[2];
  std::atomic<int> payload_active_{0};
  std::atomic<uint32_t> payload_seq_{0};
  std::atomic<bool> begin_requested_{false};
  std::atomic<bool> end_requested_{false};
  Eigen::Vector3d end_com_buf_ = Eigen::Vector3d::Zero();

  // RT-thread-only state.
  JointVec theta_ = JointVec::Zero();  // adopted at the top of each step()
  int count_[kNumJoints] = {};
  double pending_dt_ = 0.0;  // dt of skipped cycles, owed to the next good step
  uint64_t cycle_ = 0;
  JointVec slow_ = JointVec::Zero();  // first-order average of r
  JointVec impulse_theta_ = JointVec::Zero();
  uint32_t payload_seen_ = 0;
  TipPayload payload_{};
  Eigen::Vector4d payload_theta_ = Eigen::Vector4d::Zero();  // (m, m c)
  TipRegressor Y_ = TipRegressor::Zero();                    // at this cycle's q, when needed
  bool begin_pending_ = false, end_pending_ = false;
  Eigen::Vector3d weigh_com_ = Eigen::Vector3d::Zero();
  LoadChange load_ = LoadChange::kNone;
  JointVec load_ref_ = JointVec::Zero();  // r's slow average at the begin request
  JointVec tare_r_ = JointVec::Zero();
  int win_n_ = 0;
  JointVec win_r_ = JointVec::Zero(), win_q_ = JointVec::Zero();
  double win_qd_max_ = 0.0;
  ContactStatus status_{};
  Seqlock<ContactStatus> status_pub_;
};

}  // namespace kinova
