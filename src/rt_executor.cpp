#include "kinova_lowlevel/rt_executor.h"

#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <vector>

#include "kinova_lowlevel/contact_monitor.h"

namespace kinova {

namespace {

inline int64_t ns_now() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return int64_t(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

// Sleep until absolute deadline (CLOCK_MONOTONIC). Coarse nanosleep that wakes
// ~100us early, then short busy-spin to the deadline. Ported from the validated
// rtos_testing/cpp/grav_comp_test.cpp prototype.
inline void sleep_until_ns(int64_t deadline_ns) {
  int64_t remaining = deadline_ns - ns_now();
  if (remaining > 200'000) {
    struct timespec req;
    req.tv_sec = (remaining - 100'000) / 1'000'000'000LL;
    req.tv_nsec = (remaining - 100'000) % 1'000'000'000LL;
    clock_nanosleep(CLOCK_MONOTONIC, 0, &req, nullptr);
  }
  while (ns_now() < deadline_ns) { /* spin */
  }
}

// Absolute clock_nanosleep to a CLOCK_MONOTONIC deadline (lower CPU, coarser).
inline void clock_nanosleep_abs(int64_t deadline_ns) {
  struct timespec ts;
  ts.tv_sec = deadline_ns / 1'000'000'000LL;
  ts.tv_nsec = deadline_ns % 1'000'000'000LL;
  // Retry on EINTR so a signal doesn't shorten the sleep.
  while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) == EINTR) {
  }
}

inline uint32_t clamp_u32(int64_t v) {
  if (v < 0) return 0;
  if (v > int64_t(UINT32_MAX)) return UINT32_MAX;
  return uint32_t(v);
}

}  // namespace

RtExecutor::RtExecutor(Transport& t, SampleRing& ring, ExecutorConfig cfg)
    : t_(t), ring_(ring), cfg_(cfg) {
  if (cfg_.max_consecutive_misses < 1 || cfg_.max_window_misses < 1 || cfg_.miss_window < 1) {
    throw std::invalid_argument(
        "ExecutorConfig: max_consecutive_misses, max_window_misses and miss_window must be >= 1");
  }
}

void RtExecutor::request_mode(ControlMode* m) noexcept {
  requested_.store(m, std::memory_order_release);
}

void RtExecutor::attach_contact(ContactMonitor& monitor, ControlMode& on_contact) noexcept {
  contact_ = &monitor;
  on_contact_ = &on_contact;
}

void RtExecutor::run(std::atomic<bool>& stop) {
  // Put this thread's scheduling back as it was when run() returns or throws.
  // The caller may reuse the thread (ArmLink reconnects from it), and every
  // thread created from it inherits its policy and affinity: KORTEX's TCP
  // receive and keepalive threads would otherwise start at SCHED_FIFO on the RT
  // core. mlockall and the cpu_dma_latency pin stay: they are process-wide.
  struct RestoreScheduling {
    int policy = SCHED_OTHER;
    sched_param param{};
    cpu_set_t cpus;
    bool ok = false;
    RestoreScheduling() {
      CPU_ZERO(&cpus);
      ok = pthread_getschedparam(pthread_self(), &policy, &param) == 0 &&
           sched_getaffinity(0, sizeof(cpus), &cpus) == 0;
    }
    ~RestoreScheduling() {
      if (!ok) return;
      pthread_setschedparam(pthread_self(), policy, &param);
      sched_setaffinity(0, sizeof(cpus), &cpus);
    }
  } restore_scheduling;

  // Best-effort RT setup (mlockall + SCHED_FIFO + affinity). Never throws; if it
  // fails (e.g. EPERM under non-sudo) the loop still runs at SCHED_OTHER.
  enable_rt(cfg_.rt);

  const double rate_hz = cfg_.rate_hz > 0.0 ? cfg_.rate_hz : 1000.0;
  const int64_t period_ns = int64_t(1e9 / rate_hz);
  const double dt_s = 1.0 / rate_hz;
  // compute() gets the wall time since the previous compute() (#103), so the
  // time a miss blocked counts on the modes' watchdogs, ramps and integrators.
  // Capped at max_consecutive_misses x (default cyclic timeout + period): every
  // gap a run survives is shorter, since that many misses in a row is
  // CommsLost. A longer gap is a stalled loop, not handed to the modes as one
  // step.
  const double max_dt_s = cfg_.max_consecutive_misses * (kDefaultCyclicTimeoutMs * 1e-3 + dt_s);
  int64_t last_compute_ns = -1;  // < 0: no compute yet in this mode, use dt_s

  ControlMode* active = nullptr;

  // Reusable stack locals; zero-initialized. No allocation in the loop body.
  JointCommand cmd;  // benign hold: kTorque, zero torque, zero position
  JointFeedback fb;

  auto comms_lost = [](const char* why, const char* last) {
    char msg[256];
    std::snprintf(msg, sizeof(msg), "RtExecutor: comms lost: %s (last: %s)", why, last);
    std::fprintf(stderr, "%s\n", msg);
    throw CommsLost(msg);
  };

  // One exchange. false on a TransportTimeout: fb keeps the last good feedback
  // and cmd is untouched. CommsLost on max_consecutive_misses in a row, or
  // max_window_misses within miss_window exchanges. Any other exception
  // propagates unchanged.
  int misses = 0;  // consecutive
  // Exchange index of each of the last max_window_misses misses, as a ring.
  std::vector<uint64_t> miss_at(size_t(cfg_.max_window_misses), 0);
  uint64_t exchanges = 0, total_misses = 0;
  auto exchange_ok = [&]() -> bool {
    const uint64_t n = exchanges++;
    try {
      t_.exchange(cmd, fb);
    } catch (const TransportTimeout& e) {
      if (++misses >= cfg_.max_consecutive_misses) {
        comms_lost("max_consecutive_misses exchange timeouts in a row", e.what());
      }
      miss_at[total_misses++ % miss_at.size()] = n;
      // The oldest of the last max_window_misses misses, this one included.
      if (total_misses >= miss_at.size() &&
          n - miss_at[total_misses % miss_at.size()] < uint64_t(cfg_.miss_window)) {
        comms_lost("max_window_misses exchange timeouts within miss_window exchanges", e.what());
      }
      return false;
    }
    misses = 0;
    return true;
  };

  // Consecutive good exchanges reporting a fault / not in low-level (exit_on_fault).
  int faulty = 0, not_low_level = 0;
  auto check_fault = [&]() {
    faulty = fb.fault ? faulty + 1 : 0;
    not_low_level = fb.low_level ? 0 : not_low_level + 1;
    const char* why = nullptr;
    if (faulty >= kFaultConfirmExchanges) why = "the arm reports a fault";
    if (not_low_level >= kLeftLowLevelConfirmExchanges) why = "the base left low-level servoing";
    if (!why) return;
    char msg[160];
    std::snprintf(msg, sizeof(msg), "RtExecutor: arm fault: %s", why);
    std::fprintf(stderr, "%s\n", msg);
    throw ArmFault(msg);
  };

  // Re-settle the contact observer for this run; the latch is kept (a clear
  // requested between runs is consumed here), so a run that starts latched adopts
  // on_contact_ at its first boundary (step 3).
  if (contact_) contact_->on_run_start();

  // Seed one feedback read with the benign hold command so fb is populated and
  // the first control compute sees real joint state. A lost frame is retried:
  // no mode may see fb before one real read.
  while (!exchange_ok()) {
  }
  // When the last feedback read returned. The contact monitor is stepped over the
  // measured time between reads: a miss, a mode-switch handshake or an overrun takes
  // longer than one period.
  int64_t last_read_ns = ns_now();

  uint64_t cycle_index = 0;
  int64_t boundary = ns_now() + period_ns;

  // cycle_ns accounting: we record the actual time from this cycle's loop start
  // (t0, the post-pace wake instant) to the loop end (after compute), i.e. the
  // measured work span of the cycle. wake_jitter is reported separately as the
  // lateness of the wake relative to the scheduled boundary. overrun is detected
  // when, after advancing the boundary by one period, we are already past it; it
  // is attributed to the CURRENT cycle's sample (the cycle that ran long).
  while (!stop.load(std::memory_order_acquire)) {
    // 1. pace to boundary
    if (cfg_.pacing == Pacing::kSleepSpin) {
      sleep_until_ns(boundary);
    } else {
      clock_nanosleep_abs(boundary);
    }

    // 2. wake instant + jitter
    const int64_t t0 = ns_now();
    const uint32_t wake_jitter = clamp_u32(t0 - boundary);

    // 3. adopt requested mode at this boundary. Deferred while the last exchange
    //    missed: on_enter seeds from fb, which is then stale. While the contact
    //    latch is set, on_contact_ is the only mode adopted and requested_ is left
    //    pending: the Supervisor's latest request is adopted after the clear.
    ControlMode* req = nullptr;
    if (misses == 0) {
      if (contact_ && contact_->latched()) {
        req = on_contact_;
      } else {
        req = requested_.exchange(nullptr, std::memory_order_acq_rel);
      }
    }
    if (req && req != active) {
      if (active) active->on_exit();
      // The incoming mode's first command is the switch's hold, and this
      // cycle's exchange sends it too (#92; see ControlMode::compute and
      // Transport::set_actuator_modes). That compute ticks the mode's
      // dt-driven state one extra cycle.
      req->on_enter(fb);
      JointFeedback at_rest = fb;
      at_rest.qd.setZero();
      req->compute(at_rest, dt_s, cmd);
      // set_actuator_modes performs the pumping handshake; acceptable at a
      // boundary on a mode change only. It survives isolated lost pumps itself,
      // so a TransportTimeout out of it is comms loss.
      try {
        t_.set_actuator_modes(req->required_modes(), &cmd);
      } catch (const TransportTimeout& e) {
        comms_lost("cyclic pumps timed out during a mode switch", e.what());
      }
      active = req;
      last_compute_ns = -1;  // the pumps are not time the new mode lived through
    }

    // 4. comm round-trip. On a miss fb still holds the last good feedback.
    const bool missed = !exchange_ok();
    // A miss neither confirms nor clears a fault: fb is the last good feedback.
    if (cfg_.exit_on_fault && !missed) check_fault();
    const int64_t t1 = ns_now();
    const uint32_t comm_ns = clamp_u32(t1 - t0);
    const double read_dt_s = double(t1 - last_read_ns) * 1e-9;
    last_read_ns = t1;

    // 5. fault flag from feedback
    const bool fault = fb.fault;

    // 5b. contact. A trip (never on a miss: the monitor skips stale feedback)
    //     leaves on_contact_ to adopt: now when the actuator modes match, else a
    //     freeze this cycle and step 3 adopts it at the next boundary.
    const bool latched = contact_ && contact_->step(fb, read_dt_s, missed);
    bool frozen = false;
    // !missed: never on stale fb. Only reachable latched-and-not-in-float after two
    // misses in a row (step 3 deferred); the command is re-sent and the next good
    // boundary adopts on_contact_.
    if (latched && active != on_contact_ && !missed) {
      if (active && active->required_modes() == on_contact_->required_modes()) {
        active->on_exit();
        on_contact_->on_enter(fb);
        active = on_contact_;
      } else {
        cmd.position = fb.q;
        cmd.velocity.setZero();
        frozen = true;
      }
    }

    // 6. compute (or hold last cmd if no active mode). Skipped on a miss, so the
    //    next cycle re-sends this cycle's command unchanged: compute() on stale fb
    //    would still advance its integrators (JointVelocityMode's q_ref += qd*dt,
    //    the reference rate limiters, watchdogs) with the arm unobserved. Skipped
    //    on a contact freeze, which must go out as set.
    if (active && !missed && !frozen) {
      const double dt =
          last_compute_ns < 0 ? dt_s : std::min(double(t1 - last_compute_ns) * 1e-9, max_dt_s);
      last_compute_ns = t1;
      active->compute(fb, dt, cmd);
    }
    const int64_t t2 = ns_now();
    const uint32_t compute_ns = clamp_u32(t2 - t1);

    // 8. advance boundary; detect overrun for THIS cycle.
    boundary += period_ns;
    bool overrun = false;
    const int64_t now = ns_now();
    if (now > boundary) {
      overrun = true;
      boundary = now + period_ns;  // resync so we don't spiral
    }

    // 7. record sample. cycle_ns = measured work span (t0 -> t2).
    CycleSample s;
    s.cycle_index = cycle_index++;
    s.wake_jitter_ns = wake_jitter;
    s.comm_ns = comm_ns;
    s.compute_ns = compute_ns;
    s.cycle_ns = clamp_u32(t2 - t0);
    s.flags = uint16_t((overrun ? 1u : 0u) | (fault ? 2u : 0u) | (missed ? 4u : 0u) |
                       (latched ? 8u : 0u));
    ring_.push(s);  // drop-don't-block; return ignored on the RT path
  }
}

}  // namespace kinova
