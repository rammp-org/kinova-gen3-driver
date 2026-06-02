#include "kinova_lowlevel/rt_executor.h"

#include <errno.h>
#include <time.h>

#include <cstdint>

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
  while (ns_now() < deadline_ns) { /* spin */ }
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
    : t_(t), ring_(ring), cfg_(cfg) {}

void RtExecutor::request_mode(ControlMode* m) noexcept {
  requested_.store(m, std::memory_order_release);
}

void RtExecutor::run(std::atomic<bool>& stop) {
  // Best-effort RT setup (mlockall + SCHED_FIFO + affinity). Never throws; if it
  // fails (e.g. EPERM under non-sudo) the loop still runs at SCHED_OTHER.
  enable_rt(cfg_.rt);

  const double rate_hz = cfg_.rate_hz > 0.0 ? cfg_.rate_hz : 1000.0;
  const int64_t period_ns = int64_t(1e9 / rate_hz);
  const double dt_s = 1.0 / rate_hz;

  ControlMode* active = nullptr;

  // Reusable stack locals; zero-initialized. No allocation in the loop body.
  JointCommand cmd;   // benign hold: kTorque, zero torque, zero position
  JointFeedback fb;

  // Seed one feedback read with the benign hold command so fb is populated and
  // the first control compute sees real joint state.
  t_.exchange(cmd, fb);

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

    // 3. adopt requested mode at this boundary
    ControlMode* req = requested_.exchange(nullptr, std::memory_order_acq_rel);
    if (req && req != active) {
      if (active) active->on_exit();
      // set_actuator_modes performs the pumping handshake; acceptable at a
      // boundary on a mode change only.
      t_.set_actuator_modes(req->required_modes());
      req->on_enter(fb);
      active = req;
    }

    // 4. comm round-trip
    t_.exchange(cmd, fb);
    const int64_t t1 = ns_now();
    const uint32_t comm_ns = clamp_u32(t1 - t0);

    // 5. fault flag from feedback
    const bool fault = fb.fault;

    // 6. compute (or hold last cmd if no active mode)
    if (active) active->compute(fb, dt_s, cmd);
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
    s.flags = uint16_t((overrun ? 1u : 0u) | (fault ? 2u : 0u));
    ring_.push(s);  // drop-don't-block; return ignored on the RT path
  }
}

}  // namespace kinova
