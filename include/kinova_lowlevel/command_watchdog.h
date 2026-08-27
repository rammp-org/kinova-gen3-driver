#pragma once
#include <atomic>
#include <cstdint>
namespace kinova {

// Staleness watchdog for a single-writer command stream.
//
// DETECTION ONLY. What to do when a command goes stale is the control mode's
// contract and deliberately lives there: JointTorqueMode ramps its feedforward
// to zero, the position and impedance modes freeze their reference at measured
// q, a velocity mode commands zero. stale_for() serves the ramping style;
// tick()'s bool serves the rest.
//
// RT-safe: tick() makes no clock call, allocates nothing and takes no lock --
// elapsed time is accumulated from the dt the caller already has. Freshness is a
// monotonic COUNTER rather than a timestamp, so the non-RT writer needs no clock
// either.
//
// Threading: arm() and bump() belong to the ONE non-RT thread that publishes
// commands; reset(), tick(), fresh() and stale_for() belong to the RT thread.
class CommandWatchdog {
 public:
  // <= 0 disables. Armed when a streaming session opens, disarmed when it closes.
  void arm(double timeout_s) noexcept { timeout_s_.store(timeout_s, std::memory_order_release); }

  // Call from EVERY setter that publishes a command. The release pairs with
  // tick()'s acquire, so a bump the RT side observes guarantees the payload the
  // writer stored beforehand is visible too.
  void bump() noexcept { count_.fetch_add(1, std::memory_order_release); }

  // RT thread, from on_enter: adopt the current count WITHOUT treating it as
  // fresh, so a command sent before entry is not honoured after it.
  void reset() noexcept {
    last_seen_ = count_.load(std::memory_order_acquire);
    stale_s_ = 0.0;
    fresh_ = false;
  }

  // RT thread, once per cycle. True if the command is stale THIS cycle.
  bool tick(double dt_s) noexcept {
    const uint64_t c = count_.load(std::memory_order_acquire);
    fresh_ = (c != last_seen_);
    if (fresh_) { last_seen_ = c; stale_s_ = 0.0; }
    else        { stale_s_ += dt_s; }
    const double t = timeout_s_.load(std::memory_order_acquire);
    return t > 0.0 && stale_s_ >= t;
  }

  // RT thread. True if the LAST tick() observed a new command. This is what
  // gates the double-buffer read: the payload must be adopted exactly when the
  // counter moves, never merely because the stream is not yet stale -- otherwise
  // a command published before on_enter would be picked up after it.
  bool fresh() const noexcept { return fresh_; }

  double stale_for() const noexcept { return stale_s_; }
  bool   armed()     const noexcept { return timeout_s_.load(std::memory_order_acquire) > 0.0; }

 private:
  std::atomic<uint64_t> count_{0};       // writer -> RT freshness signal
  std::atomic<double>   timeout_s_{0.0};
  uint64_t last_seen_ = 0;               // RT-owned
  double   stale_s_   = 0.0;             // RT-owned
  bool     fresh_     = false;           // RT-owned: last tick() saw a new command
};
}  // namespace kinova
