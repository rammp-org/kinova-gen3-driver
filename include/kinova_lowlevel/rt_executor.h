#pragma once
#include <atomic>
#include <stdexcept>

#include "kinova_lowlevel/control_mode.h"
#include "kinova_lowlevel/rt_system.h"
#include "kinova_lowlevel/telemetry.h"
#include "kinova_lowlevel/transport.h"
namespace kinova {
class ContactMonitor;
enum class Pacing { kSleepSpin, kClockNanosleep };
struct ExecutorConfig {
  double rate_hz = 1000.0;
  Pacing pacing = Pacing::kSleepSpin;
  RtConfig rt{};
  // Consecutive exchange timeouts (TransportTimeout) treated as comms loss:
  // run() throws CommsLost. Each miss has already blocked for the transport's
  // cyclic timeout, so this is max_consecutive_misses x that timeout with no
  // reply (default: 3 x KortexTransport's 20 ms = 60 ms).
  int max_consecutive_misses = kMaxConsecutiveTimeouts;
  // Any good exchange resets that count, so a link losing every other frame would
  // never trip it. Also comms loss: max_window_misses timeouts within the last
  // miss_window exchanges. 5 in 1000 (>= 1 s at 1 kHz, more while misses block)
  // trips an every-other-frame loss after 5 timeouts (~100 ms at 20 ms each);
  // isolated misses never add up.
  int max_window_misses = 5;
  int miss_window = 1000;  // exchanges
  // End run() with ArmFault when feedback confirms the arm is no longer ours to
  // command: JointFeedback::fault on kFaultConfirmExchanges good exchanges in a
  // row, or !low_level on kLeftLowLevelConfirmExchanges. Off by default, so a
  // caller that clears faults in place while the loop keeps running (the teleop
  // app) keeps today's behaviour; ArmLink requires it.
  bool exit_on_fault = false;
};
// Good exchanges in a row reporting a fault before run() gives up the arm: one
// garbled frame must not drop it, a latched fault must, within a few ms.
inline constexpr int kFaultConfirmExchanges = 3;
// Good exchanges in a row with the base not reporting low-level servoing. Longer
// than for a fault: the base's reported state may trail the switch into
// low-level by a few frames (unmeasured), and a base that really left low-level
// rejects the cyclic commands anyway (ArmRejected).
inline constexpr int kLeftLowLevelConfirmExchanges = 50;
// Thrown by RtExecutor::run() under exit_on_fault when feedback confirms a fault
// or that the base left low-level servoing. The loop has stopped commanding.
class ArmFault : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};
// Thrown by RtExecutor::run() on comms loss: max_consecutive_misses exchange
// timeouts in a row, max_window_misses within miss_window exchanges, or a mode
// switch whose pumps time out. The loop has stopped commanding; the caller owns
// safe_shutdown().
class CommsLost : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};
class RtExecutor {
 public:
  // Throws std::invalid_argument if max_consecutive_misses, max_window_misses or
  // miss_window is < 1.
  RtExecutor(Transport& t, SampleRing& ring, ExecutorConfig cfg);
  // Ownership: the caller retains ownership of `*m`. The executor only stores a
  // raw pointer and adopts it at a cycle boundary, so the ControlMode MUST
  // outlive the executor (or at least remain valid until another mode is
  // requested and adopted). The executor never deletes or copies it.
  void request_mode(ControlMode* m) noexcept;  // supervisor sets; loop adopts at a cycle boundary
  // Unexpected-contact stop. Call before run(); both MUST outlive the executor.
  // The loop then calls monitor.on_run_start() at each run() start and
  // monitor.step() every cycle (a missed exchange included) on the RT thread, with
  // the measured time since the previous feedback read as dt, and flags latched
  // cycles (CycleSample bit3). While monitor.latched() the loop runs on_contact and
  // nothing else:
  //   - the cycle that trips switches to it at once when its actuator modes match
  //     the active mode's; otherwise that cycle's command is a freeze (the active
  //     mode's actuator modes, position = measured q, zero velocity, no compute) and
  //     the next boundary adopts on_contact through set_actuator_modes. A command
  //     goes out on the next cycle's exchange, so the freeze is sent once, after
  //     that handshake;
  //   - a run() that starts latched adopts it at its first boundary;
  //   - a request_mode() is left pending and adopted only after the clear.
  // monitor shares the modes' Dynamics: it is only ever called on the RT thread.
  void attach_contact(ContactMonitor& monitor, ControlMode& on_contact) noexcept;
  // Blocks on the calling thread and runs the RT loop. A TransportTimeout from
  // exchange() is survived: the cycle keeps the last good feedback, skips
  // compute() so the previous command is re-sent unchanged, and is flagged
  // (CycleSample bit2). compute() gets the wall time since the mode's previous
  // compute(), so the miss counts on its timers: the nominal 1/rate_hz on a
  // mode's first compute(), capped at max_consecutive_misses x
  // (kDefaultCyclicTimeoutMs + period). Throws CommsLost on comms loss (see ExecutorConfig);
  // any other exception propagates as before. With exit_on_fault, throws
  // ArmFault on a confirmed fault. May be called again after it returns or throws:
  // a mode requested in between is adopted by the new run.
  void run(std::atomic<bool>& stop);
  const ExecutorConfig& config() const noexcept { return cfg_; }

 private:
  Transport& t_;
  SampleRing& ring_;
  ExecutorConfig cfg_;
  std::atomic<ControlMode*> requested_{nullptr};
  ContactMonitor* contact_ = nullptr;  // set before run(); null = no contact stop
  ControlMode* on_contact_ = nullptr;
};
}  // namespace kinova
