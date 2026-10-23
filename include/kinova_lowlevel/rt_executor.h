#pragma once
#include <atomic>
#include <stdexcept>

#include "kinova_lowlevel/control_mode.h"
#include "kinova_lowlevel/rt_system.h"
#include "kinova_lowlevel/telemetry.h"
#include "kinova_lowlevel/transport.h"
namespace kinova {
enum class Pacing { kSleepSpin, kClockNanosleep };
struct ExecutorConfig {
  double rate_hz = 1000.0;
  Pacing pacing = Pacing::kSleepSpin;
  RtConfig rt{};
  // Consecutive exchange timeouts (TransportTimeout) treated as comms loss:
  // run() throws CommsLost. Each miss has already blocked for the transport's
  // cyclic timeout (KortexTransport: 20 ms), so 3 is ~60 ms with no reply.
  int max_consecutive_misses = 3;
  // Any good exchange resets that count, so a link losing every other frame would
  // never trip it. Also comms loss: max_window_misses timeouts within the last
  // miss_window exchanges. 5 in 1000 (>= 1 s at 1 kHz, more while misses block)
  // trips an every-other-frame loss in ~100 ms; isolated misses never add up.
  int max_window_misses = 5;
  int miss_window = 1000;  // exchanges
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
  // Throws std::invalid_argument if a miss limit or miss_window is < 1.
  RtExecutor(Transport& t, SampleRing& ring, ExecutorConfig cfg);
  // Ownership: the caller retains ownership of `*m`. The executor only stores a
  // raw pointer and adopts it at a cycle boundary, so the ControlMode MUST
  // outlive the executor (or at least remain valid until another mode is
  // requested and adopted). The executor never deletes or copies it.
  void request_mode(ControlMode* m) noexcept;  // supervisor sets; loop adopts at a cycle boundary
  // Blocks on the calling thread and runs the RT loop. A TransportTimeout from
  // exchange() is survived: the cycle keeps the last good feedback, skips
  // compute() so the previous command is re-sent unchanged, and is flagged
  // (CycleSample bit2). Throws CommsLost on comms loss (see ExecutorConfig);
  // any other exception propagates as before.
  void run(std::atomic<bool>& stop);

 private:
  Transport& t_;
  SampleRing& ring_;
  ExecutorConfig cfg_;
  std::atomic<ControlMode*> requested_{nullptr};
};
}  // namespace kinova
