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
};
// Thrown by RtExecutor::run() after max_consecutive_misses exchange timeouts in
// a row. The loop has stopped commanding; the caller owns safe_shutdown().
class CommsLost : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};
class RtExecutor {
 public:
  RtExecutor(Transport& t, SampleRing& ring, ExecutorConfig cfg);
  // Ownership: the caller retains ownership of `*m`. The executor only stores a
  // raw pointer and adopts it at a cycle boundary, so the ControlMode MUST
  // outlive the executor (or at least remain valid until another mode is
  // requested and adopted). The executor never deletes or copies it.
  void request_mode(ControlMode* m) noexcept;  // supervisor sets; loop adopts at a cycle boundary
  // Blocks on the calling thread and runs the RT loop. A TransportTimeout from
  // exchange() is survived: the cycle keeps the last good feedback, skips
  // compute() so the previous command is re-sent unchanged, and is flagged
  // (CycleSample bit2). Throws CommsLost on max_consecutive_misses in a row;
  // any other exception propagates as before.
  void run(std::atomic<bool>& stop);

 private:
  Transport& t_;
  SampleRing& ring_;
  ExecutorConfig cfg_;
  std::atomic<ControlMode*> requested_{nullptr};
};
}  // namespace kinova
