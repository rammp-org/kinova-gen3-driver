#pragma once
#include <atomic>
#include "kinova_lowlevel/transport.h"
#include "kinova_lowlevel/control_mode.h"
#include "kinova_lowlevel/telemetry.h"
#include "kinova_lowlevel/rt_system.h"
namespace kinova {
enum class Pacing { kSleepSpin, kClockNanosleep };
struct ExecutorConfig { double rate_hz = 1000.0; Pacing pacing = Pacing::kSleepSpin; RtConfig rt{}; };
class RtExecutor {
 public:
  RtExecutor(Transport& t, SampleRing& ring, ExecutorConfig cfg);
  void request_mode(ControlMode* m) noexcept;   // supervisor sets; loop adopts at a cycle boundary
  void run(std::atomic<bool>& stop);             // blocks on calling thread; runs the RT loop
 private:
  Transport& t_;
  SampleRing& ring_;
  ExecutorConfig cfg_;
  std::atomic<ControlMode*> requested_{nullptr};
};
}  // namespace kinova
