#pragma once
#include <atomic>
#include <cstdint>
#include "kinova_lowlevel/interface/value_types.h"
namespace kinova::interface {

// Which (setpoint shape, control mode) pairs this driver can actually execute.
// Anything off the table is refused at open rather than silently degraded.
bool pair_supported(SetpointKind, ControlModeKind);

// The streaming tier's lifecycle, as pure logic over injected time.
//
// Deliberately owns NO control state: it does not know what a mode is, never
// touches Dynamics or Transport, and writes no targets. The Supervisor asks it
// whether a setpoint may proceed and where it should go; the Supervisor does the
// writing. That keeps this unit testable with no robot, no URDF and no threads.
//
// is_open() is atomic because it is read by the sampler thread (for the
// expiry check) and by Supervisor::on_trajectory_goal's fast pre-check, while
// the backend thread opens and closes.
class StreamingSession {
 public:
  StreamOpenResult open(const StreamOpenRequest&, double now_s);
  void             close();
  // Returns true if this setpoint may proceed. A matching setpoint also refreshes
  // the deadline; a rejected one does NOT (a client sending the wrong shape is not
  // evidence the stream is healthy).
  bool             admit(SetpointKind, double now_s);
  bool             expired(double now_s) const;

  bool            is_open()        const { return open_.load(std::memory_order_acquire); }
  SetpointKind    kind()           const { return kind_.load(std::memory_order_relaxed); }
  ControlModeKind control_mode()   const { return mode_.load(std::memory_order_relaxed); }
  double          timeout_s()      const { return timeout_s_.load(std::memory_order_relaxed); }
  uint64_t        rejected_count() const { return rejected_.load(std::memory_order_relaxed); }

 private:
  std::atomic<bool> open_{false};
  // ATOMIC, though the release/acquire on open_ already orders the FIRST read after
  // an open. A close->re-open on the backend thread can land while the sampler is
  // mid-expired(), and that is an unsynchronised read of a plain member: harmless
  // in practice, a genuine data race to TSan, and free to close. Relaxed is enough
  // -- open_ still carries the ordering; these just must not tear.
  std::atomic<SetpointKind>    kind_{SetpointKind::kJointPosition};
  std::atomic<ControlModeKind> mode_{ControlModeKind::kPosition};
  std::atomic<double>          timeout_s_{0.1};
  std::atomic<double> last_s_{0.0};        // last accepted setpoint, or the open time
  std::atomic<uint64_t> rejected_{0};
};
}  // namespace kinova::interface
