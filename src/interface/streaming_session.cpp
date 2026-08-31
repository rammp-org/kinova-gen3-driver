#include "kinova_lowlevel/interface/streaming_session.h"
namespace kinova::interface {

// The valid-pair table. Plan 2 adds the velocity rows and EE pose -> position;
// until then those are refused, which is the table working, not a stub.
bool pair_supported(SetpointKind k, ControlModeKind m) {
  switch (k) {
    case SetpointKind::kJointPosition:
      return m == ControlModeKind::kPosition || m == ControlModeKind::kImpedance;
    case SetpointKind::kEePose:
      return m == ControlModeKind::kImpedance;      // position mode has no IK path yet (Plan 2)
    case SetpointKind::kJointTorque:
      return m == ControlModeKind::kTorque;
    case SetpointKind::kJointVelocity:
    case SetpointKind::kEeTwist:
      return false;                                  // JointVelocityMode does not exist yet (Plan 2)
  }
  return false;
}

StreamOpenResult StreamingSession::open(const StreamOpenRequest& r, double now_s) {
  if (is_open())
    return {false, result_code::kStreamRejected, "a session is already open; close it first"};
  if (r.timeout_s <= 0.0)
    return {false, result_code::kStreamRejected,
            "timeout_s must be > 0: an unbounded stream has no safe-stop"};
  if (!pair_supported(r.kind, r.control_mode))
    return {false, result_code::kStreamRejected, "unsupported (setpoint kind, control mode) pair"};

  kind_.store(r.kind, std::memory_order_relaxed);
  mode_.store(r.control_mode, std::memory_order_relaxed);
  timeout_s_.store(r.timeout_s, std::memory_order_relaxed);
  last_s_.store(now_s, std::memory_order_relaxed);
  open_.store(true, std::memory_order_release);      // marked LAST -- see the handoff rule
  return {true, 0, ""};
}

void StreamingSession::close() {
  open_.store(false, std::memory_order_release);     // marked FIRST -- see the handoff rule
}

bool StreamingSession::admit(SetpointKind k, double now_s) {
  if (!is_open() || k != kind_.load(std::memory_order_relaxed)) {
    rejected_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  last_s_.store(now_s, std::memory_order_relaxed);
  return true;
}

bool StreamingSession::expired(double now_s) const {
  if (!is_open()) return false;                      // nothing open, nothing to tear down
  return (now_s - last_s_.load(std::memory_order_relaxed)) > timeout_s_.load(std::memory_order_relaxed);
}
}  // namespace kinova::interface
