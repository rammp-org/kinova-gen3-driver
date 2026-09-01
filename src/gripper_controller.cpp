#include "kinova_lowlevel/gripper_controller.h"
namespace kinova {
namespace {
float clamp01(float v) noexcept {
  if (v < 0.0f) return 0.0f;
  if (v > 1.0f) return 1.0f;
  return v;
}
}  // namespace

void GripperController::set_target(const GripperCommand& c) noexcept {
  const int next = 1 - active_.load(std::memory_order_relaxed);
  buf_[next] = c;
  // Saturate rather than reject, matching the driver's house style elsewhere
  // (max_ref_speed, torque_limit, max_qd): an out-of-range setpoint gets clamped
  // into GripperCommand's documented contract here, once, for every caller --
  // including unvalidated data straight off a socket, and SimTransport/KortexTransport
  // downstream have no clamp of their own to fall back on.
  buf_[next].position = clamp01(c.position);
  buf_[next].speed    = clamp01(c.speed);
  buf_[next].force    = clamp01(c.force);
  active_.store(next, std::memory_order_release);
  stamping_.store(true, std::memory_order_release);   // LAST: publishes the buffer above it
}

void GripperController::release() noexcept {
  stamping_.store(false, std::memory_order_release);
}

JointCommand GripperController::stamp(const JointCommand& c) const noexcept {
  JointCommand out = c;
  // acquire on active_ pairs with the release in set_target: once the new index is
  // observed, the buffer write that preceded it is guaranteed visible, so there is no
  // first-cycle stale read. Read unconditionally -- release() only gates `active`
  // below, it does not stop the last-commanded position/speed/force from being
  // readable in the outgoing frame (that's the point: ceasing to command holds).
  out.gripper = buf_[active_.load(std::memory_order_acquire)];
  out.gripper.active = stamping_.load(std::memory_order_acquire);
  return out;
}

}  // namespace kinova
