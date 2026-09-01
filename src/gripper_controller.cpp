#include "kinova_lowlevel/gripper_controller.h"
namespace kinova {

void GripperController::set_target(const GripperCommand& c) noexcept {
  const int next = 1 - active_.load(std::memory_order_relaxed);
  buf_[next] = c;
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
