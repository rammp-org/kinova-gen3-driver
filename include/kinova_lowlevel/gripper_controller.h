#pragma once
#include <atomic>
#include "kinova_lowlevel/gripper_types.h"
#include "kinova_lowlevel/transport.h"
namespace kinova {

// Stamps the gripper command into every outgoing JointCommand.
//
// A DECORATOR, not a ControlMode, and deliberately so. Modes are mutually exclusive,
// so making the gripper a mode would mean giving up arm control to move it. The gripper
// is not a control law -- it has no feedback term and no RT computation; it is a field
// on the outgoing frame. That makes the gripper ORTHOGONAL to control modes: you can
// grip during a trajectory, an impedance hold, or a velocity stream, and nothing about
// the gripper touches mode switching or widens the RT-safe surface.
//
// This is the same shape FeedbackTap uses to decorate Transport and Arbiter uses to
// decorate CommandSink.
//
// Threading: set_target() and release() belong to ONE non-RT thread. The stamp happens
// on the RT thread inside exchange()/send(). Published through a double-buffer with a
// release store, so the RT reader never observes a torn command.
class GripperController : public Transport {
 public:
  explicit GripperController(Transport& inner) : inner_(inner) {}

  // Non-RT. Latest wins; every call carries all three fields, because speed and force
  // are deliberately NOT sticky -- see the statelessness decision in the spec.
  // position/speed/force are each clamped to [0, 1] here, so this is the one place
  // GripperCommand's documented range is enforced for every caller (including
  // unvalidated data straight off a socket).
  void set_target(const GripperCommand& c) noexcept;

  // Non-RT, the halt path. Stops STAMPING -- this decorator's own output goes to
  // cmd.gripper.active = false starting next cycle. It does NOT mean the gripper goes
  // quiet: on KortexTransport, the last commanded position/speed/force submessage stays
  // latched in the persistent cyclic command and is retransmitted every cycle regardless
  // of `active` (see the RETRANSMISSION, NOT CESSATION note in kortex_transport.cpp).
  // The grip holds either way -- the 2F-85 is effectively self-locking -- but the
  // mechanism is continuous re-command against the last target, not the motor going
  // idle. E-stop means stop moving, and opening would itself be a motion.
  void release() noexcept;

  // RT-thread-owned view of what is currently being stamped. NOT synchronized: for
  // tests and post-stop inspection only -- this is NOT the stamp path (see stamp() and
  // its ordering note below); it does not gate on stamping_ the way stamp() must.
  GripperCommand target() const noexcept { return buf_[active_.load(std::memory_order_acquire)]; }

  void connect() override { inner_.connect(); }
  void set_servoing_low_level() override { inner_.set_servoing_low_level(); }
  void set_actuator_modes(const ActuatorModes& m) override { inner_.set_actuator_modes(m); }
  void exchange(const JointCommand& c, JointFeedback& fb) override { inner_.exchange(stamp(c), fb); }
  void send(const JointCommand& c) override { inner_.send(stamp(c)); }
  void receive(JointFeedback& fb) override { inner_.receive(fb); }
  void safe_shutdown() override { inner_.safe_shutdown(); }
  void clear_faults() override { inner_.clear_faults(); }

 private:
  JointCommand stamp(const JointCommand& c) const noexcept;

  Transport& inner_;
  GripperCommand buf_[2];
  std::atomic<int> active_{0};
  // Separate from buf_[].active so release() can stop stamping without destroying the
  // target -- the last commanded position stays readable for diagnostics.
  // ORDERING: set_target's writer publishes buf_ -> active_ (release) -> stamping_
  // (release), last. The reader in stamp() MUST load stamping_ before active_ -- that
  // is what ties the index it reads from active_ to the same write generation as the
  // buffer contents. Reading active_ first can pair a fresh index with a stale/default
  // buffer entry if the writer completes between the two loads. Do not reverse this.
  std::atomic<bool> stamping_{false};
};

}  // namespace kinova
