#pragma once
#include <array>
#include <atomic>
#include <limits>
#include "kinova_lowlevel/control_mode.h"
#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/joint_target_sink.h"
namespace kinova {

struct JointPositionParams {
  // Per-joint cap on how fast the commanded reference may move [rad/s].
  //
  // Deliberately BELOW the URDF velocity limits (1.40 rad/s proximal, 1.22 at
  // the wrist). Position mode has no compliance: whatever is commanded, the
  // actuator's own servo chases it at full authority, so a target that teleports
  // drags the arm across the whole gap as fast as the joint can move. Raise it
  // once there is hardware data.
  //
  // Non-finite entries are seeded from the URDF; finite entries are clamped DOWN
  // to it. No configuration can ask for more than the hardware is rated for.
  JointVec max_ref_speed = JointVec::Constant(0.5);

  // Cap on how far the reference may lead the MEASURED position [rad]. This
  // bounds the energy release when a blocked arm comes free: without it the
  // reference marches on to the target while the arm sits still, and the arm
  // then snaps across the entire accumulated gap the moment it is released.
  // A value <= 0 disables the leash.
  double max_following_error = 0.35;

  // Emit the reference velocity alongside the position command, as a
  // feedforward for the actuator's own position loop.
  //
  // OFF by default, and expected to stay that way. Kinova's own ros2_kortex
  // driver computes this very command and then declines to send it:
  //   base_command_.mutable_actuators(i)->set_position(cmd_degrees_tmp_);
  //   // Velocity command interface not implemented properly in the kortex api
  //   // base_command_.mutable_actuators(i)->set_velocity(cmd_vel_tmp_);
  // (kortex_driver/src/hardware_interface.cpp). Nor does the KORTEX reference
  // say whether the field is a feedforward or a LIMIT in POSITION servoing, and
  // being wrong at 1 kHz is a hardware event. The mode computes the reference
  // velocity either way (see last_ref_velocity) so it can be logged and compared
  // first; this flag exists to make the experiment possible on an attended arm,
  // not because the path is expected to work. Feedforward that actually pays
  // goes through the impedance law, where we evaluate the torque ourselves.
  bool velocity_feedforward = false;

  // Software position limits [rad]. Non-finite entries are seeded from the URDF
  // at construction, so a caller-supplied tighter limit survives but the default
  // is never unbounded. Continuous joints are ±inf in the URDF and stay that way.
  JointVec q_lower = JointVec::Constant(-std::numeric_limits<double>::infinity());
  JointVec q_upper = JointVec::Constant(std::numeric_limits<double>::infinity());
};

// Joint-space position control. Commands every actuator in kPosition and lets
// the actuator's own servo close the loop, so this mode runs no dynamics at all
// — no gravity term, no mass matrix, no IK. It is the cheapest control path in
// the driver and the one with the least to go wrong in software.
//
// What it does own is the REFERENCE:
//   q_ref <- rate_limit(q_ref -> target)   bounded by max_ref_speed·dt
//          -> leash to measured q          bounded by max_following_error
//          -> wrapped (continuous joints)  kept in (-pi, pi] like the feedback
//          -> clamped to the position limits
//
// Tradeoff against JointImpedanceMode: there is no compliance whatsoever. The
// arm will not yield to contact, it will push through it until the actuator
// faults. Use this to move to known configurations and to exercise a target
// path; use joint impedance when a human is in the loop.
//
// Live setters publish via a single-writer (non-RT) double-buffer; compute()
// (RT thread) reads one snapshot per cycle.
class JointPositionMode : public ControlMode, public JointTargetSink {
 public:
  JointPositionMode(Dynamics& dyn, JointPositionParams p = {});
  ActuatorModes required_modes() const override;
  void on_enter(const JointFeedback& fb) override;
  void compute(const JointFeedback& fb, double dt_s, JointCommand& out) override;
  void on_exit() override {}

  // Non-RT setters (call from one supervisor thread).
  void set_joint_target(const JointTarget& t) noexcept override;

  // The reference velocity computed last cycle: d(q_ref)/dt, i.e. the speed the
  // arm is actually being commanded at after rate limiting and leashing. Always
  // computed, only SENT when params().velocity_feedforward is on — so it can be
  // logged and sanity-checked before it is trusted on hardware. RT-thread-owned:
  // do not call while the loop is running.
  JointVec last_ref_velocity() const noexcept { return qd_ref_; }
  void set_params(const JointPositionParams& p) noexcept;

  // Returns the ACTIVE parameters, i.e. after URDF seeding and clamping — not
  // necessarily what was passed in. Worth reading back when a speed request
  // seems to have been ignored: it was probably clamped to the hardware limit.
  JointPositionParams params() const noexcept;

  // RT-thread-owned state, for tests and post-stop inspection. NOT synchronized:
  // do not call this from another thread while the RT loop is running.
  JointVec reference() const noexcept { return q_ref_; }

 private:
  // Fills any non-finite limit with the URDF value and clamps any finite speed
  // request down to the URDF velocity limit. Runs on the non-RT thread only, and
  // reads the cached URDF values rather than Dynamics, which is not thread-safe
  // against the RT loop.
  void seed_limits(JointPositionParams& p) const noexcept;

  JointVec q_lower_urdf_ = JointVec::Zero();   // cached in ctor: set_params must
  JointVec q_upper_urdf_ = JointVec::Zero();   // not touch Dynamics off the RT thread
  JointVec v_max_urdf_ = JointVec::Zero();
  // Which joints are continuous (both URDF limits infinite). The transport wraps
  // every measured angle to (-pi, pi], so on these joints a raw target-minus-
  // reference difference can be wrong by 2*pi and MUST be wrapped — otherwise
  // the reference walks the long way round and the joint spins most of a turn.
  std::array<bool, kNumJoints> continuous_{};

  // Params double-buffer (writer: set_params; reader: compute). Seeded at ctor.
  JointPositionParams params_[2];
  std::atomic<int> params_active_{0};

  // Target source: the entry configuration (written once by on_enter on the RT
  // thread) or an external target published by set_target.
  JointVec entry_q_ = JointVec::Zero();
  JointTarget ext_target_[2];
  std::atomic<int> ext_active_{0};
  std::atomic<bool> has_ext_target_{false};

  JointVec q_ref_ = JointVec::Zero();   // integrated reference configuration
  JointVec qd_ref_ = JointVec::Zero();  // its velocity, computed every cycle
};

}  // namespace kinova
