#pragma once
#include <array>
#include <atomic>
#include <limits>
#include "kinova_lowlevel/command_watchdog.h"
#include "kinova_lowlevel/control_mode.h"
#include "kinova_lowlevel/diff_ik.h"
#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/joint_target_sink.h"
#include "kinova_lowlevel/pose_target_sink.h"
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

  // Software position limits [rad]. Non-finite entries are seeded from the URDF
  // at construction, so a caller-supplied tighter limit survives but the default
  // is never unbounded. Continuous joints are ±inf in the URDF and stay that way.
  JointVec q_lower = JointVec::Constant(-std::numeric_limits<double>::infinity());
  JointVec q_upper = JointVec::Constant(std::numeric_limits<double>::infinity());

  // Staleness watchdog for streamed targets. 0 DISABLES it, which is the default
  // and preserves the behaviour every existing caller relies on.
  double cmd_timeout_s = 0.0;

  // Sustained IK non-convergence threshold [s]. A single non-converged solve is
  // NOT a fault -- a momentarily unreachable pose is normal while a client servos
  // toward something. Expressed in TIME, not cycles: compute() runs at whatever
  // the loop rate is, so a cycle count would silently mean something different at
  // a different rate. <= 0 disables.
  double ik_fault_s = 0.1;
  DiffIkParams ik{};
};

// Joint-space position control. Commands every actuator in kPosition and lets
// the actuator's own servo close the loop.
//
// With a JOINT target this still runs no dynamics at all -- no gravity term, no
// mass matrix, no IK -- and remains the cheapest control path in the driver. Only
// a live POSE target pulls in the in-loop IK solve.
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
class JointPositionMode : public ControlMode,
                          public JointTargetSink,
                          public PoseTargetSink {
 public:
  JointPositionMode(Dynamics& dyn, JointPositionParams p = {});
  ActuatorModes required_modes() const override;
  void on_enter(const JointFeedback& fb) override;
  void compute(const JointFeedback& fb, double dt_s, JointCommand& out) override;
  void on_exit() override {}

  // Non-RT setters (call from one supervisor thread).
  void set_target(const JointVec& q_d) noexcept override;
  // Cartesian target: resolved to a joint reference by in-loop IK, which then
  // feeds the SAME rate_limit -> leash -> wrap -> clamp pipeline a joint target
  // does, so the whole safety envelope comes along unchanged (PoseTargetSink).
  void set_target(const Pose& x_d) noexcept override;
  using JointTargetSink::set_target;   // keep the JointVec overload visible
  void set_params(const JointPositionParams& p) noexcept;

  // Set when IK has failed to converge for longer than ik_fault_s. Published for
  // the sampler thread: the mode cannot end a streaming session (modes know
  // nothing about the interface layer), so it freezes the reference immediately
  // at 1 kHz and lets the lifecycle teardown catch up.
  bool ik_faulted() const noexcept { return ik_faulted_.load(std::memory_order_acquire); }

  // Re-arm the staleness watchdog. s >= 0 arms with s; s < 0 restores this
  // mode's own configured default (params().cmd_timeout_s).
  void set_command_timeout(double s) noexcept;

  // Returns the ACTIVE parameters, i.e. after URDF seeding and clamping — not
  // necessarily what was passed in. Worth reading back when a speed request
  // seems to have been ignored: it was probably clamped to the hardware limit.
  JointPositionParams params() const noexcept;

  // RT-thread-owned state, for tests and post-stop inspection. NOT synchronized:
  // do not call these from another thread while the RT loop is running.
  JointVec reference() const noexcept { return q_ref_; }
  IkResult last_ik() const noexcept { return last_ik_; }

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

  // Target source, selected by the most recent setter (single-writer, non-RT).
  // Named kEntry rather than kEntryPose because what is captured at entry is a
  // joint CONFIGURATION, not a pose.
  enum class TargetSource : int { kEntry, kPose, kJoint };
  JointVec entry_q_ = JointVec::Zero();
  JointVec ext_target_[2];
  std::atomic<int> ext_active_{0};
  Pose pose_target_[2];
  std::atomic<int> pose_active_{0};
  std::atomic<TargetSource> source_{TargetSource::kEntry};

  // Staleness detection for the streamed target. The RESPONSE -- freezing the
  // reference at measured q -- is this mode's contract and lives in compute().
  CommandWatchdog wd_;

  JointVec q_ref_ = JointVec::Zero();   // integrated reference configuration

  Dynamics& dyn_;
  DiffIkSolver ik_;
  IkResult last_ik_{};
  double ik_bad_s_ = 0.0;                  // RT-owned: summed dt while !converged
  std::atomic<bool> ik_faulted_{false};    // RT writer, non-RT (sampler) reader
  JointVec ik_q_ = JointVec::Zero();       // preallocated RT scratch for the solve
};

}  // namespace kinova
