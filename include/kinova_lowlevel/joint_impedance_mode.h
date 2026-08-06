#pragma once
#include <array>
#include <atomic>
#include "kinova_lowlevel/control_mode.h"
#include "kinova_lowlevel/diff_ik.h"
#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/pose_target_sink.h"
namespace kinova {

struct JointImpedanceParams {
  JointVec Kq = (JointVec() << 30, 30, 30, 30, 12, 12, 12).finished();  // N·m/rad
  // Damping is DERIVED, never dialed in:  Dq_i = 2·zeta·sqrt(Kq_i · M_ii(q)).
  // A flat damping vector cannot be right at more than one configuration — on
  // this arm the effective inertia at joint 1 swings ~38x between extended
  // (0.015 kg·m²) and elbow-up (0.573), so any constant is far too high at one end
  // and too low at the other. Deriving it also means retuning Kq never leaves the
  // damping stale.
  double zeta = 0.7;              // damping ratio; 1.0 = critically damped
  // Per-joint ceiling. The URDF gives joints 5-7 an effort limit of 9 N·m; the
  // single scalar CartesianImpedanceParams uses would overrun the wrist by 4x
  // under stiff joint gains.
  JointVec torque_limit = (JointVec() << 39, 39, 39, 39, 9, 9, 9).finished();
  // Spring leash: caps |q_ref - q| per joint so spring torque saturates at
  // Kq*leash while gravity compensation still passes through in full. The total
  // torque clamp cannot do this -- it eats the gravity term under load and the
  // arm sags.
  double max_tracking_error = 0.35;   // rad
  double max_ref_speed      = 1.0;    // rad/s cap on reference motion
  double gain_ramp_s        = 0.5;    // fade the spring in over this window on entry
  DiffIkParams ik{};
};

// Joint-space impedance driven by in-loop IK:
//   q_d  <- DiffIk(target, warm start q_d)          (all 7 joints commanded)
//   tau   = g(q) + ramp * ( Kq∘clamp(q_d-q, ±leash) - Dq∘qd )
// Unlike CartesianImpedanceMode this leaves no uncommanded DOF: the redundant
// joint is resolved inside the IK by posture bias + limit avoidance, so the arm
// cannot drift into an arbitrary configuration.
//
// Tradeoff: end-effector stiffness becomes J^-T Kq J^-1 -- configuration
// dependent and not diagonal in the task frame. For teleop, predictable posture
// is worth more than an exactly shaped task-space ellipsoid.
//
// Live setters publish via a single-writer (non-RT) double-buffer; compute()
// (RT thread) reads one snapshot per cycle.
class JointImpedanceMode : public ControlMode, public PoseTargetSink {
 public:
  JointImpedanceMode(Dynamics& dyn, JointImpedanceParams p = {});
  ActuatorModes required_modes() const override;
  void on_enter(const JointFeedback& fb) override;
  void compute(const JointFeedback& fb, double dt_s, JointCommand& out) override;
  void on_exit() override {}

  // Non-RT setters (call from one supervisor thread).
  void set_gains(const JointImpedanceParams& p) noexcept;
  void set_target(const Pose& x_d) noexcept override;

  // RT-thread-owned state, for tests and post-stop inspection. NOT synchronized:
  // do not call these from another thread while the RT loop is running.
  JointVec reference() const noexcept { return q_d_; }
  IkResult last_ik() const noexcept { return last_ik_; }
  // The damping actually applied last cycle, 2·zeta·sqrt(Kq·M_ii(q)). Worth
  // watching while tuning — it is configuration dependent, so it changes as the
  // arm moves even at a fixed Kq.
  JointVec last_damping() const noexcept { return Dq_last_; }

 private:
  JointImpedanceParams params() const noexcept;   // RT-safe: returns a value snapshot
  // Fills any non-finite IK limit with the URDF value cached at construction, so
  // a caller-supplied tighter software limit survives but the default does not
  // leave the solver unbounded.
  void seed_limits(JointImpedanceParams& p) const noexcept;

  Dynamics& dyn_;
  DiffIkSolver ik_;
  JointVec q_lower_urdf_ = JointVec::Zero();   // cached in ctor: set_gains must not
  JointVec q_upper_urdf_ = JointVec::Zero();   // touch Dynamics off the RT thread
  // Which joints are continuous (both URDF limits infinite). The transport wraps
  // every measured angle to (-pi, pi], so on these joints the raw reference-minus-
  // measured difference can be wrong by 2*pi and MUST be wrapped -- see compute().
  std::array<bool, kNumJoints> continuous_{};

  // Gains double-buffer (writer: set_gains; reader: compute). Seeded at ctor.
  JointImpedanceParams gains_[2];
  std::atomic<int> gains_active_{0};

  // Target source: the entry pose (written once by on_enter on the RT thread) or
  // an optional external override published by set_target. Same single-writer
  // double-buffer discipline as CartesianImpedanceMode.
  Pose entry_pose_;
  Pose ext_target_[2];
  std::atomic<int> ext_active_{0};
  std::atomic<bool> has_ext_target_{false};

  JointVec q_d_ = JointVec::Zero();    // integrated reference configuration
  IkResult last_ik_{};
  double ramp_elapsed_ = 0.0;

  JointVec Dq_last_ = JointVec::Zero();   // damping applied last cycle (derived)

  // Preallocated RT scratch.
  JointMat M_ = JointMat::Zero();
  JointVec g_ = JointVec::Zero();
  JointVec tau_ = JointVec::Zero();
};

}  // namespace kinova
