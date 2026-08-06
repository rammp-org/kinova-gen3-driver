#pragma once
#include <atomic>
#include "kinova_lowlevel/control_mode.h"
#include "kinova_lowlevel/diff_ik.h"
#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/pose_target_sink.h"
namespace kinova {

struct JointImpedanceParams {
  JointVec Kq = (JointVec() << 100, 100, 100, 100, 40, 40, 40).finished();  // N·m/rad
  JointVec Dq = (JointVec() <<  12,  12,  12,  12,  5,  5,  5).finished();  // N·m·s/rad
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

  // Preallocated RT scratch.
  JointVec g_ = JointVec::Zero();
  JointVec tau_ = JointVec::Zero();
};

}  // namespace kinova
