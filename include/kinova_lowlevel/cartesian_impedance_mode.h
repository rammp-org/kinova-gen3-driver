#pragma once
#include <atomic>
#include "kinova_lowlevel/control_mode.h"
#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/cartesian.h"
#include "kinova_lowlevel/pose_target_sink.h"
namespace kinova {

struct CartesianImpedanceParams {
  Vector6 Kx = (Vector6() << 300,300,300, 30,30,30).finished();  // N/m | N·m/rad
  Vector6 Dx = (Vector6() << 35,35,35, 5,5,5).finished();        // N·s/m | N·m·s/rad
  // Defaults are sensible starting points; tune Kx/Dx on hardware.
  double  nullspace_kp = 0.0;     // joint posture stiffness (N·m/rad) — 0: pure damping, no snap-back to q_rest
  double  nullspace_kd = 8.0;     // joint posture damping (N·m·s/rad) — viscous resistance to null-space motion
  bool    nullspace_on = true;
  // Posture target source. false (default): q_rest := entry config (legacy "hold
  // where you started"). true: q_rest := nullspace_q_rest, a FIXED config — set
  // nullspace_q_rest to an elbow-up pose and use a LOW nullspace_kp so the operator
  // can override; this is the "default toward up" behaviour for teleop.
  bool    nullspace_use_fixed_rest = false;
  JointVec nullspace_q_rest =     // PLACEHOLDER elbow-up home — TUNE ON HARDWARE
      (JointVec() << 0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57).finished();
  // Manipulability gradient ascent: drive the redundant DOF toward dexterous
  // (non-singular) configs by climbing w(q)=√det(J Jᵀ). Off by default. When on it
  // adds manip_gain·∇w(q) to the null-space torque and costs 7 EXTRA Jacobian evals
  // per cycle — keep manip_gain small; compose with a low posture term if it wanders.
  bool    manip_on = false;
  double  manip_gain = 0.0;       // gradient-ascent gain on √det(J Jᵀ)
  double  manip_fd_step = 1e-4;   // forward finite-difference step (rad) for ∇w
  double  pinv_damping = 1e-3;    // Levenberg-Marquardt lambda for the pseudo-inverse
  double  torque_limit = 39.0;    // per-joint clamp (N·m)
  double  gain_ramp_s  = 0.5;     // ramp the active wrench 0->1 over this window on entry
};

// Task-space impedance: tau = gravity + ramp * (Jᵀ F + nullspace),
//   F = Kx ∘ pose_error(x_d, fk(q)) - Dx ∘ (J qd).  Gravity is always applied in
//   full; the entry ramp fades in only the active wrench over gain_ramp_s.
// Holds the entry pose by default. Live setters publish via a single-writer
// (non-RT) double-buffer; compute() (RT thread) reads one snapshot per cycle.
class CartesianImpedanceMode : public ControlMode, public PoseTargetSink {
 public:
  CartesianImpedanceMode(Dynamics& dyn, CartesianImpedanceParams p = {});
  ActuatorModes required_modes() const override;
  void on_enter(const JointFeedback& fb) override;
  void compute(const JointFeedback& fb, double dt_s, JointCommand& out) override;
  void on_exit() override {}

  // Non-RT setters (call from one supervisor thread).
  void set_gains(const CartesianImpedanceParams& p) noexcept;
  void set_target(const Pose& x_d) noexcept override;

 private:
  CartesianImpedanceParams params() const noexcept;  // RT-safe: returns a value snapshot
  Dynamics& dyn_;

  // Gains double-buffer (writer: set_gains; reader: compute). Seeded at ctor.
  CartesianImpedanceParams gains_[2];
  std::atomic<int> gains_active_{0};

  // Target source: the entry pose (written once by on_enter on the RT thread) or
  // an optional external override published by set_target.
  Pose entry_pose_;
  JointVec q_rest_ = JointVec::Zero();   // nullspace posture target (set on_enter)
  // ext_target_ double-buffer: a SINGLE non-RT supervisor thread writes it (see
  // set_target); compute() (RT) reads it. Two buffers + index flip give the RT
  // reader a torn-read-free snapshot as long as there is only one writer.
  Pose ext_target_[2];
  std::atomic<int> ext_active_{0};
  // Written by BOTH on_enter (RT thread, store false) and set_target (non-RT,
  // store true). Benign: both are release-stores to an atomic<bool>; a false from
  // on_enter merely reverts to entry_pose_, and a true from set_target has its
  // target data published (release) before the flag flip, so the acquire-load in
  // compute() sees a consistent target.
  std::atomic<bool> has_ext_target_{false};

  double ramp_elapsed_ = 0.0;

  // Preallocated RT scratch.
  Jacobian6 J_;
  JointVec  g_;
  JointVec  tau_;
};
}  // namespace kinova
