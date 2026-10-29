#pragma once
#include "kinova_lowlevel/control_mode.h"
#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/joint_types.h"
namespace kinova {

struct FloatParams {
  JointVec damping = (JointVec() << 5, 5, 5, 5, 1.5, 1.5, 1.5).finished();  // N·m·s/rad
  // Weak anchor spring (N·m/rad). K_s·drag_band is the most it ever pulls:
  // ~1 N·m on j1-j4, ~0.3 N·m on j5-j7.
  JointVec spring = (JointVec() << 28.6, 28.6, 28.6, 28.6, 8.6, 8.6, 8.6).finished();
  double drag_band_rad = 2.0 * kDeg2Rad;  // anchor trails q by at most this
  // Per-joint ceiling on the TOTAL output (joints 5-7 have a 9 N·m effort limit).
  JointVec torque_limit = (JointVec() << 39, 39, 39, 39, 9, 9, 9).finished();
};

// The contact-stop control law: yields to any push, slows under damping, and
// stays where it is left.
//   tau = gravity(q) - D*qd - K_s*(q - q_a), clamped per joint.
// The anchor q_a starts at the entry pose and is dragged so it never trails q by
// more than drag_band_rad, so the spring only resists model-error creep.
class FloatMode : public ControlMode {
 public:
  // Throws std::invalid_argument on non-finite or negative gains, a
  // non-positive band, or a non-positive torque limit.
  FloatMode(Dynamics& dyn, FloatParams p = {});
  ActuatorModes required_modes() const override;
  void on_enter(const JointFeedback& fb) override;
  void compute(const JointFeedback& fb, double dt_s, JointCommand& out) override;
  void on_exit() override {}

  const JointVec& anchor() const noexcept { return anchor_; }

 private:
  Dynamics& dyn_;
  FloatParams p_;
  JointVec anchor_ = JointVec::Zero();  // RT-thread-only state

  // Preallocated RT scratch.
  JointVec g_;
  JointVec tau_;
};
}  // namespace kinova
