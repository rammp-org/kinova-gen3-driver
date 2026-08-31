#pragma once
#include <atomic>
#include <cstdint>
#include "kinova_lowlevel/command_watchdog.h"
#include "kinova_lowlevel/control_mode.h"
#include "kinova_lowlevel/dynamics.h"
namespace kinova {

struct JointTorqueParams {
  double scale         = 1.0;   // gravity scale
  double damping       = 0.0;   // joint velocity damping (N·m·s/rad)
  // Per-joint ceiling on the TOTAL output. The URDF gives joints 5-7 an effort
  // limit of 9 N·m; a single scalar sized for the proximal joints would overrun
  // the wrist by 4x. Mirrors JointImpedanceParams::torque_limit.
  JointVec torque_limit = (JointVec() << 39, 39, 39, 39, 9, 9, 9).finished();
  double cmd_timeout_s = 0.1;   // staleness watchdog window; <=0 disables
  double cmd_decay_s   = 0.05;  // ramp tau_ff -> 0 over this window on staleness
                                // (<=0 => hard zero)
};

// Joint-torque control: tau = scale*gravity(q) - damping*qd + tau_ff, clamped.
// tau_ff is published by ONE non-RT thread via set_torque() (two-buffer +
// atomic index) and read once per cycle by compute() (RT). A staleness watchdog
// decays tau_ff to zero if no fresh command arrives within cmd_timeout_s,
// reverting to gravity-compensation hold. With tau_ff never set, this mode is
// identical to gravity compensation.
class JointTorqueMode : public ControlMode {
 public:
  JointTorqueMode(Dynamics& dyn, JointTorqueParams p = {});
  ActuatorModes required_modes() const override;
  void on_enter(const JointFeedback& fb) override;
  void compute(const JointFeedback& fb, double dt_s, JointCommand& out) override;
  void on_exit() override {}

  // Non-RT: call from a single supervisor thread.
  void set_torque(const JointVec& tau_ff) noexcept;

  // Non-RT: re-arm the staleness watchdog. s >= 0 arms with s; s < 0 restores
  // this mode's own configured default (cmd_timeout_s), which is how a closing
  // streaming session hands the mode back to its own supervision.
  void set_command_timeout(double s) noexcept;

 private:
  Dynamics& dyn_;
  JointTorqueParams p_;

  // tau_ff publication: single writer (set_torque) -> single reader (compute).
  JointVec tau_ff_buf_[2];
  std::atomic<int> tau_ff_active_{0};

  // Staleness detection; the RESPONSE (the decay ramp below) stays here because
  // it is this mode's contract, not shared behaviour.
  CommandWatchdog wd_;

  // RT-thread-only state.
  JointVec tau_ff_target_  = JointVec::Zero();   // latest adopted command
  JointVec tau_ff_applied_ = JointVec::Zero();   // post-decay value summed in

  // Preallocated RT scratch.
  JointVec g_;
  JointVec tau_;
};
}  // namespace kinova
