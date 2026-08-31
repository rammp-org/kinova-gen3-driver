#pragma once
#include <atomic>
#include <limits>
#include <Eigen/Cholesky>
#include "kinova_lowlevel/cartesian_types.h"
#include "kinova_lowlevel/command_watchdog.h"
#include "kinova_lowlevel/control_mode.h"
#include "kinova_lowlevel/dynamics.h"
namespace kinova {

struct JointVelocityParams {
  // Per-joint cap on commanded velocity [rad/s]. Non-finite entries are seeded
  // from the URDF; finite entries are clamped DOWN to it. No configuration can
  // ask for more than the hardware is rated for.
  JointVec max_qd = JointVec::Constant(std::numeric_limits<double>::infinity());

  // Baseline Levenberg-Marquardt damping for the 6x7 twist solve.
  double dls_damping = 1e-3;
  // Manipulability w = sqrt(det(J J^T)) below which damping is raised toward
  // dls_damping_max. This is the ONLY thing keeping commanded velocity bounded
  // near a singularity: in a torque law a bad solve produces a bounded torque,
  // but here whatever is computed goes straight to the actuators.
  double w_threshold     = 0.02;
  double dls_damping_max = 0.10;

  // Null-space posture bias [1/s]. Without it the redundant DOF drifts and the
  // elbow wanders while the tool tracks the twist perfectly.
  double posture_gain = 0.5;
  JointVec q_rest =   // elbow-up home; matches DiffIkParams::q_rest
      (JointVec() << 0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57).finished();

  // Staleness watchdog. 0 DISABLES it, matching every other mode's default.
  double cmd_timeout_s = 0.0;
};

// Joint-space velocity control. Commands every actuator in kVelocity and lets the
// actuator's own servo close the loop.
//
// STIFF BY CONTRACT. This mode does not yield to contact and makes no attempt to.
// A compliant velocity law is a DIFFERENT promise and belongs in a different mode
// -- delivering compliance from something named "velocity" is exactly the silent
// semantic difference this driver refuses to ship.
//
// Two target shapes:
//   set_velocity_target(qd)  - native; commanded through unchanged (then limited)
//   set_twist_target(V)      - EE twist [linear; angular] in the base frame,
//                              mapped by damped least squares + null-space posture
//
// Staleness commands ZERO velocity and LATCHES until a fresh target arrives.
//
// Live setters publish via a single-writer (non-RT) double-buffer; compute()
// (RT thread) reads one snapshot per cycle.
class JointVelocityMode : public ControlMode {
 public:
  JointVelocityMode(Dynamics& dyn, JointVelocityParams p = {});
  ActuatorModes required_modes() const override;
  void on_enter(const JointFeedback& fb) override;
  void compute(const JointFeedback& fb, double dt_s, JointCommand& out) override;
  void on_exit() override {}

  // Non-RT setters (call from one supervisor thread). Latest setter wins: a twist
  // target supersedes a joint-velocity target and vice-versa.
  void set_velocity_target(const JointVec& qd_d) noexcept;
  void set_twist_target(const Vector6& V) noexcept;
  void set_params(const JointVelocityParams& p) noexcept;

  // s >= 0 arms with s; s < 0 restores params().cmd_timeout_s.
  void set_command_timeout(double s) noexcept;

  // ACTIVE parameters, i.e. after URDF seeding and clamping -- not necessarily
  // what was passed in. Read this back when a speed request seems ignored.
  JointVelocityParams params() const noexcept;

  // RT-thread-owned state, for tests and post-stop inspection. NOT synchronized.
  JointVec commanded() const noexcept { return qd_cmd_; }
  // Manipulability at the last twist solve, sqrt(det(J J^T)). 0 until a twist
  // has been solved. Worth watching: it is what drives the damping.
  double last_manipulability() const noexcept { return w_last_; }

 private:
  void seed_limits(JointVelocityParams& p) const noexcept;
  // RT: J(q) -> damped least squares -> null-space posture. Writes qd_out.
  void solve_twist(const JointVec& q, const Vector6& V,
                   const JointVelocityParams& p, JointVec& qd_out) noexcept;
  // RT: uniform scale so the fastest joint just reaches its cap, then a hard
  // per-joint clamp as a backstop.
  static void limit(const JointVelocityParams& p, JointVec& qd) noexcept;

  Dynamics& dyn_;
  JointVec v_max_urdf_ = JointVec::Zero();   // cached in ctor: set_params must not
                                             // touch Dynamics off the RT thread
  std::array<bool, kNumJoints> continuous_{};

  JointVelocityParams params_[2];
  std::atomic<int> params_active_{0};

  enum class Source : int { kNone, kJoint, kTwist };
  std::atomic<Source> source_{Source::kNone};
  JointVec ext_qd_[2];
  std::atomic<int> qd_active_{0};
  Vector6 ext_twist_[2];
  std::atomic<int> tw_active_{0};

  CommandWatchdog wd_;
  // RT-owned. Latched rather than recomputed per cycle so DISARMING the watchdog
  // cannot un-freeze and resurrect a target nobody is maintaining.
  bool frozen_ = false;

  // RT-owned adopted targets and preallocated scratch.
  JointVec qd_target_ = JointVec::Zero();
  Vector6  twist_target_ = Vector6::Zero();
  JointVec qd_cmd_ = JointVec::Zero();
  Jacobian6 J_ = Jacobian6::Zero();
  Eigen::Matrix<double, 6, 6> A_ = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt_;
  Vector6  y_ = Vector6::Zero();
  JointVec bias_ = JointVec::Zero();
  double   w_last_ = 0.0;
};

}  // namespace kinova
