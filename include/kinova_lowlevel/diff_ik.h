#pragma once
#include <limits>
#include "kinova_lowlevel/cartesian.h"
#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/joint_types.h"
namespace kinova {

struct DiffIkParams {
  int    max_iters      = 4;      // hard cap: this runs inside the 1 kHz cycle
  double pos_tol        = 1e-4;   // m
  double rot_tol        = 1e-3;   // rad
  double damping        = 1e-3;   // Levenberg-Marquardt lambda for the DLS inverse
  // Per-iteration clamp on the task error fed to the solve. This is what makes an
  // unreachable or teleported target safe: the reference walks toward it at a
  // bounded pace instead of producing one enormous step.
  double max_pos_err    = 0.05;   // m
  double max_rot_err    = 0.20;   // rad
  double max_joint_step = 0.05;   // rad, per joint per iteration
  // Stop iterating once the whole step (task + secondary) falls below this. With
  // secondary objectives on, task error alone is not a termination condition --
  // see the comment on solve().
  double min_step       = 1e-7;   // rad
  // Null-space posture bias. Decides the redundant DOF deterministically instead
  // of letting it drift wherever the pose trajectory drags it.
  double posture_gain   = 0.15;
  JointVec q_rest =               // elbow-up home -- TUNE ON HARDWARE
      (JointVec() << 0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57).finished();
  // Null-space push away from hard stops, active only inside limit_margin.
  double limit_gain     = 0.5;
  double limit_margin   = 0.25;   // rad
  double limit_clamp_margin = 0.02;  // rad, hard clamp offset from the stop
  // Default infinite; JointImpedanceMode fills these from the URDF.
  JointVec q_lower = JointVec::Constant(-std::numeric_limits<double>::infinity());
  JointVec q_upper = JointVec::Constant( std::numeric_limits<double>::infinity());
};

struct IkResult {
  double pos_err   = 0.0;   // m,   after the final iteration
  double rot_err   = 0.0;   // rad, after the final iteration
  int    iters     = 0;
  bool   converged = false;
};

// Bounded damped-least-squares Gauss-Newton IK, warm-started from the caller's
// current q. Redundancy is resolved by a null-space posture bias plus limit
// avoidance, so the solution is a deterministic function of (target, seed) rather
// than a free DOF. RT-safe: fixed-size scratch, no heap allocation after ctor.
class DiffIkSolver {
 public:
  DiffIkSolver(Dynamics& dyn, DiffIkParams p);
  // Refines q IN PLACE. Terminates on max_iters, or on a step below min_step, or
  // on task convergence -- but task convergence alone only terminates when NO
  // secondary objective is active. With posture bias or limit avoidance on, the
  // redundant DOF must keep drifting toward the rest posture even while the
  // commanded pose is held perfectly still; that is the whole point of the mode.
  IkResult solve(const Pose& target, JointVec& q);
  void set_params(const DiffIkParams& p) noexcept { p_ = p; }
  const DiffIkParams& params() const noexcept { return p_; }

 private:
  Dynamics& dyn_;
  DiffIkParams p_;
  Jacobian6 J_;   // preallocated RT scratch
};

}  // namespace kinova
