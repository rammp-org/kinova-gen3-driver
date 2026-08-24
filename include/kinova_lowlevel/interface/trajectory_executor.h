#pragma once
#include <vector>
#include <optional>
#include "kinova_lowlevel/joint_types.h"        // kinova::JointVec, kNumJoints
#include "kinova_lowlevel/joint_target_sink.h"  // kinova::JointTargetSink (the mode seam)
namespace kinova::interface {

// qd/qdd carry a planner's velocity/acceleration profile. They are meaningful
// only when the owning Trajectory sets the matching has_* flag; otherwise they
// are ignored (a zero qd is NOT the same as "no velocity" — see sample()).
struct JointWaypoint {
  kinova::JointVec q;
  double t_s;
  kinova::JointVec qd  = kinova::JointVec::Zero();
  kinova::JointVec qdd = kinova::JointVec::Zero();
};
struct Trajectory {
  std::vector<JointWaypoint> points;
  // Selects the interpolation order in sample(), the way ros2_control's
  // joint_trajectory_controller does: linear (positions only), cubic Hermite
  // (+ velocities), quintic (+ accelerations). Accelerations are honoured only
  // together with velocities.
  bool has_velocities = false;
  bool has_accelerations = false;
  double duration_s() const { return points.empty() ? 0.0 : points.back().t_s; }
};

kinova::JointVec sample(const Trajectory& tr, double t_s);

// The same reference, plus the derivatives of the very polynomial sample()
// evaluates — so a mode's feedforward is exactly consistent with the position
// it is being asked to track. Derivatives are reported only when the trajectory
// actually carries a profile; a positions-only trajectory yields q alone and
// the has_* flags stay false.
kinova::JointTarget sample_target(const Trajectory& tr, double t_s);

enum class Preemption { kQueue, kLatestWins };
enum class ControlModeKind { kPosition, kImpedance };
enum class SubmitResult { kAccepted, kRejectedModeChangeWhileMoving, kRejectedEmpty };

struct ExecStatus {
  static constexpr int kOk = 0;
  static constexpr int kPathToleranceViolated = -4;
  bool active; bool completed; double fraction; int error_code; bool promoted = false;
};

// The executor drives the driver's control modes through the core
// kinova::JointTargetSink (set_target(JointVec)) — both JointPositionMode and
// JointImpedanceMode implement it — so no interface-local sink is needed.
class TrajectoryExecutor {
 public:
  explicit TrajectoryExecutor(kinova::JointTargetSink& sink) : sink_(sink) {}
  SubmitResult submit(const Trajectory& tr, ControlModeKind mode, Preemption p, const kinova::JointVec& path_tol);
  bool is_active() const { return active_.has_value() || queued_.has_value(); }
  ControlModeKind active_mode() const { return mode_; }
  ExecStatus tick(double now_s, const kinova::JointVec& q_meas);

 private:
  struct Active { Trajectory tr; double start_time = 0.0; bool started = false; };
  kinova::JointTargetSink& sink_;
  ControlModeKind mode_ = ControlModeKind::kPosition;
  std::optional<Active> active_;
  std::optional<Trajectory> queued_;
  kinova::JointVec path_tol_ = kinova::JointVec::Zero();        // guards the ACTIVE trajectory
  kinova::JointVec queued_tol_ = kinova::JointVec::Zero();      // applied when queued_ is promoted (Task 6)
};

}  // namespace kinova::interface
