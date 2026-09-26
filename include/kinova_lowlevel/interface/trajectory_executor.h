#pragma once
#include <array>
#include <cmath>
#include <optional>
#include <vector>

#include "kinova_lowlevel/joint_target_sink.h"  // kinova::JointTargetSink (the mode seam)
#include "kinova_lowlevel/joint_types.h"        // kinova::JointVec, kNumJoints
namespace kinova::interface {

// qd/qdd carry a planner's velocity/acceleration profile. They are meaningful
// only when the owning Trajectory sets the matching has_* flag; otherwise they
// are ignored (a zero qd is NOT the same as "no velocity" — see sample()).
struct JointWaypoint {
  kinova::JointVec q;
  double t_s;
  kinova::JointVec qd = kinova::JointVec::Zero();
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

enum class Preemption { kQueue, kLatestWins };
enum class ControlModeKind { kPosition, kImpedance, kVelocity, kTorque };
enum class SubmitResult { kAccepted, kRejectedModeChangeWhileMoving, kRejectedEmpty };

// Floor for the effective scale. Zero would stop the clock and hang the goal
// forever; this is slow enough to be a hold in practice and still terminates.
inline constexpr double kMinSpeedScale = 0.01;

// The slower of the goal's own scale and the runtime override, clamped into
// (0, 1]. Free and inline so it is unit-testable without threads. A non-finite
// input is treated as "no request" rather than propagated: NaN compares false
// against every bound, and a NaN scale would stop the trajectory clock dead.
inline double effective_scale(double goal_scale, double override_scale) {
  const double g = std::isfinite(goal_scale) ? goal_scale : 1.0;
  const double o = std::isfinite(override_scale) ? override_scale : 1.0;
  double s = g < o ? g : o;
  if (s > 1.0) s = 1.0;
  if (s < kMinSpeedScale) s = kMinSpeedScale;
  return s;
}

struct ExecStatus {
  static constexpr int kOk = 0;
  static constexpr int kPathToleranceViolated = -4;
  bool active;
  bool completed;
  double fraction;
  int error_code;
  bool promoted = false;
};

// The executor drives the driver's control modes through the core
// kinova::JointTargetSink (set_target(JointVec)) — both JointPositionMode and
// JointImpedanceMode implement it — so no interface-local sink is needed.
class TrajectoryExecutor {
 public:
  // `continuous` marks the joints whose measurement wraps into (-pi, pi] (the
  // URDF's `type="continuous"`). The divergence guard needs it because q_meas
  // arrives wrapped from the transport while a planner emits q_desired
  // unwrapped -- see tick(). Defaults to all-false, which is the exact
  // pre-existing behaviour for a caller that does not supply it.
  explicit TrajectoryExecutor(kinova::JointTargetSink& sink,
                              const std::array<bool, kinova::kNumJoints>& continuous = {})
      : sink_(sink), continuous_(continuous) {}
  SubmitResult submit(const Trajectory& tr, ControlModeKind mode, Preemption p,
                      const kinova::JointVec& path_tol);
  bool is_active() const { return active_.has_value() || queued_.has_value(); }
  ControlModeKind active_mode() const { return mode_; }
  ExecStatus tick(double now_s, const kinova::JointVec& q_meas);

 private:
  struct Active {
    Trajectory tr;
    double start_time = 0.0;
    bool started = false;
  };
  kinova::JointTargetSink& sink_;
  std::array<bool, kinova::kNumJoints> continuous_{};
  ControlModeKind mode_ = ControlModeKind::kPosition;
  std::optional<Active> active_;
  std::optional<Trajectory> queued_;
  kinova::JointVec path_tol_ = kinova::JointVec::Zero();  // guards the ACTIVE trajectory
  kinova::JointVec queued_tol_ =
      kinova::JointVec::Zero();  // applied when queued_ is promoted (Task 6)
};

}  // namespace kinova::interface
