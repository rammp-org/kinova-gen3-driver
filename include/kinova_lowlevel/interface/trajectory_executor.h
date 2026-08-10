#pragma once
#include <vector>
#include <optional>
#include "kinova_lowlevel/joint_types.h"   // kinova::JointVec, kNumJoints
namespace kinova::interface {

struct JointWaypoint { kinova::JointVec q; double t_s; };
struct Trajectory {
  std::vector<JointWaypoint> points;
  double duration_s() const { return points.empty() ? 0.0 : points.back().t_s; }
};

kinova::JointVec sample(const Trajectory& tr, double t_s);

enum class Preemption { kQueue, kLatestWins };
enum class ControlModeKind { kPosition, kImpedance };
enum class SubmitResult { kAccepted, kRejectedModeChangeWhileMoving, kRejectedEmpty };

struct ExecStatus {
  static constexpr int kOk = 0;
  static constexpr int kPathToleranceViolated = -4;
  bool active; bool completed; double fraction; int error_code;
};

class JointTargetSink {
 public:
  virtual ~JointTargetSink() = default;
  virtual void set_joint_target(const kinova::JointVec&) = 0;
};

class TrajectoryExecutor {
 public:
  explicit TrajectoryExecutor(JointTargetSink& sink) : sink_(sink) {}
  SubmitResult submit(const Trajectory& tr, ControlModeKind mode, Preemption p, const kinova::JointVec& path_tol);
  bool is_active() const { return active_.has_value() || queued_.has_value(); }
  ControlModeKind active_mode() const { return mode_; }
  ExecStatus tick(double now_s, const kinova::JointVec& q_meas);

 private:
  struct Active { Trajectory tr; double start_time = 0.0; bool started = false; };
  JointTargetSink& sink_;
  ControlModeKind mode_ = ControlModeKind::kPosition;
  std::optional<Active> active_;
  std::optional<Trajectory> queued_;
  Preemption queued_pre_ = Preemption::kLatestWins;
  kinova::JointVec path_tol_ = kinova::JointVec::Zero();
};

}  // namespace kinova::interface
