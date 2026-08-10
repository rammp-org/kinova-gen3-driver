#include "kinova_lowlevel/interface/trajectory_executor.h"
#include <algorithm>
namespace kinova::interface {

kinova::JointVec sample(const Trajectory& tr, double t_s) {
  const auto& p = tr.points;
  if (p.empty()) return kinova::JointVec::Zero();
  if (t_s <= p.front().t_s) return p.front().q;
  if (t_s >= p.back().t_s)  return p.back().q;
  // find first waypoint with t_s greater than the query
  auto hi = std::upper_bound(p.begin(), p.end(), t_s,
      [](double t, const JointWaypoint& w){ return t < w.t_s; });
  const JointWaypoint& b = *hi;
  const JointWaypoint& a = *(hi - 1);
  const double span = b.t_s - a.t_s;
  const double u = span > 0.0 ? (t_s - a.t_s) / span : 0.0;
  return a.q + u * (b.q - a.q);
}

SubmitResult TrajectoryExecutor::submit(const Trajectory& tr, ControlModeKind mode, Preemption p) {
  if (tr.points.empty()) return SubmitResult::kRejectedEmpty;
  if (is_active() && mode != mode_) return SubmitResult::kRejectedModeChangeWhileMoving;
  if (!is_active()) {                       // idle -> adopt immediately
    mode_ = mode;
    active_ = Active{tr, 0.0, false};
    queued_.reset();
    return SubmitResult::kAccepted;
  }
  // Preemption handling lands in Tasks 5-6; for now, latest-wins replaces active.
  active_ = Active{tr, 0.0, false};
  return SubmitResult::kAccepted;
}

}  // namespace kinova::interface
