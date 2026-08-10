#include "kinova_lowlevel/interface/trajectory_executor.h"
#include <algorithm>
#include <cmath>
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

ExecStatus TrajectoryExecutor::tick(double now_s, const kinova::JointVec& /*q_meas*/) {
  if (!active_) return ExecStatus{false, false, 0.0, ExecStatus::kOk};
  Active& a = *active_;
  if (!a.started) { a.start_time = now_s; a.started = true; }
  const double elapsed = now_s - a.start_time;
  const double dur = a.tr.duration_s();
  sink_.set_joint_target(sample(a.tr, elapsed));
  const double frac = dur > 0.0 ? std::min(1.0, std::max(0.0, elapsed / dur)) : 1.0;
  if (elapsed >= dur) {
    active_.reset();
    return ExecStatus{false, true, 1.0, ExecStatus::kOk};   // time-based completion
  }
  return ExecStatus{true, false, frac, ExecStatus::kOk};
}

}  // namespace kinova::interface
