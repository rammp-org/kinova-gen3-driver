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

SubmitResult TrajectoryExecutor::submit(const Trajectory& tr, ControlModeKind mode, Preemption p, const kinova::JointVec& path_tol) {
  if (tr.points.empty()) return SubmitResult::kRejectedEmpty;
  if (is_active() && mode != mode_) return SubmitResult::kRejectedModeChangeWhileMoving;
  if (!is_active()) {                       // idle -> adopt immediately
    mode_ = mode;
    active_ = Active{tr, 0.0, false};
    path_tol_ = path_tol;                   // tolerance guards the adopted trajectory
    queued_.reset();
    return SubmitResult::kAccepted;
  }
  // active, same mode: preempt per the caller's policy.
  if (p == Preemption::kLatestWins) {
    active_ = Active{tr, 0.0, false};   // replace + reset clock (started=false)
    path_tol_ = path_tol;               // new trajectory's tolerance takes over
    queued_.reset();
    return SubmitResult::kAccepted;
  }
  // kQueue: store trajectory + its tolerance for gapless promotion on completion
  // (promotion in Task 6). Do NOT touch path_tol_: the active trajectory keeps its
  // own divergence guard until the queued goal is actually promoted.
  queued_ = tr;
  queued_tol_ = path_tol;
  return SubmitResult::kAccepted;
}

ExecStatus TrajectoryExecutor::tick(double now_s, const kinova::JointVec& q_meas) {
  if (!active_) return ExecStatus{false, false, 0.0, ExecStatus::kOk};
  Active& a = *active_;
  if (!a.started) { a.start_time = now_s; a.started = true; }
  const double elapsed = now_s - a.start_time;
  const double dur = a.tr.duration_s();
  const kinova::JointVec q_desired = sample(a.tr, elapsed);
  sink_.set_target(q_desired);
  const double frac = dur > 0.0 ? std::min(1.0, std::max(0.0, elapsed / dur)) : 1.0;

  // Check divergence guard
  for (int i = 0; i < kinova::kNumJoints; ++i) {
    if (path_tol_[i] > 0.0 && std::abs(q_meas[i] - q_desired[i]) > path_tol_[i]) {
      active_.reset();
      queued_.reset();   // a fault aborts the whole chain — don't strand a queued follow-on
      return ExecStatus{false, true, frac, ExecStatus::kPathToleranceViolated};
    }
  }

  if (elapsed >= dur) {
    if (queued_) {                                  // gapless promotion — no idle gap
      active_ = Active{*queued_, now_s, true};      // latch start to NOW (started=true)
      path_tol_ = queued_tol_;                      // adopt the promoted goal's divergence guard
      queued_.reset();
      return ExecStatus{true, false, 0.0, ExecStatus::kOk, true};   // promoted this tick
    }
    active_.reset();
    return ExecStatus{false, true, 1.0, ExecStatus::kOk};   // time-based completion
  }
  return ExecStatus{true, false, frac, ExecStatus::kOk};
}

}  // namespace kinova::interface
