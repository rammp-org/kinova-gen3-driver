#include <gtest/gtest.h>
#include "kinova_lowlevel/interface/trajectory_executor.h"
using namespace kinova::interface;

static kinova::JointVec vec7(double v) { kinova::JointVec q; q.setConstant(v); return q; }

TEST(TrajectorySample, LinearInterpBetweenWaypoints) {
  Trajectory tr;
  tr.points = { {vec7(0.0), 0.0}, {vec7(1.0), 2.0} };   // 0 -> 1 rad over 2 s
  EXPECT_NEAR(sample(tr, 0.0)[0], 0.0, 1e-9);
  EXPECT_NEAR(sample(tr, 1.0)[0], 0.5, 1e-9);           // halfway
  EXPECT_NEAR(sample(tr, 2.0)[0], 1.0, 1e-9);
  EXPECT_NEAR(sample(tr, 5.0)[0], 1.0, 1e-9);           // clamps past end
  EXPECT_NEAR(sample(tr, -1.0)[0], 0.0, 1e-9);          // clamps before start
  EXPECT_NEAR(tr.duration_s(), 2.0, 1e-9);
}

TEST(TrajectorySample, HandlesEmptyAndSingleWaypoint) {
  Trajectory empty;
  EXPECT_NEAR(sample(empty, 0.0)[0], 0.0, 1e-9);   // empty -> zero vector, no UB
  EXPECT_NEAR(empty.duration_s(), 0.0, 1e-9);
  Trajectory hold;                                 // single waypoint -> constant hold, clamps both sides
  hold.points = { {vec7(0.3), 0.0} };
  EXPECT_NEAR(sample(hold, -1.0)[0], 0.3, 1e-9);
  EXPECT_NEAR(sample(hold, 0.0)[0], 0.3, 1e-9);
  EXPECT_NEAR(sample(hold, 5.0)[0], 0.3, 1e-9);
}

namespace {
struct RecordingSink : kinova::JointTargetSink {
  std::vector<kinova::JointVec> calls;      // positions, for the existing assertions
  std::vector<kinova::JointTarget> targets; // the full reference, derivatives included
  void set_joint_target(const kinova::JointTarget& t) noexcept override {
    calls.push_back(t.q);
    targets.push_back(t);
  }
};
kinova::interface::Trajectory ramp(double dur) {  // helper: 0->1 rad over dur
  return { { {vec7(0.0), 0.0}, {vec7(1.0), dur} } };
}
}  // namespace

TEST(ExecutorSubmit, AcceptsFirstGoalAndRejectsEmpty) {
  RecordingSink sink;
  kinova::interface::TrajectoryExecutor ex(sink);
  using kinova::interface::SubmitResult; using kinova::interface::ControlModeKind;
  using kinova::interface::Preemption;
  EXPECT_EQ(ex.submit(kinova::interface::Trajectory{}, ControlModeKind::kPosition, Preemption::kLatestWins, vec7(-1.0)),
            SubmitResult::kRejectedEmpty);
  EXPECT_EQ(ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins, vec7(-1.0)),
            SubmitResult::kAccepted);
  EXPECT_TRUE(ex.is_active());
  EXPECT_EQ(ex.active_mode(), ControlModeKind::kPosition);
}

TEST(ExecutorSubmit, RejectsModeChangeWhileInFlight) {
  RecordingSink sink;
  kinova::interface::TrajectoryExecutor ex(sink);
  using kinova::interface::SubmitResult; using kinova::interface::ControlModeKind;
  using kinova::interface::Preemption;
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins, vec7(-1.0));   // now in flight, position
  EXPECT_EQ(ex.submit(ramp(2.0), ControlModeKind::kImpedance, Preemption::kLatestWins, vec7(-1.0)),
            SubmitResult::kRejectedModeChangeWhileMoving);
  // same-mode goal is fine
  EXPECT_EQ(ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins, vec7(-1.0)),
            SubmitResult::kAccepted);
}

TEST(ExecutorTick, SamplesToSinkAndCompletesOnTime) {
  RecordingSink sink;
  kinova::interface::TrajectoryExecutor ex(sink);
  using namespace kinova::interface;
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins, vec7(-1.0));

  ExecStatus s0 = ex.tick(10.0, vec7(0.0));   // start clock at t=10
  EXPECT_TRUE(s0.active); EXPECT_FALSE(s0.completed);
  EXPECT_NEAR(sink.calls.back()[0], 0.0, 1e-9);

  ExecStatus s1 = ex.tick(11.0, vec7(0.0));   // 1s in -> halfway
  EXPECT_NEAR(sink.calls.back()[0], 0.5, 1e-9);
  EXPECT_NEAR(s1.fraction, 0.5, 1e-9);
  EXPECT_FALSE(s1.completed);

  ExecStatus s2 = ex.tick(12.0, vec7(1.0));   // at/after final timestamp
  EXPECT_TRUE(s2.completed);
  EXPECT_EQ(s2.error_code, ExecStatus::kOk);
  EXPECT_FALSE(ex.is_active());               // goal left active on completion
}

TEST(ExecutorDivergence, AbortsWhenErrorExceedsPathTolerance) {
  RecordingSink sink;
  kinova::interface::TrajectoryExecutor ex(sink);
  using namespace kinova::interface;
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins, vec7(0.05));
  ex.tick(0.0, vec7(0.0));                      // start; desired 0, meas 0 -> ok
  ExecStatus s = ex.tick(1.0, vec7(0.9));       // desired 0.5, meas 0.9 -> err 0.4 > 0.05
  EXPECT_TRUE(s.completed);
  EXPECT_EQ(s.error_code, ExecStatus::kPathToleranceViolated);
  EXPECT_FALSE(ex.is_active());
}

TEST(ExecutorDivergence, AbortWithQueuedGoalClearsQueue) {
  RecordingSink sink;
  kinova::interface::TrajectoryExecutor ex(sink);
  using namespace kinova::interface;
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins, vec7(0.05)); // active, tight tol
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kQueue,     vec7(-1.0));  // queued follow-on
  ex.tick(0.0, vec7(0.0));                        // start, on-track
  ExecStatus s = ex.tick(1.0, vec7(0.9));         // active diverges (err 0.4 > 0.05) -> abort whole chain
  EXPECT_TRUE(s.completed);
  EXPECT_EQ(s.error_code, ExecStatus::kPathToleranceViolated);
  EXPECT_FALSE(ex.is_active());                   // queued follow-on dropped, not stranded
  ExecStatus after = ex.tick(2.0, vec7(0.0));     // stays idle — no phantom promotion
  EXPECT_FALSE(after.active);
  EXPECT_FALSE(after.completed);
}

TEST(ExecutorPreempt, LatestWinsReplacesAndRestartsClock) {
  RecordingSink sink;
  kinova::interface::TrajectoryExecutor ex(sink);
  using namespace kinova::interface;
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins, vec7(-1.0));
  ex.tick(0.0, vec7(0.0));
  ex.tick(1.0, vec7(0.5));                       // halfway through first ramp
  // preempt with a fresh 0->1 ramp
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins, vec7(-1.0));
  ExecStatus s = ex.tick(1.0, vec7(0.5));        // first tick of NEW goal -> desired ~0 (clock reset)
  EXPECT_NEAR(sink.calls.back()[0], 0.0, 1e-9);
  EXPECT_NEAR(s.fraction, 0.0, 1e-9);
}

TEST(ExecutorPreempt, QueueDoesNotClobberActivePathTolerance) {
  RecordingSink sink;
  kinova::interface::TrajectoryExecutor ex(sink);
  using namespace kinova::interface;
  // Active trajectory guarded with a tight tolerance.
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins, vec7(0.05));
  ex.tick(0.0, vec7(0.0));                        // start, on-track
  // Queue a second goal with a DISABLED tolerance (-1). It must NOT relax the guard
  // on the still-running active trajectory (the queued tol only applies once promoted).
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kQueue, vec7(-1.0));
  ExecStatus s = ex.tick(1.0, vec7(0.9));         // active desired 0.5, meas 0.9 -> err 0.4 > 0.05
  EXPECT_TRUE(s.completed);
  EXPECT_EQ(s.error_code, ExecStatus::kPathToleranceViolated);
}

TEST(ExecutorPreempt, QueuePromotesGaplesslyOnCompletion) {
  RecordingSink sink;
  kinova::interface::TrajectoryExecutor ex(sink);
  using namespace kinova::interface;
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins, vec7(-1.0)); // active
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kQueue,     vec7(-1.0)); // queued
  ex.tick(0.0, vec7(0.0));
  ExecStatus at_end = ex.tick(2.0, vec7(1.0));   // first ramp ends here
  EXPECT_TRUE(at_end.active);                    // NOT idle — queued promoted
  EXPECT_FALSE(at_end.completed);                // continuous motion, no completion gap
  ExecStatus mid = ex.tick(3.0, vec7(0.5));      // 1s into promoted ramp -> desired 0.5
  EXPECT_NEAR(sink.calls.back()[0], 0.5, 1e-9);
  EXPECT_TRUE(mid.active);
}

TEST(ExecutorPreempt, PromotedGoalAdoptsItsOwnPathTolerance) {
  RecordingSink sink;
  kinova::interface::TrajectoryExecutor ex(sink);
  using namespace kinova::interface;
  // Active guard DISABLED; queued goal carries a TIGHT tolerance that must take
  // effect only once it is promoted (guards the Task-5 tolerance-isolation fix).
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins, vec7(-1.0)); // active, guard off
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kQueue,     vec7(0.05));  // queued, tight
  ex.tick(0.0, vec7(0.0));
  ex.tick(2.0, vec7(1.0));                        // first ramp completes -> promote queued (adopt 0.05)
  // On the promoted ramp: desired at elapsed=1 is 0.5; meas 0.9 -> err 0.4 > 0.05 -> abort.
  ExecStatus s = ex.tick(3.0, vec7(0.9));
  EXPECT_TRUE(s.completed);
  EXPECT_EQ(s.error_code, ExecStatus::kPathToleranceViolated);
}

TEST(ExecutorPreempt, PromotionRaisesPromotedFlagExactlyOnce) {
  RecordingSink sink;
  kinova::interface::TrajectoryExecutor ex(sink);
  using namespace kinova::interface;
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins, vec7(-1.0)); // active
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kQueue,     vec7(-1.0)); // queued
  ex.tick(0.0, vec7(0.0));
  ExecStatus mid = ex.tick(1.0, vec7(0.5));  EXPECT_FALSE(mid.promoted);
  ExecStatus at_end = ex.tick(2.0, vec7(1.0));  // first ramp ends -> promote queued
  EXPECT_TRUE(at_end.promoted);
  EXPECT_TRUE(at_end.active);
  ExecStatus after = ex.tick(3.0, vec7(0.5));  EXPECT_FALSE(after.promoted);
}

// ---------------------------------------------------------------------------
// Issue #13: planner velocities/accelerations drive the interpolation order.
// cuRobo emits qd/qdd; linear interpolation of a ~20 ms-spaced plan puts a
// velocity step at every waypoint (~50/s), which reads as jerky motion.
// ---------------------------------------------------------------------------
namespace {

// Smooth analytic reference, per joint: q(t) = A sin(w t + ph).
struct Sine {
  double A, w, ph;
  double q(double t)   const { return A * std::sin(w * t + ph); }
  double qd(double t)  const { return A * w * std::cos(w * t + ph); }
  double qdd(double t) const { return -A * w * w * std::sin(w * t + ph); }
};
Sine joint_sine(int j) { return Sine{0.5 + 0.1 * j, 1.3 + 0.2 * j, 0.4 * j}; }

// Sample the analytic reference into `n` waypoints spanning [0, dur].
// `order`: 0 = positions only, 1 = + velocities, 2 = + accelerations.
Trajectory analytic_traj(int n, double dur, int order) {
  Trajectory tr;
  tr.has_velocities    = order >= 1;
  tr.has_accelerations = order >= 2;
  for (int k = 0; k < n; ++k) {
    const double t = dur * k / (n - 1);
    JointWaypoint w;
    w.t_s = t;
    for (int j = 0; j < kinova::kNumJoints; ++j) {
      const Sine s = joint_sine(j);
      w.q[j] = s.q(t); w.qd[j] = s.qd(t); w.qdd[j] = s.qdd(t);
    }
    tr.points.push_back(w);
  }
  return tr;
}

// One-sided numeric derivatives of sample(), for continuity checks at knots.
double vel_left(const Trajectory& tr, double t, int j, double h = 1e-6) {
  return (sample(tr, t)[j] - sample(tr, t - h)[j]) / h;
}
double vel_right(const Trajectory& tr, double t, int j, double h = 1e-6) {
  return (sample(tr, t + h)[j] - sample(tr, t)[j]) / h;
}
double max_abs_err(const Trajectory& tr, double dur) {  // vs the analytic reference
  double worst = 0.0;
  for (int i = 0; i <= 500; ++i) {
    const double t = dur * i / 500.0;
    const kinova::JointVec q = sample(tr, t);
    for (int j = 0; j < kinova::kNumJoints; ++j)
      worst = std::max(worst, std::abs(q[j] - joint_sine(j).q(t)));
  }
  return worst;
}

}  // namespace

TEST(TrajectorySample, PositionsOnlyStaysLinear) {
  // Regression: trajectories without velocities keep the pre-#13 behavior.
  Trajectory tr;
  tr.points = { {vec7(0.0), 0.0}, {vec7(1.0), 2.0} };
  EXPECT_FALSE(tr.has_velocities);
  EXPECT_NEAR(sample(tr, 0.5)[0], 0.25, 1e-9);   // exactly linear
  EXPECT_NEAR(sample(tr, 1.0)[0], 0.50, 1e-9);
  EXPECT_NEAR(sample(tr, 1.5)[0], 0.75, 1e-9);
}

TEST(TrajectorySample, CubicHermiteMatchesEndpointPositionsAndVelocities) {
  const double dur = 2.0;
  const Trajectory tr = analytic_traj(6, dur, /*order=*/1);
  for (const auto& w : tr.points) {
    const kinova::JointVec q = sample(tr, w.t_s);
    for (int j = 0; j < kinova::kNumJoints; ++j) {
      EXPECT_NEAR(q[j], w.q[j], 1e-9) << "position at knot t=" << w.t_s << " j=" << j;
      // Interior knots: the slope on both sides must equal the planner's qd.
      if (w.t_s > 0.0 && w.t_s < dur) {
        EXPECT_NEAR(vel_left(tr, w.t_s, j),  w.qd[j], 1e-3) << "left slope j=" << j;
        EXPECT_NEAR(vel_right(tr, w.t_s, j), w.qd[j], 1e-3) << "right slope j=" << j;
      }
    }
  }
}

TEST(TrajectorySample, CubicHermiteIsC1WhereLinearIsNot) {
  // The jerk in issue #13: linear interpolation steps velocity at every knot.
  const double dur = 2.0;
  const int n = 9;
  const Trajectory cubic  = analytic_traj(n, dur, /*order=*/1);
  const Trajectory linear = analytic_traj(n, dur, /*order=*/0);
  double worst_cubic = 0.0, worst_linear = 0.0;
  for (const auto& w : cubic.points) {
    if (w.t_s <= 0.0 || w.t_s >= dur) continue;              // interior knots only
    for (int j = 0; j < kinova::kNumJoints; ++j) {
      worst_cubic  = std::max(worst_cubic,
          std::abs(vel_right(cubic,  w.t_s, j) - vel_left(cubic,  w.t_s, j)));
      worst_linear = std::max(worst_linear,
          std::abs(vel_right(linear, w.t_s, j) - vel_left(linear, w.t_s, j)));
    }
  }
  EXPECT_LT(worst_cubic, 1e-3) << "cubic Hermite must not step velocity at a knot";
  EXPECT_GT(worst_linear, 0.05) << "linear is expected to step (this is the bug)";
}

TEST(TrajectorySample, QuinticMatchesEndpointPositionVelocityAndAcceleration) {
  const double dur = 2.0;
  const Trajectory tr = analytic_traj(6, dur, /*order=*/2);
  const double h = 1e-4;
  for (const auto& w : tr.points) {
    if (w.t_s <= 0.0 || w.t_s >= dur) continue;
    for (int j = 0; j < kinova::kNumJoints; ++j) {
      EXPECT_NEAR(sample(tr, w.t_s)[j], w.q[j], 1e-9);
      EXPECT_NEAR(vel_left(tr, w.t_s, j),  w.qd[j], 1e-3);
      EXPECT_NEAR(vel_right(tr, w.t_s, j), w.qd[j], 1e-3);
      // second derivative, one-sided, must match the planner's qdd
      const double acc_r =
          (sample(tr, w.t_s + 2 * h)[j] - 2 * sample(tr, w.t_s + h)[j] + sample(tr, w.t_s)[j]) / (h * h);
      EXPECT_NEAR(acc_r, w.qdd[j], 5e-2) << "right accel j=" << j;
    }
  }
}

TEST(TrajectorySample, HigherOrderTracksTheAnalyticPathMoreAccurately) {
  const double dur = 2.0;
  const int n = 9;
  const double e_lin  = max_abs_err(analytic_traj(n, dur, 0), dur);
  const double e_cub  = max_abs_err(analytic_traj(n, dur, 1), dur);
  const double e_quin = max_abs_err(analytic_traj(n, dur, 2), dur);
  EXPECT_LT(e_cub,  e_lin  / 10.0) << "cubic should be far closer than linear";
  EXPECT_LT(e_quin, e_cub);        // quintic closer still
}

TEST(TrajectorySample, HigherOrderDegeneraciesAreSafe) {
  Trajectory tr;                       // duplicate timestamps -> zero span, must not NaN
  tr.has_velocities = true;
  tr.points = { {vec7(0.0), 0.0}, {vec7(1.0), 0.0}, {vec7(2.0), 1.0} };
  for (int j = 0; j < kinova::kNumJoints; ++j) {
    EXPECT_TRUE(std::isfinite(sample(tr, 0.0)[j]));
    EXPECT_TRUE(std::isfinite(sample(tr, 0.5)[j]));
  }
  Trajectory single;                   // single waypoint with velocity -> constant hold
  single.has_velocities = true;
  single.points = { {vec7(0.3), 0.0} };
  EXPECT_NEAR(sample(single, 2.0)[0], 0.3, 1e-9);
}

// --- sample_target: the derivatives handed to a mode as feedforward ---------

TEST(TrajectorySampleTarget, PositionsOnlyReportsNoDerivatives) {
  Trajectory tr;
  tr.points = { {vec7(0.0), 0.0}, {vec7(1.0), 2.0} };
  const kinova::JointTarget t = sample_target(tr, 1.0);
  EXPECT_NEAR(t.q[0], 0.5, 1e-9);
  EXPECT_FALSE(t.has_velocity) << "no profile -> nothing to feed forward";
  EXPECT_FALSE(t.has_acceleration);
  EXPECT_NEAR(t.qd[0], 0.0, 1e-12);
}

TEST(TrajectorySampleTarget, DerivativesMatchTheInterpolantItself) {
  // The feedforward must describe the very curve sample() commands, so compare
  // against a numeric derivative of sample() rather than the planner's inputs.
  const double dur = 2.0;
  for (int order = 1; order <= 2; ++order) {
    const Trajectory tr = analytic_traj(9, dur, order);
    for (double t : {0.31, 0.77, 1.24, 1.85}) {
      const kinova::JointTarget got = sample_target(tr, t);
      ASSERT_TRUE(got.has_velocity) << "order=" << order;
      EXPECT_EQ(got.has_acceleration, order >= 2);
      const double h = 1e-5;
      for (int j = 0; j < kinova::kNumJoints; ++j) {
        EXPECT_NEAR(got.q[j], sample(tr, t)[j], 1e-12);
        const double vel_fd = (sample(tr, t + h)[j] - sample(tr, t - h)[j]) / (2 * h);
        EXPECT_NEAR(got.qd[j], vel_fd, 1e-4) << "order=" << order << " t=" << t << " j=" << j;
        if (order >= 2) {
          const double acc_fd =
              (sample(tr, t + h)[j] - 2 * sample(tr, t)[j] + sample(tr, t - h)[j]) / (h * h);
          EXPECT_NEAR(got.qdd[j], acc_fd, 1e-1) << "order=2 t=" << t << " j=" << j;
        }
      }
    }
  }
}

TEST(TrajectorySampleTarget, HeldOutsideTheSpanMeansZeroVelocity) {
  // Before the start and past the end the reference is parked, so feeding a
  // non-zero velocity forward would command motion that is not happening.
  const Trajectory tr = analytic_traj(6, 2.0, /*order=*/2);
  for (double t : {-0.5, 0.0, 2.0, 5.0}) {
    const kinova::JointTarget got = sample_target(tr, t);
    for (int j = 0; j < kinova::kNumJoints; ++j) {
      EXPECT_NEAR(got.qd[j], 0.0, 1e-12) << "t=" << t;
      EXPECT_NEAR(got.qdd[j], 0.0, 1e-12) << "t=" << t;
    }
  }
}
