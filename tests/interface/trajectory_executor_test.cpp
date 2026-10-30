#include "kinova_lowlevel/interface/trajectory_executor.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>

#include "kinova_lowlevel/units.h"
using namespace kinova::interface;

static kinova::JointVec vec7(double v) {
  kinova::JointVec q;
  q.setConstant(v);
  return q;
}

TEST(TrajectorySample, LinearInterpBetweenWaypoints) {
  Trajectory tr;
  tr.points = {{vec7(0.0), 0.0}, {vec7(1.0), 2.0}};  // 0 -> 1 rad over 2 s
  EXPECT_NEAR(sample(tr, 0.0)[0], 0.0, 1e-9);
  EXPECT_NEAR(sample(tr, 1.0)[0], 0.5, 1e-9);  // halfway
  EXPECT_NEAR(sample(tr, 2.0)[0], 1.0, 1e-9);
  EXPECT_NEAR(sample(tr, 5.0)[0], 1.0, 1e-9);   // clamps past end
  EXPECT_NEAR(sample(tr, -1.0)[0], 0.0, 1e-9);  // clamps before start
  EXPECT_NEAR(tr.duration_s(), 2.0, 1e-9);
}

TEST(TrajectorySample, HandlesEmptyAndSingleWaypoint) {
  Trajectory empty;
  EXPECT_NEAR(sample(empty, 0.0)[0], 0.0, 1e-9);  // empty -> zero vector, no UB
  EXPECT_NEAR(empty.duration_s(), 0.0, 1e-9);
  Trajectory hold;  // single waypoint -> constant hold, clamps both sides
  hold.points = {{vec7(0.3), 0.0}};
  EXPECT_NEAR(sample(hold, -1.0)[0], 0.3, 1e-9);
  EXPECT_NEAR(sample(hold, 0.0)[0], 0.3, 1e-9);
  EXPECT_NEAR(sample(hold, 5.0)[0], 0.3, 1e-9);
}

namespace {
struct RecordingSink : kinova::JointTargetSink {
  std::vector<kinova::JointVec> calls;
  kinova::JointVec last = kinova::JointVec::Zero();
  kinova::JointTarget last_target{};  // full reference, derivatives included
  void set_joint_target(const kinova::JointTarget& t) noexcept override {
    calls.push_back(t.q);
    last = t.q;
    last_target = t;
  }
};
kinova::interface::Trajectory ramp(double dur) {  // helper: 0->1 rad over dur
  return {{{vec7(0.0), 0.0}, {vec7(1.0), dur}}};
}
}  // namespace

TEST(ExecutorSubmit, AcceptsFirstGoalAndRejectsEmpty) {
  RecordingSink sink;
  kinova::interface::TrajectoryExecutor ex(sink);
  using kinova::interface::ControlModeKind;
  using kinova::interface::Preemption;
  using kinova::interface::SubmitResult;
  EXPECT_EQ(ex.submit(kinova::interface::Trajectory{}, ControlModeKind::kPosition,
                      Preemption::kLatestWins, vec7(-1.0)),
            SubmitResult::kRejectedEmpty);
  EXPECT_EQ(ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins, vec7(-1.0)),
            SubmitResult::kAccepted);
  EXPECT_TRUE(ex.is_active());
  EXPECT_EQ(ex.active_mode(), ControlModeKind::kPosition);
}

TEST(ExecutorSubmit, RejectsModeChangeWhileInFlight) {
  RecordingSink sink;
  kinova::interface::TrajectoryExecutor ex(sink);
  using kinova::interface::ControlModeKind;
  using kinova::interface::Preemption;
  using kinova::interface::SubmitResult;
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins,
            vec7(-1.0));  // now in flight, position
  EXPECT_EQ(ex.submit(ramp(2.0), ControlModeKind::kImpedance, Preemption::kLatestWins, vec7(-1.0)),
            SubmitResult::kRejectedModeChangeWhileMoving);
  // same-mode goal is fine
  EXPECT_EQ(ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins, vec7(-1.0)),
            SubmitResult::kAccepted);
}

TEST(ExecutorSubmit, RejectsAnOutOfRangeOrNonFiniteSpeedScale) {
  // Second layer under the Supervisor's accept-time check: a goal can reach
  // submit() without passing on_trajectory_goal, and effective_scale() clamps
  // internally -- a below-floor scale that got this far would silently run the
  // arm FASTER than asked, and a non-finite one at full speed. Refuse, never
  // clamp, on every path (direct, latest-wins).
  RecordingSink sink;
  kinova::interface::TrajectoryExecutor ex(sink);
  using kinova::interface::ControlModeKind;
  using kinova::interface::Preemption;
  using kinova::interface::SubmitResult;
  const double bad[] = {0.001,
                        0.0,
                        -1.0,
                        1.5,
                        std::numeric_limits<double>::quiet_NaN(),
                        std::numeric_limits<double>::infinity()};
  for (double s : bad) {
    EXPECT_EQ(
        ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins, vec7(-1.0), s),
        SubmitResult::kRejectedSpeedScale)
        << "scale " << s;
    EXPECT_FALSE(ex.is_active()) << "scale " << s;  // refused before any state change
  }
  // Preempting an active goal must refuse too.
  ASSERT_EQ(
      ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins, vec7(-1.0), 0.5),
      SubmitResult::kAccepted);
  EXPECT_EQ(
      ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins, vec7(-1.0), 0.001),
      SubmitResult::kRejectedSpeedScale);
}

TEST(ExecutorSubmit, RejectsWaypointsUnder10nsApart) {
  // Duplicate, out-of-order, sub-10 ns and NaN stamps would reach
  // sample_target as a step or a meaningless quintic qdd (see
  // QuinticDegeneraciesAreSafe). Exactly 10 ns is accepted.
  RecordingSink sink;
  kinova::interface::TrajectoryExecutor ex(sink);
  using namespace kinova::interface;
  auto plan = [](double t1, double t2) {
    Trajectory tr;
    tr.points = {{vec7(0.0), 0.0}, {vec7(0.1), t1}, {vec7(0.2), t2}};
    return tr;
  };
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (auto [t1, t2] : {std::pair{0.0, 1.0}, std::pair{1.0, 1.0}, std::pair{1.0, 0.5},
                        std::pair{1e-9, 1.0}, std::pair{9e-9, 1.0}, std::pair{nan, 1.0}}) {
    EXPECT_EQ(ex.submit(plan(t1, t2), ControlModeKind::kPosition, Preemption::kLatestWins,
                        vec7(-1.0)),
              SubmitResult::kRejectedShortSegment)
        << "t1 " << t1 << " t2 " << t2;
    EXPECT_FALSE(ex.is_active()) << "t1 " << t1 << " t2 " << t2;
  }
  ASSERT_EQ(
      ex.submit(plan(10e-9, 1.0), ControlModeKind::kPosition, Preemption::kLatestWins, vec7(-1.0)),
      SubmitResult::kAccepted);
  // A refused preempt leaves the active goal running.
  EXPECT_EQ(
      ex.submit(plan(1.0, 1.0), ControlModeKind::kPosition, Preemption::kLatestWins, vec7(-1.0)),
      SubmitResult::kRejectedShortSegment);
  EXPECT_TRUE(ex.is_active());
}

TEST(ExecutorTick, SamplesToSinkAndCompletesOnTime) {
  RecordingSink sink;
  kinova::interface::TrajectoryExecutor ex(sink);
  using namespace kinova::interface;
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins, vec7(-1.0));

  ExecStatus s0 = ex.tick(10.0, vec7(0.0));  // start clock at t=10
  EXPECT_TRUE(s0.active);
  EXPECT_FALSE(s0.completed);
  EXPECT_NEAR(sink.calls.back()[0], 0.0, 1e-9);

  ExecStatus s1 = ex.tick(11.0, vec7(0.0));  // 1s in -> halfway
  EXPECT_NEAR(sink.calls.back()[0], 0.5, 1e-9);
  EXPECT_NEAR(s1.fraction, 0.5, 1e-9);
  EXPECT_FALSE(s1.completed);

  ExecStatus s2 = ex.tick(12.0, vec7(1.0));  // at/after final timestamp
  EXPECT_TRUE(s2.completed);
  EXPECT_EQ(s2.error_code, ExecStatus::kOk);
  EXPECT_FALSE(ex.is_active());  // goal left active on completion
}

TEST(ExecutorDivergence, AbortsWhenErrorExceedsPathTolerance) {
  RecordingSink sink;
  kinova::interface::TrajectoryExecutor ex(sink);
  using namespace kinova::interface;
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins, vec7(0.05));
  ex.tick(0.0, vec7(0.0));                 // start; desired 0, meas 0 -> ok
  ExecStatus s = ex.tick(1.0, vec7(0.9));  // desired 0.5, meas 0.9 -> err 0.4 > 0.05
  EXPECT_TRUE(s.completed);
  EXPECT_EQ(s.error_code, ExecStatus::kPathToleranceViolated);
  EXPECT_FALSE(ex.is_active());
}

namespace {
constexpr double kPi = 3.14159265358979323846;
// joint_3 is `type="continuous"` in models/gen3_7dof.urdf, matching the arm.
std::array<bool, kinova::kNumJoints> j3_continuous() {
  std::array<bool, kinova::kNumJoints> c{};
  c[2] = true;
  return c;
}
// A plan walking joint_3 across -pi, unwrapped -- what a planner emits. 1 mrad
// of travel over 1 s, everything else parked.
kinova::interface::Trajectory across_pi() {
  kinova::JointVec a = vec7(0.0), b = vec7(0.0);
  a[2] = -kPi + 0.0005;
  b[2] = -kPi - 0.0005;  // == +3.14109 once wrapped
  return {{{a, 0.0}, {b, 1.0}}};
}
}  // namespace

// The arm tracks the plan perfectly across the boundary. The transport reports the
// measurement wrapped into (-pi, pi], so it comes back with the opposite sign and
// the RAW difference is ~2*pi. Guarding on that aborted a healthy trajectory:
// observed on the arm as joint_3 parked at -3.14154 rad and every GoToEEPose goal
// ending kPathToleranceViolated mid-motion with a final_error of zero.
TEST(ExecutorDivergence, ContinuousJointWrapIsNotDivergence) {
  RecordingSink sink;
  kinova::interface::TrajectoryExecutor ex(sink, j3_continuous());
  using namespace kinova::interface;
  const Trajectory tr = across_pi();
  ex.submit(tr, ControlModeKind::kPosition, Preemption::kLatestWins, vec7(0.35));
  ex.tick(0.0, tr.points.front().q);  // start, on-track

  kinova::JointVec meas = vec7(0.0);
  meas[2] = kPi - 0.0004;  // ~0.1 mrad from q_desired physically; ~2*pi raw
  ExecStatus s = ex.tick(0.9, meas);

  EXPECT_FALSE(s.completed) << "guard aborted a trajectory the arm was tracking";
  EXPECT_EQ(s.error_code, ExecStatus::kOk);
  EXPECT_TRUE(ex.is_active());
}

// The fold must not blind the guard: a continuous joint that is genuinely lost
// still has to trip it. 0.9 rad of real divergence is 0.9 rad after wrapping.
TEST(ExecutorDivergence, ContinuousJointStillAbortsOnRealDivergence) {
  RecordingSink sink;
  kinova::interface::TrajectoryExecutor ex(sink, j3_continuous());
  using namespace kinova::interface;
  const Trajectory tr = across_pi();
  ex.submit(tr, ControlModeKind::kPosition, Preemption::kLatestWins, vec7(0.35));
  ex.tick(0.0, tr.points.front().q);

  kinova::JointVec meas = vec7(0.0);
  meas[2] = kinova::wrap_to_pi(-kPi - 0.0005 + 0.9);  // 0.9 rad off, wrapped
  ExecStatus s = ex.tick(0.9, meas);

  EXPECT_TRUE(s.completed);
  EXPECT_EQ(s.error_code, ExecStatus::kPathToleranceViolated);
}

// A BOUNDED joint is never folded. joint_2 reaches +/-2.41 rad on this arm, so a
// divergence larger than pi is reachable and must still read as a divergence
// rather than being folded down into the tolerance.
TEST(ExecutorDivergence, BoundedJointDivergenceIsNotFolded) {
  RecordingSink sink;
  kinova::interface::TrajectoryExecutor ex(sink, j3_continuous());  // joint_2 bounded
  using namespace kinova::interface;
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins, vec7(0.35));
  ex.tick(0.0, vec7(0.0));

  kinova::JointVec meas = vec7(0.0);
  meas[1] = 2.0 * kPi + 0.5;  // would fold to 0.5 and slip under a blanket wrap
  ExecStatus s = ex.tick(1.0, meas);

  EXPECT_TRUE(s.completed);
  EXPECT_EQ(s.error_code, ExecStatus::kPathToleranceViolated);
}

TEST(ExecutorPreempt, LatestWinsReplacesAndRestartsClock) {
  RecordingSink sink;
  kinova::interface::TrajectoryExecutor ex(sink);
  using namespace kinova::interface;
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins, vec7(-1.0));
  ex.tick(0.0, vec7(0.0));
  ex.tick(1.0, vec7(0.5));  // halfway through first ramp
  // preempt with a fresh 0->1 ramp
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins, vec7(-1.0));
  ExecStatus s = ex.tick(1.0, vec7(0.5));  // first tick of NEW goal -> desired ~0 (clock reset)
  EXPECT_NEAR(sink.calls.back()[0], 0.0, 1e-9);
  EXPECT_NEAR(s.fraction, 0.0, 1e-9);
}

TEST(ExecutorPreempt, QueueWhileActiveIsRefusedAndLeavesTheActiveGoalAlone) {
  RecordingSink sink;
  kinova::interface::TrajectoryExecutor ex(sink);
  using namespace kinova::interface;
  // Active trajectory guarded with a tight tolerance.
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins, vec7(0.05));
  ex.tick(0.0, vec7(0.0));  // start, on-track
  // Queueing is disabled (#84): a QUEUE goal behind an active one is refused,
  // and its DISABLED tolerance (-1) must not relax the running goal's guard.
  EXPECT_EQ(ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kQueue, vec7(-1.0)),
            SubmitResult::kRejectedQueueWhileActive);
  ExecStatus s = ex.tick(1.0, vec7(0.9));  // active desired 0.5, meas 0.9 -> err 0.4 > 0.05
  EXPECT_TRUE(s.completed);
  EXPECT_EQ(s.error_code, ExecStatus::kPathToleranceViolated);
}

TEST(ExecutorPreempt, QueueWhileIdleIsAdopted) {
  RecordingSink sink;
  kinova::interface::TrajectoryExecutor ex(sink);
  using namespace kinova::interface;
  ASSERT_EQ(ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kQueue, vec7(-1.0)),
            SubmitResult::kAccepted);
  EXPECT_TRUE(ex.is_active());
  ex.tick(0.0, vec7(0.0));
  ExecStatus s = ex.tick(2.0, vec7(1.0));
  EXPECT_TRUE(s.completed);
  EXPECT_EQ(s.error_code, ExecStatus::kOk);
  EXPECT_FALSE(ex.is_active());
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
  double q(double t) const { return A * std::sin(w * t + ph); }
  double qd(double t) const { return A * w * std::cos(w * t + ph); }
  double qdd(double t) const { return -A * w * w * std::sin(w * t + ph); }
};
Sine joint_sine(int j) { return Sine{0.5 + 0.1 * j, 1.3 + 0.2 * j, 0.4 * j}; }

// Sample the analytic reference into `n` waypoints spanning [0, dur].
// `order`: 0 = positions only, 1 = + velocities, 2 = + accelerations.
Trajectory analytic_traj(int n, double dur, int order) {
  Trajectory tr;
  tr.has_velocities = order >= 1;
  tr.has_accelerations = order >= 2;
  for (int k = 0; k < n; ++k) {
    const double t = dur * k / (n - 1);
    JointWaypoint w;
    w.t_s = t;
    for (int j = 0; j < kinova::kNumJoints; ++j) {
      const Sine s = joint_sine(j);
      w.q[j] = s.q(t);
      w.qd[j] = s.qd(t);
      w.qdd[j] = s.qdd(t);
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
// Worst excursion of sample() beyond [min, max] of each segment's two endpoint
// positions, any joint, scanned at 1/200 of each span (issue #19).
double hull_excursion(const Trajectory& tr) {
  double worst = 0.0;
  for (size_t k = 0; k + 1 < tr.points.size(); ++k) {
    const JointWaypoint &a = tr.points[k], &b = tr.points[k + 1];
    for (int i = 0; i <= 200; ++i) {
      const kinova::JointVec q = sample(tr, a.t_s + (b.t_s - a.t_s) * i / 200.0);
      for (int j = 0; j < kinova::kNumJoints; ++j) {
        const double lo = std::min(a.q[j], b.q[j]), hi = std::max(a.q[j], b.q[j]);
        worst = std::max({worst, q[j] - hi, lo - q[j]});
      }
    }
  }
  return worst;
}

}  // namespace

TEST(TrajectorySample, PositionsOnlyStaysLinear) {
  // Regression: trajectories without velocities keep the pre-#13 behavior.
  Trajectory tr;
  tr.points = {{vec7(0.0), 0.0}, {vec7(1.0), 2.0}};
  EXPECT_FALSE(tr.has_velocities);
  EXPECT_NEAR(sample(tr, 0.5)[0], 0.25, 1e-9);  // exactly linear
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
        EXPECT_NEAR(vel_left(tr, w.t_s, j), w.qd[j], 1e-3) << "left slope j=" << j;
        EXPECT_NEAR(vel_right(tr, w.t_s, j), w.qd[j], 1e-3) << "right slope j=" << j;
      }
    }
  }
}

TEST(TrajectorySample, CubicHermiteIsC1WhereLinearIsNot) {
  // The jerk in issue #13: linear interpolation steps velocity at every knot.
  const double dur = 2.0;
  const int n = 9;
  const Trajectory cubic = analytic_traj(n, dur, /*order=*/1);
  const Trajectory linear = analytic_traj(n, dur, /*order=*/0);
  double worst_cubic = 0.0, worst_linear = 0.0;
  for (const auto& w : cubic.points) {
    if (w.t_s <= 0.0 || w.t_s >= dur) continue;  // interior knots only
    for (int j = 0; j < kinova::kNumJoints; ++j) {
      worst_cubic =
          std::max(worst_cubic, std::abs(vel_right(cubic, w.t_s, j) - vel_left(cubic, w.t_s, j)));
      worst_linear = std::max(worst_linear,
                              std::abs(vel_right(linear, w.t_s, j) - vel_left(linear, w.t_s, j)));
    }
  }
  EXPECT_LT(worst_cubic, 1e-3) << "cubic Hermite must not step velocity at a knot";
  EXPECT_GT(worst_linear, 0.05) << "linear is expected to step (this is the bug)";
}

TEST(TrajectorySample, QuinticMatchesEndpointPositionVelocityAndAcceleration) {
  // Acceleration is checked against the analytic p''(u)/span^2 (sample_target's
  // qdd) from both sides of every knot, exact at any spacing: n = 6 is the
  // original fixture (span 0.4 s), n = 55 the span-0.037 s case #22 raised, and
  // n = 144 over 2.88 s a 20 ms cuRobo plan. qdd alone could match while q
  // drops its acceleration terms, so a centred second difference of sample()
  // across each knot ties the commanded curve to it. Centred, at h = 1e-4, its
  // error is ~1e-7 roundoff plus O(h) from the jerk step at the knot; the old
  // one-sided form had O(h) truncation and needed its tolerance tuned to h.
  const double h = 1e-4;
  struct Case {
    int n;
    double dur;
  };
  for (const Case c : {Case{6, 2.0}, Case{55, 2.0}, Case{144, 2.88}}) {
    const Trajectory tr = analytic_traj(c.n, c.dur, /*order=*/2);
    for (const auto& w : tr.points) {
      if (w.t_s <= 0.0 || w.t_s >= c.dur) continue;
      // At a knot the search picks the right-hand segment (u = 0); one ulp
      // earlier is the left-hand segment at u -> 1.
      const kinova::JointTarget right = sample_target(tr, w.t_s);
      const kinova::JointTarget left = sample_target(tr, std::nextafter(w.t_s, 0.0));
      for (int j = 0; j < kinova::kNumJoints; ++j) {
        EXPECT_NEAR(right.q[j], w.q[j], 1e-9) << "n=" << c.n << " j=" << j;
        EXPECT_NEAR(vel_left(tr, w.t_s, j), w.qd[j], 1e-3) << "n=" << c.n << " j=" << j;
        EXPECT_NEAR(vel_right(tr, w.t_s, j), w.qd[j], 1e-3) << "n=" << c.n << " j=" << j;
        EXPECT_NEAR(right.qdd[j], w.qdd[j], 1e-6) << "n=" << c.n << " right accel j=" << j;
        EXPECT_NEAR(left.qdd[j], w.qdd[j], 1e-6) << "n=" << c.n << " left accel j=" << j;
        const double curve_acc =
            (sample(tr, w.t_s + h)[j] - 2.0 * sample(tr, w.t_s)[j] + sample(tr, w.t_s - h)[j]) /
            (h * h);
        EXPECT_NEAR(curve_acc, w.qdd[j], 1e-2) << "n=" << c.n << " curve accel j=" << j;
      }
    }
  }
}

TEST(TrajectorySample, HigherOrderTracksTheAnalyticPathMoreAccurately) {
  const double dur = 2.0;
  const int n = 9;
  const double e_lin = max_abs_err(analytic_traj(n, dur, 0), dur);
  const double e_cub = max_abs_err(analytic_traj(n, dur, 1), dur);
  const double e_quin = max_abs_err(analytic_traj(n, dur, 2), dur);
  EXPECT_LT(e_cub, e_lin / 10.0) << "cubic should be far closer than linear";
  EXPECT_LT(e_quin, e_cub);  // quintic closer still
}

TEST(TrajectorySample, HigherOrderDegeneraciesAreSafe) {
  Trajectory tr;  // duplicate timestamps -> zero span, must not NaN
  tr.has_velocities = true;
  tr.points = {{vec7(0.0), 0.0}, {vec7(1.0), 0.0}, {vec7(2.0), 1.0}};
  for (int j = 0; j < kinova::kNumJoints; ++j) {
    EXPECT_TRUE(std::isfinite(sample(tr, 0.0)[j]));
    EXPECT_TRUE(std::isfinite(sample(tr, 0.5)[j]));
  }
  Trajectory single;  // single waypoint with velocity -> constant hold
  single.has_velocities = true;
  single.points = {{vec7(0.3), 0.0}};
  EXPECT_NEAR(sample(single, 2.0)[0], 0.3, 1e-9);
}

TEST(TrajectorySample, QuinticDegeneraciesAreSafe) {
  // Duplicate timestamps mid-plan, with distinct positions and a full profile.
  // upper_bound brackets every query with a.t_s <= t < b.t_s, so the
  // zero-width segment is never selected (nor is the span <= 0 guard reached):
  // the reference steps from wA to wB at t = 1.
  JointWaypoint w0{vec7(0.0), 0.0}, wA{vec7(0.5), 1.0}, wB{vec7(0.6), 1.0}, w2{vec7(1.0), 2.0};
  wA.qd = vec7(0.8);
  wA.qdd = vec7(-0.4);
  wB.qd = vec7(0.3);
  wB.qdd = vec7(1.2);
  Trajectory tr;
  tr.has_velocities = true;
  tr.has_accelerations = true;
  tr.points = {w0, wA, wB, w2};
  for (double t = -0.1; t <= 2.1; t += 1e-3) {
    const kinova::JointTarget ref = sample_target(tr, t);
    EXPECT_TRUE(ref.q.allFinite() && ref.qd.allFinite() && ref.qdd.allFinite()) << "t=" << t;
  }
  EXPECT_NEAR(sample(tr, 1.0)[0], 0.6, 1e-12);                       // wB's segment, u = 0
  EXPECT_NEAR(sample(tr, std::nextafter(1.0, 0.0))[0], 0.5, 1e-12);  // wA's segment, u -> 1
  EXPECT_NEAR(sample_target(tr, 1.0).qdd[0], 1.2, 1e-9);

  // The shortest span a ROS message can express (1 ns): q and qd stay exact and
  // nothing goes non-finite. qdd is checked for finiteness only: below ~10 ns
  // the 1/span^2 scaling amplifies q's rounding until it dominates (~80 rad/s^2
  // off at 1 ns), and once span*span underflows (~2e-162 s) qdd is inf or NaN.
  // submit() refuses spans under kMinSegmentSpanSec; this exercises the sampler.
  JointWaypoint n0{vec7(0.3), 0.0}, n1{vec7(0.3 + 5e-10), 1e-9}, n2{vec7(1.0), 1.0};
  n0.qd = n1.qd = vec7(0.5);
  n0.qdd = n1.qdd = vec7(1.0);
  Trajectory ns;
  ns.has_velocities = true;
  ns.has_accelerations = true;
  ns.points = {n0, n1, n2};
  const kinova::JointTarget mid = sample_target(ns, 5e-10);
  EXPECT_TRUE(mid.q.allFinite() && mid.qd.allFinite() && mid.qdd.allFinite());
  EXPECT_NEAR(mid.q[0], 0.3 + 2.5e-10, 1e-12);
  EXPECT_NEAR(mid.qd[0], 0.5, 1e-3);
}

TEST(TrajectorySample, AccelerationsWithoutVelocitiesFallBackToLinear) {
  // Pins today's behaviour, not an endorsement: accelerations are honoured
  // only together with velocities, so this combination interpolates linearly
  // and reports no profile, silently dropping the qdd the caller supplied.
  // That silence is #20; when it is resolved, update this test with it.
  Trajectory tr = analytic_traj(6, 2.0, /*order=*/2);
  tr.has_velocities = false;
  Trajectory linear = tr;
  linear.has_accelerations = false;
  for (double t : {0.1, 0.55, 1.3, 1.9}) {
    const kinova::JointTarget ref = sample_target(tr, t);
    EXPECT_FALSE(ref.has_velocity);
    EXPECT_FALSE(ref.has_acceleration);
    for (int j = 0; j < kinova::kNumJoints; ++j)
      EXPECT_EQ(ref.q[j], sample(linear, t)[j]) << "t=" << t << " j=" << j;
  }
}

TEST(TrajectorySample, HermiteCanLeaveTheWaypointHullWhenDerivativesMismatchTimestamps) {
  // #19: cubic/quintic Hermite are not hull-preserving, and the path-tolerance
  // guard measures against the interpolant, so it cannot see an excursion.
  // Prints the numbers quoted in docs/deep-dive/trajectory-interpolation.md;
  // asserts only the consistent plan and the x10 row.
  // (a) a 20 ms cuRobo-like plan (144 points / 2.88 s) whose qd/qdd match its
  // timestamps: the only excursion is at direction reversals, where the
  // analytic curve itself peaks between samples by 2.1e-4 rad.
  const Trajectory lin = analytic_traj(144, 2.88, 0), cub = analytic_traj(144, 2.88, 1),
                   quin = analytic_traj(144, 2.88, 2);
  EXPECT_LT(hull_excursion(lin), 1e-12) << "linear is hull-preserving";
  EXPECT_LT(hull_excursion(cub), 3e-4);
  EXPECT_LT(hull_excursion(quin), 3e-4);
  // (b) the same plan re-timed (timestamps x s) without recomputing qd/qdd, as
  // when a client dilates t but forgets v*s and a*s^2. At s = 10 (a 0.1 speed
  // scale) the excursion is ~27x (cubic) and ~150x (quintic) the consistent
  // plan's: quintic's acceleration error grows as s^2 and dominates by then.
  auto retimed = [](Trajectory tr, double s) {
    for (auto& w : tr.points) w.t_s *= s;
    return tr;
  };
  for (double s : {0.5, 1.0, 2.0, 4.0, 10.0, 20.0})
    std::printf("hull excursion, re-timed x%g: cubic %.3g rad, quintic %.3g rad\n", s,
                hull_excursion(retimed(cub, s)), hull_excursion(retimed(quin, s)));
  // Downsampled, derivatives kept: the interpolant still follows the plan, but
  // the curve between samples at a reversal grows with the spacing squared.
  Trajectory every5th = quin;
  every5th.points.clear();
  for (size_t i = 0; i < quin.points.size(); i += 5) every5th.points.push_back(quin.points[i]);
  std::printf("hull excursion, every 5th point (100 ms): quintic %.3g rad\n",
              hull_excursion(every5th));
  EXPECT_GT(hull_excursion(retimed(cub, 10.0)), 5e-3);
  EXPECT_GT(hull_excursion(retimed(quin, 10.0)), 3e-2);
}

TEST(EffectiveScale, TakesTheSlowerOfGoalAndOverride) {
  EXPECT_DOUBLE_EQ(effective_scale(1.0, 1.0), 1.0);
  EXPECT_DOUBLE_EQ(effective_scale(0.25, 1.0), 0.25);
  EXPECT_DOUBLE_EQ(effective_scale(1.0, 0.3), 0.3);
  EXPECT_DOUBLE_EQ(effective_scale(0.5, 0.2), 0.2);
}

TEST(EffectiveScale, NeverExceedsOneAndNeverReachesZero) {
  // The contract is "this can only ever slow the arm down".
  EXPECT_DOUBLE_EQ(effective_scale(2.0, 1.0), 1.0);
  EXPECT_DOUBLE_EQ(effective_scale(1.0, 5.0), 1.0);
  EXPECT_GT(effective_scale(0.0, 1.0), 0.0);
  EXPECT_GT(effective_scale(-1.0, 1.0), 0.0);
}

TEST(EffectiveScale, NonFiniteFallsBackToFullSpeed) {
  // NaN compares false against every bound, so a naive clamp would pass it
  // straight through and stop the clock forever.
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_DOUBLE_EQ(effective_scale(nan, 1.0), 1.0);
  EXPECT_DOUBLE_EQ(effective_scale(1.0, nan), 1.0);
}

TEST(ExecutorSpeedScale, HalfScaleTakesTwiceAsLongInWallTime) {
  RecordingSink sink;
  TrajectoryExecutor ex(sink);
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins,
            kinova::JointVec::Constant(-1.0), 0.5);

  ex.tick(0.0, vec7(0.0));                   // latch the clock
  ExecStatus mid = ex.tick(2.0, vec7(0.0));  // 2 s wall = 1 s trajectory time
  EXPECT_TRUE(mid.active);
  EXPECT_NEAR(mid.fraction, 0.5, 1e-9) << "fraction must track the SCALED clock";
  EXPECT_NEAR(sink.last[0], 0.5, 1e-9) << "halfway along a 0->1 ramp";

  ExecStatus end = ex.tick(4.0, vec7(1.0));  // 4 s wall = 2 s trajectory time
  EXPECT_TRUE(end.completed);
  EXPECT_NEAR(end.fraction, 1.0, 1e-9);
}

TEST(ExecutorSpeedScale, FullScaleIsIdenticalToBeforeTheFeature) {
  RecordingSink sink;
  TrajectoryExecutor ex(sink);
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins,
            kinova::JointVec::Constant(-1.0));  // default scale
  ex.tick(10.0, vec7(0.0));
  ExecStatus mid = ex.tick(11.0, vec7(0.0));
  EXPECT_NEAR(mid.fraction, 0.5, 1e-9);
  EXPECT_NEAR(sink.last[0], 0.5, 1e-9);
}

TEST(ExecutorSpeedScale, LatestWinsResetsTheScaledClock) {
  RecordingSink sink;
  TrajectoryExecutor ex(sink);
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins,
            kinova::JointVec::Constant(-1.0), 1.0);
  ex.tick(0.0, vec7(0.0));
  ex.tick(1.5, vec7(0.0));  // 1.5 s into the first trajectory

  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins,
            kinova::JointVec::Constant(-1.0), 1.0);
  ex.tick(2.0, vec7(0.0));
  ExecStatus s = ex.tick(2.5, vec7(0.0));
  EXPECT_NEAR(s.fraction, 0.25, 1e-9)
      << "the replacement starts from zero, not from the old elapsed time";
}

// A goal boundary latches the applied scale to the new goal's own scale, in
// both directions, instead of slewing from the outgoing goal's. Ticks are
// 0.01 s apart, so a slew (kScaleSlewPerSec * 0.01 = 0.02) could not cover
// the 0.1 <-> 1.0 gap and would show up here.
TEST(ExecutorSpeedScale, LatestWinsLatchesToTheNewGoalsScaleBothWays) {
  RecordingSink sink;
  TrajectoryExecutor ex(sink);
  ex.submit(ramp(10.0), ControlModeKind::kPosition, Preemption::kLatestWins,
            kinova::JointVec::Constant(-1.0), 0.1);  // slow
  ex.tick(0.0, vec7(0.0));
  ex.tick(0.01, vec7(0.0));
  ASSERT_NEAR(ex.applied_scale(), 0.1, 1e-9);

  ex.submit(ramp(10.0), ControlModeKind::kPosition, Preemption::kLatestWins,
            kinova::JointVec::Constant(-1.0), 1.0);  // preempt with a fast goal
  ex.tick(0.02, vec7(0.0));
  EXPECT_NEAR(ex.applied_scale(), 1.0, 1e-9) << "must latch up, not ramp from 0.1";

  ex.tick(0.03, vec7(0.0));
  ex.submit(ramp(10.0), ControlModeKind::kPosition, Preemption::kLatestWins,
            kinova::JointVec::Constant(-1.0), 0.1);  // preempt with a slow goal
  ex.tick(0.04, vec7(0.0));
  EXPECT_NEAR(ex.applied_scale(), 0.1, 1e-9) << "must latch down, not ramp from 1.0";
}

TEST(ExecutorSpeedScale, AnOverrideChangeRampsRatherThanSteps) {
  RecordingSink sink;
  TrajectoryExecutor ex(sink);
  ex.submit(ramp(10.0), ControlModeKind::kPosition, Preemption::kLatestWins,
            kinova::JointVec::Constant(-1.0), 1.0);
  ex.tick(0.0, vec7(0.0), 1.0);
  EXPECT_NEAR(ex.applied_scale(), 1.0, 1e-9);

  // Slam the override to 0.1 and step 10 ms. A step would move the reference
  // discontinuously; the slew limit must keep the change bounded. Pinned to
  // the actual constant (kScaleSlewPerSec = 2.0 -> max_step = 2.0 * 0.01 =
  // 0.02, so applied_ = 1.0 - 0.02 = 0.98) rather than a loose (0.1, 1.0)
  // bound: that bound also passes a constant ten times too large -- a
  // full-range change in 50 ms, exactly the step this feature exists to
  // prevent.
  ex.tick(0.01, vec7(0.0), 0.1);
  EXPECT_NEAR(ex.applied_scale(), 0.98, 1e-9);

  for (double t = 0.02; t < 3.0; t += 0.01) ex.tick(t, vec7(0.0), 0.1);
  EXPECT_NEAR(ex.applied_scale(), 0.1, 1e-6) << "and must get there";
}

// Fix wave, finding 8: a negative dt_wall correctly freezes applied_ (via
// max_step = 0), but traj_t is an ACCUMULATOR now -- unlike the old
// `now_s - start_time` form, an un-guarded `traj_t += dt_wall * applied_`
// would walk it backwards permanently rather than self-correcting on the
// next forward tick. Unreachable today (secs_since uses steady_clock), but
// cheap to close at the source.
TEST(ExecutorSpeedScale, BackwardsWallClockDoesNotMoveTrajectoryTimeBackwards) {
  RecordingSink sink;
  TrajectoryExecutor ex(sink);
  ex.submit(ramp(10.0), ControlModeKind::kPosition, Preemption::kLatestWins,
            kinova::JointVec::Constant(-1.0), 1.0);
  ex.tick(5.0, vec7(0.0));                      // latch the clock at t=5
  ExecStatus before = ex.tick(6.0, vec7(0.0));  // 1 s wall -> traj_t=1.0, fraction 0.1
  EXPECT_NEAR(before.fraction, 0.1, 1e-9);

  ExecStatus back = ex.tick(5.5, vec7(0.0));  // now_s went BACKWARDS
  EXPECT_NEAR(back.fraction, before.fraction, 1e-9)
      << "a backwards wall-clock stamp must never move trajectory time backwards";
}

// ---- v1.3.0 target feedforward: sample_target + scaled derivatives ----
namespace {
// A profiled S-curve: rest -> rest over 2 s with a nonzero mid velocity.
kinova::interface::Trajectory profiled(bool with_accel) {
  kinova::interface::Trajectory tr;
  kinova::interface::JointWaypoint w0{vec7(0.0), 0.0}, w1{vec7(0.5), 1.0}, w2{vec7(1.0), 2.0};
  w1.qd = vec7(0.8);  // moving through the midpoint
  tr.points = {w0, w1, w2};
  tr.has_velocities = true;
  tr.has_accelerations = with_accel;
  return tr;
}
}  // namespace

TEST(SampleTarget, DerivativesMatchFiniteDifferencesOfSample) {
  using kinova::interface::sample;
  using kinova::interface::sample_target;
  for (bool accel : {false, true}) {
    const auto tr = profiled(accel);
    const double h = 1e-6;
    for (double t : {0.3, 0.7, 1.4}) {
      const auto ref = sample_target(tr, t);
      ASSERT_TRUE(ref.has_velocity);
      EXPECT_EQ(ref.has_acceleration, accel);
      const kinova::JointVec fd = (sample(tr, t + h) - sample(tr, t - h)) / (2.0 * h);
      EXPECT_NEAR(ref.qd[0], fd[0], 1e-3) << "accel=" << accel << " t=" << t;
      if (accel) {
        const kinova::JointVec fdd =
            (sample(tr, t + h) - 2.0 * sample(tr, t) + sample(tr, t - h)) / (h * h);
        EXPECT_NEAR(ref.qdd[0], fdd[0], 1e-2) << " t=" << t;
      }
    }
  }
}

TEST(SampleTarget, ProfiledTrajectoryAlwaysReportsAVelocityOpinion) {
  // has_velocity must hold at EVERY query of a profiled trajectory -- edges,
  // normal segments, and duplicate-timestamp (zero-span) segments alike. A
  // flag that flickered off flipped the impedance damper to absolute-velocity
  // damping for exactly those ticks (review finding).
  kinova::interface::Trajectory tr = profiled(false);
  kinova::interface::JointWaypoint dup{vec7(0.5), 1.0};  // duplicates w1's t_s
  dup.qd = vec7(0.0);
  tr.points.insert(tr.points.begin() + 2, dup);
  for (double t = -0.2; t <= 2.2; t += 0.01) {
    const auto ref = kinova::interface::sample_target(tr, t);
    EXPECT_TRUE(ref.has_velocity) << "t=" << t;
    EXPECT_FALSE(ref.qd.hasNaN()) << "t=" << t;
  }
}

TEST(SampleTarget, PositionsOnlyTrajectoryCarriesNoProfile) {
  const auto ref = kinova::interface::sample_target(ramp(2.0), 1.0);
  EXPECT_FALSE(ref.has_velocity);
  EXPECT_FALSE(ref.has_acceleration);
}

TEST(SampleTarget, OutsideTheSpanHoldsStillWithProfilePresent) {
  // Holding is a real reference state: has_velocity true, qd zero — so the
  // damper damps toward rest while holding instead of toward a phantom.
  for (double t : {-0.1, 2.1}) {
    const auto ref = kinova::interface::sample_target(profiled(false), t);
    EXPECT_TRUE(ref.has_velocity);
    EXPECT_NEAR(ref.qd.norm(), 0.0, 1e-12) << " t=" << t;
  }
}

TEST(ExecutorTick, ScalesFedForwardDerivativesIntoWallTime) {
  RecordingSink sink;
  kinova::interface::TrajectoryExecutor ex(sink);
  using namespace kinova::interface;
  ASSERT_EQ(ex.submit(profiled(false), ControlModeKind::kImpedance, Preemption::kLatestWins,
                      vec7(-1.0), 0.5),
            SubmitResult::kAccepted);
  ex.tick(10.0, vec7(0.0));  // clock starts; applied_ latches to 0.5
  ex.tick(10.4, vec7(0.0));  // 0.4 s wall -> traj_t = 0.2
  const auto analytic = sample_target(profiled(false), 0.2);
  ASSERT_TRUE(sink.last_target.has_velocity);
  EXPECT_NEAR(sink.last_target.qd[0], 0.5 * analytic.qd[0], 1e-9);
  EXPECT_NEAR(sink.last_target.q[0], analytic.q[0], 1e-9);  // position unscaled
}
