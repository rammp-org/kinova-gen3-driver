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

namespace {
struct RecordingSink : kinova::interface::JointTargetSink {
  std::vector<kinova::JointVec> calls;
  void set_joint_target(const kinova::JointVec& q) override { calls.push_back(q); }
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
