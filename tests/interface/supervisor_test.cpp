#include <gtest/gtest.h>
#include "kinova_lowlevel/interface/value_types.h"
#include "kinova_lowlevel/interface/ports.h"
#include "fake_backend.h"
#include "kinova_lowlevel/interface/supervisor.h"
#include "kinova_lowlevel/joint_position_mode.h"
#include "kinova_lowlevel/joint_impedance_mode.h"
#include "kinova_lowlevel/rt_executor.h"
#include "kinova_lowlevel/sim_transport.h"
#include "kinova_lowlevel/feedback_tap.h"
#include "kinova_lowlevel/dynamics.h"
#include <atomic>
#include <chrono>
#include <thread>
using namespace kinova;
using namespace kinova::interface;

TEST(ValueTypes, DefaultsAndResultCodes) {
  TrajectoryGoal g;                          // default-constructs
  g.control_mode = ControlModeKind::kPosition;
  g.preemption   = Preemption::kLatestWins;
  g.path_tolerance = JointVec::Constant(0.2);
  EXPECT_EQ(g.trajectory.points.size(), 0u);
  EXPECT_FALSE(g.has_gains);
  EXPECT_EQ(result_code::kSuccessful, 0);
  EXPECT_EQ(result_code::kPathToleranceViolated, -4);
  EXPECT_EQ(result_code::kPreempted, -6);
  ArmState s; s.q = JointVec::Constant(0.1);
  EXPECT_NEAR(s.q[0], 0.1, 1e-12);
  GoalId id{}; EXPECT_EQ(id.size(), 16u);
}

TEST(Ports, FakeBackendRecordsDrivenCalls) {
  FakeBackend be;
  StreamPort& sp = be; ActionServerPort& ap = be;
  ArmState s; s.q = JointVec::Constant(0.3);
  sp.publish_state(s);
  GoalId id{}; id[0] = 7;
  TrajectoryResult r; r.error_code = interface::result_code::kSuccessful;
  ap.settle(id, r);
  EXPECT_EQ(be.state_count(), 1u);
  EXPECT_NEAR(be.last_state().q[0], 0.3, 1e-12);
  EXPECT_EQ(be.result_count(), 1u);
  EXPECT_EQ(be.last_result().error_code, 0);
}

namespace {
JointFeedback make_feedback(double q0) {
  JointFeedback f;
  f.q = JointVec::Constant(q0);
  return f;
}

// Fixture wires: SimTransport -> FeedbackTap -> RtExecutor(main-ish thread) + Supervisor + FakeBackend.
// q0 seeds the initial joint position; init.q is set before SimTransport is
// constructed so the tap wraps a correctly-seeded transport from the start.
struct SupFix {
  Dynamics dyn{URDF_PATH}, pump_dyn{URDF_PATH};
  JointFeedback init;
  SimTransport sim;
  Seqlock<JointFeedback> snap;
  FeedbackTap tap{sim, snap};
  SampleRing ring{1u << 12};
  JointPositionMode pos{dyn};
  JointImpedanceMode imp{dyn};
  RtExecutor exec{tap, ring, {1000.0, kinova::Pacing::kSleepSpin, {}}};
  FakeBackend be;
  interface::Supervisor sup{pos, imp, exec, snap, pump_dyn, be, be};
  std::atomic<bool> stop{false};
  std::thread rt;

  explicit SupFix(double q0 = 0.0) : init(make_feedback(q0)), sim(init) {}

  void run_rt() { rt = std::thread([&]{ exec.run(stop); }); }
  void teardown() { stop = true; if (rt.joinable()) rt.join(); }
};
}  // namespace

TEST(Supervisor, StartStopClean) {
  SupFix f;
  f.sup.start();
  f.run_rt();
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  f.sup.stop();
  f.teardown();
  SUCCEED();          // no crash, no hang, threads joined
}

TEST(Supervisor, PumpPublishesArmStateFromFeedback) {
  SupFix f(0.25);     // seed a non-zero start pose before the tap wires up
  f.sup.start(); f.run_rt();
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  f.sup.stop(); f.teardown();
  ASSERT_GT(f.be.state_count(), 0u);
  EXPECT_NEAR(f.be.last_state().q[0], 0.25, 1e-6);   // q flowed feedback->pump->StreamPort
  // query_state returns the same latest snapshot:
  EXPECT_NEAR(f.sup.on_query_state().q[0], 0.25, 1e-6);
}

static interface::Trajectory ramp7(double from,double to,double dur){
  interface::Trajectory t; JointVec a=JointVec::Constant(from), b=JointVec::Constant(to);
  t.points = {{a,0.0},{b,dur}}; return t; }

TEST(Supervisor, PositionGoalRunsToCompletionAndSettlesSuccess) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::TrajectoryGoal g;
  g.trajectory = ramp7(0.0, 0.05, 0.4);
  g.control_mode = interface::ControlModeKind::kPosition;
  g.preemption   = interface::Preemption::kLatestWins;
  g.path_tolerance = JointVec::Constant(-1.0);       // guard off for this test (sim is static echo)
  interface::GoalId id{}; id[0]=1;
  ASSERT_EQ(f.sup.on_trajectory_goal(g), interface::GoalResponse::kAccept);
  f.sup.on_trajectory_accepted(id, g);
  std::this_thread::sleep_for(std::chrono::milliseconds(800));   // > duration + settle
  f.sup.stop(); f.teardown();
  ASSERT_EQ(f.be.result_count(), 1u);
  EXPECT_EQ(f.be.last_result().error_code, interface::result_code::kSuccessful);
  EXPECT_EQ(f.be.last_result_id()[0], 1);
  EXPECT_GT(f.be.feedback_count(), 0u);              // add feedback_count() to FakeBackend
}

TEST(Supervisor, LatestWinsPreemptionSettlesOldGoalPreempted) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::TrajectoryGoal g; g.trajectory=ramp7(0.0,0.1,2.0); g.path_tolerance=JointVec::Constant(-1.0);
  g.control_mode=interface::ControlModeKind::kPosition; g.preemption=interface::Preemption::kLatestWins;
  interface::GoalId a{}; a[0]=1; interface::GoalId b{}; b[0]=2;
  f.sup.on_trajectory_goal(g); f.sup.on_trajectory_accepted(a, g);
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  f.sup.on_trajectory_goal(g); f.sup.on_trajectory_accepted(b, g);   // latest-wins preempt
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  f.sup.stop(); f.teardown();
  bool saw_preempt_a=false;
  for (auto& pr : f.be.all_results()) if (pr.first[0]==1) saw_preempt_a = (pr.second.error_code==interface::result_code::kPreempted);
  EXPECT_TRUE(saw_preempt_a);
}

TEST(Supervisor, QueuedGoalPromotesAndBothSettleSuccessful) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::TrajectoryGoal ga; ga.trajectory=ramp7(0.0,0.05,0.2);
  ga.control_mode=interface::ControlModeKind::kPosition; ga.preemption=interface::Preemption::kLatestWins;
  ga.path_tolerance=JointVec::Constant(-1.0);
  interface::TrajectoryGoal gb=ga; gb.preemption=interface::Preemption::kQueue;   // queued follow-on
  interface::GoalId a{}; a[0]=1; interface::GoalId b{}; b[0]=2;
  ASSERT_EQ(f.sup.on_trajectory_goal(ga), interface::GoalResponse::kAccept);
  f.sup.on_trajectory_accepted(a, ga);
  std::this_thread::sleep_for(std::chrono::milliseconds(60));                       // A still active
  ASSERT_EQ(f.sup.on_trajectory_goal(gb), interface::GoalResponse::kAccept);        // kQueue now accepted
  f.sup.on_trajectory_accepted(b, gb);
  std::this_thread::sleep_for(std::chrono::milliseconds(600));                      // A completes+promotes, B completes
  f.sup.stop(); f.teardown();
  int a_code=999, b_code=999, a_n=0, b_n=0;
  for (auto& pr : f.be.all_results()) {
    if (pr.first[0]==1) { a_code = pr.second.error_code; ++a_n; }
    if (pr.first[0]==2) { b_code = pr.second.error_code; ++b_n; }
  }
  EXPECT_EQ(f.be.result_count(), 2u);                       // exactly two settlements, no double-settle
  EXPECT_EQ(a_n, 1); EXPECT_EQ(b_n, 1);                     // each id settled exactly once
  EXPECT_EQ(a_code, interface::result_code::kSuccessful);   // finished goal settles successful on promotion
  EXPECT_EQ(b_code, interface::result_code::kSuccessful);   // promoted goal settles successful on completion
}

TEST(Supervisor, RejectsCrossModeGoalThatSlipsInFlightPrecheck) {
  // in_flight_ is set only when the sampler drains a goal, so a second goal
  // submitted back-to-back (before the drain) passes on_trajectory_goal's
  // mode-change pre-check. The sampler must fail-loud on the cross-mode goal
  // rather than switch modes mid-flight and orphan it.
  SupFix f; f.sup.start(); f.run_rt();
  interface::TrajectoryGoal gp; gp.trajectory=ramp7(0.0,0.05,0.3);
  gp.control_mode=interface::ControlModeKind::kPosition; gp.preemption=interface::Preemption::kLatestWins;
  gp.path_tolerance=JointVec::Constant(-1.0);
  interface::TrajectoryGoal gi; gi.trajectory=ramp7(0.0,0.05,0.3);
  gi.control_mode=interface::ControlModeKind::kImpedance; gi.preemption=interface::Preemption::kQueue;
  gi.path_tolerance=JointVec::Constant(-1.0);
  gi.has_gains=true; gi.gains.kq=JointVec::Constant(60.0); gi.gains.zeta=0.6;
  gi.gains.torque_limit=(JointVec()<<39,39,39,39,9,9,9).finished();
  interface::GoalId p{}; p[0]=1; interface::GoalId i{}; i[0]=2;
  // Both accepted back-to-back: in_flight_ still false at the impedance goal's pre-check.
  ASSERT_EQ(f.sup.on_trajectory_goal(gp), interface::GoalResponse::kAccept);
  f.sup.on_trajectory_accepted(p, gp);
  ASSERT_EQ(f.sup.on_trajectory_goal(gi), interface::GoalResponse::kAccept);
  f.sup.on_trajectory_accepted(i, gi);
  std::this_thread::sleep_for(std::chrono::milliseconds(600));   // > position duration + margin
  f.sup.stop(); f.teardown();
  int p_code=999, i_code=999, p_n=0, i_n=0;
  for (auto& pr : f.be.all_results()) {
    if (pr.first[0]==1) { p_code = pr.second.error_code; ++p_n; }
    if (pr.first[0]==2) { i_code = pr.second.error_code; ++i_n; }
  }
  EXPECT_EQ(i_n, 1);                                                // impedance goal settled (never orphaned)
  EXPECT_EQ(i_code, interface::result_code::kInvalidGoal);          // ... as a fail-loud rejection
  EXPECT_EQ(p_n, 1);                                                // position goal settled exactly once
  EXPECT_EQ(p_code, interface::result_code::kSuccessful);
}

TEST(Supervisor, CancelSettlesActivePreempted) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::TrajectoryGoal g; g.trajectory=ramp7(0.0,0.1,2.0); g.path_tolerance=JointVec::Constant(-1.0);
  g.control_mode=interface::ControlModeKind::kPosition; g.preemption=interface::Preemption::kLatestWins;
  interface::GoalId a{}; a[0]=1;
  ASSERT_EQ(f.sup.on_trajectory_goal(g), interface::GoalResponse::kAccept);
  f.sup.on_trajectory_accepted(a, g);
  std::this_thread::sleep_for(std::chrono::milliseconds(80));                       // mid-flight
  EXPECT_EQ(f.sup.on_trajectory_cancel(a), interface::CancelResponse::kAccept);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  f.sup.stop(); f.teardown();
  ASSERT_EQ(f.be.result_count(), 1u);                                              // no further results
  EXPECT_EQ(f.be.last_result_id()[0], 1);
  EXPECT_EQ(f.be.last_result().error_code, interface::result_code::kPreempted);
}

TEST(Supervisor, RejectsModeChangeWhileInFlight) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::TrajectoryGoal g1; g1.trajectory=ramp7(0.0,0.05,1.0); g1.path_tolerance=JointVec::Constant(-1.0);
  g1.control_mode=interface::ControlModeKind::kPosition;
  interface::GoalId id1{}; id1[0]=1;
  ASSERT_EQ(f.sup.on_trajectory_goal(g1), interface::GoalResponse::kAccept);
  f.sup.on_trajectory_accepted(id1, g1);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));       // now in flight
  interface::TrajectoryGoal g2 = g1; g2.control_mode=interface::ControlModeKind::kImpedance;
  EXPECT_EQ(f.sup.on_trajectory_goal(g2), interface::GoalResponse::kReject);   // mode change mid-motion
  f.sup.stop(); f.teardown();
}

TEST(Supervisor, SwitchesToImpedanceAtRest) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::TrajectoryGoal g; g.trajectory=ramp7(0.0,0.03,0.3); g.path_tolerance=JointVec::Constant(-1.0);
  g.control_mode=interface::ControlModeKind::kImpedance;
  g.has_gains=true; g.gains.kq=JointVec::Constant(60.0); g.gains.zeta=0.6;
  g.gains.torque_limit=(JointVec()<<39,39,39,39,9,9,9).finished();
  interface::GoalId id{}; id[0]=9;
  ASSERT_EQ(f.sup.on_trajectory_goal(g), interface::GoalResponse::kAccept);
  f.sup.on_trajectory_accepted(id, g);
  std::this_thread::sleep_for(std::chrono::milliseconds(900));       // settle + duration
  f.sup.stop(); f.teardown();
  ASSERT_EQ(f.be.result_count(), 1u);
  EXPECT_EQ(f.be.last_result().error_code, interface::result_code::kSuccessful);
  // Prove the switch actually happened: the executor drove the impedance mode's
  // joint target (set_target bypasses IK), so imp.reference() tracks the ramp goal.
  // If the switch had NOT occurred, imp.reference() would remain 0 and pos.reference()
  // would hold the goal instead.
  EXPECT_NEAR(f.imp.reference()[0], 0.03, 5e-3);
  EXPECT_NEAR(f.pos.reference()[0], 0.0,  5e-3);
}

TEST(Supervisor, DivergenceAbortSettlesPathToleranceViolated) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::TrajectoryGoal g;
  g.trajectory = ramp7(0.0, 0.5, 2.0);               // moves 0.5 rad; SimTransport never moves
  g.path_tolerance = JointVec::Constant(0.2);        // guard ON -> must trip
  interface::GoalId id{}; id[0]=2;
  ASSERT_EQ(f.sup.on_trajectory_goal(g), interface::GoalResponse::kAccept);
  f.sup.on_trajectory_accepted(id, g);
  // |q_desired - q_meas| only crosses the 0.2 rad tolerance at elapsed ~= 0.8s
  // (0.2/0.5 * 2.0s ramp); wait past that with margin for sampler scheduling.
  std::this_thread::sleep_for(std::chrono::milliseconds(1200));
  f.sup.stop(); f.teardown();
  ASSERT_EQ(f.be.result_count(), 1u);
  EXPECT_EQ(f.be.last_result().error_code, interface::result_code::kPathToleranceViolated);
  EXPECT_FALSE(f.sup.on_query_state().fault);         // divergence is not a hardware fault
}
