#include <gtest/gtest.h>
#include "kinova_lowlevel/interface/value_types.h"
#include "kinova_lowlevel/interface/ports.h"
#include "fake_backend.h"
#include "kinova_lowlevel/interface/supervisor.h"
#include "kinova_lowlevel/interface/arbiter.h"
#include "kinova_lowlevel/joint_position_mode.h"
#include "kinova_lowlevel/joint_impedance_mode.h"
#include "kinova_lowlevel/joint_torque_mode.h"
#include "kinova_lowlevel/joint_velocity_mode.h"
#include "kinova_lowlevel/rt_executor.h"
#include "kinova_lowlevel/sim_transport.h"
#include "kinova_lowlevel/feedback_tap.h"
#include "kinova_lowlevel/dynamics.h"
#include <algorithm>
#include <cmath>
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

TEST(ValueTypes, ArbitrationDefaultsAndResultCodes) {
  EXPECT_EQ(interface::result_code::kNotAuthorized, -8);
  EXPECT_EQ(interface::result_code::kHalted,        -9);
  interface::TrajectoryGoal g;
  EXPECT_EQ(g.token, (interface::Token{}));          // zero-initialised, not garbage
  interface::CancelRequest c;
  EXPECT_EQ(c.token, (interface::Token{}));
  interface::GrantResult gr;
  EXPECT_FALSE(gr.accepted);
  EXPECT_EQ(gr.generation, 0u);
  interface::ArbitrationStatus st;
  EXPECT_FALSE(st.estopped);
  EXPECT_FALSE(st.owned);
  EXPECT_EQ(st.rejected_count, 0u);
  EXPECT_EQ(st.mode, interface::ArbitrationMode::kEnforced);
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
  JointTorqueMode tau{dyn};
  JointVelocityMode vel{dyn};
  RtExecutor exec{tap, ring, {1000.0, kinova::Pacing::kSleepSpin, {}}};
  FakeBackend be;
  interface::Supervisor sup{pos, imp, tau, vel, exec, snap, pump_dyn, be, be};
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
  interface::CancelRequest cr; cr.id = a;
  EXPECT_EQ(f.sup.on_trajectory_cancel(cr), interface::CancelResponse::kAccept);
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

// Regression (real-arm bug 2026-08-12): a transient failed feedback-snapshot read
// (Seqlock::load == false, e.g. the RT writer preempted mid-store) must NOT inject
// q=0 into the divergence guard — that caused a false PATH_TOLERANCE_VIOLATED abort
// mid-motion on the arm. The sampler must reuse the last-good q instead. Mirrors the
// proven last-good-q pattern in apps/trajectory_run.cpp.
TEST(SupervisorSampler, ReusesLastGoodQOnFailedSnapshotRead) {
  using kinova::interface::sampled_q;
  const kinova::JointVec last  = kinova::JointVec::Constant(1.64);
  const kinova::JointVec fresh = kinova::JointVec::Constant(1.65);
  // Successful read -> use the fresh sample.
  EXPECT_NEAR(sampled_q(true, fresh, last)[6], 1.65, 1e-12);
  // Failed read -> REUSE last-good, NOT zero (the bug injected Zero() here).
  const kinova::JointVec r = sampled_q(false, kinova::JointVec::Zero(), last);
  EXPECT_NEAR(r[0], 1.64, 1e-12);
  EXPECT_NEAR(r[6], 1.64, 1e-12);
}

// ---------------------------------------------------------------------------
// Halt path: settle everything ACCEPTed, then hold where the arm actually is.
// ---------------------------------------------------------------------------

TEST(Supervisor, HaltSettlesTheActiveGoalAsHalted) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::TrajectoryGoal g; g.trajectory = ramp7(0.0, 0.1, 2.0);
  g.control_mode = interface::ControlModeKind::kPosition;
  g.preemption   = interface::Preemption::kLatestWins;
  g.path_tolerance = JointVec::Constant(-1.0);
  interface::GoalId id{}; id[0] = 3;
  ASSERT_EQ(f.sup.on_trajectory_goal(g), interface::GoalResponse::kAccept);
  f.sup.on_trajectory_accepted(id, g);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));   // mid-motion
  f.sup.on_halt(interface::HaltReason::kOwnershipRevoked);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  f.sup.stop(); f.teardown();
  ASSERT_EQ(f.be.result_count(), 1u);
  EXPECT_EQ(f.be.last_result().error_code, interface::result_code::kHalted);
  EXPECT_EQ(f.be.last_result_id()[0], 3);
}

TEST(Supervisor, HaltSettlesTheQueuedGoalToo) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::TrajectoryGoal g; g.trajectory = ramp7(0.0, 0.1, 2.0);
  g.control_mode = interface::ControlModeKind::kPosition;
  g.path_tolerance = JointVec::Constant(-1.0);
  g.preemption = interface::Preemption::kLatestWins;
  interface::GoalId a{}; a[0] = 1;
  f.sup.on_trajectory_goal(g); f.sup.on_trajectory_accepted(a, g);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  interface::TrajectoryGoal q = g; q.preemption = interface::Preemption::kQueue;
  interface::GoalId b{}; b[0] = 2;
  f.sup.on_trajectory_goal(q); f.sup.on_trajectory_accepted(b, q);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  f.sup.on_halt(interface::HaltReason::kEmergencyStop);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  f.sup.stop(); f.teardown();
  // Both were ACCEPTed, so both must settle -- dropping the queued one silently
  // orphans a client that waits forever (the a835bf5 / d0791df bug class).
  ASSERT_EQ(f.be.result_count(), 2u);
  for (const auto& kv : f.be.all_results())
    EXPECT_EQ(kv.second.error_code, interface::result_code::kHalted);
}

TEST(Supervisor, HaltLatchesTheTargetAtMeasuredQ) {
  SupFix f(0.0); f.sup.start(); f.run_rt();
  interface::TrajectoryGoal g; g.trajectory = ramp7(0.0, 0.5, 4.0);   // slow ramp away from 0
  g.control_mode = interface::ControlModeKind::kPosition;
  g.preemption   = interface::Preemption::kLatestWins;
  g.path_tolerance = JointVec::Constant(-1.0);
  interface::GoalId id{}; id[0] = 5;
  f.sup.on_trajectory_goal(g); f.sup.on_trajectory_accepted(id, g);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  f.sup.on_halt(interface::HaltReason::kEmergencyStop);
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  f.sup.stop(); f.teardown();
  // SimTransport is a static echo: measured q never leaves the seed, so a hold at
  // MEASURED q must snap the command back to it -- not park at the last reference.
  EXPECT_NEAR(f.sim.last_command().position[0], 0.0, 1e-6);
}

// ---------------------------------------------------------------------------
// A real Arbiter in front of a real Supervisor.
// ---------------------------------------------------------------------------

TEST(ArbitrationIntegration, RevokeMidMotionHaltsAndSettlesExactlyOnce) {
  SupFix f;
  interface::Arbiter arb{f.sup, f.sup, interface::ArbitrationMode::kEnforced, 99};
  f.sup.start(); f.run_rt();
  const interface::Token t = arb.grant("planner").token;

  interface::TrajectoryGoal g; g.trajectory = ramp7(0.0, 0.4, 3.0);
  g.control_mode = interface::ControlModeKind::kPosition;
  g.preemption   = interface::Preemption::kLatestWins;
  g.path_tolerance = JointVec::Constant(-1.0);
  g.token = t;
  interface::GoalId id{}; id[0] = 11;
  ASSERT_EQ(arb.on_trajectory_goal(g), interface::GoalResponse::kAccept);
  arb.on_trajectory_accepted(id, g);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  arb.revoke();                                  // ownership pulled mid-motion
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // A command from the ex-owner after the revoke must not restart the arm.
  EXPECT_EQ(arb.on_trajectory_goal(g), interface::GoalResponse::kRejectUnauthorized);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  f.sup.stop(); f.teardown();

  ASSERT_EQ(f.be.result_count(), 1u);            // exactly once, not zero and not twice
  EXPECT_EQ(f.be.last_result().error_code, interface::result_code::kHalted);
  EXPECT_NEAR(f.sim.last_command().position[0], 0.0, 1e-6);   // held at measured q
}

TEST(ArbitrationIntegration, ANewOwnerCanSwitchControlModeAfterAHalt) {
  SupFix f;
  interface::Arbiter arb{f.sup, f.sup, interface::ArbitrationMode::kEnforced, 99};
  f.sup.start(); f.run_rt();

  const interface::Token a = arb.grant("planner").token;
  interface::TrajectoryGoal gp; gp.trajectory = ramp7(0.0, 0.4, 3.0);
  gp.control_mode = interface::ControlModeKind::kPosition;
  gp.preemption = interface::Preemption::kLatestWins;
  gp.path_tolerance = JointVec::Constant(-1.0); gp.token = a;
  interface::GoalId ida{}; ida[0] = 21;
  arb.on_trajectory_goal(gp); arb.on_trajectory_accepted(ida, gp);
  std::this_thread::sleep_for(std::chrono::milliseconds(250));

  const interface::Token b = arb.grant("teleop").token;   // re-grant halts, then grants
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // After a halt the arm is at rest, so mode-switch-at-rest is satisfied and an
  // impedance goal from the NEW owner is accepted.
  interface::TrajectoryGoal gi; gi.trajectory = ramp7(0.0, 0.05, 1.0);
  gi.control_mode = interface::ControlModeKind::kImpedance;
  gi.preemption = interface::Preemption::kLatestWins;
  gi.path_tolerance = JointVec::Constant(-1.0); gi.token = b;
  interface::GoalId idb{}; idb[0] = 22;
  ASSERT_EQ(arb.on_trajectory_goal(gi), interface::GoalResponse::kAccept);
  arb.on_trajectory_accepted(idb, gi);
  std::this_thread::sleep_for(std::chrono::milliseconds(1800));
  f.sup.stop(); f.teardown();

  const auto results = f.be.all_results();
  ASSERT_EQ(results.size(), 2u);
  EXPECT_EQ(results[0].second.error_code, interface::result_code::kHalted);      // halted position goal
  EXPECT_EQ(results[1].second.error_code, interface::result_code::kSuccessful);  // the new owner's goal
}

// Regression (fix wave, finding 1): estop() delivers its halt BEFORE it takes the
// Arbiter's m_, so a delegate that had ALREADY passed admit() is still running
// downstream -- and Supervisor::on_trajectory_accepted pushes into inbox_ *after*
// on_halt flushed it. The sampler consumes halt_pending_ in step 0, then drains that
// late arrival in step 1 and drives it: motion after an e-stop. The window is
// nanoseconds wide in production, so it is widened deterministically here by holding
// the delegate in flight -- which is also the realistic case, since a backend
// delegate may be preempted for arbitrarily long between admit and push.
namespace {
struct SlowAcceptSink : interface::CommandSink {
  interface::CommandSink& down;
  std::chrono::milliseconds hold;
  std::atomic<bool> entered{false};
  SlowAcceptSink(interface::CommandSink& d, std::chrono::milliseconds h) : down(d), hold(h) {}
  interface::GoalResponse on_trajectory_goal(const interface::TrajectoryGoal& g) override {
    return down.on_trajectory_goal(g); }
  void on_trajectory_accepted(const interface::GoalId& id, const interface::TrajectoryGoal& g) override {
    entered.store(true);
    std::this_thread::sleep_for(hold);            // admitted; the push has not happened yet
    down.on_trajectory_accepted(id, g);
  }
  interface::CancelResponse on_trajectory_cancel(const interface::CancelRequest& c) override {
    return down.on_trajectory_cancel(c); }
  interface::GainsResult on_set_gains(const interface::GainsRequest& r) override { return down.on_set_gains(r); }
  interface::ArmState    on_query_state() override { return down.on_query_state(); }
  void                   on_halt(interface::HaltReason r) override { down.on_halt(r); }
};
}  // namespace

TEST(ArbitrationIntegration, AGoalAdmittedJustBeforeAnEstopNeverReachesTheArm) {
  SupFix f;
  SlowAcceptSink slow{f.sup, std::chrono::milliseconds(150)};
  interface::Arbiter arb{slow, f.sup, interface::ArbitrationMode::kDisabled, 7};
  f.sup.start(); f.run_rt();

  interface::TrajectoryGoal g; g.trajectory = ramp7(0.0, 0.3, 0.5);
  g.control_mode = interface::ControlModeKind::kPosition;
  g.preemption = interface::Preemption::kLatestWins;
  g.path_tolerance = JointVec::Constant(-1.0);
  interface::GoalId id{}; id[0] = 51;
  ASSERT_EQ(arb.on_trajectory_goal(g), interface::GoalResponse::kAccept);
  std::thread t([&]{ arb.on_trajectory_accepted(id, g); });      // admits, then holds in flight
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!slow.entered.load() && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  if (!slow.entered.load()) {                                    // never vacuously green
    t.join(); f.sup.stop(); f.teardown();
    FAIL() << "the delegate never started; the race window was missed";
  }
  arb.estop();                    // latches + halts, then waits on m_ for the delegate
  t.join();
  std::this_thread::sleep_for(std::chrono::milliseconds(800));   // > the 0.5 s ramp
  f.sup.stop(); f.teardown();
  // The ramp asked for 0.3 rad from 0.0, so anything that ran at all is visible in the
  // command. The tolerance covers the documented residual: the sampler could still
  // drain in the microseconds between the delegate releasing m_ and the second
  // on_halt, which costs at most one sampler period of ramp (~2 mrad).
  EXPECT_LT(std::abs(f.sim.last_command().position[0]), 0.01)
      << "a goal admitted before the e-stop was executed after it";
}

TEST(ValueTypes, StreamingDefaultsAndResultCodes) {
  EXPECT_EQ(interface::result_code::kStreamRejected, -10);
  interface::StreamOpenRequest r;
  EXPECT_EQ(r.kind, interface::SetpointKind::kJointPosition);
  EXPECT_EQ(r.control_mode, interface::ControlModeKind::kPosition);
  EXPECT_NEAR(r.timeout_s, 0.1, 1e-12);          // a deadline is mandatory, so it has a default
  EXPECT_EQ(r.token, (interface::Token{}));
  interface::StreamOpenResult res;
  EXPECT_FALSE(res.accepted);
  interface::JointSetpoint js;
  EXPECT_TRUE(js.values.isZero());
  interface::TwistSetpoint ts;
  EXPECT_TRUE(ts.twist.isZero());                // Vector6, [linear; angular]
  EXPECT_EQ(ts.token, (interface::Token{}));
}

// ---------------------------------------------------------------------------
// Streaming tier (Task 6): setpoints reach the mode, and a session is exclusive
// with trajectory goals in BOTH directions.
// ---------------------------------------------------------------------------

TEST(Supervisor, StreamingJointPositionDrivesTheMode) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::StreamOpenRequest r;
  r.kind = interface::SetpointKind::kJointPosition;
  r.control_mode = interface::ControlModeKind::kPosition;
  r.timeout_s = 1.0;
  ASSERT_TRUE(f.sup.on_stream_open(r).accepted);
  interface::JointSetpoint sp; sp.values = JointVec::Constant(0.05);
  for (int i = 0; i < 20; ++i) {
    f.sup.on_setpoint_joint_position(sp);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  interface::StreamCloseRequest c;
  f.sup.on_stream_close(c);
  // Deliberately SHORT: closing the session latches hold-at-measured-q, and
  // SimTransport is a static echo, so the commanded reference immediately starts
  // walking back toward the entry configuration at max_ref_speed (0.5 rad/s ->
  // 0.05 rad in 100 ms). Sample before it arrives; the hold itself is asserted by
  // ClosingAStreamLatchesHoldAtMeasuredQ below.
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  f.sup.stop(); f.teardown();
  EXPECT_GT(f.sim.last_command().position[0], 1e-3);   // the setpoint actually reached the arm
}

TEST(Supervisor, ClosingAStreamLatchesHoldAtMeasuredQ) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::StreamOpenRequest r;
  r.kind = interface::SetpointKind::kJointPosition;
  r.control_mode = interface::ControlModeKind::kPosition;
  r.timeout_s = 1.0;
  ASSERT_TRUE(f.sup.on_stream_open(r).accepted);
  interface::JointSetpoint sp; sp.values = JointVec::Constant(0.05);
  for (int i = 0; i < 20; ++i) {
    f.sup.on_setpoint_joint_position(sp);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  interface::StreamCloseRequest c;
  f.sup.on_stream_close(c);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));   // > 0.05 rad / 0.5 rad/s
  f.sup.stop(); f.teardown();
  // The teardown commands the hold EXPLICITLY. Without it, position mode keeps the
  // last streamed target once its watchdog is disarmed and the command stays at
  // 0.05 rad -- the asymmetry against impedance that Task 4 surfaced.
  EXPECT_NEAR(f.sim.last_command().position[0], 0.0, 1e-3);
}

TEST(Supervisor, StreamOpenRefusesABadRequestBeforeSwitchingModes) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::StreamOpenRequest no_deadline; no_deadline.timeout_s = 0.0;
  auto res = f.sup.on_stream_open(no_deadline);
  EXPECT_FALSE(res.accepted);
  EXPECT_EQ(res.error_code, interface::result_code::kStreamRejected);
  interface::StreamOpenRequest negative; negative.timeout_s = -1.0;
  EXPECT_FALSE(f.sup.on_stream_open(negative).accepted);
  interface::StreamOpenRequest bad_pair;                       // velocity needs Plan 2
  bad_pair.kind = interface::SetpointKind::kJointVelocity;
  bad_pair.control_mode = interface::ControlModeKind::kImpedance;
  bad_pair.timeout_s = 1.0;
  EXPECT_FALSE(f.sup.on_stream_open(bad_pair).accepted);
  // A refused open leaves the arm exactly where it was: no mode switch happened.
  interface::JointSetpoint sp; sp.values = JointVec::Constant(0.2);
  f.sup.on_setpoint_joint_position(sp);                        // no session -> dropped
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  f.sup.stop(); f.teardown();
  EXPECT_NEAR(f.sim.last_command().position[0], 0.0, 1e-6);
}

TEST(Supervisor, AGoalIsRefusedWhileAStreamIsOpen) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::StreamOpenRequest r; r.timeout_s = 5.0;
  ASSERT_TRUE(f.sup.on_stream_open(r).accepted);
  interface::TrajectoryGoal g;
  g.trajectory = ramp7(0.0, 0.05, 0.4);
  g.control_mode = interface::ControlModeKind::kPosition;
  g.path_tolerance = JointVec::Constant(-1.0);
  EXPECT_EQ(f.sup.on_trajectory_goal(g), interface::GoalResponse::kReject);
  f.sup.stop(); f.teardown();
}

TEST(Supervisor, AStreamIsRefusedWhileAGoalIsInFlight) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::TrajectoryGoal g;
  g.trajectory = ramp7(0.0, 0.1, 2.0);
  g.control_mode = interface::ControlModeKind::kPosition;
  g.preemption = interface::Preemption::kLatestWins;
  g.path_tolerance = JointVec::Constant(-1.0);
  interface::GoalId id{}; id[0] = 1;
  f.sup.on_trajectory_goal(g); f.sup.on_trajectory_accepted(id, g);
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  interface::StreamOpenRequest r; r.timeout_s = 1.0;
  EXPECT_FALSE(f.sup.on_stream_open(r).accepted);
  f.sup.stop(); f.teardown();
}

// Regression (whole-branch review, finding 1): the accept-time exclusivity checks
// are BOTH necessary and BOTH insufficient. in_flight_ is set by the SAMPLER when it
// drains the inbox, not by on_trajectory_accepted, so for up to one sampler period
// (4 ms at 250 Hz) an accepted goal sits in inbox_ with in_flight_ still false and
// on_stream_open is admitted. Left unguarded the sampler then rebinds traj_ and ticks
// the trajectory into the very sink the backend thread is streaming into -- two
// writers on one double buffer, which is the whole safety argument for writing
// setpoints directly from the backend thread. The sampler must refuse the goal.
TEST(Supervisor, AGoalAcceptedJustBeforeAStreamOpensIsSettledInvalid) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::TrajectoryGoal g;
  g.trajectory = ramp7(0.0, 0.3, 0.5);
  g.control_mode = interface::ControlModeKind::kPosition;
  g.preemption = interface::Preemption::kLatestWins;
  g.path_tolerance = JointVec::Constant(-1.0);
  interface::GoalId id{}; id[0] = 41;
  ASSERT_EQ(f.sup.on_trajectory_goal(g), interface::GoalResponse::kAccept);
  f.sup.on_trajectory_accepted(id, g);       // inbox_ only -- in_flight_ still false
  // NO sleep: opening inside the drain window is the whole point of the test.
  interface::StreamOpenRequest r;
  r.kind = interface::SetpointKind::kJointPosition;
  r.control_mode = interface::ControlModeKind::kPosition;   // same mode: no settle sleep
  r.timeout_s = 5.0;                          // long enough not to expire during the test
  ASSERT_TRUE(f.sup.on_stream_open(r).accepted)
      << "the sampler drained before the open; the race window was missed";
  std::this_thread::sleep_for(std::chrono::milliseconds(800));   // > the 0.5 s ramp
  f.sup.stop(); f.teardown();
  ASSERT_EQ(f.be.result_count(), 1u);
  EXPECT_EQ(f.be.last_result().error_code, interface::result_code::kInvalidGoal);
  // And it really was refused rather than run: the ramp asked for 0.3 rad and no
  // setpoint was ever streamed, so the command never left the entry configuration.
  EXPECT_NEAR(f.sim.last_command().position[0], 0.0, 1e-6);
}

TEST(Supervisor, StreamTimeoutClosesTheSession) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::StreamOpenRequest r; r.timeout_s = 0.1;
  ASSERT_TRUE(f.sup.on_stream_open(r).accepted);
  interface::JointSetpoint sp; sp.values = JointVec::Constant(0.05);
  f.sup.on_setpoint_joint_position(sp);
  std::this_thread::sleep_for(std::chrono::milliseconds(400));   // let the deadline lapse
  // The session is gone, so a goal is admissible again -- that is the observable
  // consequence of the lifecycle teardown.
  interface::TrajectoryGoal g;
  g.trajectory = ramp7(0.0, 0.05, 0.4);
  g.control_mode = interface::ControlModeKind::kPosition;
  g.path_tolerance = JointVec::Constant(-1.0);
  EXPECT_EQ(f.sup.on_trajectory_goal(g), interface::GoalResponse::kAccept);
  f.sup.stop(); f.teardown();
}

TEST(Supervisor, HaltClosesAnOpenStream) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::StreamOpenRequest r; r.timeout_s = 5.0;
  ASSERT_TRUE(f.sup.on_stream_open(r).accepted);
  f.sup.on_halt(interface::HaltReason::kEmergencyStop);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  interface::JointSetpoint sp; sp.values = JointVec::Constant(0.2);
  f.sup.on_setpoint_joint_position(sp);            // must not restart a halted arm
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  f.sup.stop(); f.teardown();
  EXPECT_NEAR(f.sim.last_command().position[0], 0.0, 1e-6);
}

// Ruling B: a backend that calls on_trajectory_accepted WITHOUT a preceding
// accepted goal must not have a torque/velocity goal silently driven as position.
TEST(Supervisor, AnAcceptedTorqueGoalIsSettledInvalidNotDrivenAsPosition) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::TrajectoryGoal g;
  g.trajectory = ramp7(0.0, 0.3, 0.4);
  g.control_mode = interface::ControlModeKind::kTorque;
  g.path_tolerance = JointVec::Constant(-1.0);
  interface::GoalId id{}; id[0] = 9;
  EXPECT_EQ(f.sup.on_trajectory_goal(g), interface::GoalResponse::kReject);
  f.sup.on_trajectory_accepted(id, g);             // backend ignored the pre-check
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  f.sup.stop(); f.teardown();
  ASSERT_EQ(f.be.result_count(), 1u);
  EXPECT_EQ(f.be.last_result().error_code, interface::result_code::kInvalidGoal);
  EXPECT_NEAR(f.sim.last_command().position[0], 0.0, 1e-6);   // never executed
}

// Regression: a streaming session moves active_mode_kind_ from the BACKEND thread,
// but only the sampler may rebuild traj_. When the rebind condition compared the
// goal against active_mode_kind_, this sequence left traj_ bound to pos_ while the
// executor ran imp_: the goal settled SUCCESSFUL while the arm never moved. The
// result code alone cannot catch that, so this asserts the running mode's reference.
TEST(Supervisor, AnImpedanceGoalAfterAnImpedanceStreamDrivesTheRunningMode) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::StreamOpenRequest r;
  r.kind = interface::SetpointKind::kJointPosition;
  r.control_mode = interface::ControlModeKind::kImpedance;   // switches the executor to imp_
  r.timeout_s = 1.0;
  ASSERT_TRUE(f.sup.on_stream_open(r).accepted);
  interface::JointSetpoint sp; sp.values = JointVec::Zero();
  f.sup.on_setpoint_joint_position(sp);
  interface::StreamCloseRequest c;
  f.sup.on_stream_close(c);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  interface::TrajectoryGoal g;
  g.trajectory = ramp7(0.0, 0.2, 0.5);
  g.control_mode = interface::ControlModeKind::kImpedance;
  g.preemption = interface::Preemption::kLatestWins;
  g.path_tolerance = JointVec::Constant(-1.0);
  interface::GoalId id{}; id[0] = 31;
  ASSERT_EQ(f.sup.on_trajectory_goal(g), interface::GoalResponse::kAccept);
  f.sup.on_trajectory_accepted(id, g);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));   // settle + duration
  f.sup.stop(); f.teardown();

  ASSERT_EQ(f.be.result_count(), 1u);
  EXPECT_EQ(f.be.last_result().error_code, interface::result_code::kSuccessful);
  // The discriminating assertion: the trajectory reached the mode the EXECUTOR is
  // running. Bound to the wrong sink, imp_ would never leave its entry reference.
  EXPECT_GT(f.imp.reference()[0], 1e-2);
}

// Mirror of the case above, and the one the first fix left open. After a
// kImpedance stream closes, active_mode_kind_ is kImpedance while traj_bound_kind_
// is still kPosition. A kPosition goal MATCHES traj_bound_kind_, so a rebind
// condition that tests only that skips the switch: exec_ keeps running imp_,
// traj_ drives pos_, and the goal settles SUCCESSFUL with the arm motionless.
// Asserting pos_'s reference is what discriminates -- q_ref_ advances only inside
// compute(), which is never called for a mode the executor is not running.
TEST(Supervisor, APositionGoalAfterAnImpedanceStreamDrivesTheRunningMode) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::StreamOpenRequest r;
  r.kind = interface::SetpointKind::kJointPosition;
  r.control_mode = interface::ControlModeKind::kImpedance;   // switches the executor to imp_
  r.timeout_s = 1.0;
  ASSERT_TRUE(f.sup.on_stream_open(r).accepted);
  interface::JointSetpoint sp; sp.values = JointVec::Zero();
  f.sup.on_setpoint_joint_position(sp);
  interface::StreamCloseRequest c;
  f.sup.on_stream_close(c);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  interface::TrajectoryGoal g;
  g.trajectory = ramp7(0.0, 0.2, 0.5);
  g.control_mode = interface::ControlModeKind::kPosition;    // back to the mode traj_ is bound to
  g.preemption = interface::Preemption::kLatestWins;
  g.path_tolerance = JointVec::Constant(-1.0);
  interface::GoalId id{}; id[0] = 32;
  ASSERT_EQ(f.sup.on_trajectory_goal(g), interface::GoalResponse::kAccept);
  f.sup.on_trajectory_accepted(id, g);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));   // settle + duration
  f.sup.stop(); f.teardown();

  ASSERT_EQ(f.be.result_count(), 1u);
  EXPECT_EQ(f.be.last_result().error_code, interface::result_code::kSuccessful);
  // The discriminating assertion: the executor was switched BACK to pos_, so
  // pos_ integrated the streamed reference. Left running imp_, this stays 0.
  EXPECT_GT(f.pos.reference()[0], 1e-2);
}

// End-to-end kTorque stream: open, push setpoints, close. SimTransport is a static
// echo so the arm never moves; the observable is the COMMANDED torque, which for
// this mode is clamp(gravity(q) + tau_ff). The RT thread is stopped before
// last_command() is read so the read cannot race the 1 kHz writer, and the close
// then runs the torque branch of close_stream()'s default-restore.
TEST(Supervisor, StreamingJointTorqueDrivesTheTorqueMode) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::StreamOpenRequest r;
  r.kind = interface::SetpointKind::kJointTorque;
  r.control_mode = interface::ControlModeKind::kTorque;
  r.timeout_s = 1.0;
  ASSERT_TRUE(f.sup.on_stream_open(r).accepted);
  interface::JointSetpoint sp; sp.values = JointVec::Constant(1.0);
  for (int i = 0; i < 10; ++i) {
    f.sup.on_setpoint_joint_torque(sp);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  f.teardown();                                    // freeze last_cmd_ before reading it
  const JointCommand cmd = f.sim.last_command();
  interface::StreamCloseRequest c;
  f.sup.on_stream_close(c);                        // torque branch of the default-restore
  f.sup.stop();

  EXPECT_EQ(cmd.mode, ActuatorMode::kTorque);
  JointVec g0; f.dyn.gravity(JointVec::Zero(), g0);   // measured q never leaves 0
  const JointVec lim = JointTorqueParams{}.torque_limit;
  for (int i = 0; i < kNumJoints; ++i) {
    const double want = std::max(-lim[i], std::min(lim[i], g0[i] + 1.0));
    EXPECT_NEAR(cmd.torque[i], want, 1e-9) << "joint " << i;
  }
  // A goal is admissible again, so the session really is closed.
  interface::TrajectoryGoal g;
  g.trajectory = ramp7(0.0, 0.05, 0.4);
  g.control_mode = interface::ControlModeKind::kPosition;
  g.path_tolerance = JointVec::Constant(-1.0);
  EXPECT_EQ(f.sup.on_trajectory_goal(g), interface::GoalResponse::kAccept);
}

// The write-handoff invariant -- exactly one thread writes targets at any instant --
// cannot be proven behaviourally. It has to be raced: a writer thread hammers
// on_setpoint_joint_position() concurrently with the close, and we check whether any
// post-close setpoint reached the mode. During the session it streams a setpoint equal
// to where the arm already is (zero), so nothing moves either way; only after the close
// does it switch to a clearly different value. A test streaming one constant throughout
// could not distinguish "the handoff held" from "the handoff leaked."
TEST(StreamingHandoff, NoSetpointReachesTheModeAfterClose) {
  for (int round = 0; round < 50; ++round) {
    SupFix f; f.sup.start(); f.run_rt();
    interface::StreamOpenRequest r;
    r.kind = interface::SetpointKind::kJointPosition;
    r.control_mode = interface::ControlModeKind::kPosition;
    r.timeout_s = 5.0;                       // long: this test is about close, not expiry
    ASSERT_TRUE(f.sup.on_stream_open(r).accepted);

    std::atomic<bool> stop_writer{false};
    std::atomic<bool> closed{false};

    // Before the close: stream where the arm already is, so nothing moves.
    // After the close: stream somewhere obviously different. Any leak shows up
    // as commanded motion.
    std::thread writer([&] {
      while (!stop_writer.load(std::memory_order_acquire)) {
        interface::JointSetpoint sp;
        sp.values = closed.load(std::memory_order_acquire) ? JointVec::Constant(0.4)
                                                           : JointVec::Zero();
        f.sup.on_setpoint_joint_position(sp);
        std::this_thread::yield();
      }
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    interface::StreamCloseRequest c;
    f.sup.on_stream_close(c);
    closed.store(true, std::memory_order_release);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));   // let the writer hammer
    stop_writer.store(true, std::memory_order_release);
    writer.join();

    EXPECT_FALSE(f.sup.stream_is_open());
    f.sup.stop(); f.teardown();
    // The post-close setpoints asked for 0.4 rad. If the handoff held, none of
    // them reached the mode and the command never left the entry configuration.
    EXPECT_NEAR(f.sim.last_command().position[0], 0.0, 1e-6)
        << "a setpoint landed after the session was closed (round " << round << ")";
  }
}

TEST(StreamingHandoff, NoSetpointReachesTheModeAfterHalt) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::StreamOpenRequest r;
  r.kind = interface::SetpointKind::kJointPosition;
  r.control_mode = interface::ControlModeKind::kPosition;
  r.timeout_s = 5.0;
  ASSERT_TRUE(f.sup.on_stream_open(r).accepted);

  std::atomic<bool> stop_writer{false}, halted{false};
  std::thread writer([&] {
    while (!stop_writer.load(std::memory_order_acquire)) {
      interface::JointSetpoint sp;
      sp.values = halted.load(std::memory_order_acquire) ? JointVec::Constant(0.4)
                                                         : JointVec::Zero();
      f.sup.on_setpoint_joint_position(sp);
      std::this_thread::yield();
    }
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  f.sup.on_halt(interface::HaltReason::kEmergencyStop);
  halted.store(true, std::memory_order_release);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  stop_writer.store(true, std::memory_order_release);
  writer.join();
  f.sup.stop(); f.teardown();
  // An e-stopped arm must not be restartable by a client that has not noticed.
  EXPECT_NEAR(f.sim.last_command().position[0], 0.0, 1e-6);
}

TEST(Supervisor, StreamingJointVelocityDrivesTheVelocityMode) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::StreamOpenRequest r;
  r.kind = interface::SetpointKind::kJointVelocity;
  r.control_mode = interface::ControlModeKind::kVelocity;
  r.timeout_s = 1.0;
  ASSERT_TRUE(f.sup.on_stream_open(r).accepted);

  interface::JointSetpoint sp; sp.values = JointVec::Constant(0.05);
  for (int i = 0; i < 20; ++i) {
    f.sup.on_setpoint_joint_velocity(sp);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  // Freeze qd_cmd_ before reading it -- mirrors StreamingJointTorqueDrivesTheTorque-
  // Mode's "freeze last_cmd_ before reading it". close_stream()'s explicit
  // set_velocity_target(Zero()) is velocity mode's safe-stop (Plan 2 decision):
  // called before the RT thread is joined, it would land within a cycle or two
  // and zero exactly the value this test exists to observe.
  f.teardown();
  const JointVec cmd = f.vel.commanded();
  interface::StreamCloseRequest c;
  f.sup.on_stream_close(c);
  f.sup.stop();

  EXPECT_GT(cmd.cwiseAbs().maxCoeff(), 0.0);
}

TEST(Supervisor, StreamingATwistDrivesTheVelocityMode) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::StreamOpenRequest r;
  r.kind = interface::SetpointKind::kEeTwist;
  r.control_mode = interface::ControlModeKind::kVelocity;
  r.timeout_s = 1.0;
  ASSERT_TRUE(f.sup.on_stream_open(r).accepted);

  interface::TwistSetpoint sp; sp.twist = Vector6::Zero(); sp.twist[0] = 0.02;
  for (int i = 0; i < 20; ++i) {
    f.sup.on_setpoint_twist(sp);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  interface::StreamCloseRequest c;
  f.sup.on_stream_close(c);
  f.sup.stop(); f.teardown();

  EXPECT_GT(f.vel.last_manipulability(), 0.0);   // the DLS solve actually ran
}

TEST(Supervisor, StreamingAPoseIntoPositionModeIsAccepted) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::StreamOpenRequest r;
  r.kind = interface::SetpointKind::kEePose;
  r.control_mode = interface::ControlModeKind::kPosition;
  r.timeout_s = 1.0;
  ASSERT_TRUE(f.sup.on_stream_open(r).accepted);

  // The FK of a real configuration, so the target is reachable BY CONSTRUCTION.
  // The fixture starts at q = 0, which is the arm straight up and fully extended:
  // fk(0) + 2 cm along x is a quarter of a millimetre OUTSIDE the workspace, and
  // the pose path correctly refuses to pretend it is tracking that.
  interface::PoseSetpoint sp; sp.pose = f.pump_dyn.fk(JointVec::Constant(0.1));
  for (int i = 0; i < 20; ++i) {
    f.sup.on_setpoint_pose(sp);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  // The session must still be up: nothing here should have tripped the IK-fault
  // teardown, and a client streaming a reachable pose is entitled to keep it.
  EXPECT_TRUE(f.sup.stream_is_open());
  EXPECT_FALSE(f.pos.ik_faulted());

  // Freeze last_ik_ before reading it -- mirrors StreamingJointTorqueDrivesThe-
  // TorqueMode's "freeze last_cmd_ before reading it". close_stream()'s
  // pre-existing hold-at-measured-q latch calls pos_.set_target(JointVec), which
  // switches the target source away from kPose; JointPositionMode::compute()
  // explicitly resets last_ik_ on that branch ("last_ik() means THIS cycle's
  // solve"), so reading it after a close would see the reset, not the IK that
  // actually ran while streaming.
  f.teardown();
  const IkResult ik = f.pos.last_ik();
  const JointVec ref = f.pos.reference();
  interface::StreamCloseRequest c;
  f.sup.on_stream_close(c);
  f.sup.stop();

  // converged, not iters > 0: a default-constructed IkResult has converged=false,
  // so this still proves a solve RAN, and it additionally proves the solve reached
  // the pose. iters is no longer the right probe -- once the persistent seed has
  // settled, DiffIkSolver returns on iteration 0 by design.
  EXPECT_TRUE(ik.converged);
  // Only pose setpoints were ever sent, so a reference off the entry configuration
  // can only have come from the IK.
  EXPECT_GT(ref.norm(), 0.05);
}

TEST(Supervisor, RefusesAJointPositionSetpointInVelocityMode) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::StreamOpenRequest r;
  r.kind = interface::SetpointKind::kJointPosition;
  r.control_mode = interface::ControlModeKind::kVelocity;
  r.timeout_s = 1.0;
  EXPECT_FALSE(f.sup.on_stream_open(r).accepted);
  f.sup.stop(); f.teardown();
}

// Spec Component 3: "The sampler observes it on its next tick and tears the
// session down with a distinct reason." Without this the mode freezes at measured
// q, the session stays OPEN, the client's own setpoints keep refreshing the
// deadline, and the client streams poses believing it is tracking.
TEST(Supervisor, ASustainedIkFaultTearsDownThePoseSession) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::StreamOpenRequest r;
  r.kind = interface::SetpointKind::kEePose;
  r.control_mode = interface::ControlModeKind::kPosition;
  r.timeout_s = 1.0;                      // far longer than this test runs
  ASSERT_TRUE(f.sup.on_stream_open(r).accepted);

  interface::PoseSetpoint sp; sp.pose = f.pump_dyn.fk(f.init.q);
  sp.pose.p.x() += 5.0;                   // metres away: never reachable
  for (int i = 0; i < 60 && f.sup.stream_is_open(); ++i) {
    f.sup.on_setpoint_pose(sp);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_FALSE(f.sup.stream_is_open());
  // A DISTINCT cause: the deadline never lapsed here, and a client must be able to
  // tell "you went quiet" from "the pose you asked for is not solvable".
  EXPECT_EQ(f.sup.stream_close_cause(), interface::StreamCloseCause::kIkFault);

  // The teardown re-arms the latch, so reconnecting works. Without that, on_enter
  // is the only reset and re-opening in the SAME mode kind never re-enters the
  // mode -- every later pose session would close on its first sampler tick.
  EXPECT_FALSE(f.pos.ik_faulted());
  ASSERT_TRUE(f.sup.on_stream_open(r).accepted);
  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  EXPECT_TRUE(f.sup.stream_is_open());
  f.sup.stop(); f.teardown();
}

TEST(Supervisor, AGracefulCloseAndADeadlineLapseReportDifferentCauses) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::StreamOpenRequest r; r.timeout_s = 1.0;
  ASSERT_TRUE(f.sup.on_stream_open(r).accepted);
  interface::StreamCloseRequest c;
  f.sup.on_stream_close(c);
  EXPECT_EQ(f.sup.stream_close_cause(), interface::StreamCloseCause::kClientRequest);

  r.timeout_s = 0.05;
  ASSERT_TRUE(f.sup.on_stream_open(r).accepted);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_FALSE(f.sup.stream_is_open());
  EXPECT_EQ(f.sup.stream_close_cause(), interface::StreamCloseCause::kDeadlineExpired);
  f.sup.stop(); f.teardown();
}
