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

TEST(Supervisor, RejectsQueuePreemptionUntilTask9) {
  // kQueue promotion isn't settled anywhere yet (Task 9 wires that up); accepting
  // it today would orphan the prior active goal's result. Must reject fail-loud.
  SupFix f;
  interface::TrajectoryGoal g;
  g.trajectory = ramp7(0.0, 0.05, 0.4);
  g.control_mode = interface::ControlModeKind::kPosition;
  g.preemption   = interface::Preemption::kQueue;
  g.path_tolerance = JointVec::Constant(-1.0);
  EXPECT_EQ(f.sup.on_trajectory_goal(g), interface::GoalResponse::kReject);
}
