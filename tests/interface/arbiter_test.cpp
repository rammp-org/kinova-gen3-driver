#include <gtest/gtest.h>
#include <vector>
#include "kinova_lowlevel/interface/arbiter.h"

using namespace kinova;
using namespace kinova::interface;

namespace {
// Records what actually reached the downstream sink. Asserting on the ABSENCE of a
// delegate is the whole point: a rejected command must not merely return an error,
// it must never touch the Supervisor.
struct RecordingSink : public CommandSink, public StreamSink {
  int goals=0, accepted=0, cancels=0, gains=0, queries=0;
  int stream_opens=0, stream_closes=0, setpoints=0;
  std::vector<HaltReason> halts;
  GoalResponse   on_trajectory_goal(const TrajectoryGoal&) override { ++goals; return GoalResponse::kAccept; }
  void           on_trajectory_accepted(const GoalId&, const TrajectoryGoal&) override { ++accepted; }
  CancelResponse on_trajectory_cancel(const CancelRequest&) override { ++cancels; return CancelResponse::kAccept; }
  GainsResult    on_set_gains(const GainsRequest&) override { ++gains; return {true, ""}; }
  ArmState       on_query_state() override { ++queries; ArmState s; s.stamp_s = 42.0; return s; }
  void           on_halt(HaltReason r) override { halts.push_back(r); }

  StreamOpenResult on_stream_open(const StreamOpenRequest&) override { ++stream_opens; return {true, 0, ""}; }
  void             on_stream_close(const StreamCloseRequest&) override { ++stream_closes; }
  void             on_setpoint_joint_position(const JointSetpoint&) override { ++setpoints; }
  void             on_setpoint_joint_velocity(const JointSetpoint&) override { ++setpoints; }
  void             on_setpoint_joint_torque(const JointSetpoint&) override { ++setpoints; }
  void             on_setpoint_pose(const PoseSetpoint&) override { ++setpoints; }
  void             on_setpoint_twist(const TwistSetpoint&) override { ++setpoints; }
};
TrajectoryGoal goal_with(const Token& t){ TrajectoryGoal g; g.token = t; return g; }
}  // namespace

// ---------- grant, token minting, admission ----------

TEST(Arbiter, EnforcedRejectsCommandWithoutGrant) {
  RecordingSink sink; Arbiter arb{sink, sink, ArbitrationMode::kEnforced, 1234};
  EXPECT_EQ(arb.on_trajectory_goal(goal_with(Token{})), GoalResponse::kRejectUnauthorized);
  EXPECT_EQ(sink.goals, 0);                       // never reached the Supervisor
  EXPECT_EQ(arb.status().rejected_count, 1u);
}

TEST(Arbiter, GrantMintsUniqueTokensAndIncrementsGeneration) {
  RecordingSink sink; Arbiter arb{sink, sink, ArbitrationMode::kEnforced, 1234};
  const GrantResult a = arb.grant("planner");
  ASSERT_TRUE(a.accepted);
  EXPECT_EQ(a.generation, 1u);
  EXPECT_NE(a.token, (Token{}));                  // not left zeroed
  const GrantResult b = arb.grant("teleop");
  ASSERT_TRUE(b.accepted);
  EXPECT_EQ(b.generation, 2u);
  EXPECT_NE(b.token, a.token);                    // fresh per grant
  EXPECT_EQ(arb.status().owner_id, "teleop");
}

TEST(Arbiter, CorrectTokenDelegatesExactlyOnce) {
  RecordingSink sink; Arbiter arb{sink, sink, ArbitrationMode::kEnforced, 1234};
  const Token t = arb.grant("planner").token;
  EXPECT_EQ(arb.on_trajectory_goal(goal_with(t)), GoalResponse::kAccept);
  EXPECT_EQ(sink.goals, 1);
  EXPECT_EQ(arb.status().rejected_count, 0u);
}

TEST(Arbiter, WrongTokenRejectedAndNotDelegated) {
  RecordingSink sink; Arbiter arb{sink, sink, ArbitrationMode::kEnforced, 1234};
  arb.grant("planner");
  Token wrong{}; wrong[0] = 0xAB;
  EXPECT_EQ(arb.on_trajectory_goal(goal_with(wrong)), GoalResponse::kRejectUnauthorized);
  EXPECT_EQ(sink.goals, 0);
}

TEST(Arbiter, StaleTokenRejectedAfterRegrant) {   // the zombie-node case
  RecordingSink sink; Arbiter arb{sink, sink, ArbitrationMode::kEnforced, 1234};
  const Token old = arb.grant("planner").token;
  arb.grant("teleop");                            // ownership moved on
  EXPECT_EQ(arb.on_trajectory_goal(goal_with(old)), GoalResponse::kRejectUnauthorized);
  EXPECT_EQ(sink.goals, 0);
}

TEST(Arbiter, QueryStateIsNeverGated) {
  RecordingSink sink; Arbiter arb{sink, sink, ArbitrationMode::kEnforced, 1234};
  EXPECT_NEAR(arb.on_query_state().stamp_s, 42.0, 1e-9);   // no grant at all
  EXPECT_EQ(sink.queries, 1);
}

TEST(Arbiter, DisabledModeAdmitsAnyToken) {
  RecordingSink sink; Arbiter arb{sink, sink, ArbitrationMode::kDisabled, 1234};
  EXPECT_EQ(arb.on_trajectory_goal(goal_with(Token{})), GoalResponse::kAccept);
  EXPECT_EQ(sink.goals, 1);
  EXPECT_EQ(arb.status().mode, ArbitrationMode::kDisabled);   // visible, not just logged
}

TEST(Arbiter, SetGainsIsGated) {
  RecordingSink sink; Arbiter arb{sink, sink, ArbitrationMode::kEnforced, 1234};
  GainsRequest r;                                  // zero token
  EXPECT_FALSE(arb.on_set_gains(r).accepted);
  EXPECT_EQ(sink.gains, 0);
}

// ---------- revoke and the halt handshake ----------

TEST(Arbiter, RevokeHaltsAndRejectsTheOldToken) {
  RecordingSink sink; Arbiter arb{sink, sink, ArbitrationMode::kEnforced, 1234};
  const Token t = arb.grant("planner").token;
  arb.revoke();
  ASSERT_EQ(sink.halts.size(), 1u);
  EXPECT_EQ(sink.halts[0], HaltReason::kOwnershipRevoked);
  EXPECT_EQ(arb.on_trajectory_goal(goal_with(t)), GoalResponse::kRejectUnauthorized);
  EXPECT_EQ(sink.goals, 0);
  EXPECT_FALSE(arb.status().owned);
}

TEST(Arbiter, RevokeWithNoOwnerDoesNotHalt) {
  RecordingSink sink; Arbiter arb{sink, sink, ArbitrationMode::kEnforced, 1234};
  arb.revoke();
  EXPECT_TRUE(sink.halts.empty());     // nothing to stop; don't jolt the arm for nothing
}

TEST(Arbiter, RegrantHaltsBeforeTheNewGrantIsLive) {
  RecordingSink sink; Arbiter arb{sink, sink, ArbitrationMode::kEnforced, 1234};
  arb.grant("planner");
  const GrantResult second = arb.grant("teleop");
  ASSERT_TRUE(second.accepted);
  ASSERT_EQ(sink.halts.size(), 1u);                 // the swap stopped the arm first
  EXPECT_EQ(sink.halts[0], HaltReason::kOwnershipRevoked);
  EXPECT_EQ(arb.on_trajectory_goal(goal_with(second.token)), GoalResponse::kAccept);
}

TEST(Arbiter, CancelRequiresTheOwnerToken) {
  RecordingSink sink; Arbiter arb{sink, sink, ArbitrationMode::kEnforced, 1234};
  const Token t = arb.grant("planner").token;
  CancelRequest stranger;                                   // zero token
  EXPECT_EQ(arb.on_trajectory_cancel(stranger), CancelResponse::kReject);
  EXPECT_EQ(sink.cancels, 0);
  CancelRequest owner; owner.token = t;
  EXPECT_EQ(arb.on_trajectory_cancel(owner), CancelResponse::kAccept);
  EXPECT_EQ(sink.cancels, 1);
}

TEST(Arbiter, AcceptedRechecksTheTokenIndependently) {
  RecordingSink sink; Arbiter arb{sink, sink, ArbitrationMode::kEnforced, 1234};
  const Token t = arb.grant("planner").token;
  const TrajectoryGoal g = goal_with(t);
  GoalId id{}; id[0] = 7;
  ASSERT_EQ(arb.on_trajectory_goal(g), GoalResponse::kAccept);
  arb.revoke();                                    // ownership lost between accept and accepted
  arb.on_trajectory_accepted(id, g);
  EXPECT_EQ(sink.accepted, 0);                     // must not slip through on a prior accept
}

// ---------- e-stop latch and status ----------

TEST(Arbiter, EstopHaltsDropsTheGrantAndRejectsEverything) {
  RecordingSink sink; Arbiter arb{sink, sink, ArbitrationMode::kEnforced, 1234};
  const Token t = arb.grant("planner").token;
  arb.estop();
  ASSERT_EQ(sink.halts.size(), 1u);
  EXPECT_EQ(sink.halts[0], HaltReason::kEmergencyStop);
  EXPECT_EQ(arb.on_trajectory_goal(goal_with(t)), GoalResponse::kRejectUnauthorized);
  EXPECT_EQ(sink.goals, 0);
  EXPECT_TRUE(arb.status().estopped);
  EXPECT_FALSE(arb.status().owned);
}

TEST(Arbiter, EstopLatchesAndRefusesAFreshGrant) {
  RecordingSink sink; Arbiter arb{sink, sink, ArbitrationMode::kEnforced, 1234};
  arb.estop();
  const GrantResult g = arb.grant("planner");
  EXPECT_FALSE(g.accepted);                      // cannot grant your way out of an e-stop
  EXPECT_EQ(g.message, "e-stopped");
}

TEST(Arbiter, EstopRejectsEvenInDisabledMode) {
  RecordingSink sink; Arbiter arb{sink, sink, ArbitrationMode::kDisabled, 1234};
  EXPECT_EQ(arb.on_trajectory_goal(goal_with(Token{})), GoalResponse::kAccept);   // bypass works
  arb.estop();
  EXPECT_EQ(arb.on_trajectory_goal(goal_with(Token{})), GoalResponse::kRejectUnauthorized);
  EXPECT_EQ(sink.goals, 1);                      // the one before the e-stop, and no more
}

TEST(Arbiter, EstopClearExitsToNoOwnerNotToOwned) {
  RecordingSink sink; Arbiter arb{sink, sink, ArbitrationMode::kEnforced, 1234};
  const Token t = arb.grant("planner").token;
  arb.estop();
  arb.estop_clear();
  EXPECT_FALSE(arb.status().estopped);
  EXPECT_FALSE(arb.status().owned);
  EXPECT_EQ(arb.on_trajectory_goal(goal_with(t)), GoalResponse::kRejectUnauthorized);  // old token dead
  const GrantResult g = arb.grant("planner");
  EXPECT_TRUE(g.accepted);                       // but a fresh grant now works
}

TEST(Arbiter, EstopClearWorksInDisabledMode) {
  RecordingSink sink; Arbiter arb{sink, sink, ArbitrationMode::kDisabled, 1234};
  arb.estop();
  arb.estop_clear();                             // must not be able to strand yourself
  EXPECT_EQ(arb.on_trajectory_goal(goal_with(Token{})), GoalResponse::kAccept);
}

TEST(Arbiter, StatusReflectsOwnerAndRejectionCount) {
  RecordingSink sink; Arbiter arb{sink, sink, ArbitrationMode::kEnforced, 1234};
  arb.grant("planner");
  arb.on_trajectory_goal(goal_with(Token{}));    // reject 1
  CancelRequest c; arb.on_trajectory_cancel(c);  // reject 2
  const ArbitrationStatus st = arb.status();
  EXPECT_TRUE(st.owned);
  EXPECT_EQ(st.owner_id, "planner");
  EXPECT_EQ(st.generation, 1u);
  EXPECT_EQ(st.rejected_count, 2u);
}

// ---------- streaming tier is gated the same way ----------

TEST(Arbiter, StreamOpenRequiresTheOwnerToken) {
  RecordingSink sink; Arbiter arb{sink, sink, ArbitrationMode::kEnforced, 1234};
  StreamOpenRequest r;                                    // zero token
  EXPECT_FALSE(arb.on_stream_open(r).accepted);
  EXPECT_EQ(sink.stream_opens, 0);                        // never reached the Supervisor
  const Token t = arb.grant("servo").token;
  r.token = t;
  EXPECT_TRUE(arb.on_stream_open(r).accepted);
  EXPECT_EQ(sink.stream_opens, 1);
}

TEST(Arbiter, SetpointsAreGatedByToken) {
  RecordingSink sink; Arbiter arb{sink, sink, ArbitrationMode::kEnforced, 1234};
  const Token t = arb.grant("servo").token;
  JointSetpoint good; good.token = t;
  JointSetpoint bad;                                      // zero token
  arb.on_setpoint_joint_position(good);
  arb.on_setpoint_joint_position(bad);
  EXPECT_EQ(sink.setpoints, 1);                           // only the authorised one got through
}

TEST(Arbiter, NoSetpointLandsAfterRevoke) {
  RecordingSink sink; Arbiter arb{sink, sink, ArbitrationMode::kEnforced, 1234};
  const Token t = arb.grant("servo").token;
  JointSetpoint sp; sp.token = t;
  arb.on_setpoint_joint_position(sp);
  EXPECT_EQ(sink.setpoints, 1);
  arb.revoke();
  arb.on_setpoint_joint_position(sp);
  EXPECT_EQ(sink.setpoints, 1);                           // the ex-owner cannot keep driving
}

TEST(Arbiter, EstopBlocksStreamingEvenInDisabledMode) {
  RecordingSink sink; Arbiter arb{sink, sink, ArbitrationMode::kDisabled, 1234};
  JointSetpoint sp;                                       // no token needed in kDisabled
  arb.on_setpoint_joint_position(sp);
  EXPECT_EQ(sink.setpoints, 1);
  arb.estop();
  arb.on_setpoint_joint_position(sp);
  EXPECT_EQ(sink.setpoints, 1);                           // e-stop is what kDisabled does NOT bypass
}
