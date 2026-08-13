#include <gtest/gtest.h>
#include "kinova_lowlevel/interface/value_types.h"
#include "kinova_lowlevel/interface/ports.h"
#include "fake_backend.h"
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
