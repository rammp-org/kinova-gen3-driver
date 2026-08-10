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
