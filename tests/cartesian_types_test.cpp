#include <gtest/gtest.h>
#include "kinova_lowlevel/cartesian_types.h"
using namespace kinova;

TEST(CartesianTypes, PoseDefaultsToIdentity) {
  Pose x;
  EXPECT_TRUE(x.p.isZero());
  EXPECT_NEAR(x.R.norm(), 1.0, 1e-12);          // unit quaternion
  EXPECT_NEAR(x.R.w(), 1.0, 1e-12);             // identity rotation
}

TEST(CartesianTypes, FixedSizes) {
  EXPECT_EQ(Vector6::RowsAtCompileTime, 6);
  EXPECT_EQ(Jacobian6::RowsAtCompileTime, 6);
  EXPECT_EQ(Jacobian6::ColsAtCompileTime, kNumJoints);
}
