#include <gtest/gtest.h>
#include "kinova_lowlevel/joint_types.h"
#include "kinova_lowlevel/units.h"
using namespace kinova;
TEST(Units, DegRadRoundTrip) {
  JointVec deg; deg << 0, 90, -180, 45, 30, 270, -90;
  JointVec back = rad_to_deg(deg_to_rad(deg));
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(back[i], deg[i], 1e-9);
}
TEST(Units, WrapToPi) {
  EXPECT_NEAR(wrap_to_pi(3.0 * M_PI / 2.0), -M_PI / 2.0, 1e-9);
  EXPECT_NEAR(wrap_to_pi(-3.0 * M_PI), M_PI, 1e-9);
}
