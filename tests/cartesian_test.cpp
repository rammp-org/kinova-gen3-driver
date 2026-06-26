#include <gtest/gtest.h>
#include <cmath>
#include "kinova_lowlevel/cartesian.h"
using namespace kinova;

TEST(PoseError, IdenticalPosesGiveZero) {
  Pose a;
  a.p = Eigen::Vector3d(0.1, -0.2, 0.3);
  a.R = Eigen::Quaterniond(Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitY()));
  Vector6 e = pose_error(a, a);
  EXPECT_NEAR(e.norm(), 0.0, 1e-12);
}

TEST(PoseError, PureTranslationIsDesiredMinusCurrent) {
  Pose cur, des;
  cur.p = Eigen::Vector3d(0, 0, 0);
  des.p = Eigen::Vector3d(0.05, -0.10, 0.02);
  Vector6 e = pose_error(des, cur);
  EXPECT_NEAR((e.head<3>() - des.p).norm(), 0.0, 1e-12);
  EXPECT_NEAR(e.tail<3>().norm(), 0.0, 1e-12);
}

TEST(PoseError, PureRotationAboutZ) {
  Pose cur, des;
  const double ang = 0.3;
  des.R = Eigen::Quaterniond(Eigen::AngleAxisd(ang, Eigen::Vector3d::UnitZ()));
  Vector6 e = pose_error(des, cur);
  EXPECT_NEAR(e.head<3>().norm(), 0.0, 1e-12);
  EXPECT_NEAR(e[3], 0.0,  1e-9);
  EXPECT_NEAR(e[4], 0.0,  1e-9);
  EXPECT_NEAR(e[5], ang,  1e-9);
}

TEST(PoseError, LargeRotationTakesShortestPath) {
  // A 1.8*pi rotation about +X has a quaternion with w<0; pose_error must report
  // it as the equivalent short -0.2*pi arc (axis flips to -X), exercising the
  // shortest-geodesic branch. (At q=1.8pi: half-angle=0.9pi, w=cos(0.9pi)<0 ->
  // flip -> angle=0.2pi about -X -> rotvec[0] = -0.2pi.)
  // Rotation lives in e.tail<3>() = e[3..5]; e[3] is the X component.
  Pose cur, des;
  des.R = Eigen::Quaterniond(Eigen::AngleAxisd(1.8 * M_PI, Eigen::Vector3d::UnitX()));
  Vector6 e = pose_error(des, cur);
  EXPECT_NEAR(e.head<3>().norm(), 0.0, 1e-12);     // pure rotation, no translation
  EXPECT_NEAR(e[3], -0.2 * M_PI, 1e-9);            // short arc, not +1.8*pi
  EXPECT_NEAR(e[4], 0.0, 1e-9);
  EXPECT_NEAR(e[5], 0.0, 1e-9);
}
