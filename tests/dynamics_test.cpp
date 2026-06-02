#include <gtest/gtest.h>
#include <cmath>
#include "kinova_lowlevel/dynamics.h"
using namespace kinova;
TEST(Dynamics, LoadsModel) {
  Dynamics dyn(URDF_PATH);
  EXPECT_EQ(dyn.nv(), kNumJoints);
}
TEST(Dynamics, GravityFiniteAndLoadsJointOffAxis) {
  Dynamics dyn(URDF_PATH);
  JointVec q = JointVec::Zero(), tau;
  dyn.gravity(q, tau);
  EXPECT_TRUE(tau.allFinite());
  JointVec q2 = JointVec::Zero(); q2[1] = M_PI / 2.0;   // arm horizontal
  JointVec tau2; dyn.gravity(q2, tau2);
  EXPECT_GT(tau2.cwiseAbs().maxCoeff(), 1.0);
}
