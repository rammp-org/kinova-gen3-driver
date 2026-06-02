#include <gtest/gtest.h>
#include <cmath>
#include "kinova_lowlevel/dynamics.h"
using namespace kinova;
TEST(Dynamics, LoadsModel) {
  Dynamics dyn(URDF_PATH);
  EXPECT_EQ(dyn.nv(), kNumJoints);
}
TEST(Dynamics, ContinuousJointsInflateNq) {
  // The Gen3 7-DOF has continuous joints, which Pinocchio represents as (cos,sin)
  // pairs -> nq > nv. This guards the faithful packing path (no wide-limit hack):
  // if a build ever used bounded revolute joints instead, nq would equal nv.
  Dynamics dyn(URDF_PATH);
  EXPECT_GT(dyn.nq(), dyn.nv());
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
