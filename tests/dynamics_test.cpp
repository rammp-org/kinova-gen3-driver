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
TEST(Dynamics, DefaultEeFrameResolvesOn2f85) {
  Dynamics dyn(URDF_PATH);                 // default frame "gen3_end_effector_link"
  EXPECT_EQ(dyn.nv(), kNumJoints);         // ctor did not throw
}

TEST(Dynamics, UnknownFrameThrows) {
  EXPECT_THROW(Dynamics(URDF_PATH, "no_such_frame_xyz"), std::runtime_error);
}

TEST(DynamicsFk, NeutralPoseIsFiniteUnitQuatInReach) {
  Dynamics dyn(URDF_PATH);
  JointVec q = JointVec::Zero();
  Pose x = dyn.fk(q);
  EXPECT_TRUE(x.p.allFinite());
  EXPECT_NEAR(x.R.norm(), 1.0, 1e-9);              // unit quaternion
  EXPECT_GT(x.p.norm(), 0.05);                     // tip is away from base origin
  EXPECT_LT(x.p.norm(), 1.5);                      // within physical reach
}

TEST(DynamicsFk, BaseYawRotatesTipAboutVerticalAxis) {
  // Joint 0 is the base yaw about a vertical (world-Z) axis. Rotating it must
  // leave the tip HEIGHT unchanged while MOVING it horizontally. We deliberately
  // do NOT assert a rotation direction or an exact origin-radius: the base axis
  // is offset from the world origin, so origin-radius is not exactly conserved.
  // Height-invariance + horizontal motion is the robust, physically-honest check;
  // fk's full numeric correctness is pinned by the finite-difference Jacobian
  // test (Task 4). Independent of exact link lengths.
  Dynamics dyn(URDF_PATH);
  JointVec q = JointVec::Zero();
  Pose a = dyn.fk(q);
  q[0] = M_PI / 2.0;
  Pose b = dyn.fk(q);
  EXPECT_NEAR(b.p.z(), a.p.z(), 1e-6);                          // vertical axis: height held
  EXPECT_GT((b.p.head<2>() - a.p.head<2>()).norm(), 0.01);     // tip moved horizontally
}
