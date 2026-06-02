#include <gtest/gtest.h>
#include <cmath>
#include "kinova_lowlevel/gravity_comp_mode.h"
using namespace kinova;
TEST(GravComp, OutputsClampedGravityAndPassthrough) {
  Dynamics dyn(URDF_PATH);
  GravityCompTorqueMode m(dyn, {1.0, 0.0, 39.0});
  JointFeedback fb; fb.q.setZero(); fb.q[1]=M_PI/2; fb.qd.setZero();
  for (auto x : m.required_modes()) EXPECT_EQ(x, ActuatorMode::kTorque);
  JointCommand c; m.on_enter(fb); m.compute(fb, 0.001, c);
  JointVec g; dyn.gravity(fb.q, g);
  for (int i=0;i<kNumJoints;++i){
    EXPECT_LE(std::abs(c.torque[i]), 39.0+1e-9);
    if (std::abs(g[i])<39.0) EXPECT_NEAR(c.torque[i], g[i], 1e-6);
  }
  EXPECT_NEAR(c.position[1], fb.q[1], 1e-9);
  EXPECT_EQ(c.mode, ActuatorMode::kTorque);
}
TEST(GravComp, DampingSubtractsVelocityTerm) {
  Dynamics dyn(URDF_PATH);
  GravityCompTorqueMode m(dyn, {1.0, 2.0, 1e9});
  JointFeedback fb; fb.q.setZero(); fb.qd.setConstant(1.0);
  JointCommand c; m.on_enter(fb); m.compute(fb, 0.001, c);
  JointVec g; dyn.gravity(fb.q, g);
  for (int i=0;i<kNumJoints;++i) EXPECT_NEAR(c.torque[i], g[i]-2.0*1.0, 1e-6);
}
