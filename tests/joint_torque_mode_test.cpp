#include <gtest/gtest.h>
#include <cmath>
#include "kinova_lowlevel/joint_torque_mode.h"
using namespace kinova;

// With no feedforward ever set, JointTorqueMode must reproduce gravity-comp:
// tau = gravity(q), clamped, with position passthrough and all-torque modes.
TEST(JointTorque, ZeroFeedforwardEqualsGravityCompClampedPassthrough) {
  Dynamics dyn(URDF_PATH);
  JointTorqueMode m(dyn, {1.0, 0.0, 39.0});
  JointFeedback fb; fb.q.setZero(); fb.q[1] = M_PI / 2; fb.qd.setZero();
  for (auto x : m.required_modes()) EXPECT_EQ(x, ActuatorMode::kTorque);
  JointCommand c; m.on_enter(fb); m.compute(fb, 0.001, c);
  JointVec g; dyn.gravity(fb.q, g);
  for (int i = 0; i < kNumJoints; ++i) {
    EXPECT_LE(std::abs(c.torque[i]), 39.0 + 1e-9);
    if (std::abs(g[i]) < 39.0) { EXPECT_NEAR(c.torque[i], g[i], 1e-6); }
  }
  EXPECT_NEAR(c.position[1], fb.q[1], 1e-9);
  EXPECT_EQ(c.mode, ActuatorMode::kTorque);
}

TEST(JointTorque, DampingSubtractsVelocityTerm) {
  Dynamics dyn(URDF_PATH);
  JointTorqueMode m(dyn, {1.0, 2.0, 1e9});
  JointFeedback fb; fb.q.setZero(); fb.qd.setConstant(1.0);
  JointCommand c; m.on_enter(fb); m.compute(fb, 0.001, c);
  JointVec g; dyn.gravity(fb.q, g);
  for (int i = 0; i < kNumJoints; ++i)
    EXPECT_NEAR(c.torque[i], g[i] - 2.0 * 1.0, 1e-6);
}

TEST(JointTorque, FeedforwardAddsToGravity) {
  Dynamics dyn(URDF_PATH);
  JointTorqueMode m(dyn, {1.0, 0.0, 1e9});  // huge limit: no clamp interference
  JointFeedback fb; fb.q.setZero(); fb.q[1] = M_PI / 2; fb.qd.setZero();
  JointVec ff; ff.setConstant(3.0);
  JointCommand c; m.on_enter(fb); m.set_torque(ff); m.compute(fb, 0.001, c);
  JointVec g; dyn.gravity(fb.q, g);
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(c.torque[i], g[i] + 3.0, 1e-6);
}

TEST(JointTorque, TotalOutputClampedWithFeedforward) {
  Dynamics dyn(URDF_PATH);
  JointTorqueMode m(dyn, {1.0, 0.0, 39.0});
  JointFeedback fb; fb.q.setZero(); fb.qd.setZero();
  JointVec ff; ff.setConstant(1000.0);  // gravity + 1000 >> 39 on every joint
  JointCommand c; m.on_enter(fb); m.set_torque(ff); m.compute(fb, 0.001, c);
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(c.torque[i], 39.0, 1e-9);
}

// After cmd_timeout_s of cycles with no fresh command, the feedforward is
// dropped (hard zero with cmd_decay_s=0) and output reverts to gravity comp.
TEST(JointTorque, WatchdogZerosStaleFeedforward) {
  Dynamics dyn(URDF_PATH);
  JointTorqueMode m(dyn, {1.0, 0.0, 1e9, 0.05, 0.0});  // timeout 50ms, hard zero
  JointFeedback fb; fb.q.setZero(); fb.q[1] = M_PI / 2; fb.qd.setZero();
  JointVec g; dyn.gravity(fb.q, g);
  JointVec ff; ff.setConstant(5.0);
  JointCommand c; m.on_enter(fb); m.set_torque(ff);
  m.compute(fb, 0.001, c);                       // first cycle adopts the command
  EXPECT_NEAR(c.torque[0], g[0] + 5.0, 1e-6);
  for (int k = 0; k < 100; ++k) m.compute(fb, 0.001, c);  // 0.1s > 0.05s timeout
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(c.torque[i], g[i], 1e-6);
}

// Re-issuing the command every cycle keeps it applied; stopping lets it lapse.
TEST(JointTorque, FreshCommandResetsWatchdog) {
  Dynamics dyn(URDF_PATH);
  JointTorqueMode m(dyn, {1.0, 0.0, 1e9, 0.05, 0.0});
  JointFeedback fb; fb.q.setZero(); fb.qd.setZero();
  JointVec g; dyn.gravity(fb.q, g);
  JointVec ff; ff.setConstant(5.0);
  JointCommand c; m.on_enter(fb);
  for (int k = 0; k < 100; ++k) { m.set_torque(ff); m.compute(fb, 0.001, c); }
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(c.torque[i], g[i] + 5.0, 1e-6);
  for (int k = 0; k < 100; ++k) m.compute(fb, 0.001, c);  // stop issuing -> lapse
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(c.torque[i], g[i], 1e-6);
}

// A command issued BEFORE on_enter must be discarded on entry.
TEST(JointTorque, OnEnterDiscardsPriorCommand) {
  Dynamics dyn(URDF_PATH);
  JointTorqueMode m(dyn, {1.0, 0.0, 1e9, 0.05, 0.0});
  JointFeedback fb; fb.q.setZero(); fb.qd.setZero();
  JointVec g; dyn.gravity(fb.q, g);
  JointVec ff; ff.setConstant(7.0);
  JointCommand c;
  m.set_torque(ff);   // issued before entry
  m.on_enter(fb);     // entering discards it
  m.compute(fb, 0.001, c);
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(c.torque[i], g[i], 1e-6);
}
