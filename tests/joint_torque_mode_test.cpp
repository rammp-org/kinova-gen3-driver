#include <gtest/gtest.h>
#include <cmath>
#include "kinova_lowlevel/joint_torque_mode.h"
using namespace kinova;

// With no feedforward ever set, JointTorqueMode must reproduce gravity-comp:
// tau = gravity(q), clamped, with position passthrough and all-torque modes.
TEST(JointTorque, ZeroFeedforwardEqualsGravityCompClampedPassthrough) {
  Dynamics dyn(URDF_PATH);
  JointTorqueMode m(dyn, {1.0, 0.0, JointVec::Constant(39.0)});
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
  JointTorqueMode m(dyn, {1.0, 2.0, JointVec::Constant(1e9)});
  JointFeedback fb; fb.q.setZero(); fb.qd.setConstant(1.0);
  JointCommand c; m.on_enter(fb); m.compute(fb, 0.001, c);
  JointVec g; dyn.gravity(fb.q, g);
  for (int i = 0; i < kNumJoints; ++i)
    EXPECT_NEAR(c.torque[i], g[i] - 2.0 * 1.0, 1e-6);
}

TEST(JointTorque, FeedforwardAddsToGravity) {
  Dynamics dyn(URDF_PATH);
  JointTorqueMode m(dyn, {1.0, 0.0, JointVec::Constant(1e9)});  // huge limit: no clamp interference
  JointFeedback fb; fb.q.setZero(); fb.q[1] = M_PI / 2; fb.qd.setZero();
  JointVec ff; ff.setConstant(3.0);
  JointCommand c; m.on_enter(fb); m.set_torque(ff); m.compute(fb, 0.001, c);
  JointVec g; dyn.gravity(fb.q, g);
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(c.torque[i], g[i] + 3.0, 1e-6);
}

TEST(JointTorque, TotalOutputClampedWithFeedforward) {
  Dynamics dyn(URDF_PATH);
  JointTorqueMode m(dyn, {1.0, 0.0, JointVec::Constant(39.0)});
  JointFeedback fb; fb.q.setZero(); fb.qd.setZero();
  // Assumes gravity(q=0) + 1000 exceeds the 39 N·m limit on every joint for
  // this URDF, so the clamp dominates and output is exactly +39 on all joints.
  JointVec ff; ff.setConstant(1000.0);
  JointCommand c; m.on_enter(fb); m.set_torque(ff); m.compute(fb, 0.001, c);
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(c.torque[i], 39.0, 1e-9);
}

// After cmd_timeout_s of cycles with no fresh command, the feedforward is
// dropped (hard zero with cmd_decay_s=0) and output reverts to gravity comp.
TEST(JointTorque, WatchdogZerosStaleFeedforward) {
  Dynamics dyn(URDF_PATH);
  JointTorqueMode m(dyn, {1.0, 0.0, JointVec::Constant(1e9), 0.05, 0.0});  // timeout 50ms, hard zero
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
  JointTorqueMode m(dyn, {1.0, 0.0, JointVec::Constant(1e9), 0.05, 0.0});
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
  JointTorqueMode m(dyn, {1.0, 0.0, JointVec::Constant(1e9), 0.05, 0.0});
  JointFeedback fb; fb.q.setZero(); fb.qd.setZero();
  JointVec g; dyn.gravity(fb.q, g);
  JointVec ff; ff.setConstant(7.0);
  JointCommand c;
  m.set_torque(ff);   // issued before entry
  m.on_enter(fb);     // entering discards it
  m.compute(fb, 0.001, c);
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(c.torque[i], g[i], 1e-6);
}

// After timeout, the decay ramp must be monotone toward zero with sign
// preserved: positive targets decrease but stay >= 0; negative targets
// increase but stay <= 0. After the full cmd_decay_s window the applied
// feedforward must be exactly zero (output == gravity comp).
TEST(JointTorque, WatchdogDecayRampIsMonotoneToZero) {
  Dynamics dyn(URDF_PATH);
  // scale=1, damping=0, huge limit (no clamp), timeout=0.01s, decay=0.05s
  JointTorqueMode m(dyn, {1.0, 0.0, JointVec::Constant(1e9), 0.01, 0.05});
  JointFeedback fb; fb.q.setZero(); fb.qd.setZero();
  JointVec g; dyn.gravity(fb.q, g);

  // Mixed-sign feedforward: positive on all joints, negative on joint 3.
  JointVec ff; ff.setConstant(6.0); ff[3] = -6.0;
  JointCommand c;
  m.on_enter(fb);
  m.set_torque(ff);
  m.compute(fb, 0.001, c);  // cycle 0: adopt the command

  // Advance just past cmd_timeout_s (0.01s) so decay begins.
  // dt=0.001 -> 10 cycles reaches stale_s_ = 0.010 >= 0.01.
  for (int k = 0; k < 10; ++k) m.compute(fb, 0.001, c);

  // Now run through the decay window (cmd_decay_s=0.05 -> 50 cycles at dt=0.001).
  // dec per cycle = (0.001/0.05)*6.0 = 0.12 -> 50 cycles zeroes it exactly.
  // Check monotonicity and sign preservation each cycle.
  double prev_pos = c.torque[0] - g[0];  // applied ff on a positive-target joint
  double prev_neg = c.torque[3] - g[3];  // applied ff on the negative-target joint
  for (int k = 0; k < 55; ++k) {
    m.compute(fb, 0.001, c);
    const double cur_pos = c.torque[0] - g[0];
    const double cur_neg = c.torque[3] - g[3];
    // Positive-target: non-increasing, never goes below zero.
    EXPECT_LE(cur_pos, prev_pos + 1e-9)
        << "positive-target applied ff increased at decay cycle " << k;
    EXPECT_GE(cur_pos, -1e-9)
        << "positive-target applied ff went negative at decay cycle " << k;
    // Negative-target: non-decreasing (toward 0), never goes above zero.
    EXPECT_GE(cur_neg, prev_neg - 1e-9)
        << "negative-target applied ff decreased at decay cycle " << k;
    EXPECT_LE(cur_neg, 1e-9)
        << "negative-target applied ff went positive at decay cycle " << k;
    prev_pos = cur_pos;
    prev_neg = cur_neg;
  }

  // After 55 cycles the full decay window has elapsed; applied ff must be zero.
  for (int i = 0; i < kNumJoints; ++i)
    EXPECT_NEAR(c.torque[i], g[i], 1e-6)
        << "applied feedforward not zeroed on joint " << i << " after decay window";
}

TEST(JointTorque, WristClampsAtItsOwnLowerLimit) {
  Dynamics dyn(URDF_PATH);
  // Default limits: 39 N*m for joints 1-4, 9 N*m for the wrist (5-7).
  JointTorqueMode m(dyn, {1.0, 0.0, (JointVec() << 39,39,39,39,9,9,9).finished(), 0.0, 0.0});
  JointFeedback fb; fb.q.setZero(); fb.qd.setZero();
  m.on_enter(fb);
  m.set_torque(JointVec::Constant(1000.0));         // demand far beyond every limit
  JointCommand out;
  m.compute(fb, 0.001, out);
  EXPECT_NEAR(out.torque[0], 39.0, 1e-9);           // proximal joint at its limit
  EXPECT_NEAR(out.torque[5], 9.0, 1e-9);            // wrist must NOT be allowed 39
  EXPECT_NEAR(out.torque[6], 9.0, 1e-9);
}
