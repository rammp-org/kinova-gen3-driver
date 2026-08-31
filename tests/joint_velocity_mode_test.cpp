#include <gtest/gtest.h>
#include "kinova_lowlevel/joint_velocity_mode.h"

using namespace kinova;

namespace {
JointFeedback fb_at(const JointVec& q) { JointFeedback fb; fb.q = q; return fb; }
}  // namespace

TEST(JointVelocityMode, RequiresVelocityOnEveryActuator) {
  Dynamics dyn(URDF_PATH);
  JointVelocityMode m(dyn);
  for (auto mode : m.required_modes()) EXPECT_EQ(mode, ActuatorMode::kVelocity);
}

TEST(JointVelocityMode, CommandsZeroBeforeAnyTarget) {
  Dynamics dyn(URDF_PATH);
  JointVelocityMode m(dyn);
  m.on_enter(fb_at(JointVec::Zero()));
  JointCommand out;
  m.compute(fb_at(JointVec::Zero()), 0.001, out);
  EXPECT_EQ(out.mode, ActuatorMode::kVelocity);
  EXPECT_TRUE(out.velocity.isZero());
}

TEST(JointVelocityMode, TracksJointVelocityTargetExactly) {
  Dynamics dyn(URDF_PATH);
  JointVelocityMode m(dyn);
  m.on_enter(fb_at(JointVec::Zero()));
  JointVec qd = JointVec::Constant(0.2);
  m.set_velocity_target(qd);
  JointCommand out;
  m.compute(fb_at(JointVec::Zero()), 0.001, out);
  // Stiff mode: what was asked for is what is commanded, no shaping.
  EXPECT_TRUE(out.velocity.isApprox(qd, 1e-12));
}

TEST(JointVelocityMode, SeedsMaxQdFromUrdfWhenLeftNonFinite) {
  Dynamics dyn(URDF_PATH);
  JointVec v_max; dyn.velocity_limits(v_max);
  JointVelocityMode m(dyn);              // default params: max_qd all +inf
  const JointVelocityParams p = m.params();
  for (int i = 0; i < kNumJoints; ++i) EXPECT_DOUBLE_EQ(p.max_qd[i], v_max[i]);
}

TEST(JointVelocityMode, ClampsARequestAboveTheUrdfLimitDownToIt) {
  Dynamics dyn(URDF_PATH);
  JointVec v_max; dyn.velocity_limits(v_max);
  JointVelocityParams p;
  p.max_qd = JointVec::Constant(1e3);    // absurd request
  JointVelocityMode m(dyn, p);
  for (int i = 0; i < kNumJoints; ++i) EXPECT_DOUBLE_EQ(m.params().max_qd[i], v_max[i]);
}

TEST(JointVelocityMode, ScalesUniformlySoDirectionSurvivesSaturation) {
  Dynamics dyn(URDF_PATH);
  JointVelocityParams p;
  p.max_qd = JointVec::Constant(0.1);
  JointVelocityMode m(dyn, p);
  m.on_enter(fb_at(JointVec::Zero()));
  JointVec qd = JointVec::Zero();
  qd[0] = 1.0; qd[1] = 0.5;              // 2:1 ratio, both over the cap
  m.set_velocity_target(qd);
  JointCommand out;
  m.compute(fb_at(JointVec::Zero()), 0.001, out);
  EXPECT_NEAR(out.velocity[0], 0.1, 1e-12);
  EXPECT_NEAR(out.velocity[1], 0.05, 1e-12);   // ratio preserved, not clamped to 0.1
  EXPECT_LE(out.velocity.cwiseAbs().maxCoeff(), 0.1 + 1e-12);
}

TEST(JointVelocityMode, CommandsZeroWhenTheStreamGoesStale) {
  Dynamics dyn(URDF_PATH);
  JointVelocityParams p; p.cmd_timeout_s = 0.1;
  JointVelocityMode m(dyn, p);
  m.on_enter(fb_at(JointVec::Zero()));
  m.set_velocity_target(JointVec::Constant(0.2));
  JointCommand out;
  m.compute(fb_at(JointVec::Zero()), 0.001, out);
  ASSERT_FALSE(out.velocity.isZero());
  for (int i = 0; i < 150; ++i) m.compute(fb_at(JointVec::Zero()), 0.001, out);
  EXPECT_TRUE(out.velocity.isZero());     // stale -> zero, per decision 6
}

TEST(JointVelocityMode, DisarmingTheWatchdogDoesNotResurrectAStaleTarget) {
  Dynamics dyn(URDF_PATH);
  JointVelocityParams p; p.cmd_timeout_s = 0.1;
  JointVelocityMode m(dyn, p);
  m.on_enter(fb_at(JointVec::Zero()));
  m.set_velocity_target(JointVec::Constant(0.2));
  JointCommand out;
  for (int i = 0; i < 150; ++i) m.compute(fb_at(JointVec::Zero()), 0.001, out);
  ASSERT_TRUE(out.velocity.isZero());
  m.set_command_timeout(0.0);             // disarm
  m.compute(fb_at(JointVec::Zero()), 0.001, out);
  EXPECT_TRUE(out.velocity.isZero());     // the freeze is LATCHED
}

TEST(JointVelocityMode, AFreshTargetReleasesTheFreeze) {
  Dynamics dyn(URDF_PATH);
  JointVelocityParams p; p.cmd_timeout_s = 0.1;
  JointVelocityMode m(dyn, p);
  m.on_enter(fb_at(JointVec::Zero()));
  m.set_velocity_target(JointVec::Constant(0.2));
  JointCommand out;
  for (int i = 0; i < 150; ++i) m.compute(fb_at(JointVec::Zero()), 0.001, out);
  ASSERT_TRUE(out.velocity.isZero());
  m.set_velocity_target(JointVec::Constant(0.15));
  m.compute(fb_at(JointVec::Zero()), 0.001, out);
  EXPECT_NEAR(out.velocity[0], 0.15, 1e-12);
}

TEST(JointVelocityMode, OnEnterDropsATargetSentBeforeEntry) {
  Dynamics dyn(URDF_PATH);
  JointVelocityMode m(dyn);
  m.set_velocity_target(JointVec::Constant(0.3));   // sent BEFORE entry
  m.on_enter(fb_at(JointVec::Zero()));
  JointCommand out;
  m.compute(fb_at(JointVec::Zero()), 0.001, out);
  EXPECT_TRUE(out.velocity.isZero());
}
