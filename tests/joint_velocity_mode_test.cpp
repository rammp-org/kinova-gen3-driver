#include <cstdio>
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

TEST(JointVelocityMode, ClearsStalePositionAndTorqueOnTheFrozenPath) {
  // RtExecutor reuses one JointCommand across mode switches. A prior mode may
  // have left non-zero position/torque in it; this mode owns velocity and must
  // not let those fields survive, even on the early-return no-target path.
  Dynamics dyn(URDF_PATH);
  JointVelocityMode m(dyn);
  m.on_enter(fb_at(JointVec::Zero()));
  JointCommand out;
  out.position = JointVec::Constant(1.0);
  out.torque = JointVec::Constant(1.0);
  m.compute(fb_at(JointVec::Zero()), 0.001, out);   // no target -> frozen/no-target path
  EXPECT_TRUE(out.position.isZero());
  EXPECT_TRUE(out.torque.isZero());
}

TEST(JointVelocityMode, ClearsStalePositionAndTorqueOnTheTrackingPath) {
  Dynamics dyn(URDF_PATH);
  JointVelocityMode m(dyn);
  m.on_enter(fb_at(JointVec::Zero()));
  m.set_velocity_target(JointVec::Constant(0.2));
  JointCommand out;
  out.position = JointVec::Constant(1.0);
  out.torque = JointVec::Constant(1.0);
  m.compute(fb_at(JointVec::Zero()), 0.001, out);
  EXPECT_TRUE(out.position.isZero());
  EXPECT_TRUE(out.torque.isZero());
}

namespace {
// A comfortably non-singular, elbow-up configuration.
JointVec nominal_q() {
  return (JointVec() << 0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57).finished();
}
// Straight-arm: the classic wrist/elbow singularity for this arm.
JointVec straight_q() { return JointVec::Zero(); }
}  // namespace

TEST(JointVelocityModeTwist, ReproducesTheCommandedTwistAwayFromSingularities) {
  Dynamics dyn(URDF_PATH);
  JointVelocityParams p;
  p.posture_gain = 0.0;               // isolate the task term
  p.max_qd = JointVec::Constant(10.0);   // clamped to URDF, but well above need
  JointVelocityMode m(dyn, p);
  const JointVec q = nominal_q();
  m.on_enter(fb_at(q));

  Vector6 V = Vector6::Zero();
  V[0] = 0.05;                        // 5 cm/s along base x
  m.set_twist_target(V);
  JointCommand out;
  m.compute(fb_at(q), 0.001, out);

  Jacobian6 J; dyn.jacobian(q, J);
  const Vector6 achieved = J * out.velocity;
  // Light damping means near-exact reproduction, not exact.
  EXPECT_NEAR((achieved - V).norm(), 0.0, 1e-3);
}

TEST(JointVelocityModeTwist, StaysBoundedAtAStraightArmSingularity) {
  Dynamics dyn(URDF_PATH);
  JointVelocityParams p;
  p.max_qd = JointVec::Constant(1e3);    // seeded/clamped to URDF anyway
  JointVelocityMode m(dyn, p);
  const JointVec q = straight_q();
  m.on_enter(fb_at(q));

  Vector6 V = Vector6::Zero();
  V[2] = 0.10;                        // push along the degenerate direction
  m.set_twist_target(V);
  JointCommand out;
  m.compute(fb_at(q), 0.001, out);

  EXPECT_TRUE(out.velocity.allFinite());
  JointVec v_max; dyn.velocity_limits(v_max);
  for (int i = 0; i < kNumJoints; ++i)
    EXPECT_LE(std::abs(out.velocity[i]), v_max[i] + 1e-12);
}

TEST(JointVelocityModeTwist, ManipulabilityIsLowerAtTheSingularity) {
  Dynamics dyn(URDF_PATH);
  JointVelocityMode m(dyn);
  Vector6 V = Vector6::Zero(); V[2] = 0.05;
  JointCommand out;

  m.on_enter(fb_at(nominal_q()));
  m.set_twist_target(V);
  m.compute(fb_at(nominal_q()), 0.001, out);
  const double w_nominal = m.last_manipulability();

  m.on_enter(fb_at(straight_q()));
  m.set_twist_target(V);
  m.compute(fb_at(straight_q()), 0.001, out);
  const double w_singular = m.last_manipulability();

  EXPECT_GT(w_nominal, w_singular);
  // Records the real numbers so w_threshold's default is grounded in measurement
  // rather than guessed. Update the default if this ever prints far from it.
  std::printf("w_nominal=%.6f w_singular=%.6f\n", w_nominal, w_singular);
}

TEST(JointVelocityModeTwist, AZeroTwistCommandsZero) {
  Dynamics dyn(URDF_PATH);
  JointVelocityParams p; p.posture_gain = 0.0;
  JointVelocityMode m(dyn, p);
  m.on_enter(fb_at(nominal_q()));
  m.set_twist_target(Vector6::Zero());
  JointCommand out;
  m.compute(fb_at(nominal_q()), 0.001, out);
  EXPECT_NEAR(out.velocity.norm(), 0.0, 1e-9);
}
