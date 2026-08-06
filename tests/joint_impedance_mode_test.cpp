#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include "kinova_lowlevel/joint_impedance_mode.h"
using namespace kinova;

namespace {
JointVec sample_q() {
  JointVec q; q << 0.1, 0.3, -0.2, 0.8, 0.5, -0.4, 0.2; return q;
}
// Isolate the impedance law: no ramp, no IK motion, no reference rate limit.
JointImpedanceParams static_params() {
  JointImpedanceParams p;
  p.gain_ramp_s = 0.0;
  p.ik.max_iters = 0;          // freeze q_d at its seeded value
  p.max_ref_speed = 1e9;
  return p;
}
}  // namespace

TEST(JointImpedance, RequiresTorqueAndPassesThroughPosition) {
  Dynamics dyn(URDF_PATH);
  JointImpedanceMode m(dyn, static_params());
  JointFeedback fb; fb.q = sample_q(); fb.qd.setZero();
  for (auto x : m.required_modes()) EXPECT_EQ(x, ActuatorMode::kTorque);
  JointCommand c; m.on_enter(fb); m.compute(fb, 0.001, c);
  EXPECT_EQ(c.mode, ActuatorMode::kTorque);
  EXPECT_NEAR((c.position - fb.q).norm(), 0.0, 1e-12);
}

TEST(JointImpedance, AtReferenceZeroVelGivesGravityOnly) {
  Dynamics dyn(URDF_PATH);
  JointImpedanceMode m(dyn, static_params());
  JointFeedback fb; fb.q = sample_q(); fb.qd.setZero();
  m.on_enter(fb);                       // q_d := fb.q -> zero spring error
  JointCommand c; m.compute(fb, 0.001, c);
  JointVec g; dyn.gravity(fb.q, g);
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(c.torque[i], g[i], 1e-9);
}

TEST(JointImpedance, ReferenceSeededAtMeasuredConfigOnEnter) {
  Dynamics dyn(URDF_PATH);
  JointImpedanceMode m(dyn, static_params());
  JointFeedback fb; fb.q = sample_q(); fb.qd.setZero();
  m.on_enter(fb);
  EXPECT_NEAR((m.reference() - fb.q).norm(), 0.0, 1e-12);
}

TEST(JointImpedance, MatchesIndependentlyComputedLaw) {
  Dynamics dyn(URDF_PATH);
  JointImpedanceParams p = static_params();
  JointImpedanceMode m(dyn, p);
  JointFeedback enter; enter.q = sample_q(); enter.qd.setZero();
  m.on_enter(enter);                                  // q_d := sample_q()

  JointFeedback fb; fb.q = sample_q();
  fb.q[1] -= 0.10; fb.q[4] += 0.08;                   // displace from the reference
  fb.qd.setConstant(0.05);
  JointCommand c; m.compute(fb, 0.001, c);

  JointVec g; dyn.gravity(fb.q, g);
  JointVec expected;
  for (int i = 0; i < kNumJoints; ++i) {
    const double e = std::clamp(enter.q[i] - fb.q[i],
                                -p.max_tracking_error, p.max_tracking_error);
    expected[i] = g[i] + p.Kq[i] * e - p.Dq[i] * fb.qd[i];
    expected[i] = std::clamp(expected[i], -p.torque_limit[i], p.torque_limit[i]);
  }
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(c.torque[i], expected[i], 1e-9);
}

TEST(JointImpedance, PerJointTorqueClampHonorsWristLimit) {
  Dynamics dyn(URDF_PATH);
  JointImpedanceParams p = static_params();
  p.Kq.setConstant(5000.0);
  p.max_tracking_error = 10.0;                        // let the spring run away
  JointImpedanceMode m(dyn, p);
  JointFeedback enter; enter.q = sample_q(); enter.qd.setZero();
  m.on_enter(enter);
  JointFeedback fb = enter; fb.q.array() += 0.5;      // huge error on every joint
  JointCommand c; m.compute(fb, 0.001, c);
  for (int i = 0; i < kNumJoints; ++i)
    EXPECT_LE(std::abs(c.torque[i]), p.torque_limit[i] + 1e-9) << "joint " << i;
  EXPECT_LE(std::abs(c.torque[6]), 9.0 + 1e-9);       // wrist limit, not 39
}

TEST(JointImpedance, LeashCapsSpringButNotGravity) {
  Dynamics dyn(URDF_PATH);
  JointImpedanceParams p = static_params();
  p.max_tracking_error = 0.05;
  p.torque_limit.setConstant(1e6);                    // isolate the leash
  JointImpedanceMode m(dyn, p);
  JointFeedback enter; enter.q = sample_q(); enter.qd.setZero();
  m.on_enter(enter);
  JointFeedback fb = enter; fb.q[0] -= 1.0;           // way past the leash
  JointCommand c; m.compute(fb, 0.001, c);
  JointVec g; dyn.gravity(fb.q, g);
  // Spring saturates at Kq*leash; gravity is untouched (never scaled or clamped).
  EXPECT_NEAR(c.torque[0], g[0] + p.Kq[0] * p.max_tracking_error, 1e-9);
}

TEST(JointImpedance, RampScalesSpringButGravityStaysFull) {
  Dynamics dyn(URDF_PATH);
  JointImpedanceParams p = static_params();
  p.gain_ramp_s = 1.0;
  JointImpedanceMode m(dyn, p);
  JointFeedback enter; enter.q = sample_q(); enter.qd.setZero();
  m.on_enter(enter);
  JointFeedback fb = enter; fb.q[1] -= 0.10;
  JointVec g; dyn.gravity(fb.q, g);

  JointCommand c0; m.compute(fb, 0.25, c0);           // elapsed 0 -> ramp 0
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(c0.torque[i], g[i], 1e-9);

  JointCommand c1; m.compute(fb, 0.25, c1);           // elapsed 0.25 -> ramp 0.25
  EXPECT_NEAR(c1.torque[1], g[1] + 0.25 * p.Kq[1] * 0.10, 1e-7);
}

TEST(JointImpedance, ReferenceSpeedLimitBoundsMotionOnTeleportedTarget) {
  Dynamics dyn(URDF_PATH);
  JointImpedanceParams p;
  p.gain_ramp_s = 0.0;
  p.max_ref_speed = 0.5;                              // rad/s
  JointImpedanceMode m(dyn, p);
  JointFeedback fb; fb.q = sample_q(); fb.qd.setZero();
  m.on_enter(fb);
  Pose far = dyn.fk(fb.q); far.p += Eigen::Vector3d(0.4, 0.3, -0.2);
  m.set_target(far);

  const JointVec before = m.reference();
  JointCommand c; m.compute(fb, 0.001, c);
  const double moved = (m.reference() - before).lpNorm<Eigen::Infinity>();
  EXPECT_GT(moved, 0.0);                              // it did move toward the target
  EXPECT_LE(moved, p.max_ref_speed * 0.001 + 1e-12);  // ...but no faster than allowed
}

TEST(JointImpedance, IkLimitsSeededFromUrdf) {
  // The mode must not leave the solver unbounded just because the default params
  // carry infinite limits: driving hard at an unreachable target for 5 s must not
  // walk the reference past a hard stop.
  Dynamics dyn(URDF_PATH);
  JointImpedanceMode m(dyn, JointImpedanceParams{});
  JointVec lo, hi; dyn.joint_limits(lo, hi);

  JointFeedback fb; fb.q = sample_q(); fb.qd.setZero();
  m.on_enter(fb);
  Pose far = dyn.fk(fb.q); far.p += Eigen::Vector3d(3.0, 3.0, 3.0);
  m.set_target(far);
  for (int k = 0; k < 5000; ++k) { JointCommand c; m.compute(fb, 0.001, c); }

  const JointVec q_ref = m.reference();
  EXPECT_FALSE(q_ref.hasNaN());
  for (int i = 0; i < kNumJoints; ++i) {
    if (std::isfinite(lo[i])) {
      EXPECT_GE(q_ref[i], lo[i] - 1e-6) << "joint " << i;
    }
    if (std::isfinite(hi[i])) {
      EXPECT_LE(q_ref[i], hi[i] + 1e-6) << "joint " << i;
    }
  }
}
