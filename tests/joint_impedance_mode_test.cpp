#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include "kinova_lowlevel/joint_impedance_mode.h"
#include "kinova_lowlevel/units.h"
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
  JointMat M; dyn.mass_matrix(fb.q, M);
  JointVec expected;
  for (int i = 0; i < kNumJoints; ++i) {
    const double e = std::clamp(enter.q[i] - fb.q[i],
                                -p.max_tracking_error, p.max_tracking_error);
    const double Dq = 2.0 * p.zeta * std::sqrt(p.Kq[i] * M(i, i));
    expected[i] = g[i] + p.Kq[i] * e - Dq * fb.qd[i];
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

TEST(JointImpedance, ContinuousJointErrorTakesShortWayAroundTheWrap) {
  // j3 (index 2) is continuous, and KortexTransport wraps every measured angle to
  // (-pi, pi] (kortex_transport.cpp fill_feedback). So a reference at +3.13 and a
  // measurement at -3.13 are 0.023 rad apart the SHORT way, not 6.26. Reading the
  // raw difference saturates the spring at Kq*leash in a fixed direction, and the
  // joint spins continuously -- every wrap reasserts the same error.
  Dynamics dyn(URDF_PATH);
  JointImpedanceParams p = static_params();
  p.torque_limit.setConstant(1e6);              // isolate the spring from the clamp
  JointImpedanceMode m(dyn, p);
  JointFeedback enter; enter.q = sample_q(); enter.q[2] = 3.13; enter.qd.setZero();
  m.on_enter(enter);                            // q_d[2] := 3.13

  JointFeedback fb = enter; fb.q[2] = -3.13;    // joint crossed the wrap boundary
  JointCommand c; m.compute(fb, 0.001, c);

  JointVec g; dyn.gravity(fb.q, g);
  const double spring = c.torque[2] - g[2];
  const double true_err = wrap_to_pi(3.13 - (-3.13));   // = -0.0232 rad
  EXPECT_LT(spring, 0.0) << "spring must push the SHORT way, not saturate forward";
  EXPECT_NEAR(spring, p.Kq[2] * true_err, 1e-9);
}

TEST(JointImpedance, ContinuousReferenceStaysBounded) {
  // The reference integrates open-loop. On a continuous joint it must not grow
  // without bound, or it drifts arbitrarily far from the wrapped measurement.
  Dynamics dyn(URDF_PATH);
  JointImpedanceParams p;
  p.gain_ramp_s = 0.0;
  p.ik.q_rest = sample_q();
  p.ik.q_rest[2] = 3.14;                        // the shipped default: on the boundary
  p.ik.posture_gain = 1.0;                      // drive the posture term hard
  JointImpedanceMode m(dyn, p);
  JointFeedback fb; fb.q = sample_q(); fb.qd.setZero();
  m.on_enter(fb);
  for (int k = 0; k < 20000; ++k) { JointCommand c; m.compute(fb, 0.001, c); }

  const JointVec q_ref = m.reference();
  EXPECT_FALSE(q_ref.hasNaN());
  for (int i : {0, 2, 4, 6}) {                  // the continuous joints
    EXPECT_LE(std::abs(q_ref[i]), M_PI + 1e-9) << "joint index " << i;
  }
}

TEST(JointImpedance, DampingDerivedFromInertiaAndStiffness) {
  // Dq_i = 2*zeta*sqrt(Kq_i * M_ii(q)). Verified against an independently
  // computed mass matrix, and cross-checked through the actual torque output.
  Dynamics dyn(URDF_PATH);
  JointImpedanceParams p = static_params();
  p.zeta = 0.7;
  JointImpedanceMode m(dyn, p);
  JointFeedback fb; fb.q = sample_q(); fb.qd.setConstant(0.1);
  m.on_enter(fb);                                  // q_d := fb.q -> spring term is 0
  JointCommand c; m.compute(fb, 0.001, c);

  JointMat M; dyn.mass_matrix(fb.q, M);
  JointVec g; dyn.gravity(fb.q, g);
  for (int i = 0; i < kNumJoints; ++i) {
    const double expected = 2.0 * p.zeta * std::sqrt(p.Kq[i] * M(i, i));
    EXPECT_NEAR(m.last_damping()[i], expected, 1e-12) << "joint " << i;
    // Spring error is zero here, so the torque is exactly gravity minus damping.
    EXPECT_NEAR(c.torque[i], g[i] - expected * fb.qd[i], 1e-9) << "joint " << i;
  }
}

TEST(JointImpedance, DampingTracksConfigurationNotJustGains) {
  // The whole point of deriving damping: a flat Dq cannot be right at more than
  // one configuration. Joint 1's effective inertia swings ~38x between extended
  // and elbow-up, so the applied damping must differ substantially too.
  Dynamics dyn(URDF_PATH);
  JointImpedanceMode m1(dyn, static_params()), m2(dyn, static_params());

  JointFeedback extended; extended.q.setZero(); extended.qd.setConstant(0.1);
  JointFeedback folded; folded.qd.setConstant(0.1);
  folded.q << 0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57;

  JointCommand c;
  m1.on_enter(extended); m1.compute(extended, 0.001, c);
  m2.on_enter(folded);   m2.compute(folded, 0.001, c);

  EXPECT_GT(m2.last_damping()[0], 3.0 * m1.last_damping()[0])
      << "damping must follow the configuration, not stay constant";
}

TEST(JointImpedance, ZetaScalesDampingLinearly) {
  Dynamics dyn(URDF_PATH);
  JointImpedanceParams a = static_params(); a.zeta = 0.5;
  JointImpedanceParams b = static_params(); b.zeta = 1.0;   // critically damped
  JointImpedanceMode ma(dyn, a), mb(dyn, b);
  JointFeedback fb; fb.q = sample_q(); fb.qd.setConstant(0.1);
  JointCommand c;
  ma.on_enter(fb); ma.compute(fb, 0.001, c);
  mb.on_enter(fb); mb.compute(fb, 0.001, c);
  for (int i = 0; i < kNumJoints; ++i) {
    EXPECT_NEAR(mb.last_damping()[i], 2.0 * ma.last_damping()[i], 1e-12)
        << "joint " << i;
  }
}
