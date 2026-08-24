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
  p.max_ref_speed.setConstant(1e9);
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
  p.max_ref_speed.setConstant(0.5);                   // rad/s
  JointImpedanceMode m(dyn, p);
  JointFeedback fb; fb.q = sample_q(); fb.qd.setZero();
  m.on_enter(fb);
  Pose far = dyn.fk(fb.q); far.p += Eigen::Vector3d(0.4, 0.3, -0.2);
  m.set_target(far);

  const JointVec before = m.reference();
  JointCommand c; m.compute(fb, 0.001, c);
  const double moved = (m.reference() - before).lpNorm<Eigen::Infinity>();
  EXPECT_GT(moved, 0.0);                                 // it did move toward the target
  EXPECT_LE(moved, 0.5 * 0.001 + 1e-12);                 // ...but no faster than allowed
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

TEST(JointImpedance, RefSpeedSeededFromUrdfVelocityLimits) {
  // Left at its non-finite default, the reference rate cap must come from the
  // URDF rather than a hand-picked constant: a cap below hand speed accumulates
  // lag that only unwinds when the operator slows down, which reads as mush.
  Dynamics dyn(URDF_PATH);
  JointImpedanceParams p;
  p.gain_ramp_s = 0.0;
  JointImpedanceMode m(dyn, p);
  JointVec v_urdf; dyn.velocity_limits(v_urdf);

  JointFeedback fb; fb.q = sample_q(); fb.qd.setZero();
  m.on_enter(fb);
  Pose far = dyn.fk(fb.q); far.p += Eigen::Vector3d(1.0, 1.0, 1.0);  // pull hard
  m.set_target(far);

  const double dt = 0.001;
  for (int k = 0; k < 50; ++k) {
    const JointVec before = m.reference();
    JointCommand c; m.compute(fb, dt, c);
    const JointVec step = m.reference() - before;
    for (int i = 0; i < kNumJoints; ++i) {
      EXPECT_LE(std::abs(step[i]), v_urdf[i] * dt + 1e-12) << "joint " << i;
    }
  }
  // ...and the cap is the URDF value, not the old hardcoded 1.0 rad/s.
  EXPECT_GT(v_urdf[0], 1.0);
}

// --- Direct joint-target seam (JointTargetSink) -----------------------------

TEST(JointImpedance, JointTargetDrivesReferenceDirectlyBypassingIk) {
  Dynamics dyn(URDF_PATH);
  JointImpedanceMode m(dyn, static_params());
  JointFeedback fb; fb.q = sample_q(); fb.qd.setZero();
  m.on_enter(fb);

  JointVec q_cmd = sample_q();
  q_cmd[0] += 0.05; q_cmd[3] -= 0.07;          // small joint move, within the spring leash
  m.set_target(q_cmd);                          // JointTargetSink overload -> IK bypassed

  JointCommand c; m.compute(fb, 0.001, c);
  // The reference is the commanded joint config EXACTLY — an IK solve of some pose
  // could not reproduce an arbitrary q_cmd like this, so this pins the direct path.
  EXPECT_NEAR((m.reference() - q_cmd).norm(), 0.0, 1e-12);

  // Torque is the joint spring about q_cmd plus gravity (qd = 0, ramp = 1).
  JointVec g; dyn.gravity(fb.q, g);
  const JointImpedanceParams p = static_params();
  for (int i = 0; i < kNumJoints; ++i) {
    const double e =
        std::clamp(q_cmd[i] - fb.q[i], -p.max_tracking_error, p.max_tracking_error);
    EXPECT_NEAR(c.torque[i], g[i] + p.Kq[i] * e, 1e-9) << "joint " << i;
  }
}

// --- Feedforward from a planner profile -------------------------------------

// Without a reference velocity the damper pulls against the arm precisely
// because it is moving as commanded. Feeding qd_ref forward damps the velocity
// ERROR instead, which is what removes the standing tracking lag.
TEST(JointImpedance, ReferenceVelocityIsFedForwardOnlyWhenTheTargetCarriesOne) {
  Dynamics dyn(URDF_PATH);
  JointFeedback fb; fb.q = sample_q();
  fb.qd.setConstant(0.4);                       // the arm is genuinely moving

  // A realistic reference speed: at static_params()'s effectively-unlimited cap
  // a one-cycle step would be tens of rad/s, and the damping torque would just
  // saturate torque_limit, hiding the term under test.
  const double kRefSpeed = 0.5;                 // rad/s
  JointImpedanceParams p = static_params();
  p.max_ref_speed.setConstant(kRefSpeed);

  JointVec q_cmd = sample_q();
  q_cmd[0] += 0.05; q_cmd[3] -= 0.07;           // far enough that the limiter saturates
  const double dt = 0.001;

  // A: position-only target — the pre-existing behaviour.
  JointImpedanceMode a(dyn, p);
  a.on_enter(fb);
  a.set_target(q_cmd);
  JointCommand ca; a.compute(fb, dt, ca);
  for (int i = 0; i < kNumJoints; ++i)
    EXPECT_NEAR(a.last_ref_velocity()[i], 0.0, 1e-12) << "no profile -> no feedforward";

  // B: the same move, but the target carries a velocity profile.
  JointImpedanceMode b(dyn, p);
  b.on_enter(fb);
  JointTarget t; t.q = q_cmd; t.has_velocity = true;
  b.set_joint_target(t);
  JointCommand cb; b.compute(fb, dt, cb);

  // Both joints that move are past the cap, so the reference travels at exactly
  // max_ref_speed, signed toward the target; the rest hold still.
  JointVec expect_qd_ref = JointVec::Zero();
  expect_qd_ref[0] = +kRefSpeed;
  expect_qd_ref[3] = -kRefSpeed;
  for (int i = 0; i < kNumJoints; ++i)
    EXPECT_NEAR(b.last_ref_velocity()[i], expect_qd_ref[i], 1e-6) << "joint " << i;

  // Same spring, same gravity; the only difference is the damping term.
  for (int i = 0; i < kNumJoints; ++i)
    EXPECT_NEAR(cb.torque[i] - ca.torque[i], b.last_damping()[i] * expect_qd_ref[i], 1e-6)
        << "joint " << i;
}

// The teleop / Cartesian path has no profile and must be untouched.
TEST(JointImpedance, PoseTargetPathFeedsNothingForward) {
  Dynamics dyn(URDF_PATH);
  JointImpedanceMode m(dyn, static_params());
  JointFeedback fb; fb.q = sample_q(); fb.qd.setConstant(0.3);
  m.on_enter(fb);
  m.set_target(dyn.fk(sample_q()));
  JointCommand c; m.compute(fb, 0.001, c);
  for (int i = 0; i < kNumJoints; ++i)
    EXPECT_NEAR(m.last_ref_velocity()[i], 0.0, 1e-12) << "joint " << i;
}

// Inertial feedforward: the torque the planned acceleration needs, so the
// spring is not left to produce it out of tracking error.
TEST(JointImpedance, AccelerationFeedforwardAddsMassMatrixTimesQdd) {
  Dynamics dyn(URDF_PATH);
  JointFeedback fb; fb.q = sample_q(); fb.qd.setZero();
  const double dt = 0.001;

  JointVec qdd; qdd << 0.5, -0.3, 0.2, 0.7, -0.1, 0.4, -0.6;

  // Target the CURRENT configuration: no spring error, no reference motion, so
  // the acceleration term is the only thing left in the torque.
  JointImpedanceMode base(dyn, static_params());
  base.on_enter(fb);
  JointTarget t0; t0.q = fb.q; t0.has_velocity = true;
  base.set_joint_target(t0);
  JointCommand c0; base.compute(fb, dt, c0);

  JointImpedanceMode with_acc(dyn, static_params());
  with_acc.on_enter(fb);
  JointTarget t1; t1.q = fb.q; t1.qdd = qdd;
  t1.has_velocity = true; t1.has_acceleration = true;
  with_acc.set_joint_target(t1);
  JointCommand c1; with_acc.compute(fb, dt, c1);

  JointMat M; dyn.mass_matrix(fb.q, M);
  const JointVec expected = M * qdd;
  for (int i = 0; i < kNumJoints; ++i)
    EXPECT_NEAR(c1.torque[i] - c0.torque[i], expected[i], 1e-9) << "joint " << i;
}

// Accelerations alone cannot select the inertial term: both Hermite forms need
// velocities, and honouring qdd without qd would be a half-applied profile.
TEST(JointImpedance, AccelerationWithoutVelocityIsIgnored) {
  Dynamics dyn(URDF_PATH);
  JointFeedback fb; fb.q = sample_q(); fb.qd.setZero();
  JointImpedanceMode base(dyn, static_params());
  base.on_enter(fb);
  base.set_target(fb.q);
  JointCommand c0; base.compute(fb, 0.001, c0);

  JointImpedanceMode odd(dyn, static_params());
  odd.on_enter(fb);
  JointTarget t; t.q = fb.q; t.qdd.setConstant(1.0);
  t.has_velocity = false; t.has_acceleration = true;   // incoherent: ignore it
  odd.set_joint_target(t);
  JointCommand c1; odd.compute(fb, 0.001, c1);

  for (int i = 0; i < kNumJoints; ++i)
    EXPECT_NEAR(c1.torque[i], c0.torque[i], 1e-12) << "joint " << i;
}

TEST(JointImpedance, JointTargetSupersedesPoseTarget) {
  Dynamics dyn(URDF_PATH);
  JointImpedanceMode m(dyn, static_params());
  JointFeedback fb; fb.q = sample_q(); fb.qd.setZero();
  m.on_enter(fb);
  m.set_target(dyn.fk(sample_q()));            // Cartesian target first...
  JointVec q_cmd = sample_q(); q_cmd[2] += 0.06;
  m.set_target(q_cmd);                          // ...then a joint target: latest wins
  JointCommand c; m.compute(fb, 0.001, c);
  EXPECT_NEAR((m.reference() - q_cmd).norm(), 0.0, 1e-12);
}
