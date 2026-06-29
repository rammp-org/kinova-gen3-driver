#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include "kinova_lowlevel/cartesian_impedance_mode.h"
using namespace kinova;

namespace {
JointVec sample_q() {
  JointVec q; q << 0.1, 0.3, -0.2, 0.8, 0.5, -0.4, 0.2; return q;
}
}  // namespace

TEST(CartesianImpedance, RequiresTorqueAndPassesThroughPosition) {
  Dynamics dyn(URDF_PATH);
  CartesianImpedanceParams p; p.gain_ramp_s = 0.0; p.nullspace_on = false;
  CartesianImpedanceMode m(dyn, p);
  JointFeedback fb; fb.q = sample_q(); fb.qd.setZero();
  for (auto x : m.required_modes()) EXPECT_EQ(x, ActuatorMode::kTorque);
  JointCommand c; m.on_enter(fb); m.compute(fb, 0.001, c);
  EXPECT_EQ(c.mode, ActuatorMode::kTorque);
  EXPECT_NEAR((c.position - fb.q).norm(), 0.0, 1e-12);
}

TEST(CartesianImpedance, AtTargetZeroVelGivesGravityOnly) {
  Dynamics dyn(URDF_PATH);
  CartesianImpedanceParams p; p.gain_ramp_s = 0.0; p.nullspace_on = false;
  CartesianImpedanceMode m(dyn, p);
  JointFeedback fb; fb.q = sample_q(); fb.qd.setZero();
  m.on_enter(fb);                       // target := fk(fb.q)  -> zero error
  JointCommand c; m.compute(fb, 0.001, c);
  JointVec g; dyn.gravity(fb.q, g);
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(c.torque[i], g[i], 1e-6);
}

TEST(CartesianImpedance, MatchesIndependentlyComputedLaw) {
  Dynamics dyn(URDF_PATH);
  CartesianImpedanceParams p; p.gain_ramp_s = 0.0; p.nullspace_on = false;
  CartesianImpedanceMode m(dyn, p);
  JointFeedback fb; fb.q = sample_q(); fb.qd.setConstant(0.05);
  m.on_enter(fb);
  Pose tgt = dyn.fk(fb.q); tgt.p.x() += 0.05; tgt.p.z() -= 0.03;   // displace
  m.set_target(tgt);
  JointCommand c; m.compute(fb, 0.001, c);

  Jacobian6 J; dyn.jacobian(fb.q, J);
  Vector6 e = pose_error(tgt, dyn.fk(fb.q));
  Vector6 xd = J * fb.qd;
  Vector6 F = p.Kx.cwiseProduct(e) - p.Dx.cwiseProduct(xd);
  JointVec g; dyn.gravity(fb.q, g);
  JointVec expected = g + J.transpose() * F;
  for (int i = 0; i < kNumJoints; ++i)
    expected[i] = std::clamp(expected[i], -p.torque_limit, p.torque_limit);
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(c.torque[i], expected[i], 1e-6);
}

TEST(CartesianImpedance, TorqueIsClamped) {
  Dynamics dyn(URDF_PATH);
  CartesianImpedanceParams p; p.gain_ramp_s = 0.0; p.nullspace_on = false;
  p.torque_limit = 2.0; p.Kx.setConstant(5000);
  CartesianImpedanceMode m(dyn, p);
  JointFeedback fb; fb.q = sample_q(); fb.qd.setZero();
  m.on_enter(fb);
  Pose tgt = dyn.fk(fb.q); tgt.p += Eigen::Vector3d(0.3, 0.3, 0.3);  // big error
  m.set_target(tgt);
  JointCommand c; m.compute(fb, 0.001, c);
  for (int i = 0; i < kNumJoints; ++i) EXPECT_LE(std::abs(c.torque[i]), 2.0 + 1e-9);
}

TEST(CartesianImpedanceNullspace, ProjectorAnnihilatesTaskSpace) {
  // Verify the projector FORMULA N = I - Jᵀ (Jᵀ)⁺ exactly annihilates task space
  // and is idempotent. This uses the UNDAMPED pseudo-inverse so the projector is
  // exact (sample_q is non-singular, so J Jᵀ is well-conditioned). The live mode
  // adds a small pinv_damping for singularity robustness — that approximate path
  // is validated separately by PostureTorqueProducesNoTaskWrench (J·dtau ≈ 0).
  Dynamics dyn(URDF_PATH);
  JointVec q = sample_q();
  Jacobian6 J; dyn.jacobian(q, J);
  Eigen::Matrix<double,6,6> JJt = J * J.transpose();             // no damping: exact projector
  Eigen::Matrix<double,6,kNumJoints> JtPinv = JJt.ldlt().solve(J);   // (JJt)^-1 J
  Eigen::Matrix<double,kNumJoints,kNumJoints> N =
      Eigen::Matrix<double,kNumJoints,kNumJoints>::Identity() - J.transpose() * JtPinv;
  EXPECT_NEAR((J * N).norm(), 0.0, 1e-9);          // task rows killed (exact)
  EXPECT_NEAR((N * N - N).norm(), 0.0, 1e-9);      // idempotent (exact)
}

TEST(CartesianImpedanceNullspace, NoEffectAtRestPostureZeroVel) {
  // At q == q_rest (entry config) with zero velocity, the posture torque is 0,
  // so nullspace_on and nullspace_off must agree.
  Dynamics dyn(URDF_PATH);
  JointFeedback fb; fb.q = sample_q(); fb.qd.setZero();

  CartesianImpedanceParams off; off.gain_ramp_s = 0.0; off.nullspace_on = false;
  CartesianImpedanceMode m_off(dyn, off);
  m_off.on_enter(fb);
  JointCommand c_off; m_off.compute(fb, 0.001, c_off);

  CartesianImpedanceParams on = off; on.nullspace_on = true;
  CartesianImpedanceMode m_on(dyn, on);
  m_on.on_enter(fb);
  JointCommand c_on; m_on.compute(fb, 0.001, c_on);

  EXPECT_NEAR((c_on.torque - c_off.torque).norm(), 0.0, 1e-9);
}

TEST(CartesianImpedanceNullspace, PostureTorqueProducesNoTaskWrench) {
  // With nonzero posture error, the EXTRA torque from the nullspace term must lie
  // in null(J): J * (tau_on - tau_off) ≈ 0.  Disable the task term by putting the
  // arm exactly at target (zero error) so the only difference is the posture term.
  Dynamics dyn(URDF_PATH);
  CartesianImpedanceParams off; off.gain_ramp_s = 0.0; off.nullspace_on = false;
  CartesianImpedanceMode m_off(dyn, off);
  CartesianImpedanceParams on = off; on.nullspace_on = true; on.nullspace_kp = 8.0;
  CartesianImpedanceMode m_on(dyn, on);

  JointFeedback enter; enter.q = sample_q(); enter.qd.setZero();
  m_off.on_enter(enter); m_on.on_enter(enter);            // q_rest := sample_q()

  JointFeedback fb; fb.q = sample_q(); fb.q[2] += 0.15;   // posture error, off target
  fb.qd.setZero();
  // keep both at "their target == fk(displaced q)" so the task error is zero:
  m_off.set_target(dyn.fk(fb.q));
  m_on.set_target(dyn.fk(fb.q));

  JointCommand c_off, c_on;
  m_off.compute(fb, 0.001, c_off);
  m_on.compute(fb, 0.001, c_on);

  Jacobian6 J; dyn.jacobian(fb.q, J);
  JointVec dtau = c_on.torque - c_off.torque;
  EXPECT_GT(dtau.norm(), 1e-3);                  // the posture term is actually active
  EXPECT_NEAR((J * dtau).norm(), 0.0, 1e-5);     // ...and produces no task wrench
}

TEST(CartesianImpedanceNullspace, FixedRestPostureActiveAndInNullspace) {
  // With nullspace_use_fixed_rest, q_rest is the FIXED pose, NOT the entry config.
  // We enter AND compute at the entry config: legacy (entry-anchored) would give
  // zero posture error here, so a nonzero posture torque proves the fixed-rest path
  // took effect. Offset stays modest so the DAMPED projector's residual (set by
  // pinv_damping) keeps the leaked task wrench under tol — see the comment on
  // PostureTorqueProducesNoTaskWrench.
  Dynamics dyn(URDF_PATH);
  JointVec rest = sample_q(); rest[2] += 0.15; rest[5] -= 0.12;  // fixed rest != entry

  CartesianImpedanceParams off; off.gain_ramp_s = 0.0; off.nullspace_on = false;
  CartesianImpedanceMode m_off(dyn, off);
  CartesianImpedanceParams on = off;
  on.nullspace_on = true; on.nullspace_use_fixed_rest = true;
  on.nullspace_q_rest = rest; on.nullspace_kp = 8.0; on.nullspace_kd = 0.0;
  CartesianImpedanceMode m_on(dyn, on);

  JointFeedback fb; fb.q = sample_q(); fb.qd.setZero();   // enter & compute at entry
  m_off.on_enter(fb); m_on.on_enter(fb);                  // q_rest := rest (on), q (off)
  m_off.set_target(dyn.fk(fb.q)); m_on.set_target(dyn.fk(fb.q));  // zero task error

  JointCommand c_off, c_on;
  m_off.compute(fb, 0.001, c_off);
  m_on.compute(fb, 0.001, c_on);

  Jacobian6 J; dyn.jacobian(fb.q, J);
  JointVec dtau = c_on.torque - c_off.torque;
  EXPECT_GT(dtau.norm(), 1e-3);                  // posture toward fixed home is active
  EXPECT_NEAR((J * dtau).norm(), 0.0, 1e-5);     // ...and produces no task wrench
}

TEST(CartesianImpedanceNullspace, ManipulabilityGradientActiveAndInNullspace) {
  // The manipulability gradient term is nonzero at a generic config and, being
  // applied through the projector N, produces no task-space wrench.
  Dynamics dyn(URDF_PATH);
  CartesianImpedanceParams off; off.gain_ramp_s = 0.0; off.nullspace_on = false;
  CartesianImpedanceMode m_off(dyn, off);
  CartesianImpedanceParams on = off;
  on.nullspace_on = true; on.nullspace_kp = 0.0; on.nullspace_kd = 0.0;
  on.manip_on = true; on.manip_gain = 1.0;
  CartesianImpedanceMode m_on(dyn, on);

  JointFeedback fb; fb.q = sample_q(); fb.qd.setZero();
  m_off.on_enter(fb); m_on.on_enter(fb);
  m_off.set_target(dyn.fk(fb.q)); m_on.set_target(dyn.fk(fb.q));  // zero task error

  JointCommand c_off, c_on;
  m_off.compute(fb, 0.001, c_off);
  m_on.compute(fb, 0.001, c_on);

  Jacobian6 J; dyn.jacobian(fb.q, J);
  JointVec dtau = c_on.torque - c_off.torque;
  EXPECT_GT(dtau.norm(), 1e-6);                  // gradient-ascent term is active
  EXPECT_NEAR((J * dtau).norm(), 0.0, 1e-5);     // ...and stays in null(J)
}

TEST(CartesianImpedanceRamp, GravityHeldFullDuringRampButWrenchScales) {
  Dynamics dyn(URDF_PATH);
  CartesianImpedanceParams p; p.gain_ramp_s = 1.0; p.nullspace_on = false;
  CartesianImpedanceMode m(dyn, p);
  JointFeedback fb; fb.q = sample_q(); fb.qd.setZero();
  m.on_enter(fb);
  Pose tgt = dyn.fk(fb.q); tgt.p.x() += 0.05; m.set_target(tgt);

  JointVec g; dyn.gravity(fb.q, g);
  Jacobian6 J; dyn.jacobian(fb.q, J);
  Vector6 e = pose_error(tgt, dyn.fk(fb.q));
  JointVec wrench = J.transpose() * (p.Kx.cwiseProduct(e));   // full active term, qd=0

  // First cycle: elapsed=0 -> ramp 0 -> torque == gravity only.
  JointCommand c0; m.compute(fb, 0.25, c0);
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(c0.torque[i], g[i], 1e-6);

  // After ~0.25s accumulated: ramp ≈ 0.25 -> active term partially applied.
  JointCommand c1; m.compute(fb, 0.25, c1);
  JointVec expected1 = g + 0.25 * wrench;
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(c1.torque[i], expected1[i], 1e-5);
}

TEST(CartesianImpedanceRamp, ZeroRampMeansFullImmediately) {
  Dynamics dyn(URDF_PATH);
  CartesianImpedanceParams p; p.gain_ramp_s = 0.0; p.nullspace_on = false;
  CartesianImpedanceMode m(dyn, p);
  JointFeedback fb; fb.q = sample_q(); fb.qd.setZero();
  m.on_enter(fb);
  Pose tgt = dyn.fk(fb.q); tgt.p.x() += 0.05; m.set_target(tgt);
  JointCommand c; m.compute(fb, 0.001, c);
  JointVec g; dyn.gravity(fb.q, g);
  Jacobian6 J; dyn.jacobian(fb.q, J);
  Vector6 e = pose_error(tgt, dyn.fk(fb.q));
  JointVec expected = g + J.transpose() * (p.Kx.cwiseProduct(e));
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(c.torque[i], expected[i], 1e-5);
}
