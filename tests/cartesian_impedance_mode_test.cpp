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
