#include "kinova_lowlevel/momentum_observer.h"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "fixtures.h"
using namespace kinova;
using test_fixtures::Sinusoid;
using test_fixtures::synth_tau;

namespace {
constexpr double kDt = 1e-3;
constexpr double kGain = 25.0;
}  // namespace

TEST(MomentumObserver, RestConvergesToInjectedTorque) {
  Dynamics dyn(URDF_PATH);
  MomentumObserver obs(dyn, kGain);
  Sinusoid s;
  const JointVec q = s.q0, qd = JointVec::Zero();
  JointVec tau_e;
  tau_e << 0, 3, 0, 2, 0, 0.5, 0;
  JointVec g;
  dyn.gravity(q, g);
  const JointVec tau = g - tau_e;
  const int n = static_cast<int>(5.0 / kGain / kDt);
  for (int k = 0; k <= n; ++k) obs.step(q, qd, tau, kDt);
  for (int j = 0; j < kNumJoints; ++j)
    EXPECT_NEAR(obs.residual()[j], tau_e[j], std::max(0.01 * std::abs(tau_e[j]), 1e-3)) << j;
  EXPECT_NEAR(obs.time_since_reset(), n * kDt, 1e-9);
}

TEST(MomentumObserver, FirstStepSeedsAndResetReseeds) {
  Dynamics dyn(URDF_PATH);
  MomentumObserver obs(dyn, kGain);
  Sinusoid s;
  JointVec g;
  dyn.gravity(s.q0, g);
  EXPECT_EQ(obs.step(s.q0, JointVec::Zero(), g - JointVec::Constant(5.0), kDt).norm(), 0.0);
  for (int k = 0; k < 100; ++k) obs.step(s.q0, JointVec::Zero(), g - JointVec::Constant(5.0), kDt);
  EXPECT_GT(obs.residual().norm(), 1.0);
  obs.reset();
  EXPECT_EQ(obs.time_since_reset(), 0.0);
  EXPECT_EQ(obs.step(s.q0, JointVec::Zero(), g, kDt).norm(), 0.0);
}

TEST(MomentumObserver, MotionWithoutContactStaysNearZero) {
  Dynamics dyn(URDF_PATH);
  MomentumObserver obs(dyn, kGain);
  Sinusoid s;
  double max_r = 0.0, max_quasi_j2 = 0.0;
  for (int k = 0; k < 4000; ++k) {
    const double t = k * kDt;
    const JointVec tau = synth_tau(dyn, s, t, JointVec::Zero());
    obs.step(s.q(t), s.qd(t), tau, kDt);
    if (t < 0.5) continue;  // settle
    max_r = std::max(max_r, obs.residual().cwiseAbs().maxCoeff());
    JointVec g;
    dyn.gravity(s.q(t), g);
    max_quasi_j2 = std::max(max_quasi_j2, std::abs(g[1] - tau[1]));
  }
  EXPECT_LT(max_r, 0.05);
  EXPECT_GT(max_quasi_j2, 1.0);
}

TEST(MomentumObserver, PushDuringMotionRecovered) {
  Dynamics dyn(URDF_PATH);
  MomentumObserver obs(dyn, kGain);
  Sinusoid s;
  const double t_check = 2.0 + 5.0 / kGain;
  bool checked = false;
  for (int k = 0; k < 4000; ++k) {
    const double t = k * kDt;
    JointVec tau_e = JointVec::Zero();
    if (t >= 2.0 && t < 2.5) tau_e[3] = 4.0;
    obs.step(s.q(t), s.qd(t), synth_tau(dyn, s, t, tau_e), kDt);
    if (!checked && t >= t_check) {
      checked = true;
      for (int j = 0; j < kNumJoints; ++j) {
        const double expect = j == 3 ? 4.0 : 0.0;
        EXPECT_NEAR(obs.residual()[j], expect, j == 3 ? 0.2 : 0.1) << j;
      }
    }
  }
  EXPECT_TRUE(checked);
}

TEST(MomentumObserver, RejectsBadGain) {
  Dynamics dyn(URDF_PATH);
  EXPECT_THROW(MomentumObserver(dyn, 0.0), std::invalid_argument);
  EXPECT_THROW(MomentumObserver(dyn, -1.0), std::invalid_argument);
  EXPECT_THROW(MomentumObserver(dyn, std::nan("")), std::invalid_argument);
}
