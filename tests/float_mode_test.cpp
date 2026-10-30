#include "kinova_lowlevel/float_mode.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "kinova_lowlevel/units.h"
using namespace kinova;

namespace {
JointFeedback pose() {
  JointFeedback fb;
  fb.q << 0.1, 0.5, -0.2, 1.0, 0.3, -0.4, 0.2;
  fb.qd.setZero();
  return fb;
}
}  // namespace

TEST(FloatMode, AtRestCommandsGravityOnly) {
  Dynamics dyn(URDF_PATH);
  FloatMode m(dyn);
  for (auto x : m.required_modes()) EXPECT_EQ(x, ActuatorMode::kTorque);
  JointFeedback fb = pose();
  JointCommand c;
  m.on_enter(fb);
  m.compute(fb, 0.001, c);
  JointVec g;
  dyn.gravity(fb.q, g);
  EXPECT_EQ(c.mode, ActuatorMode::kTorque);
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(c.torque[i], g[i], 1e-9);
}

TEST(FloatMode, SpringForceBoundedByBand) {
  Dynamics dyn(URDF_PATH);
  FloatParams p;
  p.torque_limit.setConstant(1e9);
  FloatMode m(dyn, p);
  JointFeedback fb = pose();
  JointCommand c;
  m.on_enter(fb);
  const double q0 = fb.q[1];
  fb.q[1] += 10.0 * kDeg2Rad;
  m.compute(fb, 0.001, c);
  JointVec g;
  dyn.gravity(fb.q, g);
  EXPECT_LE(std::abs(c.torque[1] - g[1]), p.spring[1] * p.drag_band_rad + 1e-9);
  EXPECT_NEAR(m.anchor()[1], fb.q[1] - p.drag_band_rad, 1e-12);
  EXPECT_NE(m.anchor()[1], q0);
}

TEST(FloatMode, StaysWhereLeft) {
  Dynamics dyn(URDF_PATH);
  FloatParams p;
  FloatMode m(dyn, p);
  JointFeedback fb = pose();
  JointCommand c;
  m.on_enter(fb);
  const double trip = fb.q[2];
  for (int k = 1; k <= 100; ++k) {  // drag 10 deg in steps
    fb.q[2] = trip + k * 0.1 * kDeg2Rad;
    m.compute(fb, 0.001, c);
  }
  m.compute(fb, 0.001, c);  // arm left at the final pose
  EXPECT_LE(std::abs(fb.q[2] - m.anchor()[2]), p.drag_band_rad + 1e-12);
  // The restoring force is toward the final pose's anchor, not the trip pose.
  JointVec g;
  dyn.gravity(fb.q, g);
  EXPECT_LE(std::abs(c.torque[2] - g[2]), p.spring[2] * p.drag_band_rad + 1e-9);
}

TEST(FloatMode, DampingOpposesVelocity) {
  Dynamics dyn(URDF_PATH);
  FloatParams p;
  FloatMode m(dyn, p);
  JointFeedback fb = pose();
  JointCommand c;
  m.on_enter(fb);
  fb.qd[3] = 1.0;
  m.compute(fb, 0.001, c);
  JointVec g;
  dyn.gravity(fb.q, g);
  EXPECT_NEAR(c.torque[3] - g[3], -p.damping[3], 1e-9);
}

TEST(FloatMode, ClampsToTorqueLimit) {
  Dynamics dyn(URDF_PATH);
  FloatParams p;
  p.torque_limit.setConstant(0.5);
  FloatMode m(dyn, p);
  JointFeedback fb = pose();
  JointCommand c;
  m.on_enter(fb);
  fb.qd.setConstant(10.0);
  m.compute(fb, 0.001, c);
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(c.torque[i], -0.5, 1e-12);
  fb.qd.setConstant(-10.0);
  m.compute(fb, 0.001, c);
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(c.torque[i], 0.5, 1e-12);
}

TEST(FloatMode, RejectsBadParams) {
  Dynamics dyn(URDF_PATH);
  static const double nan = std::numeric_limits<double>::quiet_NaN();
  auto bad = [&](auto mutate) {
    FloatParams p;
    mutate(p);
    EXPECT_THROW(FloatMode(dyn, p), std::invalid_argument);
  };
  bad([](FloatParams& p) { p.damping[0] = -1; });
  bad([](FloatParams& p) { p.damping[2] = nan; });
  bad([](FloatParams& p) { p.spring[1] = -0.1; });
  bad([&](FloatParams& p) { p.spring[6] = nan; });
  bad([](FloatParams& p) { p.drag_band_rad = 0; });
  bad([&](FloatParams& p) { p.drag_band_rad = nan; });
  bad([](FloatParams& p) { p.torque_limit[4] = 0; });
  bad([&](FloatParams& p) { p.torque_limit[0] = nan; });
  EXPECT_NO_THROW(FloatMode(dyn, FloatParams{}));
}

// j3 is continuous and the transport wraps q to (-pi, pi]; home has j3 = pi, so an
// encoder dithering across the wrap must not flip the anchor to the far side
// (each flip would be a K_s*band step of spring torque).
TEST(FloatMode, AnchorFollowsWrapOnContinuousJoint) {
  Dynamics dyn(URDF_PATH);
  FloatParams p;
  p.torque_limit.setConstant(1e9);
  FloatMode m(dyn, p);
  JointFeedback fb = pose();
  JointCommand c;
  JointVec g;
  fb.q[2] = M_PI - 0.001;
  m.on_enter(fb);
  for (int k = 0; k < 10; ++k) {
    // 2 mrad apart through the wrap: past the wrap is 2 mrad ahead of the anchor, so
    // the spring pulls back by K_s * 2 mrad (negative), never the far side's +K_s*band.
    const bool past = k % 2 == 0;
    fb.q[2] = past ? -M_PI + 0.001 : M_PI - 0.001;
    m.compute(fb, 0.001, c);
    dyn.gravity(fb.q, g);
    EXPECT_NEAR(c.torque[2] - g[2], past ? -p.spring[2] * 0.002 : 0.0, 1e-9) << k;
    EXPECT_NEAR(std::abs(m.anchor()[2]), M_PI - 0.001, 1e-9) << k;  // never dragged
  }
  // A real drag through the wrap: the anchor follows across it, the band still holds.
  for (int k = 1; k <= 100; ++k) {
    fb.q[2] = wrap_to_pi(M_PI - 0.001 + k * 0.1 * kDeg2Rad);
    m.compute(fb, 0.001, c);
    dyn.gravity(fb.q, g);
    // The anchor trails q, so the spring only ever pulls back: <= 0, at most K_s*band.
    EXPECT_LE(c.torque[2] - g[2], 1e-12) << k;
    EXPECT_GE(c.torque[2] - g[2], -p.spring[2] * p.drag_band_rad - 1e-9) << k;
  }
  EXPECT_NEAR(wrap_to_pi(fb.q[2] - m.anchor()[2]), p.drag_band_rad, 1e-9);
}
