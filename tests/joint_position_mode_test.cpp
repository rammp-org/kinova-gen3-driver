#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include "kinova_lowlevel/joint_position_mode.h"
#include "kinova_lowlevel/units.h"
using namespace kinova;

namespace {

JointVec sample_q() {
  JointVec q; q << 0.1, 0.3, -0.2, 0.8, 0.5, -0.4, 0.2; return q;
}

// Index of the first joint the URDF declares continuous (both limits infinite).
// Discovered rather than hard-coded: the wrap bug this guards against is about
// the property, not about joint 3 specifically.
int first_continuous_joint(Dynamics& dyn) {
  JointVec lo, hi; dyn.joint_limits(lo, hi);
  for (int i = 0; i < kNumJoints; ++i)
    if (!std::isfinite(lo[i]) && !std::isfinite(hi[i])) return i;
  return -1;
}

// Index of the first joint with real (finite) position limits.
int first_bounded_joint(Dynamics& dyn) {
  JointVec lo, hi; dyn.joint_limits(lo, hi);
  for (int i = 0; i < kNumJoints; ++i)
    if (std::isfinite(lo[i]) && std::isfinite(hi[i])) return i;
  return -1;
}

// A perfect position servo: the arm lands exactly on the previous cycle's
// command. This is the optimistic case, and it is precisely what isolates the
// reference integrator from the leash — with zero following error the leash can
// never bind, so any clamping a test observes came from the rate limit or the
// joint limits.
struct PerfectServo {
  JointPositionMode& m;
  JointFeedback fb;
  JointCommand cmd;

  PerfectServo(JointPositionMode& mode, const JointVec& q0) : m(mode) {
    fb.q = q0;
    fb.qd.setZero();
    m.on_enter(fb);
  }
  void step(double dt = 0.001) {
    m.compute(fb, dt, cmd);
    fb.q = cmd.position;   // servo lands exactly on the command
  }
};

}  // namespace

TEST(JointPosition, RequiresPositionModeAndCommandsNoTorque) {
  Dynamics dyn(URDF_PATH);
  JointPositionMode m(dyn);
  for (auto x : m.required_modes()) EXPECT_EQ(x, ActuatorMode::kPosition);

  JointFeedback fb; fb.q = sample_q(); fb.qd.setZero();
  JointCommand c;
  m.on_enter(fb);
  m.compute(fb, 0.001, c);
  EXPECT_EQ(c.mode, ActuatorMode::kPosition);
  // Nothing may leak into the torque field: the actuator is in POSITION mode and
  // a stale torque value here would be silently ignored today and acted on the
  // moment anything reads it.
  EXPECT_NEAR(c.torque.norm(), 0.0, 1e-12);
}

TEST(JointPosition, HoldsMeasuredConfigBeforeAnyTarget) {
  Dynamics dyn(URDF_PATH);
  JointPositionMode m(dyn);
  JointFeedback fb; fb.q = sample_q(); fb.qd.setZero();
  JointCommand c;
  m.on_enter(fb);
  for (int k = 0; k < 50; ++k) m.compute(fb, 0.001, c);
  // No target was ever set: the arm must stay exactly where it was on entry.
  EXPECT_NEAR((c.position - sample_q()).norm(), 0.0, 1e-12);
}

TEST(JointPosition, ReferenceSeededAtMeasuredConfigOnEnter) {
  Dynamics dyn(URDF_PATH);
  JointPositionMode m(dyn);
  JointFeedback fb; fb.q = sample_q(); fb.qd.setZero();
  m.on_enter(fb);
  EXPECT_NEAR((m.reference() - sample_q()).norm(), 0.0, 1e-12);
}

TEST(JointPosition, AdvancesByExactlyMaxSpeedTimesDtTowardFarTarget) {
  Dynamics dyn(URDF_PATH);
  JointPositionParams p;
  p.max_ref_speed.setConstant(0.5);
  p.max_following_error = 0.0;          // disable the leash; isolate the rate limit
  JointPositionMode m(dyn, p);

  JointFeedback fb; fb.q.setZero(); fb.qd.setZero();
  m.on_enter(fb);
  JointVec target; target.setConstant(1.0);   // far relative to one cycle of travel
  m.set_target(target);

  JointCommand c;
  m.compute(fb, 0.001, c);
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(c.position[i], 0.0005, 1e-12);
}

TEST(JointPosition, RateLimitIsPerJoint) {
  Dynamics dyn(URDF_PATH);
  JointPositionParams p;
  p.max_ref_speed << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
  p.max_following_error = 0.0;
  JointPositionMode m(dyn, p);

  JointFeedback fb; fb.q.setZero(); fb.qd.setZero();
  m.on_enter(fb);
  JointVec target; target.setConstant(1.0);
  m.set_target(target);

  JointCommand c;
  m.compute(fb, 0.001, c);
  for (int i = 0; i < kNumJoints; ++i)
    EXPECT_NEAR(c.position[i], p.max_ref_speed[i] * 0.001, 1e-12);
}

TEST(JointPosition, CallerCannotRequestSpeedAboveTheUrdfLimit) {
  Dynamics dyn(URDF_PATH);
  JointVec v_max; dyn.velocity_limits(v_max);

  JointPositionParams p;
  p.max_ref_speed.setConstant(100.0);       // absurd; must be clamped to hardware
  JointPositionMode m(dyn, p);

  const JointVec effective = m.params().max_ref_speed;
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(effective[i], v_max[i], 1e-12);
}

TEST(JointPosition, NonFiniteSpeedIsSeededFromTheUrdfLimit) {
  Dynamics dyn(URDF_PATH);
  JointVec v_max; dyn.velocity_limits(v_max);

  JointPositionParams p;
  p.max_ref_speed.setConstant(std::numeric_limits<double>::infinity());
  JointPositionMode m(dyn, p);

  const JointVec effective = m.params().max_ref_speed;
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(effective[i], v_max[i], 1e-12);
}

TEST(JointPosition, CommandNeverExceedsTheUrdfJointLimit) {
  Dynamics dyn(URDF_PATH);
  const int j = first_bounded_joint(dyn);
  ASSERT_GE(j, 0) << "URDF has no joint with finite position limits";
  JointVec lo, hi; dyn.joint_limits(lo, hi);

  JointPositionParams p;
  p.max_ref_speed.setConstant(1.0);
  JointPositionMode m(dyn, p);

  PerfectServo servo(m, JointVec::Zero());
  JointVec target = JointVec::Zero();
  target[j] = hi[j] + 1.0;                  // ask for a full radian past the stop
  m.set_target(target);

  for (int k = 0; k < 5000; ++k) {
    servo.step();
    ASSERT_LE(servo.cmd.position[j], hi[j] + 1e-12)
        << "commanded past the joint limit at cycle " << k;
  }
  EXPECT_NEAR(servo.cmd.position[j], hi[j], 1e-9);   // parks exactly on the stop
}

// The j3 class of bug. Reference and measurement both live in (-pi, pi], so a
// target on the far side of the wrap reads as a ~2*pi error unless the
// difference is wrapped. Unwrapped, the reference walks the long way round —
// which on the real arm is the joint spinning most of a full turn.
TEST(JointPosition, ContinuousJointTakesTheShortWayRound) {
  Dynamics dyn(URDF_PATH);
  const int j = first_continuous_joint(dyn);
  ASSERT_GE(j, 0) << "URDF has no continuous joint";

  JointPositionParams p;
  p.max_ref_speed.setConstant(0.5);
  JointPositionMode m(dyn, p);

  JointVec q0 = JointVec::Zero(); q0[j] = 3.0;
  PerfectServo servo(m, q0);
  JointVec target = JointVec::Zero(); target[j] = -3.0;
  m.set_target(target);

  // First cycle must move in the +direction (up through pi), not down through 0.
  const double before = m.reference()[j];
  servo.step();
  EXPECT_GT(m.reference()[j], before) << "took the long way round the wrap";

  // Total path length is the honest oracle: the short way is
  // |wrap_to_pi(-3.0 - 3.0)| = 2*pi - 6 ~= 0.283 rad, the long way is ~6.0 rad.
  double travel = std::abs(wrap_to_pi(m.reference()[j] - before));
  double prev = m.reference()[j];
  for (int k = 0; k < 3000; ++k) {
    servo.step();
    travel += std::abs(wrap_to_pi(m.reference()[j] - prev));
    prev = m.reference()[j];
  }
  EXPECT_NEAR(travel, 2.0 * M_PI - 6.0, 1e-6);
  EXPECT_NEAR(wrap_to_pi(m.reference()[j] - (-3.0)), 0.0, 1e-9);
}

TEST(JointPosition, CommandedPositionStaysWrappedForContinuousJoints) {
  Dynamics dyn(URDF_PATH);
  const int j = first_continuous_joint(dyn);
  ASSERT_GE(j, 0);

  JointPositionParams p;
  p.max_ref_speed.setConstant(1.0);
  JointPositionMode m(dyn, p);

  JointVec q0 = JointVec::Zero(); q0[j] = 3.0;
  PerfectServo servo(m, q0);
  JointVec target = JointVec::Zero(); target[j] = -1.0;
  m.set_target(target);

  // Whatever the path, every commanded angle must stay in the same
  // representation the transport reports measurements in: (-pi, pi].
  for (int k = 0; k < 3000; ++k) {
    servo.step();
    ASSERT_GT(servo.cmd.position[j], -M_PI - 1e-12) << "cycle " << k;
    ASSERT_LE(servo.cmd.position[j], M_PI + 1e-12) << "cycle " << k;
  }
}

// A blocked arm must not let the reference march away. Without the leash the
// commanded position runs to the target while the arm sits still, and the arm
// then snaps across the accumulated gap the instant the obstruction clears.
TEST(JointPosition, FollowingErrorLeashStopsARunawayReference) {
  Dynamics dyn(URDF_PATH);
  JointPositionParams p;
  p.max_ref_speed.setConstant(1.0);
  p.max_following_error = 0.35;
  JointPositionMode m(dyn, p);

  JointFeedback fb; fb.q.setZero(); fb.qd.setZero();   // blocked: q never changes
  m.on_enter(fb);
  JointVec target; target.setConstant(1.0);
  m.set_target(target);

  JointCommand c;
  for (int k = 0; k < 5000; ++k) {
    m.compute(fb, 0.001, c);                            // fb.q stays at zero
    ASSERT_LE(c.position[0], 0.35 + 1e-12) << "reference ran away at cycle " << k;
  }
  EXPECT_NEAR(c.position[0], 0.35, 1e-9);               // parks at the leash
}

TEST(JointPosition, LeashIsDisabledByANonPositiveValue) {
  Dynamics dyn(URDF_PATH);
  JointPositionParams p;
  p.max_ref_speed.setConstant(1.0);
  p.max_following_error = 0.0;
  JointPositionMode m(dyn, p);

  JointFeedback fb; fb.q.setZero(); fb.qd.setZero();
  m.on_enter(fb);
  JointVec target; target.setConstant(1.0);
  m.set_target(target);

  JointCommand c;
  for (int k = 0; k < 2000; ++k) m.compute(fb, 0.001, c);
  EXPECT_NEAR(c.position[0], 1.0, 1e-9);   // reaches the target despite no motion
}

TEST(JointPosition, ConvergesExactlyOnTargetAndStaysThere) {
  Dynamics dyn(URDF_PATH);
  JointPositionParams p;
  p.max_ref_speed.setConstant(0.5);        // 0.0005 rad per 1 ms cycle
  JointPositionMode m(dyn, p);

  PerfectServo servo(m, JointVec::Zero());
  JointVec target; target.setConstant(0.001);   // exactly two cycles away
  m.set_target(target);

  servo.step();
  servo.step();
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(servo.cmd.position[i], 0.001, 1e-15);
  // No chatter: an integrator that overshoots and corrects would wobble here.
  for (int k = 0; k < 100; ++k) servo.step();
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(servo.cmd.position[i], 0.001, 1e-15);
}

TEST(JointPosition, LatestTargetWins) {
  Dynamics dyn(URDF_PATH);
  JointPositionParams p;
  p.max_ref_speed.setConstant(1.0);
  JointPositionMode m(dyn, p);

  PerfectServo servo(m, JointVec::Zero());
  JointVec a; a.setConstant(0.5);
  JointVec b; b.setConstant(-0.5);
  m.set_target(a);
  m.set_target(b);                          // published before any compute ran
  for (int k = 0; k < 2000; ++k) servo.step();
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(servo.cmd.position[i], -0.5, 1e-9);
}

TEST(JointPosition, SetParamsTakesEffectOnTheNextCycle) {
  Dynamics dyn(URDF_PATH);
  JointPositionParams p;
  p.max_ref_speed.setConstant(0.5);
  p.max_following_error = 0.0;
  JointPositionMode m(dyn, p);

  JointFeedback fb; fb.q.setZero(); fb.qd.setZero();
  m.on_enter(fb);
  JointVec target; target.setConstant(1.0);
  m.set_target(target);

  JointCommand c;
  m.compute(fb, 0.001, c);
  EXPECT_NEAR(c.position[0], 0.0005, 1e-12);

  p.max_ref_speed.setConstant(1.0);
  m.set_params(p);
  m.compute(fb, 0.001, c);
  EXPECT_NEAR(c.position[0], 0.0005 + 0.001, 1e-12);
}
