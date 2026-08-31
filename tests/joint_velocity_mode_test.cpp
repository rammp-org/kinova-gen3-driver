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
  // not let those fields survive, even on the early-return no-target path. The
  // position field ECHOES the measurement, as every other mode does -- zero there
  // is not "unset", it is "all joints at 0 rad".
  Dynamics dyn(URDF_PATH);
  JointVelocityMode m(dyn);
  const JointVec q = JointVec::Constant(0.3);
  m.on_enter(fb_at(q));
  JointCommand out;
  out.position = JointVec::Constant(1.0);
  out.torque = JointVec::Constant(1.0);
  m.compute(fb_at(q), 0.001, out);   // no target -> frozen/no-target path
  EXPECT_TRUE(out.position.isApprox(q, 1e-12));
  EXPECT_TRUE(out.torque.isZero());
}

TEST(JointVelocityMode, ClearsStalePositionAndTorqueOnTheTrackingPath) {
  Dynamics dyn(URDF_PATH);
  JointVelocityMode m(dyn);
  const JointVec q = JointVec::Constant(0.3);
  m.on_enter(fb_at(q));
  m.set_velocity_target(JointVec::Constant(0.2));
  JointCommand out;
  out.position = JointVec::Constant(1.0);
  out.torque = JointVec::Constant(1.0);
  m.compute(fb_at(q), 0.001, out);
  EXPECT_TRUE(out.position.isApprox(q, 1e-12));
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

TEST(JointVelocityModePosture, DrivesTheRedundantDofTowardQRestUnderAZeroTwist) {
  Dynamics dyn(URDF_PATH);
  JointVelocityParams p;
  p.posture_gain = 0.5;
  p.max_qd = JointVec::Constant(10.0);
  JointVelocityMode m(dyn, p);

  JointVec q = nominal_q();
  q[2] += 0.30;                         // perturb the redundant DOF off the rest posture
  m.on_enter(fb_at(q));
  m.set_twist_target(Vector6::Zero());  // hold the tool still

  JointCommand out;
  m.compute(fb_at(q), 0.001, out);
  // The posture term must push joint 2 back DOWN toward q_rest.
  EXPECT_LT(out.velocity[2], 0.0);
  EXPECT_GT(out.velocity.norm(), 1e-6);   // it is not simply doing nothing
}

TEST(JointVelocityModePosture, DoesNotDisturbTheCommandedTwist) {
  Dynamics dyn(URDF_PATH);
  JointVelocityParams p;
  p.posture_gain = 0.5;
  p.max_qd = JointVec::Constant(10.0);
  JointVelocityMode m(dyn, p);

  JointVec q = nominal_q();
  q[2] += 0.30;                         // a posture error the null-space term will fight
  m.on_enter(fb_at(q));
  Vector6 V = Vector6::Zero(); V[1] = 0.04;
  m.set_twist_target(V);
  JointCommand out;
  m.compute(fb_at(q), 0.001, out);

  Jacobian6 J; dyn.jacobian(q, J);
  const Vector6 achieved = J * out.velocity;
  // The whole point of a NULL-space term: posture correction lives where it
  // cannot show up in the task velocity.
  EXPECT_NEAR((achieved - V).norm(), 0.0, 1e-3);
}

TEST(JointVelocityModePosture, ConvergesTowardQRestOverRepeatedCycles) {
  Dynamics dyn(URDF_PATH);
  JointVelocityParams p;
  p.posture_gain = 0.5;
  p.max_qd = JointVec::Constant(10.0);
  JointVelocityMode m(dyn, p);

  JointVec q = nominal_q();
  q[2] += 0.30;
  const double err0 = std::abs(q[2] - p.q_rest[2]);
  m.on_enter(fb_at(q));

  JointCommand out;
  for (int i = 0; i < 500; ++i) {       // integrate the commanded velocity forward
    m.set_twist_target(Vector6::Zero());
    m.compute(fb_at(q), 0.001, out);
    q += out.velocity * 0.001;
  }
  EXPECT_LT(std::abs(q[2] - p.q_rest[2]), err0);
}

// Finding: the wrap guard in solve_twist was untested. The sibling test above
// perturbs q[2] to 3.44, which is ALREADY in (-pi, pi] only by accident of being
// unwrapped -- wrap_to_pi is a no-op on it and the guard never runs. On the arm,
// KortexTransport wraps every measured angle, so 3.44 arrives as -2.843: without
// the wrap, q_rest[2] - q[2] reads 5.983 instead of -0.300 and the posture bias
// drives joint 3 most of a turn the WRONG way at the limiter's cap.
TEST(JointVelocityModePosture, TakesTheShortWayOnAContinuousJointAcrossTheWrap) {
  Dynamics dyn(URDF_PATH);
  JointVelocityParams p;
  p.posture_gain = 0.5;
  p.max_qd = JointVec::Constant(10.0);
  JointVelocityMode m(dyn, p);

  // Feedback arrives wrapped to (-pi, pi], as the transport delivers it.
  JointVec q = nominal_q();
  q[2] = -2.843;                        // == 3.44 rad (q_rest[2] + 0.30), wrapped
  m.on_enter(fb_at(q));
  m.set_twist_target(Vector6::Zero());  // hold the tool still; isolate the posture term

  JointCommand out;
  m.compute(fb_at(q), 0.001, out);
  // Short way is -0.30 rad; the long way is +5.98 and saturates the limiter.
  EXPECT_LT(out.velocity[2], 0.0);
  EXPECT_LT(std::abs(out.velocity[2]), 1.0);
}


// A NEAR-singular configuration: the shoulder is 0.10 rad off straight, which puts
// manipulability at w=0.00031, well inside the w < w_threshold ramp. The exactly
// straight pose is useless for this: there the commanded direction lies in the
// null space of J^T, so the solve returns ~0 whatever the damping is.
namespace {
JointVec near_singular_q() {
  JointVec q = JointVec::Zero(); q[1] = 0.10; q[3] = -0.20; return q;
}
}  // namespace

// The damping SCHEDULE (the w < w_threshold ramp) had no test that it ever fires:
// StaysBoundedAtAStraightArmSingularity asserts a bound that limit()'s hard clamp
// guarantees whatever the DLS produced, and the nominal w=0.0325 never enters the
// ramp. This observes the damping's OWN effect instead -- it sacrifices tracking
// (that is what damping buys) and leaves the command far below the limiter's cap,
// so the bound observed here is the damping's and not the clamp's.
TEST(JointVelocityModeTwist, DampingNotTheLimiterIsWhatBoundsTheSolveNearASingularity) {
  Dynamics dyn(URDF_PATH);
  JointVelocityParams p;
  p.posture_gain = 0.0;                 // isolate the task term
  JointVelocityMode m(dyn, p);
  const JointVec q = near_singular_q();
  m.on_enter(fb_at(q));

  Vector6 V = Vector6::Zero();
  V[2] = 0.10;                          // along the near-degenerate direction
  m.set_twist_target(V);
  JointCommand out;
  m.compute(fb_at(q), 0.001, out);
  ASSERT_LT(m.last_manipulability(), p.w_threshold);   // the ramp branch really ran
  ASSERT_GT(m.last_manipulability(), 0.0);             // and it is not the trivial null case

  Jacobian6 J; dyn.jacobian(q, J);
  // Damping deliberately gives up tracking rather than producing an enormous qd.
  EXPECT_GT((J * out.velocity - V).norm(), 0.05);
  // And the result is small on its own account -- nowhere near the cap the limiter
  // would have applied (measured: max|qd| 0.31 rad/s against a 1.22 rad/s cap).
  JointVec v_max; dyn.velocity_limits(v_max);
  EXPECT_LT(out.velocity.cwiseAbs().maxCoeff(), 0.5 * v_max.minCoeff());
}

// Pins the schedule directly, and records the measurement behind the docs' claim
// about what actually bounds the command. dls_damping_max == dls_damping makes the
// ramp flat, i.e. the schedule disabled; any difference is the schedule alone.
TEST(JointVelocityModeTwist, TheDampingRampIsWhatSeparatesTheseTwoSolves) {
  Dynamics dyn(URDF_PATH);
  const JointVec q = near_singular_q();
  Vector6 V = Vector6::Zero(); V[2] = 0.10;
  JointVec v_max; dyn.velocity_limits(v_max);

  auto solve_with = [&](double dmax) {
    JointVelocityParams p;
    p.posture_gain = 0.0;
    p.dls_damping_max = dmax;
    JointVelocityMode m(dyn, p);
    m.on_enter(fb_at(q));
    m.set_twist_target(V);
    JointCommand out;
    m.compute(fb_at(q), 0.001, out);
    return out.velocity;
  };

  JointVelocityParams def;
  const JointVec flat   = solve_with(def.dls_damping);   // schedule disabled
  const JointVec ramped = solve_with(def.dls_damping_max);

  EXPECT_LT(ramped.norm(), 0.5 * flat.norm());
  // With the schedule off the solve runs straight into the LIMITER -- some joint
  // sits exactly on its URDF cap. That is the concrete evidence for what the
  // header says: limit() is the unconditional bound, damping is about the solve's
  // conditioning.
  bool at_cap = false;
  for (int i = 0; i < kNumJoints; ++i)
    if (std::abs(std::abs(flat[i]) - v_max[i]) < 1e-9) at_cap = true;
  EXPECT_TRUE(at_cap);
  EXPECT_LT(ramped.cwiseAbs().maxCoeff(), 0.5 * v_max.minCoeff());
}
