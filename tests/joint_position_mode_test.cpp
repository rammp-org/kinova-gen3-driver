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

// --- staleness watchdog ------------------------------------------------------

TEST(JointPositionMode, StaleTargetFreezesTheReferenceAtMeasuredQ) {
  Dynamics dyn(URDF_PATH);
  JointPositionParams p;
  p.max_ref_speed.setConstant(1.0);
  p.cmd_timeout_s = 0.05;
  JointPositionMode m(dyn, p);
  JointFeedback fb; fb.q.setZero(); fb.qd.setZero();
  m.on_enter(fb);
  m.set_target(JointVec::Constant(0.5));
  JointCommand out;
  for (int i = 0; i < 10; ++i) m.compute(fb, 0.001, out);   // fresh: reference advances
  EXPECT_GT(m.reference()[0], 0.0);
  for (int i = 0; i < 100; ++i) m.compute(fb, 0.001, out);  // 100 ms with no new command
  EXPECT_NEAR(m.reference()[0], fb.q[0], 1e-9);
}

TEST(JointPositionMode, ZeroTimeoutDisablesTheWatchdog) {
  Dynamics dyn(URDF_PATH);
  JointPositionParams p;
  p.max_ref_speed.setConstant(1.0);
  EXPECT_EQ(p.cmd_timeout_s, 0.0);                          // default preserves old behaviour
  JointPositionMode m(dyn, p);
  JointFeedback fb; fb.q.setZero(); fb.qd.setZero();
  m.on_enter(fb);
  m.set_target(JointVec::Constant(0.5));
  JointCommand out;
  for (int i = 0; i < 500; ++i) m.compute(fb, 0.001, out);
  EXPECT_GT(m.reference()[0], 0.1);                         // still tracking; nothing froze
}

// --- Cartesian pose target via in-loop IK ------------------------------------

TEST(JointPositionModePose, APoseTargetDrivesTheReferenceTowardIt) {
  Dynamics dyn(URDF_PATH);
  JointPositionMode m(dyn);
  const JointVec q0 = (JointVec() << 0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57).finished();
  JointFeedback fb; fb.q = q0;
  m.on_enter(fb);

  Pose target = dyn.fk(q0);
  target.p.x() += 0.05;                 // 5 cm along base x
  m.set_target(target);

  JointCommand out;
  for (int i = 0; i < 500; ++i) { m.compute(fb, 0.001, out); fb.q = out.position; }

  const Pose reached = dyn.fk(out.position);
  EXPECT_LT((reached.p - target.p).norm(), (dyn.fk(q0).p - target.p).norm());
  // "Got closer" alone is satisfied by 100 cycles of progress followed by a
  // latched fault freezing the rest, so assert the fault did NOT latch too.
  EXPECT_FALSE(m.ik_faulted());
}

TEST(JointPositionModePose, AJointTargetStillBypassesIkEntirely) {
  Dynamics dyn(URDF_PATH);
  JointPositionMode m(dyn);
  JointFeedback fb; fb.q = JointVec::Zero();
  m.on_enter(fb);
  const JointVec q_d = JointVec::Constant(0.1);
  m.set_target(q_d);                    // JointTargetSink overload
  JointCommand out;
  m.compute(fb, 0.001, out);
  // iters stays 0: a joint target must not pay for a solve it does not use.
  EXPECT_EQ(m.last_ik().iters, 0);
}

TEST(JointPositionModePose, TheLatestSetterWins) {
  Dynamics dyn(URDF_PATH);
  JointPositionMode m(dyn);
  const JointVec q0 = (JointVec() << 0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57).finished();
  JointFeedback fb; fb.q = q0;
  m.on_enter(fb);

  Pose pose = dyn.fk(q0); pose.p.x() += 0.05;
  m.set_target(pose);
  JointCommand out;
  m.compute(fb, 0.001, out);
  ASSERT_GT(m.last_ik().iters, 0);

  m.set_target(q0);                     // joint target supersedes the pose
  m.compute(fb, 0.001, out);
  EXPECT_EQ(m.last_ik().iters, 0);
}

TEST(JointPositionModePose, SustainedNonConvergenceRaisesIkFaulted) {
  Dynamics dyn(URDF_PATH);
  JointPositionParams p;
  p.ik_fault_s = 0.05;
  JointPositionMode m(dyn, p);
  const JointVec q0 = (JointVec() << 0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57).finished();
  JointFeedback fb; fb.q = q0;
  m.on_enter(fb);

  Pose unreachable = dyn.fk(q0);
  unreachable.p.x() += 5.0;             // metres away: never reachable
  m.set_target(unreachable);

  JointCommand out;
  m.compute(fb, 0.001, out);
  EXPECT_FALSE(m.ik_faulted());         // one bad solve is NOT a fault
  for (int i = 0; i < 100; ++i) m.compute(fb, 0.001, out);
  EXPECT_TRUE(m.ik_faulted());          // sustained non-convergence IS
}

TEST(JointPositionModePose, AConvergedSolveClearsTheFaultAccumulator) {
  Dynamics dyn(URDF_PATH);
  JointPositionParams p;
  p.ik_fault_s = 0.05;
  JointPositionMode m(dyn, p);
  const JointVec q0 = (JointVec() << 0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57).finished();
  JointFeedback fb; fb.q = q0;
  m.on_enter(fb);

  Pose unreachable = dyn.fk(q0); unreachable.p.x() += 5.0;
  m.set_target(unreachable);
  JointCommand out;
  for (int i = 0; i < 20; ++i) m.compute(fb, 0.001, out);   // under the threshold
  ASSERT_FALSE(m.ik_faulted());

  // The seed PERSISTS, so chasing a target 5 m away leaves it at full stretch and
  // it takes ~18 cycles of max_joint_step travel to walk back -- non-convergence
  // the whole way, and rightly counted. 20 + 18 fits inside the 50-cycle budget;
  // the earlier 40 did not, and that is a property of the seed being persistent,
  // not of the accumulator failing to reset. The long recovery window is what
  // gives this test its teeth: if a converged solve did NOT clear ik_bad_s_, 220
  // accumulated cycles would blow past ik_fault_s four times over.
  m.set_target(dyn.fk(q0));             // reachable
  for (int i = 0; i < 200; ++i) m.compute(fb, 0.001, out);
  EXPECT_FALSE(m.ik_faulted());
  EXPECT_TRUE(m.last_ik().converged);   // and it really did converge, not just not-fault
}

TEST(JointPositionModePose, OnEnterClearsAPreviousIkFault) {
  Dynamics dyn(URDF_PATH);
  JointPositionParams p;
  p.ik_fault_s = 0.05;
  JointPositionMode m(dyn, p);
  const JointVec q0 = (JointVec() << 0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57).finished();
  JointFeedback fb; fb.q = q0;
  m.on_enter(fb);
  Pose unreachable = dyn.fk(q0); unreachable.p.x() += 5.0;
  m.set_target(unreachable);
  JointCommand out;
  for (int i = 0; i < 100; ++i) m.compute(fb, 0.001, out);
  ASSERT_TRUE(m.ik_faulted());

  m.on_enter(fb);
  EXPECT_FALSE(m.ik_faulted());
}

// A frozen (stale) cycle runs no solve at all -- last_ik() must read that as "no
// IK ran this cycle," not carry forward the result of the last solve before the
// freeze. Otherwise a caller polling last_ik() during a freeze sees a stale
// converged=true from before the stream stopped.
TEST(JointPositionModePose, StaleWatchdogClearsLastIkFromAPreviousPoseSolve) {
  Dynamics dyn(URDF_PATH);
  JointPositionParams p;
  p.cmd_timeout_s = 0.05;
  JointPositionMode m(dyn, p);
  const JointVec q0 = (JointVec() << 0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57).finished();
  JointFeedback fb; fb.q = q0;
  m.on_enter(fb);

  Pose target = dyn.fk(q0); target.p.x() += 0.05;
  m.set_target(target);

  JointCommand out;
  m.compute(fb, 0.001, out);
  ASSERT_GT(m.last_ik().iters, 0);            // a solve ran this cycle

  for (int i = 0; i < 100; ++i) m.compute(fb, 0.001, out);  // 100 ms with no new command
  EXPECT_EQ(m.last_ik().iters, 0);            // frozen: no solve ran this cycle
}

// The staleness reset must NOT reach ik_faulted_: it is a latch cleared only by
// on_enter, and the sampler thread depends on it staying set until the lifecycle
// teardown observes it. Going stale after a fault has already latched must not
// quietly erase the fault behind the freeze.
TEST(JointPositionModePose, StaleWatchdogDoesNotClearAnAlreadyLatchedIkFault) {
  Dynamics dyn(URDF_PATH);
  JointPositionParams p;
  p.ik_fault_s = 0.02;
  p.cmd_timeout_s = 0.05;
  JointPositionMode m(dyn, p);
  const JointVec q0 = (JointVec() << 0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57).finished();
  JointFeedback fb; fb.q = q0;
  m.on_enter(fb);

  Pose unreachable = dyn.fk(q0); unreachable.p.x() += 5.0;  // never reachable
  m.set_target(unreachable);

  JointCommand out;
  // 100 ms: the fault latches well before the 50 ms watchdog timeout (~cycle 20
  // vs ~cycle 50), then the watchdog itself goes stale and freezes the reference
  // for the remainder of the loop.
  for (int i = 0; i < 100; ++i) m.compute(fb, 0.001, out);

  EXPECT_TRUE(m.ik_faulted());   // the latch survives the staleness reset
  EXPECT_EQ(m.last_ik().iters, 0);   // but last_ik() still reads "no solve this cycle"
}

// The pose path's seed must ACCUMULATE across cycles. Re-seeding the solve from
// q_ref_ every cycle throws IK progress away: q_ref_ advances at
// max_ref_speed*dt (0.0005 rad/cycle by default) while a single solve can move
// at most max_iters*max_joint_step (0.2 rad), so any pose needing more than that
// much joint travel is !converged on EVERY cycle and ik_fault_s latches a
// permanent freeze on a perfectly reachable target.
TEST(JointPositionModePose, AReachableButDistantPoseDoesNotFault) {
  Dynamics dyn(URDF_PATH);
  JointPositionMode m(dyn);                       // defaults: ik_fault_s = 0.1
  const JointVec q0 = (JointVec() << 0.0,0.26,3.14,-2.27,0.0,0.96,1.57).finished();
  JointFeedback fb; fb.q = q0; m.on_enter(fb);
  Pose target = dyn.fk(q0); target.p.x() += 0.25;  // reachable, ordinary magnitude
  m.set_target(target);
  JointCommand out;
  for (int i = 0; i < 3000; ++i) { m.compute(fb, 0.001, out); fb.q = out.position; }
  EXPECT_FALSE(m.ik_faulted());
  EXPECT_LT((dyn.fk(out.position).p - target.p).norm(), 1e-3);
}


// The persistent seed must be RE-ANCHORED when the source transitions back into
// kPose, or a solution left over from an earlier pose session silently becomes the
// warm start for the next one -- and the first solve of a session then reports the
// old configuration's error instead of the arm's.
TEST(JointPositionModePose, ReanchorsTheSeedOnTheTransitionBackIntoPose) {
  Dynamics dyn(URDF_PATH);
  JointPositionMode m(dyn);
  const JointVec q0 = (JointVec() << 0.0,0.26,3.14,-2.27,0.0,0.96,1.57).finished();
  JointFeedback fb; fb.q = q0; m.on_enter(fb);
  JointCommand out;

  // Session 1: chase something far away so the seed ends up nowhere near q0.
  Pose far_away = dyn.fk(q0); far_away.p.x() += 5.0;
  m.set_target(far_away);
  for (int i = 0; i < 40; ++i) m.compute(fb, 0.001, out);
  ASSERT_FALSE(m.last_ik().converged);

  // A joint target ends the pose session.
  m.set_target(q0);
  m.compute(fb, 0.001, out);

  // Session 2, on a pose the arm is already AT: with the seed re-anchored to the
  // reference this converges on the very first solve. Carrying the stale seed over
  // would leave a metres-large pos_err on that same first solve.
  m.set_target(dyn.fk(m.reference()));
  m.compute(fb, 0.001, out);
  EXPECT_TRUE(m.last_ik().converged);
  EXPECT_LT(m.last_ik().pos_err, 1e-4);
}


