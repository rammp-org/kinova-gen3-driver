// Walking-skeleton integration test: wire the non-RT TrajectoryExecutor to a
// REAL control mode (via the core kinova::JointTargetSink seam) and drive a
// joint trajectory closed-loop, one cycle at a time.
//
// Plant model: SimTransport is a static echo (its exchange() returns the initial
// state unchanged — it never integrates motion), so it cannot stand in for the
// arm here. For JointPositionMode the faithful plant is an IDEAL POSITION SERVO:
// the actuator reaches whatever position it was commanded, so next-cycle measured
// q == this-cycle commanded q. That is exactly the loop RtExecutor runs per cycle
// (exchange -> tick -> compute), minus the threading, run deterministically.
#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/interface/trajectory_executor.h"
#include "kinova_lowlevel/joint_impedance_mode.h"
#include "kinova_lowlevel/joint_position_mode.h"
#include "kinova_lowlevel/sim_transport.h"
#include "kinova_lowlevel/units.h"

using namespace kinova;
using kinova::interface::ControlModeKind;
using kinova::interface::ExecStatus;
using kinova::interface::Preemption;
using kinova::interface::Trajectory;
using kinova::interface::TrajectoryExecutor;

namespace {
JointVec vecn(std::initializer_list<double> v) {
  JointVec q;
  int i = 0;
  for (double x : v) q[i++] = x;
  return q;
}
Trajectory line(const JointVec& q0, const JointVec& q1, double dur_s) {
  return {{{q0, 0.0}, {q1, dur_s}}};
}
const JointVec kDisabledTol = JointVec::Constant(-1.0);  // per-joint guard off
constexpr double kDt = 0.001;                            // 1 kHz
}  // namespace

// A trajectory submitted to JointPositionMode drives the commanded joint
// reference to the goal, and an ideal-servo arm converges to the endpoint by the
// trajectory's final timestamp; the executor reports completion there.
TEST(ExecutionIntegration, PositionTrajectoryConvergesToGoalOnSchedule) {
  Dynamics dyn(URDF_PATH);
  JointPositionMode mode(dyn);    // default params: 0.5 rad/s cap, 0.35 leash
  TrajectoryExecutor exec(mode);  // mode IS-A kinova::JointTargetSink

  const JointVec q0 = vecn({0.1, 0.3, -0.2, 0.8, 0.5, -0.4, 0.2});
  const JointVec q1 = q0 + JointVec::Constant(0.30);  // 0.30 rad move...
  const double dur = 4.0;                             // ...over 4 s -> 0.075 rad/s << 0.5 cap

  JointFeedback fb;
  fb.q = q0;
  fb.qd.setZero();
  mode.on_enter(fb);
  ASSERT_EQ(exec.submit(line(q0, q1, dur), ControlModeKind::kPosition, Preemption::kLatestWins,
                        kDisabledTol),
            kinova::interface::SubmitResult::kAccepted);

  double t = 0.0;
  ExecStatus st{};
  JointCommand cmd;
  for (int step = 0; step < 6000; ++step) {  // 4 s traj + settle margin
    st = exec.tick(t, fb.q);                 // publish sampled q_d to the mode
    mode.compute(fb, kDt, cmd);              // mode -> position command
    fb.q = cmd.position;                     // ideal position servo: arm reaches it
    fb.qd.setZero();
    t += kDt;
    if (st.completed && (fb.q - q1).norm() < 1e-3) break;  // done + settled
  }
  EXPECT_TRUE(st.completed) << "executor never reported completion";
  EXPECT_NEAR((fb.q - q1).norm(), 0.0, 1e-3) << "arm did not reach the goal";
  EXPECT_GE(t, dur - 1e-9) << "completed before the trajectory's final timestamp";
}

// A stalled arm (measured q frozen) that falls outside the path tolerance makes
// the executor abort the goal end-to-end with a REAL mode in the loop.
TEST(ExecutionIntegration, StalledArmTripsPathToleranceAbort) {
  Dynamics dyn(URDF_PATH);
  JointPositionMode mode(dyn);
  TrajectoryExecutor exec(mode);

  const JointVec q0 = vecn({0.0, 0.2, 0.0, 0.5, 0.0, -0.3, 0.0});
  const JointVec q1 = q0 + JointVec::Constant(0.30);
  JointFeedback fb;
  fb.q = q0;
  fb.qd.setZero();
  mode.on_enter(fb);
  exec.submit(line(q0, q1, 3.0), ControlModeKind::kPosition, Preemption::kLatestWins,
              JointVec::Constant(0.05));  // tight guard

  double t = 0.0;
  ExecStatus st{};
  JointCommand cmd;
  for (int step = 0; step < 3000; ++step) {
    st = exec.tick(t, fb.q);     // measured q stays q0 (arm stuck)
    mode.compute(fb, kDt, cmd);  // command ignored by the "stuck" plant
    t += kDt;
    if (st.completed) break;
  }
  EXPECT_TRUE(st.completed);
  EXPECT_EQ(st.error_code, ExecStatus::kPathToleranceViolated);
  EXPECT_FALSE(exec.is_active());
  EXPECT_LT(t, 1.0) << "should have aborted early, once |q_desired - q0| > 0.05";
}

// The same executor drives JointImpedanceMode through the joint-target seam: its
// integrated reference tracks the sampled trajectory and the commanded torque
// stays within the per-joint limits. (Closed-loop impedance MOTION needs a
// physics sim, which SimTransport is not — deferred; this pins the wiring.)
TEST(ExecutionIntegration, ImpedanceModeReferenceTracksTrajectory) {
  Dynamics dyn(URDF_PATH);
  JointImpedanceParams p;
  p.gain_ramp_s = 0.0;
  p.max_ref_speed.setConstant(1e9);  // don't rate-limit the reference here
  JointImpedanceMode mode(dyn, p);
  TrajectoryExecutor exec(mode);  // JointImpedanceMode IS-A JointTargetSink now

  const JointVec q0 = vecn({0.1, 0.3, -0.2, 0.8, 0.5, -0.4, 0.2});
  const JointVec q1 = q0 + JointVec::Constant(0.20);
  const double dur = 2.0;
  JointFeedback fb;
  fb.q = q0;
  fb.qd.setZero();
  mode.on_enter(fb);
  exec.submit(line(q0, q1, dur), ControlModeKind::kImpedance, Preemption::kLatestWins,
              kDisabledTol);

  double t = 0.0;
  ExecStatus st{};
  JointCommand cmd;
  for (int step = 0; step < 3000; ++step) {
    st = exec.tick(t, fb.q);  // publish sampled q_d as the joint reference
    mode.compute(fb, kDt, cmd);
    // Reference must equal the trajectory sample at this time (IK bypassed).
    const JointVec expected = kinova::interface::sample(line(q0, q1, dur), t);
    EXPECT_NEAR((mode.reference() - expected).norm(), 0.0, 1e-9) << "t=" << t;
    for (int i = 0; i < kNumJoints; ++i) {
      EXPECT_LE(std::abs(cmd.torque[i]), p.torque_limit[i] + 1e-9) << "joint " << i;
      EXPECT_TRUE(std::isfinite(cmd.torque[i]));
    }
    t += kDt;
    if (st.completed) break;
  }
  EXPECT_TRUE(st.completed);
  EXPECT_NEAR((mode.reference() - q1).norm(), 0.0, 1e-9);  // reference reached the goal
}

// --- continuous-joint wrap sweep (#52) ---------------------------------------
//
// The end-to-end test that would have caught #52. It drives a continuous joint
// ACROSS the +/-pi boundary with the divergence guard armed, through a real mode
// and a real Dynamics, with the measurement wrapped exactly as both transports now
// report it. Before the fix the guard read the wrap as ~2*pi of divergence and
// aborted an arm that was tracking perfectly.
//
// Parameterised over every continuous joint rather than joint_3 alone: 1/3/5/7 are
// all `type="continuous"` in the URDF, the arm can park near the boundary on any of
// them, and a guard that folds only the joint we happened to debug is not a fix.
class ContinuousJointWrapSweep : public ::testing::TestWithParam<int> {};

TEST_P(ContinuousJointWrapSweep, CrossingPiIsNotDivergence) {
  const int j = GetParam();
  Dynamics dyn(URDF_PATH);

  JointVec lower, upper;
  dyn.joint_limits(lower, upper);
  ASSERT_FALSE(std::isfinite(lower[j])) << "joint " << j << " is not continuous in the URDF";
  std::array<bool, kNumJoints> continuous{};
  for (int i = 0; i < kNumJoints; ++i)
    continuous[i] = !std::isfinite(lower[i]) && !std::isfinite(upper[i]);

  JointPositionMode mode(dyn);
  TrajectoryExecutor exec(mode, continuous);

  // Park the joint just short of -pi and walk it 0.2 rad further negative, so the
  // trajectory crosses the boundary partway through. The plan is UNWRAPPED, which
  // is what a planner emits; the measurement is wrapped, which is what the arm
  // reports. Everything else is parked.
  constexpr double kPi = 3.14159265358979323846;
  JointVec q0 = JointVec::Zero();
  q0[j] = -kPi + 0.1;
  JointVec q1 = q0;
  q1[j] = -kPi - 0.1;  // == +3.04159 once wrapped

  JointFeedback fb;
  fb.q = q0;
  fb.qd.setZero();
  mode.on_enter(fb);
  ASSERT_EQ(exec.submit(line(q0, q1, 4.0), ControlModeKind::kPosition, Preemption::kLatestWins,
                        JointVec::Constant(0.35)),  // the guard GoToEEPose uses
            kinova::interface::SubmitResult::kAccepted);

  double t = 0.0;
  ExecStatus st{};
  JointCommand cmd;
  bool crossed = false;
  for (int step = 0; step < 6000; ++step) {
    st = exec.tick(t, fb.q);
    ASSERT_NE(st.error_code, ExecStatus::kPathToleranceViolated)
        << "joint " << j << " aborted at t=" << t << " while tracking; q_meas=" << fb.q[j];
    mode.compute(fb, kDt, cmd);
    // Ideal position servo, reporting like the transport does: wrapped.
    for (int i = 0; i < kNumJoints; ++i) fb.q[i] = wrap_to_pi(cmd.position[i]);
    fb.qd.setZero();
    if (fb.q[j] > 0.0) crossed = true;  // measurement has flipped sign at the boundary
    t += kDt;
    if (st.completed) break;
  }
  EXPECT_TRUE(st.completed) << "never completed";
  EXPECT_EQ(st.error_code, ExecStatus::kOk);
  EXPECT_TRUE(crossed) << "test never actually crossed the boundary -- it proves nothing";
  EXPECT_NEAR(wrap_to_pi(fb.q[j] - q1[j]), 0.0, 1e-2) << "did not reach the goal";
}

INSTANTIATE_TEST_SUITE_P(AllContinuousJoints, ContinuousJointWrapSweep,
                         ::testing::Values(0, 2, 4, 6));

// The two transports must agree on the representation every consumer downstream
// honours. SimTransport used not to wrap, which is what made #52 unreachable from
// CI; this pins the parity so it cannot drift back.
TEST(SimHardwareParity, SimTransportReportsWrappedAngles) {
  constexpr double kPi = 3.14159265358979323846;
  JointFeedback initial;
  initial.q.setZero();
  initial.q[2] = -kPi - 0.05;  // an unwrapped angle, as a planner or a test might set
  SimTransport sim(initial);

  JointFeedback fb;
  sim.receive(fb);
  EXPECT_NEAR(fb.q[2], kPi - 0.05, 1e-12) << "sim must report wrapped, like KortexTransport";
  EXPECT_LE(fb.q[2], kPi);
  EXPECT_GT(fb.q[2], -kPi);
}
