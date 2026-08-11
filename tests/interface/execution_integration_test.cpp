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
#include <cmath>
#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/joint_impedance_mode.h"
#include "kinova_lowlevel/joint_position_mode.h"
#include "kinova_lowlevel/interface/trajectory_executor.h"

using namespace kinova;
using kinova::interface::ControlModeKind;
using kinova::interface::ExecStatus;
using kinova::interface::Preemption;
using kinova::interface::Trajectory;
using kinova::interface::TrajectoryExecutor;

namespace {
JointVec vecn(std::initializer_list<double> v) {
  JointVec q; int i = 0; for (double x : v) q[i++] = x; return q;
}
Trajectory line(const JointVec& q0, const JointVec& q1, double dur_s) {
  return { { {q0, 0.0}, {q1, dur_s} } };
}
const JointVec kDisabledTol = JointVec::Constant(-1.0);   // per-joint guard off
constexpr double kDt = 0.001;                             // 1 kHz
}  // namespace

// A trajectory submitted to JointPositionMode drives the commanded joint
// reference to the goal, and an ideal-servo arm converges to the endpoint by the
// trajectory's final timestamp; the executor reports completion there.
TEST(ExecutionIntegration, PositionTrajectoryConvergesToGoalOnSchedule) {
  Dynamics dyn(URDF_PATH);
  JointPositionMode mode(dyn);                 // default params: 0.5 rad/s cap, 0.35 leash
  TrajectoryExecutor exec(mode);               // mode IS-A kinova::JointTargetSink

  const JointVec q0 = vecn({0.1, 0.3, -0.2, 0.8, 0.5, -0.4, 0.2});
  const JointVec q1 = q0 + JointVec::Constant(0.30);   // 0.30 rad move...
  const double dur = 4.0;                              // ...over 4 s -> 0.075 rad/s << 0.5 cap

  JointFeedback fb; fb.q = q0; fb.qd.setZero();
  mode.on_enter(fb);
  ASSERT_EQ(exec.submit(line(q0, q1, dur), ControlModeKind::kPosition,
                        Preemption::kLatestWins, kDisabledTol),
            kinova::interface::SubmitResult::kAccepted);

  double t = 0.0;
  ExecStatus st{};
  JointCommand cmd;
  for (int step = 0; step < 6000; ++step) {    // 4 s traj + settle margin
    st = exec.tick(t, fb.q);                    // publish sampled q_d to the mode
    mode.compute(fb, kDt, cmd);                 // mode -> position command
    fb.q = cmd.position;                        // ideal position servo: arm reaches it
    fb.qd.setZero();
    t += kDt;
    if (st.completed && (fb.q - q1).norm() < 1e-3) break;   // done + settled
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
  JointFeedback fb; fb.q = q0; fb.qd.setZero();
  mode.on_enter(fb);
  exec.submit(line(q0, q1, 3.0), ControlModeKind::kPosition,
              Preemption::kLatestWins, JointVec::Constant(0.05));   // tight guard

  double t = 0.0;
  ExecStatus st{};
  JointCommand cmd;
  for (int step = 0; step < 3000; ++step) {
    st = exec.tick(t, fb.q);                    // measured q stays q0 (arm stuck)
    mode.compute(fb, kDt, cmd);                 // command ignored by the "stuck" plant
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
  p.max_ref_speed.setConstant(1e9);            // don't rate-limit the reference here
  JointImpedanceMode mode(dyn, p);
  TrajectoryExecutor exec(mode);               // JointImpedanceMode IS-A JointTargetSink now

  const JointVec q0 = vecn({0.1, 0.3, -0.2, 0.8, 0.5, -0.4, 0.2});
  const JointVec q1 = q0 + JointVec::Constant(0.20);
  const double dur = 2.0;
  JointFeedback fb; fb.q = q0; fb.qd.setZero();
  mode.on_enter(fb);
  exec.submit(line(q0, q1, dur), ControlModeKind::kImpedance,
              Preemption::kLatestWins, kDisabledTol);

  double t = 0.0;
  ExecStatus st{};
  JointCommand cmd;
  for (int step = 0; step < 3000; ++step) {
    st = exec.tick(t, fb.q);                    // publish sampled q_d as the joint reference
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
  EXPECT_NEAR((mode.reference() - q1).norm(), 0.0, 1e-9);   // reference reached the goal
}
