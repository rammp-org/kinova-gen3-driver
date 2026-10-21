#include "kinova_lowlevel/rt_executor.h"

#include <gtest/gtest.h>

#include <atomic>
#include <string>
#include <vector>

#include "kinova_lowlevel/cartesian_impedance_mode.h"
#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/joint_impedance_mode.h"
#include "kinova_lowlevel/joint_position_mode.h"
#include "kinova_lowlevel/joint_torque_mode.h"
#include "kinova_lowlevel/telemetry.h"
#include "kinova_lowlevel/transport.h"

using namespace kinova;

namespace {
// Records what the executor hands the transport around mode switches: the hold
// command for each switch and the first command exchanged after the first one.
// Optionally requests a second mode mid-run, then stops the loop after a fixed
// number of exchanges so the test is deterministic, no sleeps.
struct RecordingTransport : Transport {
  JointFeedback state;
  std::atomic<bool>* stop = nullptr;
  RtExecutor* ex = nullptr;
  ControlMode* next_mode = nullptr;  // requested after the 3rd exchange, if set
  std::vector<std::string>* log = nullptr;
  std::vector<JointCommand> holds;
  bool saw_first_after_switch = false;
  JointCommand first_after_switch;
  int exchanges = 0;

  void connect() override {}
  void set_servoing_low_level() override {}
  void set_actuator_modes(const ActuatorModes&, const JointCommand& h) override {
    if (log) log->push_back("switch");
    holds.push_back(h);
  }
  void exchange(const JointCommand& c, JointFeedback& fb) override {
    if (!holds.empty() && !saw_first_after_switch) {
      saw_first_after_switch = true;
      first_after_switch = c;
    }
    fb = state;
    ++exchanges;
    if (exchanges == 3 && next_mode) ex->request_mode(next_mode);
    if (exchanges >= 6) stop->store(true);
  }
  void send(const JointCommand&) override {}
  void receive(JointFeedback& fb) override { fb = state; }
  void safe_shutdown() override {}
};

// Forwards to a real mode and logs the calls, for the call-order check.
struct LoggingMode : ControlMode {
  ControlMode& inner;
  std::vector<std::string>& log;
  LoggingMode(ControlMode& m, std::vector<std::string>& l) : inner(m), log(l) {}
  ActuatorModes required_modes() const override { return inner.required_modes(); }
  void on_enter(const JointFeedback& fb) override {
    log.push_back("on_enter");
    inner.on_enter(fb);
  }
  void compute(const JointFeedback& fb, double dt, JointCommand& out) override {
    log.push_back("compute");
    inner.compute(fb, dt, out);
  }
  void on_exit() override { inner.on_exit(); }
};

// Home: (0, 15, 180, -130, 0, 55, 90) deg -- gravity loads j2/j4/j6.
JointVec home() {
  JointVec q;
  q << 0.0, 0.2618, 3.1416, -2.2689, 0.0, 0.9599, 1.5708;
  return q;
}

void run_with(RecordingTransport& t, ControlMode& first) {
  SampleRing ring(64);
  RtExecutor ex(t, ring, {1000.0, Pacing::kSleepSpin, {0, -1, false, false}});
  std::atomic<bool> stop{false};
  t.stop = &stop;
  t.ex = &ex;
  ex.request_mode(&first);
  ex.run(stop);
}
}  // namespace

// #92: the actuators switch to torque one at a time over ~160 ms. Whatever the
// transport sends meanwhile is what holds the arm, so it must be the incoming
// mode's first command (gravity comp at the current pose), never zero torque.
TEST(RtExecutor, ModeSwitchHoldsWithIncomingModesFirstCommand) {
  Dynamics dyn(URDF_PATH);
  RecordingTransport t;
  t.state.q = home();
  JointVec g;
  dyn.gravity(t.state.q, g);
  ASSERT_GT(g.norm(), 1.0);  // the pose actually loads the arm

  JointTorqueMode mode(dyn);  // tau_ff never set == gravity-comp hold
  run_with(t, mode);

  ASSERT_EQ(t.holds.size(), 1u);
  EXPECT_EQ(t.holds[0].mode, ActuatorMode::kTorque);
  EXPECT_TRUE(t.holds[0].torque.isApprox(g, 1e-9)) << t.holds[0].torque.transpose();
  // And the first cycle after the switch must not fall back to the executor's
  // zero-initialized hold either.
  ASSERT_TRUE(t.saw_first_after_switch);
  EXPECT_TRUE(t.first_after_switch.torque.isApprox(g, 1e-9))
      << t.first_after_switch.torque.transpose();
}

// A mid-run switch reuses the executor's JointCommand: the hold must be the
// incoming mode's command, not the outgoing mode's last one.
TEST(RtExecutor, MidRunSwitchHoldsWithIncomingModeNotOutgoing) {
  Dynamics dyn(URDF_PATH);
  RecordingTransport t;
  t.state.q = home();
  JointVec g;
  dyn.gravity(t.state.q, g);

  JointPositionMode pos(dyn);
  JointTorqueMode torque(dyn);
  t.next_mode = &torque;
  run_with(t, pos);

  ASSERT_EQ(t.holds.size(), 2u);
  EXPECT_EQ(t.holds[0].mode, ActuatorMode::kPosition);
  EXPECT_EQ(t.holds[1].mode, ActuatorMode::kTorque);
  EXPECT_TRUE(t.holds[1].torque.isApprox(g, 1e-9)) << t.holds[1].torque.transpose();
}

// Both impedance modes enter with the spring ramped to zero, so their hold is
// pure gravity comp: no spring or damper term acts open-loop through the switch.
TEST(RtExecutor, ImpedanceModesHoldWithPureGravity) {
  Dynamics dyn(URDF_PATH);
  JointVec g;
  dyn.gravity(home(), g);
  CartesianImpedanceMode cart(dyn);
  JointImpedanceMode joint(dyn);
  for (ControlMode* m : {static_cast<ControlMode*>(&cart), static_cast<ControlMode*>(&joint)}) {
    RecordingTransport t;
    t.state.q = home();
    run_with(t, *m);
    ASSERT_EQ(t.holds.size(), 1u);
    EXPECT_TRUE(t.holds[0].torque.isApprox(g, 1e-9)) << t.holds[0].torque.transpose();
  }
}

TEST(RtExecutor, ModeComputesItsHoldBeforeTheActuatorsSwitch) {
  Dynamics dyn(URDF_PATH);
  std::vector<std::string> log;
  RecordingTransport t;
  t.state.q = home();
  t.log = &log;
  JointTorqueMode inner(dyn);
  LoggingMode mode(inner, log);
  run_with(t, mode);

  ASSERT_GE(log.size(), 4u);
  EXPECT_EQ(log[0], "on_enter");
  EXPECT_EQ(log[1], "compute");
  EXPECT_EQ(log[2], "switch");
  EXPECT_EQ(log[3], "compute");  // then the normal cycle, on fresh feedback
}
