#include "kinova_lowlevel/rt_executor.h"

#include <gtest/gtest.h>

#include <atomic>

#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/joint_torque_mode.h"
#include "kinova_lowlevel/telemetry.h"
#include "kinova_lowlevel/transport.h"

using namespace kinova;

namespace {
// Records what the executor hands the transport around a mode switch: the hold
// command for the switch itself and the first command exchanged after it. Stops
// the loop after a few exchanges so the test is deterministic, no sleeps.
struct RecordingTransport : Transport {
  JointFeedback state;
  std::atomic<bool>* stop = nullptr;
  int switches = 0;
  JointCommand hold;
  bool saw_first_after_switch = false;
  JointCommand first_after_switch;
  int exchanges = 0;

  void connect() override {}
  void set_servoing_low_level() override {}
  void set_actuator_modes(const ActuatorModes&, const JointCommand& h) override {
    ++switches;
    hold = h;
  }
  void exchange(const JointCommand& c, JointFeedback& fb) override {
    if (switches > 0 && !saw_first_after_switch) {
      saw_first_after_switch = true;
      first_after_switch = c;
    }
    fb = state;
    if (++exchanges >= 5) stop->store(true);
  }
  void send(const JointCommand&) override {}
  void receive(JointFeedback& fb) override { fb = state; }
  void safe_shutdown() override {}
};
}  // namespace

// #92: the actuators switch to torque one at a time over ~160 ms. Whatever the
// transport sends meanwhile is what holds the arm, so it must be the incoming
// mode's first command (gravity comp at the current pose), never zero torque.
TEST(RtExecutor, ModeSwitchHoldsWithIncomingModesFirstCommand) {
  Dynamics dyn(URDF_PATH);
  RecordingTransport t;
  // Home: (0, 15, 180, -130, 0, 55, 90) deg -- gravity loads j2/j4/j6.
  t.state.q << 0.0, 0.2618, 3.1416, -2.2689, 0.0, 0.9599, 1.5708;
  JointVec g;
  dyn.gravity(t.state.q, g);
  ASSERT_GT(g.norm(), 1.0);  // the pose actually loads the arm

  JointTorqueMode mode(dyn);  // tau_ff never set == gravity-comp hold
  SampleRing ring(64);
  RtExecutor ex(t, ring, {1000.0, Pacing::kSleepSpin, {0, -1, false, false}});
  std::atomic<bool> stop{false};
  t.stop = &stop;
  ex.request_mode(&mode);
  ex.run(stop);

  ASSERT_EQ(t.switches, 1);
  EXPECT_EQ(t.hold.mode, ActuatorMode::kTorque);
  EXPECT_TRUE(t.hold.torque.isApprox(g, 1e-9)) << t.hold.torque.transpose();
  // And the first cycle after the switch must not fall back to the executor's
  // zero-initialized hold either.
  ASSERT_TRUE(t.saw_first_after_switch);
  EXPECT_TRUE(t.first_after_switch.torque.isApprox(g, 1e-9))
      << t.first_after_switch.torque.transpose();
}
