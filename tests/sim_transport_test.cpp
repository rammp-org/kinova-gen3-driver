#include <gtest/gtest.h>
#include "kinova_lowlevel/sim_transport.h"
using namespace kinova;
TEST(SimTransport, EchoesStateAndAdvancesFrame) {
  JointFeedback init; init.q.setConstant(0.1);
  SimTransport t(init);
  t.connect(); t.set_servoing_low_level();
  JointCommand c; c.mode = ActuatorMode::kTorque;
  JointFeedback fb;
  t.exchange(c, fb);
  EXPECT_NEAR(fb.q[0], 0.1, 1e-9);
  uint64_t f0 = fb.frame_id;
  t.exchange(c, fb);
  EXPECT_GT(fb.frame_id, f0);
}
TEST(SimTransport, SendReceiveRoundTrips) {
  JointFeedback init; init.q.setZero();
  SimTransport t(init);
  t.connect();
  JointCommand c; c.torque.setConstant(1.0);
  t.send(c);
  JointFeedback fb; t.receive(fb);
  EXPECT_TRUE(fb.q.allFinite());
}
// SimTransport has no fault concept, so it inherits Transport's default-no-op
// clear_faults(). The teleop server calls clear_faults() on whatever Transport
// it holds; this guards that the sim path is a safe no-op (no throw, state
// unchanged) so protocol bring-up under --sim never trips on it.
TEST(SimTransport, ClearFaultsIsNoOp) {
  JointFeedback init;
  SimTransport t(init);
  Transport& base = t;  // call through the base interface
  EXPECT_NO_THROW(base.clear_faults());
}

TEST(SimTransport, EchoesGripperWhenActive) {
  JointFeedback init;
  SimTransport t(init);
  t.connect();
  JointCommand c;
  c.gripper = 0.42f;
  c.gripper_active = true;
  JointFeedback fb;
  t.exchange(c, fb);
  EXPECT_NEAR(fb.gripper, 0.42f, 1e-6f);
}

TEST(SimTransport, LeavesGripperUntouchedWhenInactive) {
  JointFeedback init;
  init.gripper = 0.7f;       // pre-existing measured position
  SimTransport t(init);
  t.connect();
  JointCommand c;
  c.gripper = 0.1f;
  c.gripper_active = false;  // no gripper intent
  JointFeedback fb;
  t.exchange(c, fb);
  EXPECT_NEAR(fb.gripper, 0.7f, 1e-6f);  // unchanged
}
