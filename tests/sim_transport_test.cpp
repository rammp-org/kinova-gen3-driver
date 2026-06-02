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
