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
  c.gripper.position = 0.42f;
  c.gripper.active   = true;
  JointFeedback fb;
  t.exchange(c, fb);
  EXPECT_NEAR(fb.gripper.position, 0.42f, 1e-6f);
}

TEST(SimTransport, LeavesGripperUntouchedWhenInactive) {
  JointFeedback init;
  init.gripper.position = 0.7f;   // pre-existing measured position
  SimTransport t(init);
  t.connect();
  JointCommand c;
  c.gripper.position = 0.1f;
  c.gripper.active   = false;     // no gripper intent
  JointFeedback fb;
  t.exchange(c, fb);
  EXPECT_NEAR(fb.gripper.position, 0.7f, 1e-6f);   // unchanged
}

TEST(SimTransport, SendEchoesGripperToReceive) {
  JointFeedback init;
  SimTransport t(init);
  t.connect();
  JointCommand c;
  c.gripper.position = 0.33f;
  c.gripper.active   = true;
  t.send(c);
  JointFeedback fb;
  t.receive(fb);
  EXPECT_NEAR(fb.gripper.position, 0.33f, 1e-6f);
}

TEST(GripperTypes, CommandDefaultsMatchTodaysHardcodedConstants) {
  // speed 1.0 and force 0.5 ARE the old kGripperVelocityPct=100 / kGripperForcePct=50.
  // A caller that sets only position must produce what the driver sent before this change.
  GripperCommand c;
  EXPECT_FLOAT_EQ(c.position, 0.0f);
  EXPECT_FLOAT_EQ(c.speed,    1.0f);
  EXPECT_FLOAT_EQ(c.force,    0.5f);
  EXPECT_FALSE(c.active);
}

TEST(GripperTypes, FeedbackDefaultsToAbsentRatherThanOpen) {
  // present=false is the whole point: position 0 with no gripper attached must not
  // be indistinguishable from an attached, fully-open gripper.
  GripperFeedback f;
  EXPECT_FALSE(f.present);
  EXPECT_FLOAT_EQ(f.position, 0.0f);
  EXPECT_FLOAT_EQ(f.velocity, 0.0f);
  EXPECT_FLOAT_EQ(f.effort,   0.0f);
  EXPECT_FLOAT_EQ(f.current,  0.0f);
}

TEST(SimTransport, GripperApproachesTheTargetRatherThanTeleporting) {
  JointFeedback init;
  SimTransport t(init);
  t.connect();
  t.set_gripper_lag(0.25f);          // closes 25% of the remaining gap per cycle
  JointCommand c;
  c.gripper.position = 1.0f;
  c.gripper.active   = true;
  JointFeedback fb;
  t.exchange(c, fb);
  EXPECT_NEAR(fb.gripper.position, 0.25f, 1e-5f);
  t.exchange(c, fb);
  EXPECT_NEAR(fb.gripper.position, 0.4375f, 1e-5f);   // 0.25 + 0.75*0.25
  EXPECT_GT(fb.gripper.velocity, 0.0f);               // moving, and closing
}

TEST(SimTransport, GripperVelocityIsZeroWhenSettled) {
  JointFeedback init;
  SimTransport t(init);
  t.connect();
  t.set_gripper_lag(1.0f);           // reach the target immediately
  JointCommand c;
  c.gripper.position = 0.6f;
  c.gripper.active   = true;
  JointFeedback fb;
  t.exchange(c, fb);                 // moves 0 -> 0.6
  t.exchange(c, fb);                 // already there
  EXPECT_NEAR(fb.gripper.position, 0.6f, 1e-6f);
  EXPECT_NEAR(fb.gripper.velocity, 0.0f, 1e-6f);
}

TEST(SimTransport, ABlockedGripperStallsShortOfTheTargetAndLoadsUp) {
  JointFeedback init;
  SimTransport t(init);
  t.connect();
  t.set_gripper_lag(0.5f);
  t.set_gripper_blocked_at(0.4f);    // an object stops the fingers here
  JointCommand c;
  c.gripper.position = 1.0f;         // ask for fully closed
  c.gripper.active   = true;
  c.gripper.force    = 0.8f;
  JointFeedback fb;
  for (int i = 0; i < 20; ++i) t.exchange(c, fb);
  EXPECT_NEAR(fb.gripper.position, 0.4f, 1e-5f);      // stopped by the object
  EXPECT_NEAR(fb.gripper.velocity, 0.0f, 1e-5f);      // not moving
  EXPECT_NEAR(fb.gripper.effort, 0.8f, 1e-5f);        // loaded to the commanded cap
}

TEST(SimTransport, AnObjectPresentButUncommandedTargetDoesNotLoadEffort) {
  // An object exists further along, but the commanded target never reaches it --
  // the fingers should settle on the commanded target, untouched, with zero effort.
  JointFeedback init;
  SimTransport t(init);
  t.connect();
  t.set_gripper_lag(0.5f);
  t.set_gripper_blocked_at(0.4f);    // an object exists, but...
  JointCommand c;
  c.gripper.position = 0.2f;         // ...we never ask to close past it
  c.gripper.active   = true;
  c.gripper.force    = 0.8f;
  JointFeedback fb;
  for (int i = 0; i < 20; ++i) t.exchange(c, fb);
  EXPECT_NEAR(fb.gripper.position, 0.2f, 1e-5f);      // reached the commanded target
  EXPECT_NEAR(fb.gripper.effort, 0.0f, 1e-6f);        // never touched the object
}

TEST(SimTransport, ReportsTheGripperAsPresent) {
  JointFeedback init;
  SimTransport t(init);
  t.connect();
  JointCommand c;
  JointFeedback fb;
  t.exchange(c, fb);
  EXPECT_TRUE(fb.gripper.present);   // the sim always has one
}
