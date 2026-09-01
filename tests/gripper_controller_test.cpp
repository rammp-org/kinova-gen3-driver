#include <gtest/gtest.h>
#include "kinova_lowlevel/gripper_controller.h"
#include "kinova_lowlevel/sim_transport.h"

using namespace kinova;

TEST(GripperController, StampsNothingBeforeTheFirstCommand) {
  JointFeedback init;
  SimTransport sim(init);
  GripperController gc(sim);
  gc.connect();
  JointCommand c;
  JointFeedback fb;
  gc.exchange(c, fb);
  // Untouched: the gripper stays limp at startup rather than being driven to a default.
  EXPECT_FALSE(sim.last_command().gripper.active);
}

TEST(GripperController, StampsThePositionItWasGiven) {
  JointFeedback init;
  SimTransport sim(init);
  GripperController gc(sim);
  gc.connect();
  GripperCommand g;
  g.position = 0.65f;
  gc.set_target(g);
  JointCommand c;
  JointFeedback fb;
  gc.exchange(c, fb);
  EXPECT_TRUE(sim.last_command().gripper.active);
  EXPECT_NEAR(sim.last_command().gripper.position, 0.65f, 1e-6f);
}

TEST(GripperController, APositionOnlyCommandCarriesTheDefaultSpeedAndForce) {
  JointFeedback init;
  SimTransport sim(init);
  GripperController gc(sim);
  gc.connect();
  GripperCommand g;
  g.position = 0.3f;               // speed and force left at their defaults
  gc.set_target(g);
  JointCommand c;
  JointFeedback fb;
  gc.exchange(c, fb);
  EXPECT_NEAR(sim.last_command().gripper.speed, 1.0f, 1e-6f);
  EXPECT_NEAR(sim.last_command().gripper.force, 0.5f, 1e-6f);
}

TEST(GripperController, SpeedAndForceDoNotPersistBetweenCommands) {
  // The statelessness decision, pinned. A caller that sets force once and then sends a
  // position-only command gets the DEFAULT force back -- not the earlier value. This is
  // the streaming tier's rule: a setpoint is a command, never an increment.
  JointFeedback init;
  SimTransport sim(init);
  GripperController gc(sim);
  gc.connect();
  JointCommand c;
  JointFeedback fb;

  GripperCommand strong;
  strong.position = 1.0f;
  strong.force    = 0.9f;
  gc.set_target(strong);
  gc.exchange(c, fb);
  ASSERT_NEAR(sim.last_command().gripper.force, 0.9f, 1e-6f);

  GripperCommand plain;
  plain.position = 0.5f;           // force not set
  gc.set_target(plain);
  gc.exchange(c, fb);
  EXPECT_NEAR(sim.last_command().gripper.force, 0.5f, 1e-6f);
}

TEST(GripperController, ReleaseStopsStampingAndDoesNotOpenTheGripper) {
  // The halt decision: e-stop means stop moving, and opening is itself a motion. The
  // hardware self-locks, so ceasing to command IS holding.
  JointFeedback init;
  SimTransport sim(init);
  GripperController gc(sim);
  gc.connect();
  GripperCommand g;
  g.position = 0.8f;
  gc.set_target(g);
  JointCommand c;
  JointFeedback fb;
  gc.exchange(c, fb);
  ASSERT_TRUE(sim.last_command().gripper.active);

  gc.release();
  gc.exchange(c, fb);
  EXPECT_FALSE(sim.last_command().gripper.active);   // no longer commanded
  // and specifically NOT commanded open:
  EXPECT_NE(sim.last_command().gripper.position, 0.0f);
}

TEST(GripperController, PassesFeedbackThroughUntouched) {
  JointFeedback init;
  init.q[0] = 0.25;
  SimTransport sim(init);
  GripperController gc(sim);
  gc.connect();
  JointCommand c;
  JointFeedback fb;
  gc.exchange(c, fb);
  EXPECT_NEAR(fb.q[0], 0.25, 1e-9);      // the decorator is transparent to arm state
  EXPECT_TRUE(fb.gripper.present);
}

TEST(GripperController, StampsSendTheSameWayAsExchange) {
  // send() and exchange() must stamp identically, or a send-driven caller silently
  // loses its gripper command.
  JointFeedback init;
  SimTransport sim(init);
  GripperController gc(sim);
  gc.connect();
  GripperCommand g;
  g.position = 0.42f;
  gc.set_target(g);
  JointCommand c;
  gc.send(c);
  EXPECT_TRUE(sim.last_command().gripper.active);
  EXPECT_NEAR(sim.last_command().gripper.position, 0.42f, 1e-6f);
}
