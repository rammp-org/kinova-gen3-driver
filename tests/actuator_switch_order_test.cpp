#include "kinova_lowlevel/actuator_switch_order.h"

#include <gtest/gtest.h>

using namespace kinova;

namespace {
ActuatorModes all(ActuatorMode m) {
  ActuatorModes modes;
  modes.fill(m);
  return modes;
}
JointVec vec(double a, double b, double c, double d, double e, double f, double g) {
  JointVec v;
  v << a, b, c, d, e, f, g;
  return v;
}
using Order = std::array<int, kNumJoints>;
}  // namespace

// Into torque: a switched joint holds open-loop, so the most-loaded switch last.
TEST(ActuatorSwitchOrder, IntoTorqueSwitchesLeastLoadedFirst) {
  const JointVec hold = vec(0.0, -16.5, -1.1, 9.7, -0.2, 2.4, 0.0);  // ~gravity at a pose
  const Order o = actuator_switch_order(all(ActuatorMode::kTorque), hold, JointVec::Zero());
  EXPECT_EQ(o, (Order{0, 6, 4, 2, 5, 3, 1}));  // ties keep joint order
}

// Into position: a switched joint becomes stiff, while one still in torque keeps
// the outgoing torque open-loop -- so the joints carrying the most switch first.
TEST(ActuatorSwitchOrder, IntoPositionSwitchesMostLoadedFirst) {
  const JointVec outgoing = vec(0.1, -16.5, -1.1, 9.7, -0.2, 2.4, 0.0);
  const JointVec hold = JointVec::Zero();  // position commands carry no torque
  const Order o = actuator_switch_order(all(ActuatorMode::kPosition), hold, outgoing);
  EXPECT_EQ(o, (Order{1, 3, 5, 2, 4, 0, 6}));
}

TEST(ActuatorSwitchOrder, NoLoadKeepsJointOrder) {
  const Order o =
      actuator_switch_order(all(ActuatorMode::kTorque), JointVec::Zero(), JointVec::Zero());
  EXPECT_EQ(o, (Order{0, 1, 2, 3, 4, 5, 6}));
}
