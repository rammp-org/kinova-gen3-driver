#pragma once
#include <array>
#include <cmath>
#include <utility>

#include "kinova_lowlevel/joint_types.h"
#include "kinova_lowlevel/transport.h"
namespace kinova {
// Order in which a transport switches actuator control modes one at a time (#92).
// Through the switch, nothing is closed-loop: a switched actuator receives the
// fixed hold, and one not yet switched keeps what it received before.
//  - Into torque/current, a switched joint holds open-loop with the hold torque
//    while an unswitched position-mode joint stays stiff, so the joints with the
//    largest |hold_torque| switch LAST.
//  - Into position/velocity, a switched joint becomes stiff while an unswitched
//    torque-mode joint keeps the outgoing torque open-loop, so the joints with the
//    largest |outgoing_torque| switch FIRST.
// Ties keep joint order. Returns 0-based joint indices. No allocation.
inline std::array<int, kNumJoints> actuator_switch_order(const ActuatorModes& modes,
                                                         const JointVec& hold_torque,
                                                         const JointVec& outgoing_torque) {
  std::array<double, kNumJoints> key;
  std::array<int, kNumJoints> order;
  for (int j = 0; j < kNumJoints; ++j) {
    order[j] = j;
    const bool into_torque =
        modes[j] == ActuatorMode::kTorque || modes[j] == ActuatorMode::kCurrent;
    key[j] = into_torque ? std::abs(hold_torque[j]) : -std::abs(outgoing_torque[j]);
  }
  for (int i = 1; i < kNumJoints; ++i)  // stable insertion sort on key
    for (int k = i; k > 0 && key[order[k]] < key[order[k - 1]]; --k)
      std::swap(order[k], order[k - 1]);
  return order;
}
}  // namespace kinova
