#pragma once
#include <array>
#include <cstdint>
#include <Eigen/Core>

namespace kinova {

inline constexpr int kNumJoints = 7;
using JointVec = Eigen::Matrix<double, kNumJoints, 1>;   // SI: rad, rad/s, N·m

enum class ActuatorMode : uint8_t { kPosition, kVelocity, kTorque, kCurrent };

struct JointFeedback {
  JointVec q   = JointVec::Zero();
  JointVec qd  = JointVec::Zero();
  JointVec tau = JointVec::Zero();
  JointVec current = JointVec::Zero();
  uint64_t frame_id = 0;
  bool fault = false;
};

struct JointCommand {
  ActuatorMode mode = ActuatorMode::kTorque;
  JointVec position = JointVec::Zero();
  JointVec velocity = JointVec::Zero();
  JointVec torque   = JointVec::Zero();
};

constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;

}  // namespace kinova
