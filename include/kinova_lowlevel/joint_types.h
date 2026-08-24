#pragma once
#include <array>
#include <cstdint>
#include <Eigen/Core>

namespace kinova {

inline constexpr int kNumJoints = 7;
using JointVec = Eigen::Matrix<double, kNumJoints, 1>;   // SI: rad, rad/s, N·m
// Joint-space square matrix (e.g. the mass matrix M(q)).
using JointMat = Eigen::Matrix<double, kNumJoints, kNumJoints>;

enum class ActuatorMode : uint8_t { kPosition, kVelocity, kTorque, kCurrent };

struct JointFeedback {
  JointVec q   = JointVec::Zero();
  JointVec qd  = JointVec::Zero();
  JointVec tau = JointVec::Zero();
  JointVec current = JointVec::Zero();
  uint64_t frame_id = 0;
  bool fault = false;
  float gripper = 0.0f;   // measured gripper position, 0 (open) .. 1 (closed)
};

struct JointCommand {
  ActuatorMode mode = ActuatorMode::kTorque;
  JointVec position = JointVec::Zero();
  JointVec velocity = JointVec::Zero();
  JointVec torque   = JointVec::Zero();
  // Whether `velocity` is a real setpoint in kPosition mode (a feedforward for
  // the actuator's own loop). Mirrors gripper_active: when false the transport
  // emits no velocity at all, rather than an explicit zero, because a zero in
  // that field may read as a velocity LIMIT of zero rather than "no feedforward".
  // Ignored in kVelocity, where velocity is the command itself.
  bool velocity_active = false;
  float gripper = 0.0f;        // target gripper position, 0 (open) .. 1 (closed)
  bool  gripper_active = false; // when false, no gripper command is emitted
};

constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;

}  // namespace kinova
