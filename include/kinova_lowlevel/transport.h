#pragma once
#include <array>
#include "kinova_lowlevel/joint_types.h"
namespace kinova {
using ActuatorModes = std::array<ActuatorMode, kNumJoints>;
class Transport {
 public:
  virtual ~Transport() = default;
  virtual void connect() = 0;
  virtual void set_servoing_low_level() = 0;
  virtual void set_actuator_modes(const ActuatorModes&) = 0;
  virtual void exchange(const JointCommand&, JointFeedback&) = 0;  // blocking round-trip
  virtual void send(const JointCommand&) = 0;                      // non-blocking
  virtual void receive(JointFeedback&) = 0;
  virtual void safe_shutdown() = 0;
};
}  // namespace kinova
