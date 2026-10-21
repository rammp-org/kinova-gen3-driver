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
  // Switches the actuators' control modes. `hold` is the incoming mode's first
  // command, sent unchanged while the switch is in progress: actuators that have
  // switched follow it instead of going limp (#92). Actuators not yet switched
  // follow what they received before. Never send zero torque in its place.
  virtual void set_actuator_modes(const ActuatorModes&, const JointCommand& hold) = 0;
  virtual void exchange(const JointCommand&, JointFeedback&) = 0;  // blocking round-trip
  virtual void send(const JointCommand&) = 0;                      // non-blocking
  virtual void receive(JointFeedback&) = 0;
  virtual void safe_shutdown() = 0;
  // Runtime fault recovery (e.g. after a protective stop). Default no-op so
  // transports without faults (SimTransport) need not implement it.
  virtual void clear_faults() {}
};
}  // namespace kinova
