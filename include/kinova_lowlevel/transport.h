#pragma once
#include <array>
#include <stdexcept>

#include "kinova_lowlevel/joint_types.h"
namespace kinova {
// Thrown by exchange() when one round-trip did not complete within the
// transport's cyclic timeout: a single lost request or reply (KORTEX UDP has no
// retransmission). The caller's JointFeedback is left untouched. RtExecutor
// survives isolated ones; any other exception from exchange() stays fatal.
class TransportTimeout : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};
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
  // Runtime fault recovery (e.g. after a protective stop). Default no-op so
  // transports without faults (SimTransport) need not implement it.
  virtual void clear_faults() {}
};
}  // namespace kinova
