#pragma once
#include <array>
#include <cstdint>
#include <stdexcept>

#include "kinova_lowlevel/joint_types.h"
namespace kinova {
// Thrown by a cyclic call (exchange(), send(), receive(), the pumps in
// set_actuator_modes()) when one round-trip did not complete within the
// transport's cyclic timeout: a lost request or reply (KORTEX UDP has no
// retransmission). The caller's JointFeedback is left untouched. RtExecutor
// survives isolated ones from exchange(); any other exception stays fatal.
class TransportTimeout : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};
// Thrown when the arm refused a request: an error reply (KORTEX's
// KDetailedException, e.g. WRONG_SERVOING_MODE after a protective stop or when
// another client changed the servoing mode). Not a lost frame: retrying the same
// request gets the same answer until the arm's state changes. ArmLink treats it
// as a fault that waits for an explicit clear.
class ArmRejected : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};
// TransportTimeouts in a row taken as the link being down rather than dropping
// frames: that many cyclic timeouts with no reply (60 ms at KortexTransport's
// default 20 ms). ExecutorConfig::max_consecutive_misses defaults to it; the
// pumps in KortexTransport::set_actuator_modes() always use it.
inline constexpr int kMaxConsecutiveTimeouts = 3;
// KortexTransport's default bound on one cyclic round-trip, in ms. RtExecutor
// caps the dt it hands compute() at max_consecutive_misses of these.
inline constexpr uint32_t kDefaultCyclicTimeoutMs = 20;
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
  // Hand the arm back to its base: actuators to POSITION, base to single-level
  // servoing. Keeps the connection, unlike safe_shutdown(). No-op when not in
  // low-level servoing. Best-effort: never throws.
  virtual void release_low_level() = 0;
  // Whether the base reports itself in fault, asked over the control channel so
  // it answers in any servoing mode. Throws TransportTimeout if unanswered.
  virtual bool arm_in_fault() = 0;
  // Runtime fault recovery (e.g. after a protective stop). Default no-op so
  // transports without faults (SimTransport) need not implement it.
  virtual void clear_faults() {}
};
}  // namespace kinova
