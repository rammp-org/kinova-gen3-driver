#pragma once
#include "kinova_lowlevel/transport.h"
namespace kinova {
// Fake robot for tests/benchmarks with no hardware. Echoes state, advances a
// frame counter, optionally busy-waits `latency_us` to mimic the round-trip.
class SimTransport : public Transport {
 public:
  explicit SimTransport(const JointFeedback& initial, int latency_us = 0);
  void connect() override;
  void set_servoing_low_level() override;
  void set_actuator_modes(const ActuatorModes&) override;
  void exchange(const JointCommand&, JointFeedback&) override;
  void send(const JointCommand&) override;
  void receive(JointFeedback&) override;
  void safe_shutdown() override;
 private:
  JointFeedback state_;
  JointCommand last_cmd_;
  int latency_us_ = 0;
  uint64_t frame_ = 0;
};
}  // namespace kinova
