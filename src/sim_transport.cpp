#include "kinova_lowlevel/sim_transport.h"

#include <time.h>

#include <array>

namespace kinova {

namespace {
inline int64_t ns_now() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return int64_t(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}
}  // namespace

SimTransport::SimTransport(const JointFeedback& initial, int latency_us)
    : state_(initial), latency_us_(latency_us) {}

void SimTransport::connect() {}
void SimTransport::set_servoing_low_level() {}
void SimTransport::set_actuator_modes(const ActuatorModes&) {}
void SimTransport::safe_shutdown() {}

void SimTransport::exchange(const JointCommand& cmd, JointFeedback& fb) {
  last_cmd_ = cmd;
  if (latency_us_ > 0) {
    const int64_t deadline = ns_now() + int64_t(latency_us_) * 1000LL;
    while (ns_now() < deadline) { /* busy-wait, off-RT friendly */ }
  }
  ++frame_;
  state_.frame_id = frame_;
  fb = state_;
}

void SimTransport::send(const JointCommand& cmd) {
  last_cmd_ = cmd;
  ++frame_;
  state_.frame_id = frame_;
}

void SimTransport::receive(JointFeedback& fb) { fb = state_; }

}  // namespace kinova
