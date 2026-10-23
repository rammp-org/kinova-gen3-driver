#pragma once
#include <cstdint>
#include <memory>
#include <string>

#include "kinova_lowlevel/transport.h"
namespace kinova {
// Real Gen3 over the KORTEX low-level cyclic API. The ONLY unit that includes
// KORTEX/protobuf headers (kept out of this header via pimpl).
class KortexTransport : public Transport {
 public:
  // rx_priority / rx_cpu: SCHED_FIFO priority and CPU for the SDK's UDP receive thread, which
  // every exchange() waits on; pass one below the executor's priority and its CPU. 0 leaves it a
  // normal thread.
  //
  // cyclic_timeout_ms bounds each cyclic round-trip on the RT path: exchange(),
  // send(), receive() and the pumps in set_actuator_modes() (KORTEX's own default
  // is 3000 ms). Normal RTT is ~0.4 ms (p99 0.7 ms, worst seen 7.9 ms under
  // network load); 20 ms clears that with margin, and expiry throws
  // TransportTimeout. set_servoing_low_level()'s seed reads keep 3000 ms.
  KortexTransport(std::string ip, std::string user = "admin", std::string password = "admin",
                  int rx_priority = 0, int rx_cpu = -1, uint32_t cyclic_timeout_ms = 20);
  ~KortexTransport() override;
  void connect() override;
  void set_servoing_low_level() override;
  void set_actuator_modes(const ActuatorModes&) override;
  void exchange(const JointCommand&, JointFeedback&) override;
  void send(const JointCommand&) override;
  void receive(JointFeedback&) override;
  void safe_shutdown() override;
  void clear_faults() override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace kinova
