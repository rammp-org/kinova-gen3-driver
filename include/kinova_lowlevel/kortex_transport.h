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
                  int rx_priority = 0, int rx_cpu = -1,
                  uint32_t cyclic_timeout_ms = kDefaultCyclicTimeoutMs);
  ~KortexTransport() override;
  void connect() override;
  void set_servoing_low_level() override;
  void set_actuator_modes(const ActuatorModes&, const JointCommand* hold) override;
  void exchange(const JointCommand&, JointFeedback&) override;
  void send(const JointCommand&) override;
  void receive(JointFeedback&) override;
  // release_low_level() + close the sessions and sockets. Idempotent.
  void safe_shutdown() override;
  // Every RPC bounded at 500 ms; moves past a refusal, stops at the first unanswered.
  void release_low_level() override;
  // GetArmState over TCP (500 ms bound) == ARMSTATE_IN_FAULT.
  bool arm_in_fault() override;
  void clear_faults() override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace kinova
