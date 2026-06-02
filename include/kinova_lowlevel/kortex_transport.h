#pragma once
#include <memory>
#include <string>
#include "kinova_lowlevel/transport.h"
namespace kinova {
// Real Gen3 over the KORTEX low-level cyclic API. The ONLY unit that includes
// KORTEX/protobuf headers (kept out of this header via pimpl).
class KortexTransport : public Transport {
 public:
  KortexTransport(std::string ip, std::string user = "admin", std::string password = "admin");
  ~KortexTransport() override;
  void connect() override;
  void set_servoing_low_level() override;
  void set_actuator_modes(const ActuatorModes&) override;
  void exchange(const JointCommand&, JointFeedback&) override;
  void send(const JointCommand&) override;
  void receive(JointFeedback&) override;
  void safe_shutdown() override;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace kinova
