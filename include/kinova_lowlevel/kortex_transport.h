#pragma once
#include <memory>
#include <string>

#include "kinova_lowlevel/joint_types.h"
#include "kinova_lowlevel/transport.h"
namespace kinova {
// Real Gen3 over the KORTEX low-level cyclic API. The ONLY unit that includes
// KORTEX/protobuf headers (kept out of this header via pimpl).
class KortexTransport : public Transport {
 public:
  KortexTransport(std::string ip, std::string user = "admin", std::string password = "admin");
  ~KortexTransport() override;
  // Per-joint torque-sensor offset [N*m] of THIS arm: a robot-specific calibration
  // (measure it with tools/gravity_id). The actuators' torque loop tracks their own
  // sensor, so a sensor that reads `true + offset` makes the joint deliver
  // `command - offset` (verified on dojo 2026-10-20: compensating j3's +1.9 N*m cut
  // its gravity-comp drift 27x). Compensated here, the boundary that already owns
  // units and the reaction sign: torque commands go out as `tau + offset`, feedback
  // torque comes back as `measured - offset`. Applies to kTorque only. Call before
  // connect(); throws if called after, or on a non-finite or > 5 N*m entry.
  void set_torque_offset(const JointVec& offset);
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
