#include "kinova_lowlevel/kortex_transport.h"

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "kinova_lowlevel/units.h"

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>

namespace k_api = Kinova::Api;

namespace kinova {

namespace {
constexpr int kTcpPort = 10000;
constexpr int kUdpPort = 10001;

// Map our ActuatorMode -> KORTEX ActuatorConfig control mode.
k_api::ActuatorConfig::ControlMode to_kortex_mode(ActuatorMode m) {
  switch (m) {
    case ActuatorMode::kPosition: return k_api::ActuatorConfig::ControlMode::POSITION;
    case ActuatorMode::kVelocity: return k_api::ActuatorConfig::ControlMode::VELOCITY;
    case ActuatorMode::kTorque:   return k_api::ActuatorConfig::ControlMode::TORQUE;
    case ActuatorMode::kCurrent:  return k_api::ActuatorConfig::ControlMode::CURRENT;
  }
  return k_api::ActuatorConfig::ControlMode::POSITION;
}
}  // namespace

struct KortexTransport::Impl {
  std::string ip, user, password;

  // RAII-owned KORTEX objects (creation order matters; destroyed in reverse).
  std::unique_ptr<k_api::TransportClientTcp> tcp;
  std::unique_ptr<k_api::TransportClientUdp> udp;
  std::unique_ptr<k_api::RouterClient> tcp_router;
  std::unique_ptr<k_api::RouterClient> udp_router;
  std::unique_ptr<k_api::SessionManager> tcp_sess;
  std::unique_ptr<k_api::SessionManager> udp_sess;
  std::unique_ptr<k_api::Base::BaseClient> base;
  std::unique_ptr<k_api::BaseCyclic::BaseCyclicClient> base_cyclic;
  std::unique_ptr<k_api::ActuatorConfig::ActuatorConfigClient> act_cfg;

  k_api::BaseCyclic::Command cmd_;
  k_api::BaseCyclic::Feedback fb_;  // latest feedback (kept to seed passthrough positions)

  int n_ = 0;
  uint16_t frame_id_ = 0;
  bool connected_ = false;
  bool low_level_ = false;

  explicit Impl(std::string ip_in, std::string user_in, std::string password_in)
      : ip(std::move(ip_in)), user(std::move(user_in)), password(std::move(password_in)) {}

  // Refresh that re-sends current measured positions + zero torque, keeping the
  // cyclic channel alive (used between per-actuator control-mode switches).
  void pump() {
    for (int i = 0; i < n_; ++i) {
      cmd_.mutable_actuators(i)->set_position(fb_.actuators(i).position());
      cmd_.mutable_actuators(i)->set_torque_joint(0.0f);
    }
    fb_ = base_cyclic->Refresh(cmd_, 0);
  }

  void fill_feedback(JointFeedback& fb) {
    for (int i = 0; i < n_; ++i) {
      fb.q[i] = wrap_to_pi(double(fb_.actuators(i).position()) * kDeg2Rad);
      fb.qd[i] = double(fb_.actuators(i).velocity()) * kDeg2Rad;
      fb.tau[i] = double(fb_.actuators(i).torque());
      fb.current[i] = double(fb_.actuators(i).current_motor());
    }
    fb.frame_id = fb_.frame_id();
    bool fault = false;
    for (int i = 0; i < n_; ++i) {
      if (fb_.actuators(i).fault_bank_a() != 0u ||
          fb_.actuators(i).fault_bank_b() != 0u) {
        fault = true;
      }
    }
    fb.fault = fault;
  }

  // Write a JointCommand into cmd_, bumping frame_id_ and per-actuator command_ids.
  void write_command(const JointCommand& cmd) {
    frame_id_ = uint16_t(frame_id_ + 1);
    cmd_.set_frame_id(frame_id_);
    for (int i = 0; i < n_; ++i) {
      auto* a = cmd_.mutable_actuators(i);
      // KORTEX requires a sane position setpoint in every mode (avoids a step on
      // mode revert); each branch sets it — commanded position in kPosition,
      // measured-position passthrough otherwise.
      switch (cmd.mode) {
        case ActuatorMode::kPosition:
          a->set_position(float(rad_to_deg(cmd.position)[i]));
          break;
        case ActuatorMode::kVelocity:
          a->set_position(fb_.actuators(i).position());
          a->set_velocity(float(rad_to_deg(cmd.velocity)[i]));
          break;
        case ActuatorMode::kTorque:
          a->set_position(fb_.actuators(i).position());
          a->set_torque_joint(float(cmd.torque[i]));
          break;
        case ActuatorMode::kCurrent:
          // NOTE: JointCommand has no dedicated current field yet, so kCurrent
          // reuses cmd.torque as the motor-current setpoint [A]. kCurrent is not
          // exercised by any ControlMode today; add a `current` field to
          // JointCommand before relying on this path. See no-silent-footgun.
          a->set_position(fb_.actuators(i).position());
          a->set_current_motor(float(cmd.torque[i]));
          break;
      }
      a->set_command_id(frame_id_);
    }
  }
};

KortexTransport::KortexTransport(std::string ip, std::string user, std::string password)
    : impl_(std::make_unique<Impl>(std::move(ip), std::move(user), std::move(password))) {}

KortexTransport::~KortexTransport() {
  // Defensive: revert + tear down if still connected. Never throw from dtor.
  try {
    safe_shutdown();
  } catch (...) {
  }
}

void KortexTransport::connect() {
  auto& I = *impl_;
  I.tcp = std::make_unique<k_api::TransportClientTcp>();
  I.udp = std::make_unique<k_api::TransportClientUdp>();
  I.tcp->connect(I.ip, kTcpPort);
  I.udp->connect(I.ip, kUdpPort);

  auto on_err = [](k_api::KError) {};
  I.tcp_router = std::make_unique<k_api::RouterClient>(I.tcp.get(), on_err);
  I.udp_router = std::make_unique<k_api::RouterClient>(I.udp.get(), on_err);

  auto sinfo = k_api::Session::CreateSessionInfo();
  sinfo.set_username(I.user);
  sinfo.set_password(I.password);
  sinfo.set_session_inactivity_timeout(60000);
  sinfo.set_connection_inactivity_timeout(2000);

  I.tcp_sess = std::make_unique<k_api::SessionManager>(I.tcp_router.get());
  I.udp_sess = std::make_unique<k_api::SessionManager>(I.udp_router.get());
  I.tcp_sess->CreateSession(sinfo);
  I.udp_sess->CreateSession(sinfo);

  I.base = std::make_unique<k_api::Base::BaseClient>(I.tcp_router.get());
  I.base_cyclic = std::make_unique<k_api::BaseCyclic::BaseCyclicClient>(I.udp_router.get());
  I.act_cfg = std::make_unique<k_api::ActuatorConfig::ActuatorConfigClient>(I.tcp_router.get());

  try {
    I.base->ClearFaults();
  } catch (...) {
  }

  I.n_ = I.base->GetActuatorCount().count();
  // Guard the fixed-size JointVec: this build is compiled for kNumJoints DOF.
  // A robot reporting a different count would otherwise index out of bounds in
  // fill_feedback/write_command. Hard throw (see no-silent-footgun principle).
  if (I.n_ != kNumJoints) {
    throw std::runtime_error(
        "KortexTransport: robot reports " + std::to_string(I.n_) +
        " actuators but this build expects kNumJoints=" + std::to_string(kNumJoints));
  }
  I.connected_ = true;
}

void KortexTransport::set_servoing_low_level() {
  auto& I = *impl_;
  k_api::Base::ServoingModeInformation sm;
  sm.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
  I.base->SetServoingMode(sm);

  // Seed: read feedback, build a command echoing current positions, one Refresh.
  I.fb_ = I.base_cyclic->RefreshFeedback();
  I.cmd_.clear_actuators();
  for (int i = 0; i < I.n_; ++i) {
    I.cmd_.add_actuators()->set_position(I.fb_.actuators(i).position());
  }
  I.fb_ = I.base_cyclic->Refresh(I.cmd_);
  I.low_level_ = true;
}

void KortexTransport::set_actuator_modes(const ActuatorModes& modes) {
  auto& I = *impl_;
  // Per-actuator SetControlMode (1-indexed device id), pumping the cyclic
  // channel between each call so it does not time out during the switch.
  for (int idx = 1; idx <= I.n_; ++idx) {
    k_api::ActuatorConfig::ControlModeInformation cm_msg;
    cm_msg.set_control_mode(to_kortex_mode(modes[idx - 1]));
    I.act_cfg->SetControlMode(cm_msg, idx);
    I.pump();
  }
  for (int k = 0; k < 10; ++k) I.pump();
}

void KortexTransport::exchange(const JointCommand& cmd, JointFeedback& fb) {
  auto& I = *impl_;
  I.write_command(cmd);
  I.fb_ = I.base_cyclic->Refresh(I.cmd_, 0);
  I.fill_feedback(fb);
}

void KortexTransport::send(const JointCommand& cmd) {
  auto& I = *impl_;
  I.write_command(cmd);
  I.base_cyclic->RefreshCommand(I.cmd_, 0);
}

void KortexTransport::receive(JointFeedback& fb) {
  auto& I = *impl_;
  I.fb_ = I.base_cyclic->RefreshFeedback();
  I.fill_feedback(fb);
}

void KortexTransport::safe_shutdown() {
  auto& I = *impl_;
  // NOTE: we do NOT early-return on !connected_. A connect() that threw partway
  // leaves some network objects alive; they must still be torn down. We only
  // skip the robot-STATE revert (below) when we never entered low-level mode.

  // Revert the robot to a safe state only if we actually put it in low-level
  // servoing / changed actuator control modes.
  if (I.low_level_) {
    if (I.act_cfg && I.n_ > 0) {
      k_api::ActuatorConfig::ControlModeInformation cm_msg;
      cm_msg.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
      for (int idx = 1; idx <= I.n_; ++idx) {
        try {
          I.act_cfg->SetControlMode(cm_msg, idx);
        } catch (...) {
        }
      }
    }
    if (I.base) {
      k_api::Base::ServoingModeInformation sm;
      sm.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
      try {
        I.base->SetServoingMode(sm);
      } catch (...) {
      }
    }
    // Let the mode/servoing revert settle before tearing the session down
    // (mirrors the validated prototype; avoids closing mid-transition).
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  if (I.tcp_sess) {
    try { I.tcp_sess->CloseSession(); } catch (...) {}
  }
  if (I.udp_sess) {
    try { I.udp_sess->CloseSession(); } catch (...) {}
  }
  if (I.tcp_router) I.tcp_router->SetActivationStatus(false);
  if (I.udp_router) I.udp_router->SetActivationStatus(false);
  if (I.tcp) { try { I.tcp->disconnect(); } catch (...) {} }
  if (I.udp) { try { I.udp->disconnect(); } catch (...) {} }

  // Tear down clients/sessions/routers/transports (reverse of creation).
  I.act_cfg.reset();
  I.base_cyclic.reset();
  I.base.reset();
  I.tcp_sess.reset();
  I.udp_sess.reset();
  I.tcp_router.reset();
  I.udp_router.reset();
  I.tcp.reset();
  I.udp.reset();

  I.connected_ = false;
  I.low_level_ = false;
}

}  // namespace kinova
