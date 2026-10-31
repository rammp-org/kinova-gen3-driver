#include "kinova_lowlevel/kortex_transport.h"

#include <ActuatorConfigClientRpc.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <KDetailedException.h>
#include <RouterClient.h>
#include <SessionManager.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "kinova_lowlevel/arm_route.h"
#include "kinova_lowlevel/rt_system.h"
#include "kinova_lowlevel/units.h"
#include "kortex_cyclic.h"

namespace k_api = Kinova::Api;

namespace kinova {

namespace {
constexpr int kTcpPort = 10000;
constexpr int kUdpPort = 10001;

// KORTEX speaks percent (0..100) for gripper position, velocity and force; the driver
// speaks 0..1. This is the only place that conversion happens.
constexpr float kPctPerUnit = 100.0f;

inline float clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }

// Map our ActuatorMode -> KORTEX ActuatorConfig control mode.
k_api::ActuatorConfig::ControlMode to_kortex_mode(ActuatorMode m) {
  switch (m) {
    case ActuatorMode::kPosition:
      return k_api::ActuatorConfig::ControlMode::POSITION;
    case ActuatorMode::kVelocity:
      return k_api::ActuatorConfig::ControlMode::VELOCITY;
    case ActuatorMode::kTorque:
      return k_api::ActuatorConfig::ControlMode::TORQUE;
    case ActuatorMode::kCurrent:
      return k_api::ActuatorConfig::ControlMode::CURRENT;
  }
  return k_api::ActuatorConfig::ControlMode::POSITION;
}

// Bound for the cyclic reads in set_servoing_low_level(): setup, not the RT path,
// and the first replies after entering low-level servoing are unmeasured, so it
// keeps KORTEX's own default.
constexpr std::chrono::milliseconds kSetupTimeout{3000};

// Bound for the control-channel RPCs that hand the arm back or ask its state
// (release_low_level(), arm_in_fault()). KORTEX's default is 3000 ms; a revert
// over a dead TCP link (8 RPCs) would then block ~24 s.
const k_api::RouterClientSendOptions kShortRpc{false, 0, 500};

// One control-channel (TCP) RPC, with KORTEX's errors classified at the
// boundary so no KORTEX type escapes: an error reply from the arm is
// ArmRejected, an RPC timeout is TransportTimeout. KORTEX 2.8.0 reports an RPC
// timeout only as a std::runtime_error whose text starts "timeout detected"
// (e.g. "timeout detected: BaseClient::SetServoingMode", #59) -- the text is all
// that tells it apart. Anything else propagates unchanged.
template <class F>
auto rpc(F&& f) -> decltype(f()) {
  try {
    return f();
  } catch (const k_api::KDetailedException& e) {
    throw ArmRejected(std::string("KortexTransport: arm rejected the request: ") + e.what());
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()).rfind("timeout detected", 0) == 0) throw TransportTimeout(e.what());
    throw;
  }
}
}  // namespace

struct KortexTransport::Impl {
  std::string ip, user, password;
  int rx_priority, rx_cpu;

  // RAII-owned KORTEX objects (creation order matters; destroyed in reverse).
  std::unique_ptr<k_api::TransportClientTcp> tcp;
  std::unique_ptr<k_api::TransportClientUdp> udp;
  std::unique_ptr<k_api::RouterClient> tcp_router;
  std::unique_ptr<k_api::RouterClient> udp_router;
  std::unique_ptr<k_api::SessionManager> tcp_sess;
  std::unique_ptr<k_api::SessionManager> udp_sess;
  std::unique_ptr<k_api::Base::BaseClient> base;
  // Unused for calls (they go through kortex_cyclic_call()); kept for what its
  // constructor registers with udp_router.
  std::unique_ptr<k_api::BaseCyclic::BaseCyclicClient> base_cyclic;
  std::unique_ptr<k_api::ActuatorConfig::ActuatorConfigClient> act_cfg;

  k_api::BaseCyclic::Command cmd_;
  k_api::BaseCyclic::Feedback fb_;  // latest feedback (kept to seed passthrough positions)
  std::chrono::milliseconds cyclic_timeout_;  // every cyclic call on the RT path

  int n_ = 0;
  uint16_t frame_id_ = 0;
  bool connected_ = false;
  bool unanswered_ = false;  // the last revert hit an RPC the arm never answered
  bool low_level_ = false;

  explicit Impl(std::string ip_in, std::string user_in, std::string password_in, int rx_prio,
                int rx_cpu_in, uint32_t cyclic_timeout_ms)
      : ip(std::move(ip_in)),
        user(std::move(user_in)),
        password(std::move(password_in)),
        rx_priority(rx_prio),
        rx_cpu(rx_cpu_in),
        cyclic_timeout_(cyclic_timeout_ms) {}

  // See kortex_cyclic_call(): bounded, and a timeout is a TransportTimeout.
  k_api::Frame cyclic_call(uint32_t fct, const std::string& payload,
                           std::chrono::milliseconds timeout) {
    return kortex_cyclic_call(*udp_router, fct, payload, timeout);
  }

  // Refresh: send cmd_, store the feedback in fb_. fb_ is untouched on a timeout.
  void refresh(std::chrono::milliseconds timeout) {
    k_api::Frame reply =
        cyclic_call(k_api::BaseCyclic::eUidRefresh, cmd_.SerializeAsString(), timeout);
    parse_feedback(reply);
  }

  void parse_feedback(const k_api::Frame& reply) {
    if (!fb_.ParseFromString(reply.payload())) {
      throw std::runtime_error("KortexTransport: unparseable BaseCyclic feedback");
    }
  }

  // Refresh that re-sends current measured positions + the given torque, keeping
  // the cyclic channel alive (used between per-actuator control-mode switches).
  void pump(const std::array<float, kNumJoints>& torque) {
    for (int i = 0; i < n_; ++i) {
      cmd_.mutable_actuators(i)->set_position(fb_.actuators(i).position());
      cmd_.mutable_actuators(i)->set_torque_joint(torque[i]);
    }
    refresh(cyclic_timeout_);
  }

  void fill_feedback(JointFeedback& fb) {
    for (int i = 0; i < n_; ++i) {
      fb.q[i] = wrap_to_pi(double(fb_.actuators(i).position()) * kDeg2Rad);
      fb.qd[i] = double(fb_.actuators(i).velocity()) * kDeg2Rad;
      // KORTEX reports feedback torque REACTION-signed (measured ~ -g(q) at free
      // hold, verified on the lab arm) while it takes commands motor-signed.
      // Normalize here, the same boundary that turns degrees into radians, so
      // everything past Transport speaks ONE convention: the torque the actuator
      // applies, ~ +g(q) at free hold. See JointFeedback::tau.
      fb.tau[i] = -double(fb_.actuators(i).torque());
      fb.current[i] = double(fb_.actuators(i).current_motor());
    }
    fb.frame_id = fb_.frame_id();
    bool fault = false;
    for (int i = 0; i < n_; ++i) {
      if (fb_.actuators(i).fault_bank_a() != 0u || fb_.actuators(i).fault_bank_b() != 0u) {
        fault = true;
      }
    }
    // The base's own fault banks and state too: an e-stop or a protective stop
    // can fault the base with every actuator bank clear.
    const auto& base = fb_.base();
    if (base.fault_bank_a() != 0u || base.fault_bank_b() != 0u ||
        base.active_state() == k_api::Common::ARMSTATE_IN_FAULT) {
      fault = true;
    }
    fb.fault = fault;
    fb.low_level = base.active_state() == k_api::Common::ARMSTATE_SERVOING_LOW_LEVEL;
    // Gripper feedback from the interconnect. A robot with no interconnect gripper
    // reports zero motors -- record that as ABSENT rather than leaving position at 0,
    // which is indistinguishable from an attached, fully-open gripper.
    const auto& ic = fb_.interconnect();
    if (ic.gripper_feedback().motor_size() > 0) {
      const auto& m = ic.gripper_feedback().motor(0);
      fb.gripper.present = true;
      fb.gripper.position = float(m.position()) / kPctPerUnit;
      fb.gripper.current = float(m.current_motor());
      // MotorFeedback has no force field; effort is derived, normalized, and NOT Newtons.
      const float e = std::fabs(fb.gripper.current) / kGripperMaxCurrentA;
      fb.gripper.effort = e > 1.0f ? 1.0f : e;
    } else {
      // Zero the rest too: fb is a caller-owned, reused JointFeedback, and leaving
      // position/effort/current at whatever it held before would let a
      // gripper that just disappeared look like one still reporting its last state.
      fb.gripper = GripperFeedback{};
    }
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
    // Gripper rides inside the same cyclic command, in the interconnect's
    // gripper_command motor message. Position, velocity (speed) and force all convert
    // the same way, 0..1 -> percent (0..100). The source is a library GripperController
    // (any caller stamping through it), not a specific app or path.
    if (cmd.gripper.active) {
      // NOTE: the interconnect/gripper submessages are allocated lazily on the first
      // commanded cycle -- a one-time heap alloc on the RT path. Deliberate: it keeps
      // the gripper UNcommanded until the first gripper command, so the gripper stays
      // limp at startup rather than being actuated by a seeded default. Pre-seeding in
      // set_servoing_low_level() would make this alloc-free but would actuate from
      // cycle 1; still to be validated on hardware.
      //
      // RETRANSMISSION, NOT CESSATION: once allocated here, this submessage stays in
      // cmd_ and is re-sent whole, every cycle, by Refresh(cmd_, 0) -- including every
      // cycle where cmd.gripper.active is false, because this block simply does not run
      // then and nothing clears the submessage. So after release() (see
      // GripperController), the gripper keeps receiving "go to position P at speed S,
      // force cap F" at 1 kHz indefinitely; it is not going quiet. The grip holds, but
      // by continuous re-command against the last target, not by the arm ceasing to
      // command it -- a duty-cycle/thermal consideration on a gripper stalled against
      // an object. See GripperController::release()'s comment.
      auto* gripper = cmd_.mutable_interconnect()->mutable_gripper_command();
      if (gripper->motor_cmd_size() == 0) gripper->add_motor_cmd();
      auto* m = gripper->mutable_motor_cmd(0);
      m->set_position(clamp01(cmd.gripper.position) * kPctPerUnit);
      m->set_velocity(clamp01(cmd.gripper.speed) * kPctPerUnit);
      // A CEILING on motor current, not a force setpoint -- see GripperCommand.
      m->set_force(clamp01(cmd.gripper.force) * kPctPerUnit);
    }
  }
};

KortexTransport::KortexTransport(std::string ip, std::string user, std::string password,
                                 int rx_priority, int rx_cpu, uint32_t cyclic_timeout_ms)
    : impl_(std::make_unique<Impl>(std::move(ip), std::move(user), std::move(password), rx_priority,
                                   rx_cpu, cyclic_timeout_ms)) {}

KortexTransport::~KortexTransport() {
  // Defensive: revert + tear down if still connected. Never throw from dtor.
  try {
    safe_shutdown();
  } catch (...) {
  }
}

void KortexTransport::connect() {
  auto& I = *impl_;
  // With no direct route, connect() leaves by the default route and blocks ~127 s in TCP SYN
  // retries. Fail the attempt at once instead, saying why.
  if (const std::string p = arm_route_problem(I.ip); !p.empty())
    throw std::runtime_error("KortexTransport: " + p);
  I.tcp = std::make_unique<k_api::TransportClientTcp>();
  I.udp = std::make_unique<k_api::TransportClientUdp>();
  I.tcp->connect(I.ip, kTcpPort);
  I.udp->connect(I.ip, kUdpPort);
  // connect() starts the SDK's UDP receive thread, which reads every Refresh() reply off the
  // socket, so the RT loop waits on it each cycle. Raise only that one: the TCP receive and
  // session keepalive threads stay normal.
  if (I.rx_priority > 0) {
    // A connect() that failed may not have started the thread, and a default std::thread's
    // native_handle() is not a valid pthread_t.
    const std::string err =
        I.udp->m_receiveThread.joinable()
            ? raise_thread(I.udp->m_receiveThread.native_handle(), I.rx_priority, I.rx_cpu)
            : "not started";
    if (err.empty()) {
      std::fprintf(stderr, "KortexTransport: UDP receive thread SCHED_FIFO %d, CPU %d\n",
                   I.rx_priority, I.rx_cpu);
    } else {
      std::fprintf(stderr, "KortexTransport: UDP receive thread: %s\n", err.c_str());
    }
  }

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

  // No ClearFaults() here: an arm that is faulted at connect stays faulted until
  // someone clears it explicitly (ArmLink waits for that).
  I.n_ = rpc([&] { return I.base->GetActuatorCount().count(); });
  // A booting arm answers on its base before its actuators are up, and reports
  // none: not ready yet (ArmLink retries), unlike a wrong count.
  if (I.n_ == 0)
    throw std::runtime_error("KortexTransport: the arm reports no actuators yet (still booting?)");
  // Guard the fixed-size JointVec: this build is compiled for kNumJoints DOF.
  // A robot reporting a different count would otherwise index out of bounds in
  // fill_feedback/write_command. Hard throw (see no-silent-footgun principle).
  if (I.n_ != kNumJoints) {
    // invalid_argument (a logic_error): a configuration error, not an arm event,
    // so ArmLink lets it escape instead of retrying forever.
    throw std::invalid_argument(
        "KortexTransport: robot reports " + std::to_string(I.n_) +
        " actuators but this build expects kNumJoints=" + std::to_string(kNumJoints));
  }
  // The count can arrive before every actuator has registered: right after a boot the
  // base accepted low-level servoing, then refused SetControlMode on device 1
  // (INVALID_DEVICE). A read on each actuator is refused the same way
  // until it is up, and a failure here is retried (ArmLink), not latched.
  for (int id = 1; id <= I.n_; ++id) {
    try {
      rpc([&] { return I.act_cfg->GetControlMode(id); });
    } catch (const ArmRejected& e) {
      throw std::runtime_error("KortexTransport: actuator " + std::to_string(id) +
                               " not registered yet (still booting?): " + e.what());
    }
  }
  I.connected_ = true;
}

void KortexTransport::set_servoing_low_level() {
  auto& I = *impl_;
  k_api::Base::ServoingModeInformation sm;
  sm.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
  rpc([&] { I.base->SetServoingMode(sm); });
  // Set now, not after the seed: if the seed fails the base is already in
  // low-level, and release_low_level() must still hand it back.
  I.low_level_ = true;

  // Seed: read feedback, build a command echoing current positions, one Refresh.
  I.parse_feedback(I.cyclic_call(k_api::BaseCyclic::eUidRefreshFeedback, "", kSetupTimeout));
  I.cmd_.clear_actuators();
  // Re-entry (after a reconnect or a cleared fault) must not resend the gripper
  // target from before the outage: the operator may have moved it since. Clearing
  // the submessage leaves the gripper uncommanded, as at first startup, until the
  // next gripper command. (Cost: write_command's one-time allocation of that
  // submessage now happens once per re-entry rather than once per process.)
  I.cmd_.clear_interconnect();
  // Seed torque with the measured torque (reaction-signed, so negated) rather
  // than 0: after a link loss the revert may never have reached the arm, and an
  // actuator still in TORQUE mode would take this seed as "apply nothing" and
  // drop. In POSITION mode the field is ignored. The pumps that follow during a
  // mode switch hold the same way (set_actuator_modes).
  for (int i = 0; i < I.n_; ++i) {
    auto* a = I.cmd_.add_actuators();
    a->set_position(I.fb_.actuators(i).position());
    a->set_torque_joint(-I.fb_.actuators(i).torque());
  }
  I.refresh(kSetupTimeout);
}

void KortexTransport::set_actuator_modes(const ActuatorModes& modes) {
  auto& I = *impl_;
  // Per-actuator SetControlMode (1-indexed device id), pumping the cyclic
  // channel between each call so it does not time out during the switch. A pump
  // only keeps the channel alive, so a lost one is skipped;
  // kMaxConsecutiveTimeouts in a row rethrows the TransportTimeout.
  //
  // Every pump commands the torque measured once, here, before the first switch
  // (reaction-signed, so negated, as the seed does) rather than 0: an actuator
  // in TORQUE mode takes a pump's torque as its command, so one left in TORQUE
  // would drop until its own turn, one switched to it until the switch ends
  // (#92). Not re-read each pump: in TORQUE the actuator drives its own sensor
  // to the command, so feeding the reading back would let the sensor offset
  // accumulate. Uncalibrated, the hold is off by each joint's sensor offset for
  // the length of the switch. In POSITION mode the field is ignored.
  std::array<float, kNumJoints> hold{};
  for (int i = 0; i < I.n_; ++i) hold[i] = -I.fb_.actuators(i).torque();
  int misses = 0;
  auto pump = [&] {
    try {
      I.pump(hold);
      misses = 0;
    } catch (const TransportTimeout&) {
      if (++misses >= kMaxConsecutiveTimeouts) throw;
    }
  };
  for (int idx = 1; idx <= I.n_; ++idx) {
    k_api::ActuatorConfig::ControlModeInformation cm_msg;
    cm_msg.set_control_mode(to_kortex_mode(modes[idx - 1]));
    rpc([&] { I.act_cfg->SetControlMode(cm_msg, idx); });
    pump();
  }
  for (int k = 0; k < 10; ++k) pump();
}

void KortexTransport::exchange(const JointCommand& cmd, JointFeedback& fb) {
  auto& I = *impl_;
  I.write_command(cmd);
  I.refresh(I.cyclic_timeout_);  // on a TransportTimeout fb keeps the last good feedback
  I.fill_feedback(fb);
}

void KortexTransport::send(const JointCommand& cmd) {
  auto& I = *impl_;
  I.write_command(cmd);
  I.cyclic_call(k_api::BaseCyclic::eUidRefreshCommand, I.cmd_.SerializeAsString(),
                I.cyclic_timeout_);
}

void KortexTransport::receive(JointFeedback& fb) {
  auto& I = *impl_;
  I.parse_feedback(I.cyclic_call(k_api::BaseCyclic::eUidRefreshFeedback, "", I.cyclic_timeout_));
  I.fill_feedback(fb);
}

void KortexTransport::release_low_level() {
  auto& I = *impl_;
  // Revert the robot only if we actually put it in low-level servoing / changed
  // actuator control modes. Each RPC is bounded at kShortRpc. A refusal (an error
  // reply: the link is up, that one request was rejected, e.g. a faulted
  // actuator) moves on to the next; anything else (a timeout) stops the revert:
  // a link that did not answer one will not answer the rest, and each would
  // otherwise block for its own timeout.
  if (!I.low_level_) return;
  I.low_level_ = false;
  bool ok = true;
  auto attempt = [&](auto&& call) {
    try {
      call();
    } catch (const k_api::KDetailedException&) {
    } catch (...) {
      ok = false;
    }
  };
  if (I.act_cfg && I.n_ > 0) {
    k_api::ActuatorConfig::ControlModeInformation cm_msg;
    cm_msg.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
    for (int idx = 1; ok && idx <= I.n_; ++idx)
      attempt([&] { I.act_cfg->SetControlMode(cm_msg, idx, kShortRpc); });
  }
  if (ok && I.base) {
    k_api::Base::ServoingModeInformation sm;
    sm.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    attempt([&] { I.base->SetServoingMode(sm, 0, kShortRpc); });
  }
  // Let the mode/servoing revert settle before anything else talks to the arm
  // (mirrors the validated prototype; avoids closing mid-transition). Skipped
  // when the arm stopped answering: there is no transition to wait for.
  if (ok) std::this_thread::sleep_for(std::chrono::milliseconds(500));
  I.unanswered_ = !ok;
}

bool KortexTransport::arm_in_fault() {
  auto& I = *impl_;
  if (!I.base) throw TransportTimeout("KortexTransport: not connected");
  const auto st = rpc([&] { return I.base->GetArmState(0, kShortRpc); });
  return st.active_state() == k_api::Common::ARMSTATE_IN_FAULT;
}

void KortexTransport::safe_shutdown() {
  auto& I = *impl_;
  // NOTE: we do NOT early-return on !connected_. A connect() that threw partway
  // leaves some network objects alive; they must still be torn down.
  release_low_level();

  // CloseSession() takes no send options and waits KORTEX's default 3 s for a
  // reply. Skip it when the revert found the arm not answering: the arm drops the
  // session itself after the 2 s connection inactivity timeout set at connect.
  if (I.tcp_sess && !I.unanswered_) {
    try {
      I.tcp_sess->CloseSession();
    } catch (...) {
    }
  }
  if (I.udp_sess && !I.unanswered_) {
    try {
      I.udp_sess->CloseSession();
    } catch (...) {
    }
  }
  I.unanswered_ = false;
  if (I.tcp_router) I.tcp_router->SetActivationStatus(false);
  if (I.udp_router) I.udp_router->SetActivationStatus(false);
  if (I.tcp) {
    try {
      I.tcp->disconnect();
    } catch (...) {
    }
  }
  if (I.udp) {
    try {
      I.udp->disconnect();
    } catch (...) {
    }
  }

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
}

void KortexTransport::clear_faults() {
  auto& I = *impl_;
  if (!I.connected_ || !I.base) return;
  try {
    I.base->ClearFaults();
  } catch (...) {
  }
  // A protective stop can drop the robot out of low-level servoing. If we were
  // in low-level mode, re-enter + re-seed so the RT loop can resume commanding.
  if (I.low_level_) {
    try {
      set_servoing_low_level();
    } catch (...) {
    }
  }
}

}  // namespace kinova
