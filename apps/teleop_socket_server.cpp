// teleop_socket_server — bridges the Python VR-teleop supervisor to an impedance
// control mode over UDP. Two modes are selectable at startup, both driven through
// the same PoseTargetSink seam, so the Python side is identical either way:
//
//   default            CartesianImpedanceMode — task-space spring. The 7th DOF is
//                      uncommanded (only null-space-biased), so the elbow drifts.
//   --joint-impedance  JointImpedanceMode — solves IK for the commanded pose and
//                      servos ALL 7 joints. No free DOF; posture is deterministic.
//                      Use this when teleop keeps wandering into awkward configs.
//
// The RT loop (RtExecutor) owns the main thread; two helper threads handle the
// socket:
//
//   * rx thread:       receive POSE_TARGET / SET_GAINS / CONTROL, apply via the
//                      mode's non-RT setters (single writer, as required). Also
//                      forwards POSE_TARGET.gripper (0–1) via a GripperInjector
//                      decorator, stamping it into each JointCommand.
//   * feedback thread: read the latest JointFeedback snapshot, compute EE pose
//                      via its own Dynamics, and stream FEEDBACK back to the
//                      last client address. Reports the measured gripper position
//                      in FEEDBACK.gripper_state.
//
// Robot feedback is captured without touching the driver core: FeedbackTap wraps
// the Transport and snapshots `fb` at the exact point the RT loop reads it.
//
//   ./teleop_socket_server --sim  --urdf ../models/gen3_7dof_2f85.urdf --port 9095
//   ./teleop_socket_server --ip 192.168.1.10 --urdf ../models/gen3_7dof_2f85.urdf
//   ./teleop_socket_server --ip 192.168.1.10 --joint-impedance --jkp 80 --zeta 0.5
//
// SimTransport is echo-only (the arm will not move); use it for protocol
// bring-up. Real motion requires the KORTEX build against the arm.
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cstring>
#include <csignal>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <Eigen/Dense>

#include "kinova_lowlevel/cartesian_impedance_mode.h"
#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/feedback_tap.h"
#include "kinova_lowlevel/joint_impedance_mode.h"
#include "kinova_lowlevel/pose_target_sink.h"
#include "kinova_lowlevel/rt_executor.h"
#include "kinova_lowlevel/sim_transport.h"
#include "kinova_lowlevel/telemetry.h"
#include "kinova_lowlevel/teleop_protocol.h"
#include "kinova_lowlevel/transport.h"
#ifndef KINOVA_NO_KORTEX
#include "kinova_lowlevel/kortex_transport.h"
#endif

using namespace kinova;
namespace tp = kinova::teleop;

namespace {
std::atomic<bool> g_stop{false};
void on_sigint(int) { g_stop.store(true); }

int64_t ns_now() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return int64_t(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

// Transport decorator: carries the latest gripper target (set by the rx thread)
// and stamps it into each JointCommand on its way to the wrapped transport. Keeps
// gripper control orthogonal to any ControlMode. The atomics make the rx-thread
// write / RT-thread read race-free; the JointCommand copy is POD-sized (no alloc).
class GripperInjector : public Transport {
 public:
  explicit GripperInjector(Transport& inner) : inner_(inner) {}

  // Called by the rx thread when a POSE_TARGET arrives.
  void set_gripper(float g) {
    if (g < 0.0f) g = 0.0f;
    if (g > 1.0f) g = 1.0f;
    gripper_.store(g, std::memory_order_relaxed);
    active_.store(true, std::memory_order_release);  // publish gripper_ before active_
  }

  void connect() override { inner_.connect(); }
  void set_servoing_low_level() override { inner_.set_servoing_low_level(); }
  void set_actuator_modes(const ActuatorModes& m) override {
    inner_.set_actuator_modes(m);
  }
  void exchange(const JointCommand& c, JointFeedback& fb) override {
    inner_.exchange(stamp(c), fb);
  }
  void send(const JointCommand& c) override { inner_.send(stamp(c)); }
  void receive(JointFeedback& fb) override { inner_.receive(fb); }
  void safe_shutdown() override { inner_.safe_shutdown(); }
  void clear_faults() override { inner_.clear_faults(); }

 private:
  JointCommand stamp(const JointCommand& c) {
    JointCommand out = c;
    // acquire on active_ pairs with the release in set_gripper: once active_ reads
    // true, the gripper_ value stored before it is guaranteed visible (no first-cycle
    // stale read). Single rx-thread writer / single RT-thread reader.
    out.gripper_active = active_.load(std::memory_order_acquire);
    out.gripper = gripper_.load(std::memory_order_relaxed);
    return out;
  }
  Transport& inner_;
  std::atomic<float> gripper_{0.0f};
  std::atomic<bool> active_{false};
};

// Parse either a single scalar (applied to every joint) or exactly kNumJoints
// comma-separated values. Returns false on anything malformed — a silently
// half-parsed gain vector is the kind of thing you only discover on the robot.
bool parse_joint_vec(const std::string& s, JointVec& out) {
  double vals[kNumJoints];
  int n = 0;
  size_t pos = 0;
  while (true) {
    if (n == kNumJoints) return false;            // more values than joints
    const size_t comma = s.find(',', pos);
    const std::string tok =
        (comma == std::string::npos) ? s.substr(pos) : s.substr(pos, comma - pos);
    if (tok.empty()) return false;
    try {
      size_t used = 0;
      vals[n++] = std::stod(tok, &used);
      if (used != tok.size()) return false;       // trailing garbage in the token
    } catch (const std::exception&) {
      return false;
    }
    if (comma == std::string::npos) break;
    pos = comma + 1;
  }
  if (n == 1) {                                   // scalar broadcasts to all joints
    out.setConstant(vals[0]);
    return true;
  }
  if (n == kNumJoints) {
    for (int i = 0; i < kNumJoints; ++i) out[i] = vals[i];
    return true;
  }
  return false;
}

Pose pose_from_packet(const tp::PoseTargetPacket& p) {
  Pose x;
  x.p = Eigen::Vector3d(p.pos[0], p.pos[1], p.pos[2]);
  x.R = Eigen::Quaterniond(p.quat_wxyz[0], p.quat_wxyz[1], p.quat_wxyz[2],
                           p.quat_wxyz[3]);
  x.R.normalize();
  return x;
}

}  // namespace

int main(int argc, char** argv) {
  std::string ip;
  std::string urdf = "../models/gen3_7dof_2f85.urdf";
  bool use_sim = false;
  int port = 9095;
  double rate_hz = 1000.0;
  int cpu = -1;
  int rt_priority = 80;
  CartesianImpedanceParams gains;
  JointImpedanceParams jgains;
  bool joint_mode = false;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&](const char* name) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << name << " needs a value\n";
        std::exit(2);
      }
      return argv[++i];
    };
    // Parse a scalar-or-7-value joint vector, exiting on anything malformed.
    auto next_joint_vec = [&](const char* name, JointVec& out) {
      const std::string s = next(name);
      if (!parse_joint_vec(s, out)) {
        std::cerr << name << " needs 1 or " << kNumJoints
                  << " comma-separated numbers, got: " << s << "\n";
        std::exit(2);
      }
    };
    if (a == "--ip") ip = next("--ip");
    else if (a == "--sim") use_sim = true;
    else if (a == "--urdf") urdf = next("--urdf");
    else if (a == "--port") port = std::stoi(next("--port"));
    else if (a == "--rate") rate_hz = std::stod(next("--rate"));
    else if (a == "--cpu") cpu = std::stoi(next("--cpu"));
    else if (a == "--rt-priority") rt_priority = std::stoi(next("--rt-priority"));
    // --- null-space secondary objective tuning (default OFF) ------------------
    else if (a == "--ns-kp") gains.nullspace_kp = std::stod(next("--ns-kp"));
    else if (a == "--ns-kd") gains.nullspace_kd = std::stod(next("--ns-kd"));
    else if (a == "--ns-fixed-rest") gains.nullspace_use_fixed_rest = true;
    // 7 comma-separated joint angles (rad), e.g. --ns-qrest 0,0.26,3.14,-2.27,0,0.96,1.57
    else if (a == "--ns-qrest") next_joint_vec("--ns-qrest", gains.nullspace_q_rest);
    // --manip-gain enables manipulability gradient ascent (0 disables).
    else if (a == "--manip-gain") {
      gains.manip_gain = std::stod(next("--manip-gain"));
      gains.manip_on = (gains.manip_gain != 0.0);
    }
    // --- joint-space impedance (IK in the loop) -------------------------------
    // Constrains ALL 7 joints instead of leaving the redundant DOF free. Use when
    // Cartesian teleop keeps wandering into awkward elbow configurations.
    else if (a == "--joint-impedance") joint_mode = true;
    else if (a == "--jkp") next_joint_vec("--jkp", jgains.Kq);
    else if (a == "--zeta") jgains.zeta = std::stod(next("--zeta"));
    else if (a == "--jtau-limit") next_joint_vec("--jtau-limit", jgains.torque_limit);
    else if (a == "--leash") jgains.max_tracking_error = std::stod(next("--leash"));
    else if (a == "--ref-speed") next_joint_vec("--ref-speed", jgains.max_ref_speed);
    else if (a == "--ik-iters") jgains.ik.max_iters = std::stoi(next("--ik-iters"));
    else if (a == "--ik-posture-gain")
      jgains.ik.posture_gain = std::stod(next("--ik-posture-gain"));
    else if (a == "--ik-qrest") next_joint_vec("--ik-qrest", jgains.ik.q_rest);
    else {
      std::cerr << "unknown arg: " << a << "\n";
      std::exit(2);
    }
  }

  std::cout << "[teleop-srv] urdf=" << urdf << " rate=" << rate_hz << "Hz port="
            << port << " sim=" << (use_sim ? "yes" : "no") << "\n";

  // Three Dynamics instances — Dynamics::fk mutates internal Pinocchio state and
  // is NOT thread-safe. Each thread that calls fk owns its own instance so no
  // Dynamics object is ever shared across threads:
  //   dyn    — CartesianImpedanceMode / RT thread
  //   dyn_fb — feedback thread (periodic ~150 Hz fk)
  //   dyn_rx — rx thread, FREEZE handler only
  Dynamics dyn(urdf);
  Dynamics dyn_fb(urdf);
  Dynamics dyn_rx(urdf);  // rx-thread exclusive: prevents data race with dyn_fb

  std::unique_ptr<Transport> base_transport;
  if (use_sim) {
    JointFeedback init;
    base_transport = std::make_unique<SimTransport>(init);
  } else {
#ifndef KINOVA_NO_KORTEX
    if (ip.empty()) {
      std::cerr << "real-robot mode requires --ip <addr> (or pass --sim)\n";
      return 2;
    }
    base_transport = std::make_unique<KortexTransport>(ip);
#else
    std::cerr << "built without KORTEX; only --sim is available\n";
    return 2;
#endif
  }

  Seqlock<JointFeedback> snapshot;
  GripperInjector injector(*base_transport);
  FeedbackTap transport(injector, snapshot);

  std::signal(SIGINT, on_sigint);

  // --- UDP socket ---------------------------------------------------------
  int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    std::perror("socket");
    return 1;
  }
  sockaddr_in local{};
  local.sin_family = AF_INET;
  local.sin_addr.s_addr = htonl(INADDR_ANY);
  local.sin_port = htons(static_cast<uint16_t>(port));
  if (::bind(sock, reinterpret_cast<sockaddr*>(&local), sizeof(local)) < 0) {
    std::perror("bind");
    return 1;
  }
  timeval rx_timeout{0, 200'000};  // 200 ms so the rx thread can observe g_stop
  ::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &rx_timeout, sizeof(rx_timeout));

  // Shared state between socket threads.
  std::mutex client_mu;
  sockaddr_in client_addr{};
  bool have_client = false;
  std::atomic<uint32_t> last_control_seq{0};

  transport.connect();
  transport.set_servoing_low_level();

  // Seed the home pose (for REHOME) from the current joint state.
  JointFeedback seed_fb;
  transport.receive(seed_fb);
  const Pose home_pose = dyn_fb.fk(seed_fb.q);

  // Both modes implement PoseTargetSink, so the rx thread does not care which is
  // live. Only the selected one is constructed; the other stays null.
  std::unique_ptr<CartesianImpedanceMode> cart_mode;
  std::unique_ptr<JointImpedanceMode> joint_impedance;
  ControlMode* mode = nullptr;
  PoseTargetSink* sink = nullptr;
  if (joint_mode) {
    joint_impedance = std::make_unique<JointImpedanceMode>(dyn, jgains);
    mode = joint_impedance.get();
    sink = joint_impedance.get();
  } else {
    cart_mode = std::make_unique<CartesianImpedanceMode>(dyn, gains);
    mode = cart_mode.get();
    sink = cart_mode.get();
  }
  std::cout << "[teleop-srv] control mode: "
            << (joint_mode ? "joint-space impedance (IK in loop, all 7 joints)"
                           : "cartesian impedance")
            << "\n";

  // --- rx thread ----------------------------------------------------------
  std::thread rx([&] {
    char buf[512];
    while (!g_stop.load(std::memory_order_acquire)) {
      sockaddr_in src{};
      socklen_t srclen = sizeof(src);
      const ssize_t n = ::recvfrom(sock, buf, sizeof(buf), 0,
                                   reinterpret_cast<sockaddr*>(&src), &srclen);
      if (n < static_cast<ssize_t>(sizeof(tp::Header))) continue;
      tp::Header h;
      std::memcpy(&h, buf, sizeof(h));
      if (h.magic != tp::kMagic || h.version != tp::kVersion) continue;

      {  // remember who to stream feedback to
        std::lock_guard<std::mutex> lk(client_mu);
        client_addr = src;
        have_client = true;
      }

      switch (static_cast<tp::MsgType>(h.msg_type)) {
        case tp::MsgType::kPoseTarget: {
          if (n < static_cast<ssize_t>(sizeof(tp::PoseTargetPacket))) break;
          tp::PoseTargetPacket pkt;
          std::memcpy(&pkt, buf, sizeof(pkt));
          sink->set_target(pose_from_packet(pkt));
          injector.set_gripper(pkt.gripper);
          break;
        }
        case tp::MsgType::kSetGains: {
          if (n < static_cast<ssize_t>(sizeof(tp::GainsPacket))) break;
          tp::GainsPacket pkt;
          std::memcpy(&pkt, buf, sizeof(pkt));
          if (joint_mode) {
            // GainsPacket is a Cartesian-impedance payload; Kx/Dx and the
            // null-space fields have no counterpart in joint space (6 task DOF vs
            // 7 joint gains — there is no honest mapping). Apply what does carry
            // over and say so ONCE: silently dropping operator-supplied gains is
            // an hour of confused hardware debugging.
            static std::once_flag warned;
            std::call_once(warned, [] {
              std::cerr << "[teleop-srv] SET_GAINS: joint-space mode ignores Kx, Dx, "
                           "nullspace_kp, nullspace_kd, pinv_damping and nullspace_on. "
                           "Set joint gains with --jkp/--zeta. Applying torque_limit "
                           "and gain_ramp_s.\n";
            });
            JointImpedanceParams jp = jgains;
            jp.torque_limit.setConstant(pkt.torque_limit);
            jp.gain_ramp_s = pkt.gain_ramp_s;
            joint_impedance->set_gains(jp);
            break;
          }
          CartesianImpedanceParams p;
          for (int k = 0; k < 6; ++k) {
            p.Kx[k] = pkt.Kx[k];
            p.Dx[k] = pkt.Dx[k];
          }
          p.nullspace_kp = pkt.nullspace_kp;
          p.nullspace_kd = pkt.nullspace_kd;
          p.pinv_damping = pkt.pinv_damping;
          p.torque_limit = pkt.torque_limit;
          p.gain_ramp_s = pkt.gain_ramp_s;
          p.nullspace_on = pkt.nullspace_on != 0;
          cart_mode->set_gains(p);
          break;
        }
        case tp::MsgType::kControl: {
          if (n < static_cast<ssize_t>(sizeof(tp::ControlPacket))) break;
          tp::ControlPacket pkt;
          std::memcpy(&pkt, buf, sizeof(pkt));
          switch (static_cast<tp::ControlCmd>(pkt.command)) {
            case tp::ControlCmd::kClearFaults:
              transport.clear_faults();
              break;
            case tp::ControlCmd::kRehome:
              sink->set_target(home_pose);
              break;
            case tp::ControlCmd::kFreeze: {
              JointFeedback fb;
              // Use dyn_rx (rx-thread-exclusive) — dyn_fb belongs to the
              // feedback thread and must not be touched here.
              if (snapshot.load(fb)) sink->set_target(dyn_rx.fk(fb.q));
              break;
            }
            case tp::ControlCmd::kShutdown:
              g_stop.store(true, std::memory_order_release);
              break;
          }
          // Acknowledge regardless so the supervisor's resend-until-ack stops.
          last_control_seq.store(pkt.control_seq, std::memory_order_release);
          break;
        }
        default:
          break;
      }
    }
  });

  // --- feedback thread ----------------------------------------------------
  std::thread feedback([&] {
    // Its own Dynamics for thread-safe fk. ~150 Hz publish.
    while (!g_stop.load(std::memory_order_acquire)) {
      sockaddr_in dest{};
      bool send = false;
      {
        std::lock_guard<std::mutex> lk(client_mu);
        if (have_client) {
          dest = client_addr;
          send = true;
        }
      }
      JointFeedback fb;
      if (send && snapshot.load(fb)) {
        const Pose ee = dyn_fb.fk(fb.q);
        tp::FeedbackPacket pkt{};
        pkt.h.magic = tp::kMagic;
        pkt.h.version = tp::kVersion;
        pkt.h.msg_type = static_cast<uint16_t>(tp::MsgType::kFeedback);
        pkt.h.seq = 0;
        pkt.h.timestamp_ns = static_cast<uint64_t>(ns_now());
        for (int k = 0; k < kNumJoints; ++k) {
          pkt.q[k] = fb.q[k];
          pkt.qd[k] = fb.qd[k];
          pkt.tau[k] = fb.tau[k];
        }
        pkt.ee_pos[0] = ee.p.x();
        pkt.ee_pos[1] = ee.p.y();
        pkt.ee_pos[2] = ee.p.z();
        pkt.ee_quat_wxyz[0] = ee.R.w();
        pkt.ee_quat_wxyz[1] = ee.R.x();
        pkt.ee_quat_wxyz[2] = ee.R.y();
        pkt.ee_quat_wxyz[3] = ee.R.z();
        pkt.gripper_state = fb.gripper;  // actual measured position from snapshot
        pkt.fault = fb.fault ? 1 : 0;
        pkt.frame_id = fb.frame_id;
        pkt.last_control_seq = last_control_seq.load(std::memory_order_acquire);
        ::sendto(sock, &pkt, sizeof(pkt), 0,
                 reinterpret_cast<sockaddr*>(&dest), sizeof(dest));
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(6));  // ~150 Hz
    }
  });

  // Telemetry ring is required by RtExecutor but not consumed here (timing
  // telemetry is not part of teleop); it drops-don't-block when full.
  SampleRing ring(1 << 16);
  RtExecutor ex(transport, ring,
                {rate_hz, Pacing::kSleepSpin, {rt_priority, cpu, true}});
  ex.request_mode(mode);

  std::cout << "[teleop-srv] listening; RT loop running. Ctrl-C to stop.\n";
  ex.run(g_stop);  // blocks on the main (RT) thread until stop

  // Stop threads before tearing down the transport: the rx thread can call
  // transport.clear_faults() and the feedback thread uses the socket, so both
  // must be fully stopped before safe_shutdown and close(sock).
  g_stop.store(true, std::memory_order_release);
  rx.join();
  feedback.join();
  transport.safe_shutdown();
  ::close(sock);
  std::cout << "[teleop-srv] stopped.\n";
  return 0;
}
