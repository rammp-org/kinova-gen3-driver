// teleop_socket_server — bridges the Python VR-teleop supervisor to the
// CartesianImpedanceMode seam over UDP. The RT loop (RtExecutor) owns the main
// thread; two helper threads handle the socket:
//
//   * rx thread:       receive POSE_TARGET / SET_GAINS / CONTROL, apply via the
//                      mode's non-RT setters (single writer, as required).
//   * feedback thread: read the latest JointFeedback snapshot, compute EE pose
//                      via its own Dynamics, and stream FEEDBACK back to the
//                      last client address.
//
// Robot feedback is captured without touching the driver core: FeedbackTap wraps
// the Transport and snapshots `fb` at the exact point the RT loop reads it.
//
//   ./teleop_socket_server --sim  --urdf ../models/gen3_7dof_2f85.urdf --port 9095
//   ./teleop_socket_server --ip 192.168.1.10 --urdf ../models/gen3_7dof_2f85.urdf
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

// Single-writer / single-reader lock-free latest-value snapshot (seqlock). The
// RT thread is the only writer; the feedback thread is the only reader.
// T need not be trivially copyable — fixed-size Eigen members store data inline,
// so a torn read yields transient garbage numbers but never a bad pointer.
template <class T>
class Seqlock {
 public:
  void store(const T& v) {  // writer (RT thread): bounded, no alloc/lock
    const uint32_t s = seq_.load(std::memory_order_relaxed);
    seq_.store(s + 1, std::memory_order_release);  // mark write in progress (odd)
    std::atomic_thread_fence(std::memory_order_release);
    data_ = v;
    std::atomic_thread_fence(std::memory_order_release);
    seq_.store(s + 2, std::memory_order_release);  // publish (even)
  }
  bool load(T& out) const {  // reader: retry while a write is in flight
    for (int i = 0; i < 16; ++i) {
      const uint32_t s1 = seq_.load(std::memory_order_acquire);
      if (s1 & 1u) continue;
      std::atomic_thread_fence(std::memory_order_acquire);
      out = data_;
      std::atomic_thread_fence(std::memory_order_acquire);
      if (seq_.load(std::memory_order_acquire) == s1) return true;
    }
    return false;
  }

 private:
  std::atomic<uint32_t> seq_{0};
  T data_{};
};

// Transport decorator: forwards everything to the wrapped transport and, on each
// feedback-producing call, publishes the latest JointFeedback into a snapshot.
// This is what lets the feedback thread read robot state without any change to
// RtExecutor or the driver library.
class FeedbackTap : public Transport {
 public:
  FeedbackTap(Transport& inner, Seqlock<JointFeedback>& snap)
      : inner_(inner), snap_(snap) {}
  void connect() override { inner_.connect(); }
  void set_servoing_low_level() override { inner_.set_servoing_low_level(); }
  void set_actuator_modes(const ActuatorModes& m) override {
    inner_.set_actuator_modes(m);
  }
  void exchange(const JointCommand& c, JointFeedback& fb) override {
    inner_.exchange(c, fb);
    snap_.store(fb);
  }
  void send(const JointCommand& c) override { inner_.send(c); }
  void receive(JointFeedback& fb) override {
    inner_.receive(fb);
    snap_.store(fb);
  }
  void safe_shutdown() override { inner_.safe_shutdown(); }
  void clear_faults() override { inner_.clear_faults(); }

 private:
  Transport& inner_;
  Seqlock<JointFeedback>& snap_;
};

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
    active_.store(true, std::memory_order_relaxed);
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
    out.gripper = gripper_.load(std::memory_order_relaxed);
    out.gripper_active = active_.load(std::memory_order_relaxed);
    return out;
  }
  Transport& inner_;
  std::atomic<float> gripper_{0.0f};
  std::atomic<bool> active_{false};
};

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

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&](const char* name) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << name << " needs a value\n";
        std::exit(2);
      }
      return argv[++i];
    };
    if (a == "--ip") ip = next("--ip");
    else if (a == "--sim") use_sim = true;
    else if (a == "--urdf") urdf = next("--urdf");
    else if (a == "--port") port = std::stoi(next("--port"));
    else if (a == "--rate") rate_hz = std::stod(next("--rate"));
    else if (a == "--cpu") cpu = std::stoi(next("--cpu"));
    else if (a == "--rt-priority") rt_priority = std::stoi(next("--rt-priority"));
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

  CartesianImpedanceMode mode(dyn, gains);

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
          mode.set_target(pose_from_packet(pkt));
          injector.set_gripper(pkt.gripper);
          break;
        }
        case tp::MsgType::kSetGains: {
          if (n < static_cast<ssize_t>(sizeof(tp::GainsPacket))) break;
          tp::GainsPacket pkt;
          std::memcpy(&pkt, buf, sizeof(pkt));
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
          mode.set_gains(p);
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
              mode.set_target(home_pose);
              break;
            case tp::ControlCmd::kFreeze: {
              JointFeedback fb;
              // Use dyn_rx (rx-thread-exclusive) — dyn_fb belongs to the
              // feedback thread and must not be touched here.
              if (snapshot.load(fb)) mode.set_target(dyn_rx.fk(fb.q));
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
  ex.request_mode(&mode);

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
