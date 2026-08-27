// TEMPORARY — bring-up harness, not a maintained tool. Delete this app, its
// CMake block and docs/integration/velocity_mode_probe.md once the kVelocity
// question below has an answer on the real arm. Nothing in the library depends
// on it; removal is `git rm` plus one CMake block. Do not build on it.
//
// velocity_probe — DIAGNOSTIC. Answers one question: does this arm honour
// ActuatorMode::kVelocity at all?
//
// `KortexTransport` maps kVelocity to ActuatorConfig::ControlMode::VELOCITY and
// writes JointCommand::velocity, but NO ControlMode in this driver has ever used
// it, and it cannot be settled in sim — SimTransport is a static echo with no
// plant, so a velocity command produces no motion there by construction.
//
// The reason to care: joint_position_mode.h records that Kinova's own
// ros2_kortex driver computes a velocity command and then declines to send it —
//   // Velocity command interface not implemented properly in the kortex api
// That comment is about the velocity field in POSITION servoing, which is a
// different path, but it means velocity commands in the KORTEX API are a known
// trouble spot. The streaming-setpoint design
// (docs/superpowers/specs/2026-08-26-streaming-setpoints-design.md) puts a whole
// control mode on top of kVelocity, so this probe runs BEFORE that is written.
//
// The outcome is one of exactly three, and the report names which:
//   TRACKS  - measured qd follows the command and q moves
//   IGNORES - measured qd stays ~0 and q does not move (command silently dropped)
//   FAULTS  - the transport reports a fault
//
// Velocity mode has NO compliance: the actuator servo chases the commanded speed
// at full authority and will keep chasing it into an obstruction. Defaults are
// therefore deliberately tiny — ONE joint, 0.05 rad/s, 2 s, which is 0.1 rad of
// travel — and the command is RAMPED in and out rather than stepped, so nothing
// here is a jolt.
//
// This is a probe, not a driver capability: the constant-velocity mode below
// lives in this file on purpose and is not part of the library.
//
//   # 1. READ-ONLY: connects, reads, commands nothing, never enters servoing.
//   ./velocity_probe --ip 192.168.1.10 --dry-run
//
//   # 2. the probe itself: wrist joint, 0.05 rad/s, 2 s
//   ./velocity_probe --ip 192.168.1.10
//
//   # 3. a different joint / speed, still small
//   ./velocity_probe --ip 192.168.1.10 --joint 5 --qd 0.08 --duration 3
//
// Under --sim this exercises the plumbing only; it will always report IGNORES,
// because the sim transport has no plant. That is expected and is not a result.

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "kinova_lowlevel/control_mode.h"
#include "kinova_lowlevel/rt_executor.h"
#include "kinova_lowlevel/rt_system.h"
#include "kinova_lowlevel/sim_transport.h"
#include "kinova_lowlevel/telemetry.h"
#include "kinova_lowlevel/transport.h"
#include "kinova_lowlevel/units.h"
#ifndef KINOVA_NO_KORTEX
#include "kinova_lowlevel/kortex_transport.h"
#endif

using namespace kinova;

namespace {
std::atomic<bool> g_stop{false};
void on_sigint(int) { g_stop.store(true); }

// Commands a constant velocity on ONE joint, ramped in and out. Deliberately
// local to this app: it is a diagnostic, not a control law the driver offers.
//
// RT-safe: no allocation, no locks, no clock calls — elapsed time is accumulated
// from the dt_s the executor passes in, the same way JointTorqueMode tracks
// staleness.
class ConstVelocityMode : public ControlMode {
 public:
  ConstVelocityMode(int joint, double qd, double duration_s, double ramp_s)
      : joint_(joint), qd_(qd), duration_s_(duration_s), ramp_s_(ramp_s) {}

  ActuatorModes required_modes() const override {
    ActuatorModes m;
    m.fill(ActuatorMode::kVelocity);
    return m;
  }

  void on_enter(const JointFeedback& fb) override {
    t_ = 0.0;
    q_start_ = fb.q;
    qd_peak_ = 0.0;
    qd_sum_ = 0.0;
    hold_cycles_ = 0;
    faulted_ = false;
  }

  void compute(const JointFeedback& fb, double dt_s, JointCommand& out) override {
    t_ += dt_s;
    if (fb.fault) faulted_ = true;

    // Trapezoid: ramp up, hold, ramp down. Never a step.
    double scale = 0.0;
    const double tail = duration_s_ - ramp_s_;
    if (t_ < ramp_s_)        scale = (ramp_s_ > 0.0) ? t_ / ramp_s_ : 1.0;
    else if (t_ < tail)      scale = 1.0;
    else if (t_ < duration_s_) scale = (ramp_s_ > 0.0) ? (duration_s_ - t_) / ramp_s_ : 1.0;
    else                     scale = 0.0;
    if (scale < 0.0) scale = 0.0;

    // Sample tracking only during the flat hold, where the command is steady and
    // a comparison against measured qd actually means something.
    if (t_ >= ramp_s_ && t_ < tail) {
      const double m = fb.qd[joint_];
      qd_sum_ += m;
      ++hold_cycles_;
      if (std::abs(m) > std::abs(qd_peak_)) qd_peak_ = m;
    }

    out.mode = ActuatorMode::kVelocity;
    out.velocity.setZero();
    out.velocity[joint_] = qd_ * scale;
    out.position = fb.q;   // passthrough, as the other modes do
    q_last_ = fb.q;
  }

  void on_exit() override {}

  // RT-thread-owned. Read only after the executor has stopped.
  double   mean_hold_qd() const { return hold_cycles_ ? qd_sum_ / hold_cycles_ : 0.0; }
  double   peak_hold_qd() const { return qd_peak_; }
  bool     faulted()      const { return faulted_; }
  JointVec q_start()      const { return q_start_; }
  JointVec q_last()       const { return q_last_; }

 private:
  const int joint_;
  const double qd_, duration_s_, ramp_s_;
  double t_ = 0.0, qd_peak_ = 0.0, qd_sum_ = 0.0;
  uint64_t hold_cycles_ = 0;
  bool faulted_ = false;
  JointVec q_start_ = JointVec::Zero(), q_last_ = JointVec::Zero();
};
}  // namespace

int main(int argc, char** argv) {
  std::string ip;
  std::string pacing_str = "sleepspin";
  bool use_sim = false, dry_run = false;
  int joint = kNumJoints - 1;         // wrist: lightest, least able to hurt anything
  double qd = 0.05;                   // rad/s
  double duration_s = 2.0;
  double ramp_s = 0.5;
  double rate_hz = 1000.0;
  int cpu = -1, rt_priority = 80;

  auto next = [&](const char* what) -> std::string {
    static int i = 1;
    (void)what;
    return (++i < argc) ? argv[i] : std::string();
  };
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto val = [&]() -> std::string {
      if (i + 1 >= argc) { std::cerr << "missing value for " << a << "\n"; std::exit(2); }
      return argv[++i];
    };
    if (a == "--ip") ip = val();
    else if (a == "--sim") use_sim = true;
    else if (a == "--dry-run") dry_run = true;
    else if (a == "--joint") joint = std::stoi(val());
    else if (a == "--qd") qd = std::stod(val());
    else if (a == "--duration") duration_s = std::stod(val());
    else if (a == "--ramp") ramp_s = std::stod(val());
    else if (a == "--rate") rate_hz = std::stod(val());
    else if (a == "--cpu") cpu = std::stoi(val());
    else if (a == "--rt-priority") rt_priority = std::stoi(val());
    else if (a == "--pacing") pacing_str = val();
    else { std::cerr << "unknown arg: " << a << "\n"; std::exit(2); }
  }
  (void)next;

  Pacing pacing = Pacing::kSleepSpin;
  if (pacing_str == "nanosleep") pacing = Pacing::kClockNanosleep;
  else if (pacing_str != "sleepspin") {
    std::cerr << "--pacing must be sleepspin|nanosleep\n"; return 2;
  }
  if (joint < 0 || joint >= kNumJoints) {
    std::cerr << "--joint must be 0.." << (kNumJoints - 1) << "\n"; return 2;
  }
  // Guard rails: this is a probe, not a motion tool. Refuse to be turned into one.
  if (std::abs(qd) > 0.3) {
    std::cerr << "--qd " << qd << " rad/s is too fast for a probe (cap 0.3). "
                 "This app exists to answer yes/no, not to move the arm.\n";
    return 2;
  }
  if (duration_s > 10.0) {
    std::cerr << "--duration " << duration_s << "s is too long for a probe (cap 10).\n";
    return 2;
  }
  if (ramp_s * 2.0 > duration_s) {
    std::cerr << "--ramp " << ramp_s << "s twice over exceeds --duration "
              << duration_s << "s; there would be no steady phase to measure.\n";
    return 2;
  }

  const double travel = std::abs(qd) * (duration_s - ramp_s);   // trapezoid area
  std::printf("[vprobe] joint=j%d qd=%+.4f rad/s duration=%.2fs ramp=%.2fs "
              "rate=%.0fHz sim=%s dry_run=%s\n",
              joint, qd, duration_s, ramp_s, rate_hz,
              use_sim ? "yes" : "no", dry_run ? "yes" : "no");
  std::printf("[vprobe] expected travel if it TRACKS: about %+.4f rad (%+.2f deg)\n",
              travel * (qd < 0 ? -1 : 1), travel * kRad2Deg * (qd < 0 ? -1 : 1));

  std::unique_ptr<Transport> transport;
  if (use_sim) {
    JointFeedback init;
    transport = std::make_unique<SimTransport>(init);
  } else {
#ifndef KINOVA_NO_KORTEX
    if (ip.empty()) { std::cerr << "real-robot mode requires --ip <addr> (or --sim)\n"; return 2; }
    transport = std::make_unique<KortexTransport>(ip);
#else
    std::cerr << "built without KORTEX; only --sim is available\n"; return 2;
#endif
  }
  Transport& t = *transport;
  std::signal(SIGINT, on_sigint);

  // --- dry-run: READ-ONLY. Never enters low-level servoing. --------------------
  if (dry_run) {
    t.connect();
    JointFeedback fb;
    t.receive(fb);
    std::printf("\n[dry-run] READ-ONLY — nothing is commanded.\n");
    std::printf("[dry-run] current q:  ");
    for (int i = 0; i < kNumJoints; ++i) std::printf("%+7.4f ", fb.q[i]);
    std::printf("\n[dry-run] current qd: ");
    for (int i = 0; i < kNumJoints; ++i) std::printf("%+7.4f ", fb.qd[i]);
    std::printf("\n[dry-run] would put ALL actuators in kVelocity and command "
                "j%d at %+.4f rad/s (ramped), others at 0.\n", joint, qd);
    std::printf("[dry-run] j%d would end up near %+.4f rad if the arm tracks it.\n",
                joint, fb.q[joint] + travel * (qd < 0 ? -1 : 1));
    t.safe_shutdown();
    return 0;
  }

  t.connect();
  JointFeedback entry;
  t.receive(entry);
  std::printf("\n[vprobe] entry q[j%d] = %+.4f rad\n", joint, entry.q[joint]);
  std::printf("[vprobe] ALL actuators go to VELOCITY mode. This path has never "
              "run on this arm.\n"
              "[vprobe] starting in 3s — e-stop in reach. Ctrl-C aborts.\n");
  std::this_thread::sleep_for(std::chrono::seconds(3));

  t.set_servoing_low_level();

  ConstVelocityMode mode(joint, qd, duration_s, ramp_s);
  SampleRing ring(8192);
  RtExecutor ex(t, ring, {rate_hz, pacing, {rt_priority, cpu, true, true}});
  ex.request_mode(&mode);

  std::atomic<bool> stop{false};
  std::thread rt([&] { ex.run(stop); });
  const auto t0 = std::chrono::steady_clock::now();
  while (!g_stop.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    const double el = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    if (el > duration_s + 0.5) break;      // +0.5s so the ramp-down completes
  }
  stop = true;
  rt.join();
  t.safe_shutdown();

  // --- verdict ----------------------------------------------------------------
  JointFeedback after;
  const JointVec q0 = mode.q_start(), q1 = mode.q_last();
  const double moved = q1[joint] - q0[joint];
  const double mean_qd = mode.mean_hold_qd();
  const double peak_qd = mode.peak_hold_qd();

  std::printf("\n[vprobe] commanded qd (hold phase): %+.4f rad/s\n", qd);
  std::printf("[vprobe] measured  qd (hold mean):   %+.4f rad/s   (peak %+.4f)\n",
              mean_qd, peak_qd);
  std::printf("[vprobe] q[j%d]: %+.4f -> %+.4f  (moved %+.4f rad, %+.2f deg)\n",
              joint, q0[joint], q1[joint], moved, moved * kRad2Deg);

  const char* verdict;
  if (mode.faulted())                          verdict = "FAULTS  — the transport reported a fault";
  else if (std::abs(mean_qd) > 0.25 * std::abs(qd) && std::abs(moved) > 0.2 * travel)
                                               verdict = "TRACKS  — velocity mode is honoured";
  else if (std::abs(moved) < 0.05 * travel)    verdict = "IGNORES — command accepted and silently dropped";
  else                                         verdict = "PARTIAL — it moved, but not as commanded; read the numbers above";
  std::printf("\n[vprobe] VERDICT: %s\n", verdict);
  if (use_sim) {
    std::printf("[vprobe] (--sim has no plant, so IGNORES here is expected and "
                "is not a result about the hardware.)\n");
  }
  return 0;
}
