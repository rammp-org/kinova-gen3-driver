// joint_torque_check — hardware validation harness for JointTorqueMode. The
// procedure that uses it is docs/integration/joint_torque_check.md.
//
// benchmark_grav_comp already covers the zero-feedforward case (gravity-comp
// hold). What has NEVER run on the arm is the thing the mode exists for:
// set_torque() publishing a real feedforward, and the staleness watchdog
// decaying it back to zero when the commands stop. The streaming-setpoint tier
// will lean on exactly that watchdog, so it gets exercised here first.
//
// Torque mode's safety posture sits between the other two. Unlike position mode
// it IS compliant — the arm yields, because you are only ever adding torque on
// top of gravity compensation. But unlike impedance mode there is no spring
// pulling it back to a reference: a sustained feedforward accelerates the joint
// for as long as you command it. The per-joint clamp (39/39/39/39/9/9/9 N*m by
// default) is what bounds the outcome, and the wrist's 9 N*m is the number that
// matters.
//
// Defaults are deliberately far below that: ONE joint, 1.0 N*m, 2 s of command
// followed by a watchdog observation window. 1 N*m on the wrist is roughly a
// tenth of its rating.
//
// --dry-run is READ-ONLY: connects, reads feedback, prints what WOULD be
// commanded, and never enters low-level servoing. Run it first.
//
//   # 1. READ-ONLY, nothing moves
//   ./joint_torque_check --ip 192.168.1.10 --dry-run
//
//   # 2. gravity-comp hold — no feedforward at all. The arm should just hold.
//   ./joint_torque_check --ip 192.168.1.10 --tau 0
//
//   # 3. the real check: 1 N*m on the wrist, then STOP commanding and watch the
//   #    watchdog decay it to zero (the arm settles back to gravity-comp hold)
//   ./joint_torque_check --ip 192.168.1.10 --joint 6 --tau 1.0
//
// Under --sim this exercises plumbing only: SimTransport is a static echo, so
// measured torque never responds and the arm never moves.

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
#include <vector>

#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/feedback_tap.h"
#include "kinova_lowlevel/joint_torque_mode.h"
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

struct Sample { double t; double tau_meas; double q; };
}  // namespace

int main(int argc, char** argv) {
  std::string ip;
  std::string urdf = "../models/gen3_7dof_2f85.urdf";
  std::string pacing_str = "sleepspin";
  bool use_sim = false, dry_run = false;
  int joint = kNumJoints - 1;      // wrist by default
  double tau_ff = 1.0;             // N*m
  double command_s = 2.0;          // how long we keep publishing
  double observe_s = 1.0;          // watchdog observation window after we stop
  double cmd_rate_hz = 100.0;      // publisher rate — a plausible stream rate
  double rate_hz = 1000.0;
  int cpu = -1, rt_priority = 80;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto val = [&]() -> std::string {
      if (i + 1 >= argc) { std::cerr << "missing value for " << a << "\n"; std::exit(2); }
      return argv[++i];
    };
    if (a == "--ip") ip = val();
    else if (a == "--urdf") urdf = val();
    else if (a == "--sim") use_sim = true;
    else if (a == "--dry-run") dry_run = true;
    else if (a == "--joint") joint = std::stoi(val());
    else if (a == "--tau") tau_ff = std::stod(val());
    else if (a == "--command-s") command_s = std::stod(val());
    else if (a == "--observe-s") observe_s = std::stod(val());
    else if (a == "--cmd-rate") cmd_rate_hz = std::stod(val());
    else if (a == "--rate") rate_hz = std::stod(val());
    else if (a == "--cpu") cpu = std::stoi(val());
    else if (a == "--rt-priority") rt_priority = std::stoi(val());
    else if (a == "--pacing") pacing_str = val();
    else { std::cerr << "unknown arg: " << a << "\n"; std::exit(2); }
  }

  Pacing pacing = Pacing::kSleepSpin;
  if (pacing_str == "nanosleep") pacing = Pacing::kClockNanosleep;
  else if (pacing_str != "sleepspin") { std::cerr << "--pacing must be sleepspin|nanosleep\n"; return 2; }
  if (joint < 0 || joint >= kNumJoints) {
    std::cerr << "--joint must be 0.." << (kNumJoints - 1) << "\n"; return 2;
  }

  JointTorqueParams p;                       // per-joint defaults (39x4, 9x3)
  const double limit = p.torque_limit[joint];
  if (std::abs(tau_ff) > 0.5 * limit) {
    std::cerr << "--tau " << tau_ff << " N*m is more than half j" << joint
              << "'s " << limit << " N*m clamp. This is a validation harness, "
                 "not a motion tool — keep the feedforward small.\n";
    return 2;
  }

  std::printf("[jtau] joint=j%d tau_ff=%+.3f N*m (clamp %.1f) command=%.1fs "
              "observe=%.1fs cmd_rate=%.0fHz rate=%.0fHz sim=%s dry_run=%s\n",
              joint, tau_ff, limit, command_s, observe_s, cmd_rate_hz, rate_hz,
              use_sim ? "yes" : "no", dry_run ? "yes" : "no");
  std::printf("[jtau] watchdog: cmd_timeout=%.3fs decay=%.3fs — after the "
              "publisher stops, tau_ff should reach zero about %.2fs later.\n",
              p.cmd_timeout_s, p.cmd_decay_s, p.cmd_timeout_s + p.cmd_decay_s);

  Dynamics dyn(urdf);

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
  Transport& raw = *transport;
  std::signal(SIGINT, on_sigint);

  // --- dry-run: READ-ONLY, never enters low-level servoing --------------------
  if (dry_run) {
    raw.connect();
    JointFeedback fb;
    raw.receive(fb);
    JointVec g;
    dyn.gravity(fb.q, g);
    std::printf("\n[dry-run] READ-ONLY — nothing is commanded.\n");
    for (int i = 0; i < kNumJoints; ++i)
      std::printf("  j%d  q=%+8.4f rad  tau_meas=%+8.3f  gravity=%+8.3f  clamp=%.1f N*m\n",
                  i, fb.q[i], fb.tau[i], g[i], p.torque_limit[i]);
    std::printf("\n[dry-run] would command gravity comp on every joint, plus "
                "%+.3f N*m of feedforward on j%d.\n", tau_ff, joint);
    std::printf("[dry-run] j%d total would be about %+.3f N*m, clamped to +/-%.1f.\n",
                joint, g[joint] + tau_ff, limit);
    raw.safe_shutdown();
    return 0;
  }

  raw.connect();
  JointFeedback entry;
  raw.receive(entry);
  std::printf("\n[jtau] entry: q[j%d]=%+.4f rad  tau_meas[j%d]=%+.3f N*m\n",
              joint, entry.q[joint], joint, entry.tau[joint]);
  std::printf("[jtau] the arm is COMPLIANT here — it will yield if you push it, "
              "but a sustained feedforward keeps accelerating the joint.\n"
              "[jtau] starting in 3s — e-stop in reach. Ctrl-C aborts.\n");
  std::this_thread::sleep_for(std::chrono::seconds(3));

  raw.set_servoing_low_level();

  // FeedbackTap snapshots the arm's state at the RT read point, so this thread
  // can watch the torque respond without racing the RT loop. Same pattern
  // trajectory_run uses for q.
  Seqlock<JointFeedback> snap;
  FeedbackTap tap(raw, snap);

  JointTorqueMode mode(dyn, p);
  SampleRing ring(8192);
  RtExecutor ex(tap, ring, {rate_hz, pacing, {rt_priority, cpu, true, true}});
  ex.request_mode(&mode);

  std::atomic<bool> stop{false};
  std::thread rt([&] { ex.run(stop); });

  // Publisher: stream the feedforward for command_s, then STOP — the stopping is
  // the watchdog test, not an afterthought.
  std::vector<Sample> trace;
  trace.reserve(size_t((command_s + observe_s) * 50) + 16);
  const auto t0 = std::chrono::steady_clock::now();
  auto elapsed = [&] {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  };
  JointVec ff = JointVec::Zero();
  ff[joint] = tau_ff;

  bool announced_stop = false;
  const auto cmd_period = std::chrono::duration<double>(1.0 / cmd_rate_hz);
  auto next_sample = 0.0;
  while (!g_stop.load()) {
    const double el = elapsed();
    if (el >= command_s + observe_s) break;
    if (el < command_s) {
      mode.set_torque(ff);
    } else if (!announced_stop) {
      announced_stop = true;
      std::printf("\n[jtau] PUBLISHER STOPPED at %.2fs — watchdog should now "
                  "decay the feedforward to zero.\n", el);
    }
    if (el >= next_sample) {
      JointFeedback fb;
      if (snap.load(fb)) trace.push_back({el, fb.tau[joint], fb.q[joint]});
      next_sample = el + 0.05;   // 20 Hz trace
    }
    std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::steady_clock::duration>(cmd_period));
  }
  stop = true;
  rt.join();
  raw.safe_shutdown();

  // --- report ------------------------------------------------------------------
  std::printf("\n[jtau] trace (t, measured tau[j%d], q[j%d]):\n", joint, joint);
  for (const auto& s : trace)
    std::printf("  %5.2fs  tau=%+8.3f N*m  q=%+8.4f rad %s\n", s.t, s.tau_meas, s.q,
                s.t < command_s ? "[commanding]" : "[watchdog]");

  double q_drift = trace.empty() ? 0.0 : trace.back().q - entry.q[joint];
  std::printf("\n[jtau] q[j%d] drift over the run: %+.4f rad (%+.2f deg)\n",
              joint, q_drift, q_drift * kRad2Deg);
  std::printf("[jtau] WHAT TO CHECK:\n"
              "   * during [commanding]: measured tau on j%d sits above its "
              "gravity-only value by roughly %+.3f N*m.\n"
              "   * at the PUBLISHER STOPPED line: that excess ramps away within "
              "about %.2fs and does not snap.\n"
              "   * after the decay: the arm holds under gravity comp alone, and "
              "q stops drifting.\n"
              "   * throughout: the arm still yields when you push it by hand.\n",
              joint, tau_ff, p.cmd_timeout_s + p.cmd_decay_s);
  if (use_sim)
    std::printf("[jtau] (--sim is a static echo: measured tau never responds and "
                "q never moves. Plumbing only.)\n");
  return 0;
}
