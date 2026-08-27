// TEMPORARY — bring-up harness, not a maintained tool. Delete this app, its CMake
// block and docs/integration/stream_check.md once the streaming tier has been
// validated on the arm. Nothing in the library depends on it; removal is
// `git rm` plus one CMake block. Do not build on it.
//
// stream_check — hardware validation harness for the streaming setpoint tier. The
// procedure that uses it is docs/integration/stream_check.md.
//
// The unit tests prove the session state machine, the write handoff and the
// watchdog in simulation. What no test can prove is what the ARM does when a
// stream goes stale or a session closes, because SimTransport is a static echo:
// it never moves, so "the reference froze at measured q" and "the reference
// froze at zero" are indistinguishable there. That is what this exists for.
//
// The run is four phases, and the interesting one is the second:
//
//   1. STREAM   — push setpoints at --cmd-rate, walking one joint by --delta
//   2. STALE    — STOP pushing for --stale-s. This is the test, not a pause.
//                 The mode's own watchdog should make the output safe at 1 kHz:
//                 position and impedance freeze the reference at measured q,
//                 torque ramps its feedforward to zero (gravity-comp hold).
//   3. RESUME   — push again, confirming the freeze releases on a fresh command
//   4. CLOSE    — close the session; the teardown latches hold-at-measured-q
//
// Safety posture depends on which mode you select, and they are not alike:
//   --mode position    NO compliance. The actuator servo chases the command at
//                      full authority. Nothing absorbs a mistake.
//   --mode impedance   Compliant. The arm yields to contact.
//   --mode torque      Compliant, but no spring pulls it back: a sustained
//                      feedforward keeps accelerating the joint.
//
// Defaults are deliberately small: one wrist joint, 0.05 rad of travel, and the
// app refuses a --delta above 0.2 rad or a --tau above half the joint's clamp.
//
// --dry-run is READ-ONLY: connects, reads feedback, prints the plan, and never
// enters low-level servoing. Run it first.
//
//   # 1. READ-ONLY, nothing moves
//   ./stream_check --ip 192.168.1.10 --dry-run
//
//   # 2. stream joint positions into impedance mode (compliant — start here)
//   ./stream_check --ip 192.168.1.10 --kind joint-position --mode impedance
//
//   # 3. stream joint torque (feedforward on top of gravity comp)
//   ./stream_check --ip 192.168.1.10 --kind joint-torque --mode torque --tau 1.0
//
// Under --sim this exercises plumbing only: the arm never moves, so the trace is
// flat and the STALE phase proves nothing about the hardware.

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
#include "kinova_lowlevel/interface/ports.h"
#include "kinova_lowlevel/interface/supervisor.h"
#include "kinova_lowlevel/joint_impedance_mode.h"
#include "kinova_lowlevel/joint_position_mode.h"
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
using namespace kinova::interface;

namespace {
std::atomic<bool> g_stop{false};
void on_sigint(int) { g_stop.store(true); }

// The Supervisor needs both driven ports. Streaming uses neither (there is no
// goal to settle), so this only keeps the latest state for the trace and reports
// anything unexpected arriving on the action port.
class ConsoleBackend : public StreamPort, public ActionServerPort {
 public:
  void publish_state(const ArmState& s) override { last_ = s; ++states_; }
  void publish_feedback(const GoalId&, const TrajectoryFeedback&) override {}
  void settle(const GoalId&, const TrajectoryResult& r) override {
    std::printf("[stream] UNEXPECTED goal settle during a stream: code=%d %s\n",
                r.error_code, r.error_string.c_str());
  }
  ArmState last() const { return last_; }
  uint64_t states() const { return states_; }
 private:
  ArmState last_{};
  uint64_t states_ = 0;
};

struct Sample { double t; const char* phase; JointVec q; JointVec tau; };

const char* phase_of(double t, double stream_s, double stale_s, double resume_s) {
  if (t < stream_s)                              return "STREAM";
  if (t < stream_s + stale_s)                    return "STALE ";
  if (t < stream_s + stale_s + resume_s)         return "RESUME";
  return "CLOSE ";
}
}  // namespace

int main(int argc, char** argv) {
  std::string ip, urdf = "../models/gen3_7dof_2f85.urdf", pacing_str = "sleepspin";
  std::string kind_s = "joint-position", mode_s = "impedance";
  bool use_sim = false, dry_run = false;
  int joint = kNumJoints - 1;
  double delta = 0.05;          // rad, for position/pose kinds
  double tau_ff = 1.0;          // N*m, for the torque kind
  double stream_s = 2.0, stale_s = 1.0, resume_s = 1.0;
  double cmd_rate_hz = 100.0, timeout_s = 0.1;
  double rate_hz = 1000.0;
  int cpu = -1, rt_priority = 80;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto val = [&]() -> std::string {
      if (i + 1 >= argc) { std::cerr << "missing value for " << a << "\n"; std::exit(2); }
      return argv[++i];
    };
    if      (a == "--ip") ip = val();
    else if (a == "--urdf") urdf = val();
    else if (a == "--sim") use_sim = true;
    else if (a == "--dry-run") dry_run = true;
    else if (a == "--kind") kind_s = val();
    else if (a == "--mode") mode_s = val();
    else if (a == "--joint") joint = std::stoi(val());
    else if (a == "--delta") delta = std::stod(val());
    else if (a == "--tau") tau_ff = std::stod(val());
    else if (a == "--stream-s") stream_s = std::stod(val());
    else if (a == "--stale-s") stale_s = std::stod(val());
    else if (a == "--resume-s") resume_s = std::stod(val());
    else if (a == "--cmd-rate") cmd_rate_hz = std::stod(val());
    else if (a == "--timeout") timeout_s = std::stod(val());
    else if (a == "--rate") rate_hz = std::stod(val());
    else if (a == "--cpu") cpu = std::stoi(val());
    else if (a == "--rt-priority") rt_priority = std::stoi(val());
    else if (a == "--pacing") pacing_str = val();
    else { std::cerr << "unknown arg: " << a << "\n"; std::exit(2); }
  }

  Pacing pacing = Pacing::kSleepSpin;
  if (pacing_str == "nanosleep") pacing = Pacing::kClockNanosleep;
  else if (pacing_str != "sleepspin") { std::cerr << "--pacing must be sleepspin|nanosleep\n"; return 2; }

  SetpointKind kind;
  if      (kind_s == "joint-position") kind = SetpointKind::kJointPosition;
  else if (kind_s == "pose")           kind = SetpointKind::kEePose;
  else if (kind_s == "joint-torque")   kind = SetpointKind::kJointTorque;
  else { std::cerr << "--kind must be joint-position|pose|joint-torque "
                      "(the velocity kinds need JointVelocityMode, which does not exist yet)\n"; return 2; }

  ControlModeKind mode;
  if      (mode_s == "position")  mode = ControlModeKind::kPosition;
  else if (mode_s == "impedance") mode = ControlModeKind::kImpedance;
  else if (mode_s == "torque")    mode = ControlModeKind::kTorque;
  else { std::cerr << "--mode must be position|impedance|torque\n"; return 2; }

  // Same table the driver enforces. Checking it here too means an unsupported
  // pair is refused before we connect to an arm, rather than after.
  if (!pair_supported(kind, mode)) {
    std::cerr << "--kind " << kind_s << " is not supported with --mode " << mode_s
              << ". Supported: joint-position + position|impedance, pose + impedance, "
                 "joint-torque + torque.\n";
    return 2;
  }
  if (joint < 0 || joint >= kNumJoints) { std::cerr << "--joint must be 0.." << (kNumJoints-1) << "\n"; return 2; }
  if (std::abs(delta) > 0.2) {
    std::cerr << "--delta " << delta << " rad is too large for a bring-up harness (cap 0.2).\n"; return 2;
  }
  if (timeout_s <= 0.0) {
    std::cerr << "--timeout must be > 0: an unbounded stream has no safe-stop, and the driver "
                 "refuses it at open anyway.\n"; return 2;
  }

  JointTorqueParams tp;
  if (kind == SetpointKind::kJointTorque && std::abs(tau_ff) > 0.5 * tp.torque_limit[joint]) {
    std::cerr << "--tau " << tau_ff << " N*m is more than half j" << joint << "'s "
              << tp.torque_limit[joint] << " N*m clamp. Keep the feedforward small.\n";
    return 2;
  }

  std::printf("[stream] kind=%s mode=%s joint=j%d timeout=%.3fs cmd_rate=%.0fHz "
              "phases stream=%.1fs stale=%.1fs resume=%.1fs sim=%s dry_run=%s\n",
              kind_s.c_str(), mode_s.c_str(), joint, timeout_s, cmd_rate_hz,
              stream_s, stale_s, resume_s, use_sim ? "yes" : "no", dry_run ? "yes" : "no");
  std::printf("[stream] on STALE, %s\n",
              mode == ControlModeKind::kTorque
                  ? "torque mode should ramp its feedforward to zero and settle to gravity-comp hold"
                  : "the mode should freeze its reference at MEASURED q and stop advancing");

  Dynamics dyn(urdf), pump_dyn(urdf);

  std::unique_ptr<Transport> transport;
  if (use_sim) { JointFeedback init; transport = std::make_unique<SimTransport>(init); }
  else {
#ifndef KINOVA_NO_KORTEX
    if (ip.empty()) { std::cerr << "real-robot mode requires --ip <addr> (or --sim)\n"; return 2; }
    transport = std::make_unique<KortexTransport>(ip);
#else
    std::cerr << "built without KORTEX; only --sim is available\n"; return 2;
#endif
  }
  Transport& raw = *transport;
  std::signal(SIGINT, on_sigint);

  // --- dry-run: READ-ONLY, never enters low-level servoing ---------------------
  if (dry_run) {
    raw.connect();
    JointFeedback fb;
    raw.receive(fb);
    std::printf("\n[dry-run] READ-ONLY — nothing is commanded.\n");
    for (int i = 0; i < kNumJoints; ++i)
      std::printf("  j%d  q=%+8.4f rad  tau=%+8.3f N*m\n", i, fb.q[i], fb.tau[i]);
    if (kind == SetpointKind::kJointTorque)
      std::printf("\n[dry-run] would stream %+.3f N*m of feedforward on j%d, on top of gravity comp.\n",
                  tau_ff, joint);
    else
      std::printf("\n[dry-run] would stream j%d from %+.4f to %+.4f rad (%+.2f deg).\n",
                  joint, fb.q[joint], fb.q[joint] + delta, delta * kRad2Deg);
    std::printf("[dry-run] then STOP streaming for %.1fs to exercise the watchdog, resume, and close.\n",
                stale_s);
    raw.safe_shutdown();
    return 0;
  }

  raw.connect();
  JointFeedback entry;
  raw.receive(entry);
  std::printf("\n[stream] entry: q[j%d]=%+.4f rad  tau[j%d]=%+.3f N*m\n",
              joint, entry.q[joint], joint, entry.tau[joint]);
  if (mode == ControlModeKind::kPosition)
    std::printf("[stream] POSITION mode has NO compliance — the servo chases the command at full "
                "authority. Nothing absorbs a mistake.\n");
  std::printf("[stream] starting in 3s — e-stop in reach. Ctrl-C aborts.\n");
  std::this_thread::sleep_for(std::chrono::seconds(3));

  raw.set_servoing_low_level();

  Seqlock<JointFeedback> snap;
  FeedbackTap tap(raw, snap);

  JointPositionMode pos(dyn);
  JointImpedanceMode imp(dyn);
  JointTorqueMode tau(dyn);
  SampleRing ring(8192);
  RtExecutor ex(tap, ring, {rate_hz, pacing, {rt_priority, cpu, true, true}});
  ConsoleBackend backend;
  Supervisor sup(pos, imp, tau, ex, snap, pump_dyn, backend, backend);

  sup.start();
  std::atomic<bool> stop{false};
  std::thread rt([&] { ex.run(stop); });

  StreamOpenRequest open_req;
  open_req.kind = kind;
  open_req.control_mode = mode;
  open_req.timeout_s = timeout_s;
  const StreamOpenResult opened = sup.on_stream_open(open_req);
  if (!opened.accepted) {
    std::printf("\n[stream] session REFUSED: code=%d %s\n", opened.error_code, opened.message.c_str());
    stop = true; rt.join(); sup.stop(); raw.safe_shutdown();
    return 1;
  }
  std::printf("[stream] session open.\n\n");

  std::vector<Sample> trace;
  const double total = stream_s + stale_s + resume_s;
  trace.reserve(size_t(total * 25) + 16);
  const auto t0 = std::chrono::steady_clock::now();
  auto elapsed = [&] {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  };
  const auto cmd_period = std::chrono::duration<double>(1.0 / cmd_rate_hz);
  double next_sample = 0.0;
  const char* last_phase = "";

  while (!g_stop.load()) {
    const double el = elapsed();
    if (el >= total) break;
    const char* ph = phase_of(el, stream_s, stale_s, resume_s);
    if (ph != last_phase) {
      std::printf("\n[stream] --- phase %s at %.2fs ---\n", ph, el);
      if (std::string(ph) == "STALE ")
        std::printf("[stream] NOT streaming. Watch the arm: it should go safe, not drift.\n");
      last_phase = ph;
    }

    // Push setpoints in every phase EXCEPT the deliberate stall.
    if (std::string(ph) != "STALE ") {
      JointFeedback fb;
      const bool ok = snap.load(fb);
      if (ok) {
        if (kind == SetpointKind::kJointTorque) {
          JointSetpoint sp; sp.values.setZero(); sp.values[joint] = tau_ff;
          sup.on_setpoint_joint_torque(sp);
        } else if (kind == SetpointKind::kEePose) {
          JointVec target = entry.q; target[joint] = entry.q[joint] + delta;
          PoseSetpoint sp; sp.pose = dyn.fk(target);
          sup.on_setpoint_pose(sp);
        } else {
          JointSetpoint sp; sp.values = entry.q; sp.values[joint] = entry.q[joint] + delta;
          sup.on_setpoint_joint_position(sp);
        }
      }
    }

    if (el >= next_sample) {
      JointFeedback fb;
      if (snap.load(fb)) trace.push_back({el, ph, fb.q, fb.tau});
      next_sample = el + 0.05;   // 20 Hz
    }
    std::this_thread::sleep_for(
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(cmd_period));
  }

  std::printf("\n[stream] --- phase CLOSE  at %.2fs ---\n", elapsed());
  StreamCloseRequest close_req;
  sup.on_stream_close(close_req);
  std::printf("[stream] session closed; the teardown latches hold-at-measured-q.\n");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  JointFeedback after;
  const bool after_ok = snap.load(after);
  stop = true;
  rt.join();
  sup.stop();
  raw.safe_shutdown();

  // --- report ------------------------------------------------------------------
  std::printf("\n[stream] trace (t, phase, q[j%d], tau[j%d]):\n", joint, joint);
  for (const auto& s : trace)
    std::printf("  %5.2fs  %s  q=%+8.4f  tau=%+8.3f\n", s.t, s.phase, s.q[joint], s.tau[joint]);

  if (after_ok)
    std::printf("\n[stream] after close: q[j%d]=%+.4f rad (entry was %+.4f, moved %+.4f)\n",
                joint, after.q[joint], entry.q[joint], after.q[joint] - entry.q[joint]);
  std::printf("[stream] state publications seen: %llu\n",
              (unsigned long long)backend.states());

  std::printf("\n[stream] WHAT TO CHECK:\n"
              "   * STREAM: j%d moves toward the commanded value, smoothly and rate-limited.\n"
              "   * STALE:  motion STOPS within about %.3fs of the last setpoint. %s\n"
              "   * RESUME: motion picks up again — the freeze released on a fresh command.\n"
              "   * CLOSE:  the arm holds where it is and does NOT slew back toward the\n"
              "             last streamed setpoint. A slew here means the teardown hold failed.\n"
              "   * throughout: no goal settle lines appear (nothing but the stream is driving).\n",
              joint, timeout_s,
              mode == ControlModeKind::kTorque
                  ? "Torque: the feedforward ramps out and the arm settles to gravity-comp hold."
                  : "Position/impedance: the reference freezes at measured q.");
  if (use_sim)
    std::printf("[stream] (--sim is a static echo: the arm never moves and the trace is flat. "
                "Plumbing only — the STALE phase proves nothing about hardware.)\n");
  return 0;
}
