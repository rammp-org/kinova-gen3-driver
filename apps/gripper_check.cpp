// TEMPORARY — bring-up harness, not a maintained tool. Delete this app and its
// CMake block once the gripper tier has been validated on the arm. Nothing in
// the library depends on it; removal is `git rm` plus one CMake block.
//
// gripper_check — hardware validation for the gripper core plumbing.
//
// Answers three things that cannot be settled in sim, because SimTransport's
// gripper is a model we wrote rather than a measurement:
//
//   1. REGRESSION. Speed and force are now transmitted per-command; they used to
//      be pinned to constants (100% / 50%) inside KortexTransport. That wire path
//      has never run on hardware. Does the gripper still open and close correctly?
//
//   2. kGripperMaxCurrentA. GripperFeedback::effort is |current| / this constant,
//      and the constant is currently the datasheet's rated stall current, not a
//      measurement. Every effort value downstream is a fraction of it. Closing
//      fully with force=1.0 stalls the fingers against each other -- a hard stop
//      the gripper is designed for, with nothing in the way to damage -- and the
//      current there is the number we want.
//
//   3. MotorFeedback::velocity's UNITS AND SIGN. The SDK's generated header
//      carries no units, so the driver's /100 conversion is inferred from the
//      field name and nothing else. This matters more than a normal TODO: this
//      branch is what authorizes kinova_arm_ros2 to publish that number into
//      /joint_states, a units-bearing ROS field. An open-close-open cycle logged
//      against the position derivative settles both the scale and the sign.
//
// The arm is held by JointPositionMode at its entry configuration -- stiff, and
// measured on this arm to hold to four decimal places over seconds -- so the arm
// stays put while the gripper moves. That also exercises the claim that the
// gripper is orthogonal to control modes: GripperController is a Transport
// decorator, so it stamps the gripper regardless of which mode is running.
//
// THE GRIPPER MUST BE EMPTY. It closes fully, at the commanded force.
//
//   # 1. READ-ONLY, nothing moves
//   ./gripper_check --ip 192.168.1.10 --dry-run
//
//   # 2. the real check, logging a cycle for the velocity analysis
//   ./gripper_check --ip 192.168.1.10 --csv /tmp/gripper.csv
//
// Under --sim this exercises plumbing only: the simulated gripper follows a
// first-order model we wrote, so it can confirm the wiring but proves nothing
// about the hardware questions above.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/feedback_tap.h"
#include "kinova_lowlevel/gripper_controller.h"
#include "kinova_lowlevel/gripper_types.h"
#include "kinova_lowlevel/joint_position_mode.h"
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

enum class Phase { kOpen1, kClose, kOpen2 };
const char* phase_name(Phase p) {
  switch (p) {
    case Phase::kOpen1: return "OPEN ";
    case Phase::kClose: return "CLOSE";
    case Phase::kOpen2: return "OPEN2";
  }
  return "?????";
}

struct Sample {
  double t;
  Phase  phase;
  float  position, effort, current;
  bool   present;
};
}  // namespace

int main(int argc, char** argv) {
  std::string ip, csv_path;
  std::string urdf = "../models/gen3_7dof_2f85.urdf";
  std::string pacing_str = "sleepspin";
  bool use_sim = false, dry_run = false;
  float force = 1.0f;      // full force: the stall current is the point
  float speed = 0.5f;      // half speed -- gentler, and more samples while moving
  double hold_s = 2.5;     // per phase
  double rate_hz = 1000.0;
  double sample_hz = 50.0; // trace rate; the velocity analysis needs resolution
  int cpu = -1, rt_priority = 80;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto val = [&]() -> std::string {
      if (i + 1 >= argc) { std::cerr << "missing value for " << a << "\n"; std::exit(2); }
      return argv[++i];
    };
    if (a == "--ip") ip = val();
    else if (a == "--urdf") urdf = val();
    else if (a == "--csv") csv_path = val();
    else if (a == "--sim") use_sim = true;
    else if (a == "--dry-run") dry_run = true;
    else if (a == "--force") force = std::stof(val());
    else if (a == "--speed") speed = std::stof(val());
    else if (a == "--hold-s") hold_s = std::stod(val());
    else if (a == "--rate") rate_hz = std::stod(val());
    else if (a == "--sample-hz") sample_hz = std::stod(val());
    else if (a == "--cpu") cpu = std::stoi(val());
    else if (a == "--rt-priority") rt_priority = std::stoi(val());
    else if (a == "--pacing") pacing_str = val();
    else { std::cerr << "unknown arg: " << a << "\n"; std::exit(2); }
  }

  Pacing pacing = Pacing::kSleepSpin;
  if (pacing_str == "nanosleep") pacing = Pacing::kClockNanosleep;
  else if (pacing_str != "sleepspin") { std::cerr << "--pacing must be sleepspin|nanosleep\n"; return 2; }
  if (force < 0.0f || force > 1.0f || speed <= 0.0f || speed > 1.0f) {
    std::cerr << "--force must be 0..1 and --speed must be >0..1\n"; return 2;
  }

  std::printf("[grip] force=%.2f speed=%.2f hold=%.1fs/phase rate=%.0fHz "
              "sample=%.0fHz sim=%s dry_run=%s\n",
              force, speed, hold_s, rate_hz, sample_hz,
              use_sim ? "yes" : "no", dry_run ? "yes" : "no");
  std::printf("[grip] effort is reported as |current| / kGripperMaxCurrentA "
              "(currently %.3f A, from the datasheet, NOT measured).\n",
              kGripperMaxCurrentA);

  Dynamics dyn(urdf);

  std::unique_ptr<Transport> transport;
  if (use_sim) {
    JointFeedback init;
    auto sim = std::make_unique<SimTransport>(init);
    sim->set_gripper_lag(0.02f);   // a plausible closing rate, purely cosmetic
    transport = std::move(sim);
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
    std::printf("\n[dry-run] READ-ONLY — nothing is commanded.\n");
    std::printf("  gripper present=%s position=%.4f effort=%.4f current=%.4f A\n",
                fb.gripper.present ? "yes" : "NO", fb.gripper.position,
                fb.gripper.effort, fb.gripper.current);
    if (!fb.gripper.present)
      std::printf("  NOTE: no interconnect gripper reported. Everything below would be a no-op.\n");
    std::printf("  arm would be held at its entry configuration by JointPositionMode:\n");
    for (int i = 0; i < kNumJoints; ++i) std::printf("    j%d q=%+8.4f rad\n", i, fb.q[i]);
    std::printf("\n[dry-run] would cycle the gripper OPEN -> CLOSE -> OPEN at "
                "force=%.2f speed=%.2f, %.1fs per phase.\n", force, speed, hold_s);
    raw.safe_shutdown();
    return 0;
  }

  raw.connect();
  JointFeedback entry;
  raw.receive(entry);
  if (!entry.gripper.present) {
    std::cerr << "\n[grip] no interconnect gripper reported (present=false). "
                 "Nothing to test — refusing to run.\n";
    raw.safe_shutdown();
    return 3;
  }
  std::printf("\n[grip] entry: gripper position=%.4f current=%.4f A\n",
              entry.gripper.position, entry.gripper.current);
  std::printf("[grip] *** THE GRIPPER MUST BE EMPTY *** it closes FULLY at "
              "force=%.2f. The fingers stall against each other, which is a stop "
              "the gripper is built for — but anything held will be crushed.\n", force);
  std::printf("[grip] the ARM is held stiffly by JointPositionMode and should not "
              "move at all. Only the gripper moves.\n");
  std::printf("[grip] starting in 3s — e-stop in reach. Ctrl-C aborts.\n");
  std::this_thread::sleep_for(std::chrono::seconds(3));

  raw.set_servoing_low_level();

  // Chain: raw -> GripperController (stamps the gripper) -> FeedbackTap
  // (snapshots feedback for this thread) -> RtExecutor. Same shape as the
  // RT-safety case, and the reason the gripper needs no mode of its own.
  Seqlock<JointFeedback> snap;
  GripperController gc(raw);
  FeedbackTap tap(gc, snap);

  JointPositionMode mode(dyn);      // holds at entry q; stiff, no drift
  SampleRing ring(8192);
  RtExecutor ex(tap, ring, {rate_hz, pacing, {rt_priority, cpu, true, true}});
  ex.request_mode(&mode);

  std::atomic<bool> stop{false};
  std::thread rt([&] { ex.run(stop); });

  std::vector<Sample> trace;
  trace.reserve(size_t(3.0 * hold_s * sample_hz) + 32);
  const auto t0 = std::chrono::steady_clock::now();
  auto elapsed = [&] {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  };

  const double sample_dt = 1.0 / sample_hz;
  double next_sample = 0.0;
  Phase phase = Phase::kOpen1;
  Phase announced = Phase::kOpen2;   // force the first announcement
  bool first = true;

  while (!g_stop.load()) {
    const double el = elapsed();
    if (el >= 3.0 * hold_s) break;

    if (el < hold_s)            phase = Phase::kOpen1;
    else if (el < 2.0 * hold_s) phase = Phase::kClose;
    else                        phase = Phase::kOpen2;

    if (first || phase != announced) {
      std::printf("\n[grip] --- phase %s at %.2fs ---\n", phase_name(phase), el);
      announced = phase; first = false;
    }

    // Stateless by design: every command carries all three fields.
    GripperCommand g;
    g.position = (phase == Phase::kClose) ? 1.0f : 0.0f;
    g.speed    = speed;
    g.force    = force;
    gc.set_target(g);

    if (el >= next_sample) {
      JointFeedback fb;
      if (snap.load(fb))
        trace.push_back({el, phase, fb.gripper.position,
                         fb.gripper.effort, fb.gripper.current, fb.gripper.present});
      next_sample = el + sample_dt;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));   // ~200 Hz publisher
  }
  stop = true;
  rt.join();
  raw.safe_shutdown();

  // --- report ----------------------------------------------------------------
  if (csv_path.size()) {
    std::ofstream f(csv_path);
    f << "t,phase,position,effort,current,present\n";
    for (const auto& s : trace)
      f << s.t << "," << phase_name(s.phase) << "," << s.position
        << "," << s.effort << "," << s.current << "," << (s.present ? 1 : 0) << "\n";
    std::printf("\n[grip] wrote %zu samples to %s\n", trace.size(), csv_path.c_str());
  }

  std::printf("\n[grip] trace (t, phase, position, effort, current):\n");
  for (const auto& s : trace)
    std::printf("  %5.2fs  %s  pos=%.4f  eff=%.4f  cur=%.4f A\n",
                s.t, phase_name(s.phase), s.position, s.effort, s.current);

  // Peak current while stalled closed -- the kGripperMaxCurrentA candidate. Take
  // it from the LATER half of the close phase, once the fingers have met; the
  // earlier half is the free-closing transient, which is not a stall.
  float peak_stall = 0.0f, pos_at_stall = 0.0f;
  for (const auto& s : trace)
    if (s.phase == Phase::kClose && s.t > 1.5 * hold_s && std::fabs(s.current) > peak_stall) {
      peak_stall = std::fabs(s.current);
      pos_at_stall = s.position;
    }

  // Peak closing rate from the POSITION derivative -- the only honest rate available,
  // now that GripperFeedback carries no velocity field. MotorFeedback's velocity was
  // measured to be the commanded speed echoed back, so it was removed rather than
  // published; see gripper_types.h.
  auto peak_rate = [&](Phase ph) {
    double best = 0.0;
    for (size_t i = 1; i < trace.size(); ++i) {
      if (trace[i].phase != ph) continue;
      const double dt = trace[i].t - trace[i - 1].t;
      if (dt <= 0.0) continue;
      best = std::max(best, std::fabs(double(trace[i].position) - double(trace[i - 1].position)) / dt);
    }
    return best;
  };

  std::printf("\n[grip] RESULTS\n");
  std::printf(" 1. REGRESSION — did it move?\n");
  if (trace.size() >= 2) {
    float pmin = 1.0f, pmax = 0.0f;
    for (const auto& s : trace) { pmin = std::min(pmin, s.position); pmax = std::max(pmax, s.position); }
    std::printf("    position swept %.4f .. %.4f (a full cycle should approach 0 .. 1)\n", pmin, pmax);
  }
  std::printf(" 2. kGripperMaxCurrentA — peak |current| while stalled closed: %.4f A "
              "(at position %.4f)\n", peak_stall, pos_at_stall);
  std::printf("    current constant is %.3f A. If these disagree, update it in "
              "include/kinova_lowlevel/gripper_types.h — every effort value is a fraction of it.\n",
              kGripperMaxCurrentA);
  std::printf(" 3. position rate (from the position derivative):\n");
  std::printf("    closing peak %.3f /s, opening peak %.3f /s at speed=%.2f\n",
              peak_rate(Phase::kClose), peak_rate(Phase::kOpen2), speed);
  std::printf("    Full-scale rate is this divided by --speed. MotorFeedback's own\n"
              "    velocity field is NOT reported: it was measured to be the commanded\n"
              "    speed echoed back, so it was removed rather than published.\n");
  if (use_sim)
    std::printf("\n[grip] (--sim follows a model we wrote. It confirms wiring and "
                "proves nothing about 2 or 3.)\n");
  return 0;
}
