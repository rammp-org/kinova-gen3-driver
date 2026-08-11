// joint_position_check — drives JointPositionMode through the RtExecutor and
// reports timing. This is the hardware validation harness for joint-space
// position control; the procedure that uses it is
// docs/integration/joint_position_hardware_check.md.
//
// Position mode has NO compliance. The actuator's own servo chases whatever is
// commanded at full authority, so unlike the impedance benchmarks there is no
// spring to absorb a mistake. Defaults here are deliberately more conservative
// than the mode's own: 0.2 rad/s reference speed, 5 s duration, zero motion.
//
// --dry-run is READ-ONLY: connects, reads feedback, and prints what WOULD be
// commanded — never enters low-level servoing, never commands anything. Run it
// first. It is also the test for the open question about the KORTEX position
// command format: the transport sends rad_to_deg(cmd.position) unwrapped while
// feedback is wrapped to (-pi, pi], so continuous joints past half a turn come
// back as NEGATIVE degrees. Dry-run shows you exactly which joints those are
// before anything moves.
//
//   # 1. read-only, no servoing, nothing moves
//   ./joint_position_check --ip 192.168.1.10 --dry-run --duration 10
//
//   # 2. hold the entry configuration (no target => commands where it already is)
//   ./joint_position_check --ip 192.168.1.10 --duration 5
//
//   # 3. move ONE joint a small amount
//   ./joint_position_check --ip 192.168.1.10 --joint 5 --delta 0.2 --duration 8
//
//   # 4. scripted VISUAL CHECK: one joint at a time, each returning to start,
//   #    then home. Read the plan first with --dry-run; it commands nothing.
//   ./joint_position_check --ip 192.168.1.10 --sequence --dry-run
//   ./joint_position_check --ip 192.168.1.10 --sequence
//   ./joint_position_check --ip 192.168.1.10 --sequence --from-joint 1   # whole arm
//
//   # sim (no robot); protocol/plumbing only, the arm will not move
//   ./joint_position_check --sim --urdf ../models/gen3_7dof_2f85.urdf

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
#include <Eigen/Dense>

#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/joint_position_mode.h"
#include "kinova_lowlevel/rt_executor.h"
#include "kinova_lowlevel/rt_system.h"
#include "kinova_lowlevel/sim_transport.h"
#include "kinova_lowlevel/telemetry.h"
#include "kinova_lowlevel/telemetry_consumers.h"
#include "kinova_lowlevel/transport.h"
#include "kinova_lowlevel/units.h"
#ifndef KINOVA_NO_KORTEX
#include "kinova_lowlevel/kortex_transport.h"
#endif

using namespace kinova;

namespace {
std::atomic<bool> g_stop{false};
void on_sigint(int) { g_stop.store(true); }

// Print measured state alongside the degree value the transport would put on the
// wire, so the sign question is visible rather than inferred.
void report_state(const JointFeedback& fb, const JointVec& lo, const JointVec& hi) {
  int negatives = 0;
  for (int i = 0; i < kNumJoints; ++i) {
    const bool continuous = !std::isfinite(lo[i]) && !std::isfinite(hi[i]);
    const double deg = fb.q[i] * kRad2Deg;
    if (deg < 0.0) ++negatives;
    std::printf("  j%d  q=%+8.4f rad  %+9.3f deg  ", i, fb.q[i], deg);
    if (continuous) std::printf("[continuous]      ");
    else std::printf("[%+.2f,%+.2f]   ", lo[i], hi[i]);
    std::printf("-> would command %+9.3f deg\n", deg);
  }
  if (negatives > 0) {
    std::printf(
        "\n  NOTE: %d joint(s) would be commanded as a NEGATIVE degree value.\n"
        "        KORTEX reports positions in [0,360); whether it accepts a\n"
        "        negative setpoint in POSITION mode is the open question this\n"
        "        check exists to answer. Watch those joints closely in step 2.\n",
        negatives);
  }
}
// One step of the visual verification sequence.
struct Waypoint {
  std::string label;
  JointVec q;
  double settle_s;   // how long to sit here once the travel time has elapsed
};

// Scripted sequence for eyeballing the mode on the real arm. Deliberately moves
// ONE joint at a time and returns to the start after each, so "only the named
// joint moved" and "it came back" are both things you can see rather than infer.
//
// Ordered wrist-first (j6 -> j<from_joint>): the distal joints are the lightest
// and the least able to hurt anything if a direction is wrong. Waypoints are
// clamped to the URDF limits here as well as in the mode, so the printed plan is
// what will actually happen rather than what was asked for.
std::vector<Waypoint> build_sequence(const JointVec& home, int from_joint,
                                     double delta, double speed,
                                     const JointVec& lo, const JointVec& hi) {
  std::vector<Waypoint> wps;
  const double travel = std::abs(delta) / std::max(1e-9, speed);
  wps.push_back({"settle — NOTHING should move here", home, 2.0});
  for (int j = kNumJoints - 1; j >= from_joint; --j) {
    JointVec away = home;
    away[j] += delta;
    if (std::isfinite(lo[j]) && std::isfinite(hi[j]))
      away[j] = std::clamp(away[j], lo[j], hi[j]);
    const double moved = away[j] - home[j];
    char buf[160];
    std::snprintf(buf, sizeof(buf),
                  "j%d %+.3f rad (%+.1f deg) — ONLY this joint should move", j,
                  moved, moved * kRad2Deg);
    wps.push_back({buf, away, 0.7});
    std::snprintf(buf, sizeof(buf), "j%d back to start", j);
    wps.push_back({buf, home, 0.7});
  }
  wps.push_back({"HOME — compare against where the arm started", home, 2.0});
  for (auto& w : wps) w.settle_s += travel;   // allow for the move itself
  return wps;
}
}  // namespace

int main(int argc, char** argv) {
  std::string ip;
  std::string urdf = "../models/gen3_7dof_2f85.urdf";
  std::string csv_path;
  std::string pacing_str = "sleepspin";
  bool use_sim = false;
  bool dry_run = false;
  double rate_hz = 1000.0;
  int cpu = -1;
  int rt_priority = 80;
  double duration_s = 5.0;
  int move_joint = -1;
  double delta = 0.0;
  double speed = 0.2;       // rad/s; below the mode's own 0.5 default
  double leash = 0.35;
  bool sequence = false;
  int from_joint = 4;       // wrist only by default; lower it deliberately
  bool duration_set = false;

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
    else if (a == "--dry-run") dry_run = true;
    else if (a == "--urdf") urdf = next("--urdf");
    else if (a == "--rate") rate_hz = std::stod(next("--rate"));
    else if (a == "--cpu") cpu = std::stoi(next("--cpu"));
    else if (a == "--rt-priority") rt_priority = std::stoi(next("--rt-priority"));
    else if (a == "--duration") { duration_s = std::stod(next("--duration")); duration_set = true; }
    else if (a == "--sequence") sequence = true;
    else if (a == "--from-joint") from_joint = std::stoi(next("--from-joint"));
    else if (a == "--pacing") pacing_str = next("--pacing");
    else if (a == "--csv") csv_path = next("--csv");
    else if (a == "--joint") move_joint = std::stoi(next("--joint"));
    else if (a == "--delta") delta = std::stod(next("--delta"));
    else if (a == "--speed") speed = std::stod(next("--speed"));
    else if (a == "--leash") leash = std::stod(next("--leash"));
    else {
      std::cerr << "unknown arg: " << a << "\n";
      std::exit(2);
    }
  }

  Pacing pacing = Pacing::kSleepSpin;
  if (pacing_str == "nanosleep") pacing = Pacing::kClockNanosleep;
  else if (pacing_str != "sleepspin") {
    std::cerr << "--pacing must be sleepspin|nanosleep\n";
    std::exit(2);
  }
  if (move_joint >= kNumJoints || (move_joint < 0 && move_joint != -1)) {
    std::cerr << "--joint must be 0.." << (kNumJoints - 1) << "\n";
    return 2;
  }
  if (move_joint == -1 && delta != 0.0 && !sequence) {
    std::cerr << "--delta given without --joint; refusing to guess which joint\n";
    return 2;
  }
  if (sequence && move_joint >= 0) {
    std::cerr << "--sequence and --joint are different runs; pick one\n";
    return 2;
  }
  if (from_joint < 0 || from_joint >= kNumJoints) {
    std::cerr << "--from-joint must be 0.." << (kNumJoints - 1) << "\n";
    return 2;
  }
  if (sequence && delta == 0.0) delta = 0.2;   // sequence needs some motion

  std::cout << "[jpos] urdf=" << urdf << " rate=" << rate_hz << "Hz pacing="
            << pacing_str << " cpu=" << cpu << " prio=" << rt_priority
            << " duration=" << duration_s << "s sim=" << (use_sim ? "yes" : "no")
            << " dry_run=" << (dry_run ? "yes" : "no")
            << " speed=" << speed << "rad/s leash=" << leash << "rad\n";

  Dynamics dyn(urdf);
  JointVec lo, hi;
  dyn.joint_limits(lo, hi);

  std::unique_ptr<Transport> transport;
  if (use_sim) {
    JointFeedback init;  // zero state
    transport = std::make_unique<SimTransport>(init);
  } else {
#ifndef KINOVA_NO_KORTEX
    if (ip.empty()) {
      std::cerr << "real-robot mode requires --ip <addr> (or pass --sim)\n";
      return 2;
    }
    transport = std::make_unique<KortexTransport>(ip);
#else
    std::cerr << "built without KORTEX; only --sim is available\n";
    return 2;
#endif
  }
  Transport& t = *transport;

  std::signal(SIGINT, on_sigint);

  // --- dry-run: READ-ONLY, never enters low-level servoing ----------------------
  if (dry_run) {
    t.connect();
    // --sequence --dry-run: print exactly what the sequence WOULD do, from the
    // arm's current configuration, and command nothing. Read this before running
    // it for real.
    if (sequence) {
      JointFeedback fb;
      t.receive(fb);
      std::cout << "\n[dry-run] current configuration:\n";
      report_state(fb, lo, hi);
      const auto wps = build_sequence(fb.q, from_joint, delta, speed, lo, hi);
      double total = 0.0;
      std::cout << "\n[dry-run] sequence plan (" << wps.size()
                << " waypoints, joints j" << from_joint << "..j"
                << (kNumJoints - 1) << " at " << speed << " rad/s):\n";
      for (size_t k = 0; k < wps.size(); ++k) {
        total += wps[k].settle_s;
        std::printf("  %2zu. [%5.1fs] %s\n", k + 1, total, wps[k].label.c_str());
      }
      std::printf("\n[dry-run] total ~%.0f s. NOTHING was commanded.\n", total);
      t.safe_shutdown();
      return 0;
    }
    std::cout << "[dry-run] READ-ONLY — nothing is commanded, the arm stays "
                 "under its own control.\n"
                 "          Move it by hand/pendant to see the values change.\n";
    JointFeedback fb;
    const auto start = std::chrono::steady_clock::now();
    auto last_print = start - std::chrono::seconds(1);
    while (!g_stop.load(std::memory_order_acquire)) {
      t.receive(fb);
      const auto now = std::chrono::steady_clock::now();
      if (now - last_print >= std::chrono::milliseconds(1000)) {
        std::printf("\n--- measured ---\n");
        report_state(fb, lo, hi);
        last_print = now;
      }
      if (duration_s > 0.0 &&
          std::chrono::duration<double>(now - start).count() >= duration_s) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));  // ~50 Hz read
    }
    t.safe_shutdown();  // never went low-level; just closes the session
    std::cout << "\n[dry-run] done — nothing was ever commanded.\n";
    return 0;
  }

  // --- live: read the entry configuration BEFORE servoing, to build the target --
  t.connect();
  JointFeedback entry;
  t.receive(entry);
  std::cout << "\n[jpos] entry configuration:\n";
  report_state(entry, lo, hi);

  std::vector<Waypoint> wps;
  JointVec target = entry.q;
  if (sequence) {
    wps = build_sequence(entry.q, from_joint, delta, speed, lo, hi);
    double total = 0.0;
    std::cout << "\n[jpos] VISUAL CHECK SEQUENCE — joints j" << from_joint
              << "..j" << (kNumJoints - 1) << " at " << speed << " rad/s:\n";
    for (size_t k = 0; k < wps.size(); ++k) {
      total += wps[k].settle_s;
      std::printf("  %2zu. [%5.1fs] %s\n", k + 1, total, wps[k].label.c_str());
    }
    std::cout <<
        "\n[jpos] WHAT TO WATCH FOR:\n"
        "   * step 1: the arm does not move at all. Any twitch here is a bug.\n"
        "   * each move: ONLY the named joint turns, in the direction printed.\n"
        "   * each return: that joint goes back, the others never moved.\n"
        "   * the end: the arm is visibly where it started, and every residual\n"
        "     in the report reads ~0.0000. That is the reference being exact.\n"
        "   * throughout: motion is smooth and rate-limited, never a snap.\n";
    std::printf("[jpos] total ~%.0f s.\n", total);
    // Hard outer cap so the run always terminates even if the sequence thread
    // wedges. The sequence normally stops the loop itself.
    if (!duration_set) duration_s = total + 5.0;
  } else if (move_joint >= 0) {
    target[move_joint] += delta;
    std::printf("\n[jpos] MOVING j%d by %+.4f rad (%+.2f deg): %+.4f -> %+.4f rad\n",
                move_joint, delta, delta * kRad2Deg, entry.q[move_joint],
                target[move_joint]);
    std::printf("[jpos] at %.2f rad/s that takes about %.1f s of travel.\n",
                speed, std::abs(delta) / std::max(1e-9, speed));
  } else {
    std::cout << "\n[jpos] HOLD ONLY — target is the entry configuration, no "
                 "motion is requested.\n";
  }
  std::cout << "[jpos] starting in 2s — e-stop in reach. Ctrl-C aborts.\n";
  std::this_thread::sleep_for(std::chrono::seconds(2));

  t.set_servoing_low_level();

  JointPositionParams p;
  p.max_ref_speed.setConstant(speed);
  p.max_following_error = leash;
  JointPositionMode mode(dyn, p);
  {
    const JointVec eff = mode.params().max_ref_speed;
    std::printf("[jpos] effective speed after URDF clamp: %.3f .. %.3f rad/s\n",
                eff.minCoeff(), eff.maxCoeff());
  }

  SampleRing ring(1 << 16);
  TelemetrySink sink(csv_path);
  std::atomic<bool> draining{true};
  std::thread drain([&] {
    CycleSample s;
    auto last_print = std::chrono::steady_clock::now();
    while (draining.load(std::memory_order_acquire)) {
      while (ring.pop(s)) sink.consume(s);
      auto now = std::chrono::steady_clock::now();
      if (now - last_print >= std::chrono::seconds(1)) {
        std::cout << sink.console_line() << " dropped=" << ring.dropped() << "\n";
        last_print = now;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    while (ring.pop(s)) sink.consume(s);   // final drain
  });

  RtExecutor ex(t, ring, {rate_hz, pacing, {rt_priority, cpu, true}});
  ex.request_mode(&mode);

  // Publish targets only AFTER the executor has adopted the mode: on_enter
  // deliberately drops any target set before entry, so a target published now
  // would be silently discarded.
  std::thread publisher([&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    if (g_stop.load(std::memory_order_acquire)) return;
    if (!sequence) {
      mode.set_target(target);
      return;
    }
    for (size_t k = 0; k < wps.size(); ++k) {
      if (g_stop.load(std::memory_order_acquire)) return;
      std::printf("\n[jpos] %2zu/%zu  %s\n", k + 1, wps.size(),
                  wps[k].label.c_str());
      std::fflush(stdout);
      mode.set_target(wps[k].q);
      // Sleep in slices so Ctrl-C aborts promptly mid-waypoint rather than at
      // the end of it.
      const int slices = int(wps[k].settle_s * 20.0);
      for (int s = 0; s < slices; ++s) {
        if (g_stop.load(std::memory_order_acquire)) return;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }
    std::cout << "\n[jpos] sequence complete.\n";
    g_stop.store(true, std::memory_order_release);
  });

  std::thread watchdog;
  if (duration_s > 0.0) {
    watchdog = std::thread([&] {
      auto deadline = std::chrono::steady_clock::now() +
                      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                          std::chrono::duration<double>(duration_s));
      while (!g_stop.load(std::memory_order_acquire)) {
        if (std::chrono::steady_clock::now() >= deadline) {
          g_stop.store(true, std::memory_order_release);
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    });
  }

  ResourceUsage usage_before = read_usage();   // main thread IS the RT loop thread
  ex.run(g_stop);
  ResourceUsage usage_after = read_usage();

  t.safe_shutdown();

  if (watchdog.joinable()) watchdog.join();
  publisher.join();
  draining.store(false, std::memory_order_release);
  drain.join();

  const JointVec q_ref = mode.reference();
  std::cout << "\n==== joint position check report ====\n";
  std::cout << introspect() << "\n";
  for (int i = 0; i < kNumJoints; ++i) {
    std::printf("  j%d  target=%+8.4f  final_ref=%+8.4f  residual=%+8.4f rad\n",
                i, target[i], q_ref[i], wrap_to_pi(target[i] - q_ref[i]));
  }
  const auto& ch = sink.cycle_hist();
  const auto& mh = sink.compute_hist();
  std::cout << "cycle_ns   n=" << ch.count() << " min=" << ch.min()
            << " mean=" << ch.mean() << " p50=" << ch.percentile(0.50)
            << " p99=" << ch.percentile(0.99) << " p99.9=" << ch.percentile(0.999)
            << " max=" << ch.max() << "\n";
  std::cout << "compute_ns n=" << mh.count() << " min=" << mh.min()
            << " mean=" << mh.mean() << " p50=" << mh.percentile(0.50)
            << " p99=" << mh.percentile(0.99) << " p99.9=" << mh.percentile(0.999)
            << " max=" << mh.max() << "\n";
  std::cout << "dropped=" << ring.dropped() << "\n";
  std::cout << "page faults: minflt+=" << (usage_after.minflt - usage_before.minflt)
            << " majflt+=" << (usage_after.majflt - usage_before.majflt) << "\n";
  std::cout << "=====================================\n";
  return 0;
}
