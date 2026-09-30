// trajectory_run — drive a joint-space TRAJECTORY on the arm through the non-RT
// TrajectoryExecutor, JointPositionMode, and the RtExecutor. This is the first
// on-robot exercise of the interface execution core: a time-parameterized
// trajectory is sampled cycle-by-cycle off the RT thread and published to the
// mode via the core kinova::JointTargetSink seam; the RT loop runs the mode at
// 1 kHz against the transport.
//
// Position mode has NO compliance — the actuator servo chases the command at
// full authority. Defaults are conservative (0.2 rad/s peak, small move) and the
// trajectory is auto-timed to stay under the speed cap so the mode tracks it.
//
// The path-tolerance divergence guard is LIVE here: a FeedbackTap snapshots the
// arm's q at the RT read point, and the publisher thread reads the latest snapshot
// each tick and feeds it to the executor, which aborts if any joint diverges from
// the sampled trajectory by more than --path-tol (default 0.2 rad). --no-guard
// disables it. This backstops the mode's following-error leash + the slow speed cap.
//
//   # 1. READ-ONLY: prints the trajectory plan, commands nothing.
//   ./trajectory_run --ip 192.168.1.10 --joint 5 --delta 0.2 --dry-run
//
//   # 2. hold the entry configuration (no --delta => zero-motion trajectory)
//   ./trajectory_run --ip 192.168.1.10 --duration 4
//
//   # 3. move ONE joint a small amount along a timed trajectory
//   ./trajectory_run --ip 192.168.1.10 --joint 5 --delta 0.2
//
//   # sim (no robot): plumbing only — SimTransport does not move.
//   ./trajectory_run --sim --urdf ../models/gen3_7dof_2f85.urdf --joint 5 --delta 0.2
//
// SPEED SCALE. The executor dilates its own clock (traj_t += dt * scale), so the
// PATH is untouched and only the pace changes; velocity scales with s, and
// acceleration with s^2, for free. Two dials, and the SLOWER of the two wins:
//
//   # 4. the goal's own scale — what a planner client sets on submit.
//   #    Half speed => the same move takes twice as long.
//   ./trajectory_run --sim --urdf ../models/gen3_7dof_2f85.urdf \
//       --joint 5 --delta 0.2 --speed-scale 0.5
//
//   # 5. the RUNTIME override, changed mid-flight at t=T — the operator's dial.
//   #    This is the one that shows the slew limiter: the applied scale ramps at
//   #    kScaleSlewPerSec rather than stepping, because a step in scale is a step
//   #    in commanded velocity. Watch the "applied" column move.
//   ./trajectory_run --sim --urdf ../models/gen3_7dof_2f85.urdf \
//       --joint 5 --delta 0.4 --duration 8 --scale-at 3.0:0.2
//
// Out-of-range is REFUSED, not clamped: a scale below the floor would otherwise
// be silently sped UP to it, running the arm faster than asked.
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

#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/feedback_tap.h"
#include "kinova_lowlevel/interface/trajectory_executor.h"
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

void report_state(const JointFeedback& fb) {
  for (int i = 0; i < kNumJoints; ++i)
    std::printf("  j%d  q=%+8.4f rad  %+9.3f deg\n", i, fb.q[i], fb.q[i] * kRad2Deg);
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
  double duration_s = 0.0;  // trajectory duration; 0 => auto from delta/speed
  bool duration_set = false;
  int move_joint = -1;
  double delta = 0.0;
  double speed = 0.2;  // rad/s peak cap; below the mode's own 0.5 default
  double leash = 0.35;
  double tick_hz = 250.0;  // rate the publisher samples the trajectory at
  double path_tol = 0.2;   // rad; per-joint divergence guard (live feedback)
  bool no_guard = false;   // escape hatch: disable the divergence guard
  double speed_scale = 1.0;    // the GOAL's own scale, handed to submit()
  double scale_at_t = -1.0;    // wall time to change the runtime override at; <0 => never
  double scale_at_s = 1.0;     // the override to change to at scale_at_t

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&](const char* name) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << name << " needs a value\n";
        std::exit(2);
      }
      return argv[++i];
    };
    if (a == "--ip")
      ip = next("--ip");
    else if (a == "--sim")
      use_sim = true;
    else if (a == "--dry-run")
      dry_run = true;
    else if (a == "--urdf")
      urdf = next("--urdf");
    else if (a == "--rate")
      rate_hz = std::stod(next("--rate"));
    else if (a == "--cpu")
      cpu = std::stoi(next("--cpu"));
    else if (a == "--rt-priority")
      rt_priority = std::stoi(next("--rt-priority"));
    else if (a == "--duration") {
      duration_s = std::stod(next("--duration"));
      duration_set = true;
    } else if (a == "--joint")
      move_joint = std::stoi(next("--joint"));
    else if (a == "--delta")
      delta = std::stod(next("--delta"));
    else if (a == "--speed")
      speed = std::stod(next("--speed"));
    else if (a == "--leash")
      leash = std::stod(next("--leash"));
    else if (a == "--pacing")
      pacing_str = next("--pacing");
    else if (a == "--tick-rate")
      tick_hz = std::stod(next("--tick-rate"));
    else if (a == "--path-tol")
      path_tol = std::stod(next("--path-tol"));
    else if (a == "--no-guard")
      no_guard = true;
    else if (a == "--speed-scale")
      speed_scale = std::stod(next("--speed-scale"));
    else if (a == "--scale-at") {
      // T:S — at wall time T seconds, change the runtime override to S.
      const std::string v = next("--scale-at");
      const auto colon = v.find(':');
      if (colon == std::string::npos) {
        std::cerr << "--scale-at wants T:S, e.g. 3.0:0.2\n";
        std::exit(2);
      }
      scale_at_t = std::stod(v.substr(0, colon));
      scale_at_s = std::stod(v.substr(colon + 1));
    }
    else if (a == "--csv")
      csv_path = next("--csv");
    else {
      std::cerr << "unknown arg: " << a << "\n";
      std::exit(2);
    }
  }

  Pacing pacing = Pacing::kSleepSpin;
  if (pacing_str == "nanosleep")
    pacing = Pacing::kClockNanosleep;
  else if (pacing_str != "sleepspin") {
    std::cerr << "--pacing must be sleepspin|nanosleep\n";
    return 2;
  }
  if (move_joint >= kNumJoints || (move_joint < 0 && move_joint != -1)) {
    std::cerr << "--joint must be 0.." << (kNumJoints - 1) << "\n";
    return 2;
  }
  if (delta != 0.0 && move_joint < 0) {
    std::cerr << "--delta given without --joint; refusing to guess which joint\n";
    return 2;
  }
  if (speed <= 0.0) {
    std::cerr << "--speed must be > 0\n";
    return 2;
  }
  if (!no_guard && path_tol <= 0.0) {
    std::cerr << "--path-tol must be > 0 (or pass --no-guard)\n";
    return 2;
  }
  // Refused, never clamped: clamping a below-floor scale would silently run the
  // arm FASTER than asked, which is the wrong direction to fail in.
  auto check_scale = [](double s, const char* flag) {
    if (!std::isfinite(s) || s < interface::kMinSpeedScale || s > 1.0) {
      std::cerr << flag << " must be in [" << interface::kMinSpeedScale << ", 1.0]; got " << s
                << " (refused, not clamped)\n";
      std::exit(2);
    }
  };
  check_scale(speed_scale, "--speed-scale");
  if (scale_at_t >= 0.0) check_scale(scale_at_s, "--scale-at S");

  Dynamics dyn(urdf);

  std::unique_ptr<Transport> transport;
  if (use_sim) {
    JointFeedback init;
    transport = std::make_unique<SimTransport>(init);
  } else {
#ifndef KINOVA_NO_KORTEX
    if (ip.empty()) {
      std::cerr << "real-robot mode requires --ip <addr> (or --sim)\n";
      return 2;
    }
    transport = std::make_unique<KortexTransport>(ip);
#else
    std::cerr << "built without KORTEX; only --sim is available\n";
    return 2;
#endif
  }
  // Tap the transport so the publisher thread can read the arm's latest q for the
  // divergence guard. The RT loop is the only writer (it snapshots fb at exchange);
  // the publisher is the only reader. Every access below goes through the tap.
  Seqlock<JointFeedback> snapshot;
  FeedbackTap tapped(*transport, snapshot);
  Transport& t = tapped;
  std::signal(SIGINT, on_sigint);

  // Read the entry configuration first (read-only), so the trajectory starts
  // exactly where the arm is.
  t.connect();
  JointFeedback entry;
  t.receive(entry);
  std::cout << "\n[traj] entry configuration:\n";
  report_state(entry);

  // Build the target and auto-time the trajectory so its peak velocity sits
  // safely under the speed cap (0.8x), which lets the mode track it rather than
  // clip it at its own rate limit.
  JointVec target = entry.q;
  if (move_joint >= 0) target[move_joint] += delta;
  const double travel_speed = 0.8 * speed;  // rad/s the trajectory moves at
  if (!duration_set) {
    duration_s = (delta != 0.0) ? std::abs(delta) / travel_speed : 4.0;
  }
  const double peak_v = (duration_s > 0.0) ? std::abs(delta) / duration_s : 0.0;

  std::printf("\n[traj] plan: %s over %.2f s (peak %.3f rad/s, speed cap %.3f rad/s)\n",
              (delta != 0.0 ? "1-joint move" : "HOLD (no motion)"), duration_s, peak_v, speed);
  if (speed_scale != 1.0)
    std::printf("[traj]   speed-scale %.3f => same path, %.2f s wall, peak %.3f rad/s\n",
                speed_scale, duration_s / speed_scale, peak_v * speed_scale);
  if (scale_at_t >= 0.0)
    std::printf("[traj]   runtime override -> %.3f at t=%.2f s (slew-limited at %.1f /s)\n",
                scale_at_s, scale_at_t, interface::kScaleSlewPerSec);
  if (move_joint >= 0)
    std::printf("[traj]   j%d: %+.4f -> %+.4f rad (%+.2f deg)\n", move_joint, entry.q[move_joint],
                target[move_joint], delta * kRad2Deg);
  if (peak_v > speed + 1e-9)
    std::printf("[traj]   WARNING: peak velocity exceeds the speed cap — the mode will lag.\n");

  if (dry_run) {
    std::cout << "\n[dry-run] READ-ONLY — never entered low-level servoing, "
                 "nothing was commanded.\n";
    t.safe_shutdown();
    return 0;
  }

  std::cout << "[traj] starting in 2s — e-stop in reach. Ctrl-C aborts.\n";
  std::this_thread::sleep_for(std::chrono::seconds(2));
  if (g_stop.load(std::memory_order_acquire)) {
    t.safe_shutdown();
    return 0;
  }

  t.set_servoing_low_level();

  JointPositionParams p;
  p.max_ref_speed.setConstant(speed);
  p.max_following_error = leash;
  JointPositionMode mode(dyn, p);

  interface::Trajectory tr;
  tr.points = {{entry.q, 0.0}, {target, duration_s}};

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
    while (ring.pop(s)) sink.consume(s);
  });

  RtExecutor ex(t, ring, {rate_hz, pacing, {rt_priority, cpu, true}});
  ex.request_mode(&mode);

  // Publisher: after the executor adopts the mode (on_enter drops any target set
  // before entry), tick the TrajectoryExecutor at tick_hz. Each tick samples the
  // trajectory and publishes the joint reference through mode.set_target().
  std::thread publisher([&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    if (g_stop.load(std::memory_order_acquire)) return;

    interface::TrajectoryExecutor exec(mode);  // JointPositionMode IS-A JointTargetSink
    const JointVec tol = no_guard ? JointVec::Constant(-1.0)  // guard disabled
                                  : JointVec::Constant(path_tol);
    exec.submit(tr, interface::ControlModeKind::kPosition, interface::Preemption::kLatestWins, tol,
                speed_scale);

    const auto t0 = std::chrono::steady_clock::now();
    const auto period = std::chrono::duration<double>(1.0 / tick_hz);
    JointVec q_meas = entry.q;  // last good measured q; the tap seeds it from the entry read
    double last_applied = 1.0;  // so the slew ramp prints only when it moves
    while (!g_stop.load(std::memory_order_acquire)) {
      const double now_s =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      JointFeedback fb;
      if (snapshot.load(fb)) q_meas = fb.q;  // else keep last good q (no spurious abort)
      // The operator's dial: 1.0 until scale_at_t, then scale_at_s. The executor
      // slews toward it rather than stepping, so applied_scale() lags this.
      const double override_scale =
          (scale_at_t >= 0.0 && now_s >= scale_at_t) ? scale_at_s : 1.0;
      const interface::ExecStatus st = exec.tick(now_s, q_meas, override_scale);
      const double applied = exec.applied_scale();
      if (std::abs(applied - last_applied) > 1e-3) {
        std::printf("[traj]   t=%.2f s  override=%.3f  applied=%.3f (slewing)\n", now_s,
                    override_scale, applied);
        last_applied = applied;
      }
      if (st.completed) {
        if (st.error_code == interface::ExecStatus::kPathToleranceViolated)
          std::printf(
              "\n[traj] DIVERGENCE ABORT at t=%.2f s — a joint left the "
              "%.3f rad path tolerance. Arm holds; run stops.\n",
              now_s, path_tol);
        else
          std::printf("\n[traj] trajectory complete at t=%.2f s (code=%d).\n", now_s,
                      st.error_code);
        break;
      }
      std::this_thread::sleep_for(
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(period));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));  // let the arm settle on the goal
    g_stop.store(true, std::memory_order_release);
  });

  ResourceUsage usage_before = read_usage();  // main thread IS the RT loop thread
  ex.run(g_stop);
  ResourceUsage usage_after = read_usage();

  t.safe_shutdown();
  publisher.join();
  draining.store(false, std::memory_order_release);
  drain.join();

  const JointVec q_ref = mode.reference();
  std::cout << "\n==== trajectory run report ====\n";
  std::cout << introspect() << "\n";
  for (int i = 0; i < kNumJoints; ++i)
    std::printf("  j%d  goal=%+8.4f  final_ref=%+8.4f  residual=%+8.4f rad\n", i, target[i],
                q_ref[i], wrap_to_pi(target[i] - q_ref[i]));
  const auto& ch = sink.cycle_hist();
  const auto& mh = sink.compute_hist();
  std::cout << "cycle_ns   n=" << ch.count() << " p50=" << ch.percentile(0.50)
            << " p99=" << ch.percentile(0.99) << " p99.9=" << ch.percentile(0.999)
            << " max=" << ch.max() << "\n";
  std::cout << "compute_ns n=" << mh.count() << " p50=" << mh.percentile(0.50)
            << " p99=" << mh.percentile(0.99) << " max=" << mh.max() << "\n";
  std::cout << "dropped=" << ring.dropped()
            << "  majflt+=" << (usage_after.majflt - usage_before.majflt) << "\n";
  std::cout << "===============================\n";
  return 0;
}
