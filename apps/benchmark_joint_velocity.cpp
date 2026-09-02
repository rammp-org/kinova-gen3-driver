// benchmark_joint_velocity — runs JointVelocityMode through the RtExecutor at a
// fixed rate and reports timing. SIM by default (--sim); the real KortexTransport
// path is compiled only with KORTEX and never run unattended.
//
// --kind joint (default): drives the native set_velocity_target() path, which is
//   pass-through-then-limit and cheap by inspection.
// --kind twist: drives set_twist_target() with a slowly ROTATING unit twist, so
//   the Jacobian + two 6x6 LDLT decompositions + null-space posture solve runs
//   every cycle rather than short-circuiting on a stationary/zero target. This is
//   the only genuinely new per-cycle cost this mode introduces, and the delta
//   between --kind joint and --kind twist IS that cost.
//
//   ./benchmark_joint_velocity --sim --urdf ../models/gen3_7dof_2f85.urdf \
//       --rate 1000 --duration 5 --kind twist --csv /tmp/bench.csv

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
#include <Eigen/Dense>

#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/joint_velocity_mode.h"
#include "kinova_lowlevel/rt_executor.h"
#include "kinova_lowlevel/rt_system.h"
#include "kinova_lowlevel/sim_transport.h"
#include "kinova_lowlevel/telemetry.h"
#include "kinova_lowlevel/telemetry_consumers.h"
#include "kinova_lowlevel/transport.h"
#ifndef KINOVA_NO_KORTEX
#include "kinova_lowlevel/kortex_transport.h"
#endif

using namespace kinova;

namespace {
std::atomic<bool> g_stop{false};
void on_sigint(int) { g_stop.store(true); }
}  // namespace

int main(int argc, char** argv) {
  std::string ip;
  std::string urdf = "../models/gen3_7dof_2f85.urdf";
  std::string csv_path;
  std::string pacing_str = "sleepspin";
  std::string kind_str = "joint";
  bool use_sim = false;
  double rate_hz = 1000.0;
  int cpu = -1;
  int rt_priority = 80;
  double duration_s = 10.0;
  JointVelocityParams p;

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
    else if (a == "--rate") rate_hz = std::stod(next("--rate"));
    else if (a == "--cpu") cpu = std::stoi(next("--cpu"));
    else if (a == "--rt-priority") rt_priority = std::stoi(next("--rt-priority"));
    else if (a == "--duration") duration_s = std::stod(next("--duration"));
    else if (a == "--pacing") pacing_str = next("--pacing");
    else if (a == "--csv") csv_path = next("--csv");
    else if (a == "--kind") kind_str = next("--kind");
    else if (a == "--dls-damping") p.dls_damping = std::stod(next("--dls-damping"));
    else if (a == "--posture-gain") p.posture_gain = std::stod(next("--posture-gain"));
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

  const bool twist = (kind_str == "twist");
  if (!twist && kind_str != "joint") {
    std::cerr << "--kind must be joint|twist\n";
    std::exit(2);
  }

  std::cout << "[vel] urdf=" << urdf << " rate=" << rate_hz << "Hz pacing="
            << pacing_str << " cpu=" << cpu << " prio=" << rt_priority
            << " duration=" << duration_s << "s sim=" << (use_sim ? "yes" : "no")
            << " kind=" << kind_str << "\n";

  Dynamics dyn(urdf);

  // Build transport. SimTransport is the only path exercised here.
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
    // final drain
    while (ring.pop(s)) sink.consume(s);
  });

  t.connect();
  t.set_servoing_low_level();

  JointVelocityMode mode(dyn, p);

  RtExecutor ex(t, ring, {rate_hz, pacing, {rt_priority, cpu, true}});
  ex.request_mode(&mode);

  // Setpoint driver: publishes from a non-RT thread at ~200 Hz, well inside any
  // reasonable staleness window (the default cmd_timeout_s=0 disables the
  // watchdog here anyway). --kind joint holds a small constant joint-velocity
  // target (pass-through-then-limit, no per-cycle solve). --kind twist rotates a
  // unit-magnitude twist slowly in the XY plane so its components are never
  // simultaneously zero and the DLS solve + null-space projection runs on a
  // genuinely changing target every single cycle, not a cached/short-circuited
  // one.
  std::thread setpoint([&] {
    const auto t0 = std::chrono::steady_clock::now();
    while (!g_stop.load(std::memory_order_acquire)) {
      const double t_s =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      if (twist) {
        Vector6 V = Vector6::Zero();
        const double w = 0.5;  // rad/s rotation rate of the twist direction itself
        V[0] = 0.05 * std::cos(w * t_s);
        V[1] = 0.05 * std::sin(w * t_s);
        mode.set_twist_target(V);
      } else {
        JointVec qd = JointVec::Constant(0.05);
        mode.set_velocity_target(qd);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
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

  ResourceUsage usage_before = read_usage();  // NOTE: RUSAGE_THREAD; this is the
                                               // main thread, which IS the RT loop
                                               // thread since ex.run() runs here.
  ex.run(g_stop);  // blocks on the main (RT) thread until stop
  ResourceUsage usage_after = read_usage();

  t.safe_shutdown();

  if (watchdog.joinable()) watchdog.join();
  setpoint.join();
  draining.store(false, std::memory_order_release);
  drain.join();

  const auto& ch = sink.cycle_hist();
  const auto& mh = sink.compute_hist();
  std::cout << "\n==== joint velocity benchmark report (kind=" << kind_str << ") ====\n";
  std::cout << introspect() << "\n";
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
  std::cout << "====================================\n";
  return 0;
}
