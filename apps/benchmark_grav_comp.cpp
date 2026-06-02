// benchmark_grav_comp — runs the gravity-compensation control mode through the
// RtExecutor at a fixed rate and reports timing statistics.
//
// SIM-ONLY in CI/dev: pass --sim to drive the SimTransport fake robot. The real
// KortexTransport path is constructed only when --ip is given and --sim is not;
// it is never exercised here while the robot is powered off.
//
//   ./benchmark_grav_comp --sim --urdf ../models/gen3_7dof.urdf --rate 1000
//       --duration 5 --csv /tmp/bench.csv

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/gravity_comp_mode.h"
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
  std::string urdf = "../models/gen3_7dof.urdf";
  std::string csv_path;
  std::string pacing_str = "sleepspin";
  bool use_sim = false;
  double rate_hz = 1000.0;
  int cpu = -1;
  int rt_priority = 80;
  double duration_s = 10.0;
  double scale = 1.0;
  double damping = 0.0;
  double torque_limit = 39.0;

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
    else if (a == "--pacing") pacing_str = next("--pacing");
    else if (a == "--duration") duration_s = std::stod(next("--duration"));
    else if (a == "--csv") csv_path = next("--csv");
    else if (a == "--scale") scale = std::stod(next("--scale"));
    else if (a == "--damping") damping = std::stod(next("--damping"));
    else if (a == "--torque-limit") torque_limit = std::stod(next("--torque-limit"));
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

  std::cout << "[bench] urdf=" << urdf << " rate=" << rate_hz << "Hz pacing="
            << pacing_str << " cpu=" << cpu << " prio=" << rt_priority
            << " duration=" << duration_s << "s sim=" << (use_sim ? "yes" : "no")
            << "\n";

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

  GravityCompTorqueMode mode(dyn, {scale, damping, torque_limit});

  RtExecutor ex(t, ring, {rate_hz, pacing, {rt_priority, cpu, true}});
  ex.request_mode(&mode);

  std::signal(SIGINT, on_sigint);

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
  draining.store(false, std::memory_order_release);
  drain.join();

  const auto& ch = sink.cycle_hist();
  const auto& mh = sink.compute_hist();
  std::cout << "\n==== benchmark report ====\n";
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
  std::cout << "==========================\n";
  return 0;
}
