// benchmark_cartesian_impedance — runs CartesianImpedanceMode through the
// RtExecutor at a fixed rate and reports timing. SIM by default (--sim); the
// real KortexTransport path is compiled only with KORTEX and never run
// unattended. Holds the pose captured at entry (compliant hold: push the arm
// and it springs back). --dry-run is READ-ONLY: prints fk(q), Jacobian
// condition number, and the would-be task wrench WITHOUT commanding torque.
//
//   ./benchmark_cartesian_impedance --sim --urdf ../models/gen3_7dof_2f85.urdf
//       --rate 1000 --duration 5 --csv /tmp/bench.csv
//
// --dry-run: READ-ONLY, never enters low-level servoing, never commands torque.
// Prints fk(q) and Jacobian condition number so dynamics can be validated
// against the real arm BEFORE any torque is commanded.
//   ./benchmark_cartesian_impedance --ip 192.168.1.10 --dry-run
//       --urdf ../models/gen3_7dof_2f85.urdf --duration 0

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

#include "kinova_lowlevel/cartesian_impedance_mode.h"
#include "kinova_lowlevel/dynamics.h"
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
  bool use_sim = false;
  bool dry_run = false;
  double rate_hz = 1000.0;
  int cpu = -1;
  int rt_priority = 80;
  double duration_s = 10.0;
  CartesianImpedanceParams p;

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
    else if (a == "--duration") duration_s = std::stod(next("--duration"));
    else if (a == "--pacing") pacing_str = next("--pacing");
    else if (a == "--csv") csv_path = next("--csv");
    else if (a == "--kx-trans") p.Kx.head<3>().setConstant(std::stod(next("--kx-trans")));
    else if (a == "--kx-rot") p.Kx.tail<3>().setConstant(std::stod(next("--kx-rot")));
    else if (a == "--dx-trans") p.Dx.head<3>().setConstant(std::stod(next("--dx-trans")));
    else if (a == "--dx-rot") p.Dx.tail<3>().setConstant(std::stod(next("--dx-rot")));
    else if (a == "--nullspace-kp") p.nullspace_kp = std::stod(next("--nullspace-kp"));
    else if (a == "--nullspace-kd") p.nullspace_kd = std::stod(next("--nullspace-kd"));
    else if (a == "--gain-ramp-s") p.gain_ramp_s = std::stod(next("--gain-ramp-s"));
    else if (a == "--pinv-damping") p.pinv_damping = std::stod(next("--pinv-damping"));
    else if (a == "--torque-limit") p.torque_limit = std::stod(next("--torque-limit"));
    else if (a == "--no-nullspace") p.nullspace_on = false;
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

  std::cout << "[imp] urdf=" << urdf << " rate=" << rate_hz << "Hz pacing="
            << pacing_str << " cpu=" << cpu << " prio=" << rt_priority
            << " duration=" << duration_s << "s sim=" << (use_sim ? "yes" : "no")
            << " dry_run=" << (dry_run ? "yes" : "no") << "\n";

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

  // --- dry-run: READ-ONLY, never commands torque --------------------------------
  // Connects and reads feedback only (never enters LOW_LEVEL_SERVOING, never
  // commands torque). Prints fk(q) and Jacobian condition number so dynamics
  // can be validated against the real arm before trusting any torque.
  if (dry_run) {
    t.connect();
    std::cout << "[dry-run] READ-ONLY — NO torque commanded, arm stays under "
                 "its own control.\n"
                 "          Move it to a few poses (pendant/web app) to "
                 "validate FK and Jacobian.\n";
    JointFeedback fb;
    Jacobian6 J;
    const auto start = std::chrono::steady_clock::now();
    auto last_print = start - std::chrono::seconds(1);
    while (!g_stop.load(std::memory_order_acquire)) {
      t.receive(fb);
      Pose x = dyn.fk(fb.q);
      dyn.jacobian(fb.q, J);
      Eigen::JacobiSVD<Eigen::Matrix<double, 6, kNumJoints>> svd(J);
      double cond = svd.singularValues()(0) /
                    std::max(1e-12, svd.singularValues()(svd.singularValues().size() - 1));
      const auto now = std::chrono::steady_clock::now();
      if (now - last_print >= std::chrono::milliseconds(500)) {
        std::printf(
            "pos[m]=(%+.3f %+.3f %+.3f)  quat=(%+.3f %+.3f %+.3f %+.3f)  "
            "Jcond=%.1f\n",
            x.p.x(), x.p.y(), x.p.z(), x.R.w(), x.R.x(), x.R.y(), x.R.z(),
            cond);
        last_print = now;
      }
      if (duration_s > 0.0 &&
          std::chrono::duration<double>(now - start).count() >= duration_s) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));  // ~50 Hz read
    }
    t.safe_shutdown();  // no-op revert (never went low-level); just closes session
    std::cout << "[dry-run] done — no torque was ever commanded.\n";
    return 0;
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
    // final drain
    while (ring.pop(s)) sink.consume(s);
  });

  t.connect();
  t.set_servoing_low_level();

  CartesianImpedanceMode mode(dyn, p);

  RtExecutor ex(t, ring, {rate_hz, pacing, {rt_priority, cpu, true}});
  ex.request_mode(&mode);

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
  std::cout << "\n==== impedance benchmark report ====\n";
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
