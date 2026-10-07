// velocity_hold — standalone KORTEX-API-only repro for issue #34 (joint 2 creeps
// under gravity at a zero low-level VELOCITY command). Uses NOTHING from this
// driver, so it can be sent to Kinova as-is.
//
// Build (on the arm computer, next to the KORTEX SDK):
//   K=~/kortex_api_2.8.0_aarch64
//   g++ -std=c++17 -O2 -D_OS_UNIX velocity_hold.cpp \
//     -I$K/include -I$K/include/client -I$K/include/client_stubs \
//     -I$K/include/common -I$K/include/messages -I$K/include/google \
//     $K/lib/release/libKortexApiCpp.a -lpthread -ldl -o velocity_hold
//
// Modes:
//   ./velocity_hold --ip IP                      READ-ONLY: firmware, active control
//                                                loop, loop parameters. Nothing moves.
//   ./velocity_hold --ip IP --variant echo       zero velocity on ALL joints, position
//                                                field = latest measured position
//                                                (Kinova example 200/01 pattern)
//   ./velocity_hold --ip IP --variant latch      zero velocity, position field = position
//                                                at entry, held constant
//   options: --duration S (default 5)  --max-drift-deg D (default 4; aborts past it)
//
// Aborting, finishing, or Ctrl-C all revert every actuator to POSITION mode at
// its measured position, then SINGLE_LEVEL_SERVOING.

#include <ActuatorConfigClientRpc.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <DeviceConfigClientRpc.h>
#include <RouterClient.h>
#include <SessionManager.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

namespace k = Kinova::Api;
using Clock = std::chrono::steady_clock;

static std::atomic<bool> g_stop{false};

static void print_loop(k::ActuatorConfig::ActuatorConfigClient& cfg, int id,
                       k::ActuatorConfig::ControlLoopSelection sel, const char* name) {
  k::ActuatorConfig::LoopSelection ls;
  ls.set_loop_selection(sel);
  try {
    auto p = cfg.GetControlLoopParameters(ls, id);
    std::printf("    %-16s %s\n", name, p.ShortDebugString().c_str());
    // An integrator is a root of the denominator at z = 1. Try both coefficient
    // arrays, since which one is the denominator is not documented.
    double sa = 0, sb = 0;
    for (float v : p.kaz()) sa += v;
    for (float v : p.kbz()) sb += v;
    std::printf("    %-16s sum(kAz)=%.6g  sum(kBz)=%.6g  (~0 => pole/zero at z=1)\n", "", sa, sb);
  } catch (const std::exception& e) {
    std::printf("    %-16s <error: %s>\n", name, e.what());
  }
}

static void print_active_loops(k::ActuatorConfig::ActuatorConfigClient& cfg, int n) {
  for (int id = 1; id <= n; ++id) {
    auto cl = cfg.GetActivatedControlLoop(id);
    auto cm = cfg.GetControlMode(id);
    std::printf("  actuator %d: control_mode=%s activated_loop_bitmask=0x%x\n", id,
                k::ActuatorConfig::ControlMode_Name(cm.control_mode()).c_str(),
                cl.control_loop());
  }
}

int main(int argc, char** argv) {
  std::string ip = "192.168.1.10", variant;
  double duration = 5.0, max_drift_deg = 4.0;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&] { return std::string(i + 1 < argc ? argv[++i] : ""); };
    if (a == "--ip") ip = next();
    else if (a == "--variant") variant = next();
    else if (a == "--duration" || a == "--max-drift-deg") {
      // stod on a flag given no value throws uncaught; die with usage instead.
      try {
        (a == "--duration" ? duration : max_drift_deg) = std::stod(next());
      } catch (const std::exception&) {
        std::fprintf(stderr, "%s needs a numeric value\n", a.c_str());
        return 2;
      }
    }
    else { std::fprintf(stderr, "unknown arg %s\n", a.c_str()); return 2; }
  }
  if (!variant.empty() && variant != "echo" && variant != "latch") {
    std::fprintf(stderr, "--variant must be echo or latch\n");
    return 2;
  }
  std::signal(SIGINT, [](int) { g_stop = true; });

  auto on_err = [](k::KError e) { std::fprintf(stderr, "router error: %s\n", e.toString().c_str()); };
  k::TransportClientTcp tcp;
  k::TransportClientUdp udp;
  tcp.connect(ip, 10000);
  udp.connect(ip, 10001);
  k::RouterClient tcp_router(&tcp, on_err), udp_router(&udp, on_err);
  auto si = k::Session::CreateSessionInfo();
  si.set_username("admin");
  si.set_password("admin");
  si.set_session_inactivity_timeout(60000);
  si.set_connection_inactivity_timeout(2000);
  k::SessionManager tcp_sess(&tcp_router), udp_sess(&udp_router);
  tcp_sess.CreateSession(si);
  udp_sess.CreateSession(si);
  k::Base::BaseClient base(&tcp_router);
  k::BaseCyclic::BaseCyclicClient cyclic(&udp_router);
  k::ActuatorConfig::ActuatorConfigClient cfg(&tcp_router);
  k::DeviceConfig::DeviceConfigClient dev(&tcp_router);

  const int n = base.GetActuatorCount().count();
  std::printf("firmware bundle: %s\n", base.GetFirmwareBundleVersions().main_bundle_version().c_str());
  for (int id = 1; id <= n; ++id) {
    std::printf("actuator %d firmware: %s\n", id, dev.GetFirmwareVersion(id).ShortDebugString().c_str());
  }

  if (variant.empty()) {
    std::printf("\n[read-only] active loops:\n");
    print_active_loops(cfg, n);
    std::printf("\n[read-only] control loop parameters:\n");
    for (int id = 1; id <= n; ++id) {
      std::printf("  actuator %d\n", id);
      print_loop(cfg, id, k::ActuatorConfig::JOINT_POSITION, "JOINT_POSITION");
      print_loop(cfg, id, k::ActuatorConfig::JOINT_VELOCITY, "JOINT_VELOCITY");
      print_loop(cfg, id, k::ActuatorConfig::MOTOR_VELOCITY, "MOTOR_VELOCITY");
      print_loop(cfg, id, k::ActuatorConfig::JOINT_TORQUE, "JOINT_TORQUE");
    }
    tcp_sess.CloseSession();
    udp_sess.CloseSession();
    tcp_router.SetActivationStatus(false);
    udp_router.SetActivationStatus(false);
    tcp.disconnect();
    udp.disconnect();
    return 0;
  }

  try { base.ClearFaults(); } catch (...) {}
  k::Base::ServoingModeInformation sm;
  sm.set_servoing_mode(k::Base::ServoingMode::LOW_LEVEL_SERVOING);
  base.SetServoingMode(sm);

  k::BaseCyclic::Command cmd;
  auto fb = cyclic.RefreshFeedback();
  std::vector<float> q0(n);
  for (int i = 0; i < n; ++i) {
    q0[i] = fb.actuators(i).position();
    cmd.add_actuators()->set_position(q0[i]);
  }
  fb = cyclic.Refresh(cmd);

  // Switch every actuator to VELOCITY, pumping the cyclic channel between calls.
  k::ActuatorConfig::ControlModeInformation cmi;
  cmi.set_control_mode(k::ActuatorConfig::ControlMode::VELOCITY);
  for (int id = 1; id <= n; ++id) {
    cfg.SetControlMode(cmi, id);
    fb = cyclic.Refresh(cmd);
  }
  std::printf("\nactive loops in velocity mode:\n");
  print_active_loops(cfg, n);

  for (int i = 0; i < n; ++i) q0[i] = fb.actuators(i).position();
  std::vector<double> tau_sum(n, 0.0), qd_sum(n, 0.0);
  long ticks = 0;
  bool aborted = false;
  uint32_t frame = 0;
  auto t0 = Clock::now(), next = t0;
  std::vector<float> dq(n, 0.0f);
  while (!g_stop && Clock::now() - t0 < std::chrono::duration<double>(duration)) {
    ++frame;
    cmd.set_frame_id(frame);
    for (int i = 0; i < n; ++i) {
      auto* a = cmd.mutable_actuators(i);
      a->set_command_id(frame);
      a->set_velocity(0.0f);
      a->set_position(variant == "echo" ? fb.actuators(i).position() : q0[i]);
    }
    fb = cyclic.Refresh(cmd, 0);
    ++ticks;
    for (int i = 0; i < n; ++i) {
      float d = fb.actuators(i).position() - q0[i];
      d = std::remainder(d, 360.0f);  // continuous joints report [0, 360)
      dq[i] = d;
      tau_sum[i] += fb.actuators(i).torque();
      qd_sum[i] += fb.actuators(i).velocity();
      if (std::fabs(d) > max_drift_deg) aborted = true;
    }
    if (aborted) break;
    next += std::chrono::milliseconds(1);
    std::this_thread::sleep_until(next);
  }
  double elapsed = std::chrono::duration<double>(Clock::now() - t0).count();

  // Revert: position setpoint = measured, POSITION mode, then single-level.
  for (int i = 0; i < n; ++i) cmd.mutable_actuators(i)->set_position(fb.actuators(i).position());
  cmi.set_control_mode(k::ActuatorConfig::ControlMode::POSITION);
  for (int id = 1; id <= n; ++id) {
    try { cfg.SetControlMode(cmi, id); } catch (...) {}
    try { fb = cyclic.Refresh(cmd); } catch (...) {}
  }
  sm.set_servoing_mode(k::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
  try { base.SetServoingMode(sm); } catch (...) {}
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Zero-iteration exit (Ctrl-C before the first Refresh, duration <= 0)
  // would otherwise print NaN/inf in a report meant to go to Kinova as-is.
  if (ticks == 0 || elapsed <= 0.0) {
    std::printf("\nvariant=%s  elapsed=%.2fs  ticks=%ld -- no cycles ran, no rates to report\n",
                variant.c_str(), elapsed, ticks);
  } else {
    std::printf("\nvariant=%s  elapsed=%.2fs  ticks=%ld (%.0f Hz)%s\n", variant.c_str(), elapsed,
                ticks, ticks / elapsed, aborted ? "  ABORTED: drift limit" : "");
    std::printf("joint  drift_deg  drift_rate_deg_s  mean_qd_deg_s  mean_torque_Nm\n");
    for (int i = 0; i < n; ++i) {
      std::printf("%5d  %+9.3f  %+16.4f  %+13.4f  %+14.3f\n", i + 1, dq[i], dq[i] / elapsed,
                  qd_sum[i] / ticks, tau_sum[i] / ticks);
    }
  }

  tcp_sess.CloseSession();
  udp_sess.CloseSession();
  tcp_router.SetActivationStatus(false);
  udp_router.SetActivationStatus(false);
  tcp.disconnect();
  udp.disconnect();
  return aborted ? 1 : 0;
}
