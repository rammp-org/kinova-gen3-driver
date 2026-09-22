// velocity_hold_check — TEMPORARY attended harness for JointVelocityMode on the
// real arm, after #34 moved the mode onto an integrated position reference.
//
// Three checks, run ONE AT A TIME with --phase, each ending in a verdict:
//
//   hold   streams a ZERO velocity for --hold-s and measures per-joint drift.
//          Before the fix joint 2 crept ~0.038 rad/s (+0.227 rad in 6 s).
//          HOLDS if every joint moved less than 5 mrad.
//   stale  jogs ONE joint up to --qd and then, mid-motion, STOPS publishing with
//          the watchdog armed at --timeout. The mode must zero the velocity,
//          freeze at the MEASURED position and latch. FREEZES if the joint
//          travels less than 20 mrad after the last setpoint.
//   track  jogs ONE joint out at +qd and back at -qd (trapezoids), comparing
//          measured qd against the commanded schedule on the flat parts. The
//          0.1 rad leash must stay dormant while tracking: TRACKS if the mean
//          measured qd is above 90% of the command and the estimated lead (the
//          integral of the command minus the travel) stays under 0.08 rad.
//
// Uses the library's JointVelocityMode itself: nothing here is a control law.
// Setpoints are published from a non-RT thread at --cmd-rate, exactly as the
// streaming tier does; measurements come through FeedbackTap, never from the
// mode's RT-owned state.
//
//   ./velocity_hold_check --ip 192.168.1.10 --dry-run              # READ-ONLY
//   ./velocity_hold_check --ip 192.168.1.10 --phase hold --hold-s 10
//   ./velocity_hold_check --ip 192.168.1.10 --phase stale
//   ./velocity_hold_check --ip 192.168.1.10 --phase track --qd 0.3
//
// Defaults are deliberately small: the wrist joint, 0.05 rad/s, 2 s jogs, ramped.
// --qd is refused above 1.0 rad/s and --hold-s above 120 s. Under --sim this
// exercises the plumbing only; the sim transport has no plant.

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
#include "kinova_lowlevel/joint_velocity_mode.h"
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

struct Sample {
  double t;
  JointVec q, qd;
  double cmd;  // the scalar commanded on the jogged joint at this tick
};

// Trapezoid: ramp up over ramp_s, flat, ramp down over ramp_s. Never a step.
double trapezoid(double t, double qd, double dur, double ramp) {
  if (t < 0.0 || t >= dur) return 0.0;
  if (ramp <= 0.0) return qd;
  if (t < ramp) return qd * t / ramp;
  if (t < dur - ramp) return qd;
  return qd * (dur - t) / ramp;
}

// Ramp up over ramp_s, then flat FOREVER: the stale phase cuts the stream while
// this is still commanding motion.
double ramp_then_flat(double t, double qd, double ramp) {
  if (ramp <= 0.0 || t >= ramp) return qd;
  return qd * t / ramp;
}

void print_vec(const char* label, const JointVec& v) {
  std::printf("%s", label);
  for (int i = 0; i < kNumJoints; ++i) std::printf("%+8.4f ", v[i]);
  std::printf("\n");
}
}  // namespace

int main(int argc, char** argv) {
  std::setvbuf(stdout, nullptr, _IOLBF, 0);  // keep the record if the process aborts
  std::string ip, urdf = "../models/gen3_7dof_2f85.urdf", pacing_str = "sleepspin";
  std::string phase;
  bool use_sim = false, dry_run = false;
  int joint = kNumJoints - 1;  // wrist: lightest, least able to hurt anything
  double qd = 0.05;            // rad/s
  double hold_s = 10.0, jog_s = 2.0, ramp_s = 0.5, timeout_s = 0.2;
  double cmd_rate_hz = 100.0, rate_hz = 1000.0;
  int cpu = -1, rt_priority = 80;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto val = [&]() -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "missing value for " << a << "\n";
        std::exit(2);
      }
      return argv[++i];
    };
    if (a == "--ip")
      ip = val();
    else if (a == "--urdf")
      urdf = val();
    else if (a == "--sim")
      use_sim = true;
    else if (a == "--dry-run")
      dry_run = true;
    else if (a == "--phase")
      phase = val();
    else if (a == "--joint")
      joint = std::stoi(val());
    else if (a == "--qd")
      qd = std::stod(val());
    else if (a == "--hold-s")
      hold_s = std::stod(val());
    else if (a == "--jog-s")
      jog_s = std::stod(val());
    else if (a == "--ramp")
      ramp_s = std::stod(val());
    else if (a == "--timeout")
      timeout_s = std::stod(val());
    else if (a == "--cmd-rate")
      cmd_rate_hz = std::stod(val());
    else if (a == "--rate")
      rate_hz = std::stod(val());
    else if (a == "--cpu")
      cpu = std::stoi(val());
    else if (a == "--rt-priority")
      rt_priority = std::stoi(val());
    else if (a == "--pacing")
      pacing_str = val();
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
  if (!dry_run && phase != "hold" && phase != "stale" && phase != "track") {
    std::cerr << "--phase must be hold|stale|track (or use --dry-run)\n";
    return 2;
  }
  if (joint < 0 || joint >= kNumJoints) {
    std::cerr << "--joint must be 0.." << (kNumJoints - 1) << "\n";
    return 2;
  }
  // Guard rails: this is a check, not a motion tool. Refuse to be turned into one.
  if (std::abs(qd) > 1.0) {
    std::cerr << "--qd " << qd << " rad/s is too fast for this harness (cap 1.0).\n";
    return 2;
  }
  if (hold_s <= 0.0 || hold_s > 120.0) {
    std::cerr << "--hold-s must be in (0, 120].\n";
    return 2;
  }
  if (jog_s <= 0.0 || jog_s > 5.0) {
    std::cerr << "--jog-s must be in (0, 5].\n";
    return 2;
  }
  if (ramp_s < 0.0 || ramp_s * 2.0 > jog_s) {
    std::cerr << "--ramp " << ramp_s << "s twice over exceeds --jog-s " << jog_s
              << "s; there would be no flat part to measure.\n";
    return 2;
  }
  if (timeout_s <= 0.0 || timeout_s > 1.0) {
    std::cerr << "--timeout must be in (0, 1] s.\n";
    return 2;
  }
  const double jog_travel = std::abs(qd) * (jog_s - ramp_s);  // trapezoid area
  if (jog_travel > 1.5) {
    std::cerr << "a jog of " << jog_travel << " rad is too far for this harness (cap 1.5).\n";
    return 2;
  }
  if (cmd_rate_hz <= 0.0 || 1.0 / cmd_rate_hz >= timeout_s) {
    std::cerr << "--cmd-rate " << cmd_rate_hz << " Hz cannot keep a " << timeout_s
              << " s watchdog fresh.\n";
    return 2;
  }

  Dynamics dyn(urdf);
  JointVec v_max;
  dyn.velocity_limits(v_max);

  std::printf(
      "[vhold] phase=%s joint=j%d qd=%+.3f rad/s (URDF cap %.3f) hold=%.1fs jog=%.1fs "
      "ramp=%.2fs timeout=%.2fs cmd_rate=%.0fHz rate=%.0fHz sim=%s dry_run=%s\n",
      dry_run ? "-" : phase.c_str(), joint, qd, v_max[joint], hold_s, jog_s, ramp_s, timeout_s,
      cmd_rate_hz, rate_hz, use_sim ? "yes" : "no", dry_run ? "yes" : "no");

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
  Transport& raw = *transport;
  std::signal(SIGINT, on_sigint);

  // --- dry-run: READ-ONLY. Never enters low-level servoing. --------------------
  if (dry_run) {
    raw.connect();
    JointFeedback fb;
    raw.receive(fb);
    std::printf("\n[dry-run] READ-ONLY: nothing is commanded.\n");
    print_vec("[dry-run] current q:  ", fb.q);
    print_vec("[dry-run] current qd: ", fb.qd);
    std::printf(
        "[dry-run] every phase puts ALL actuators in POSITION mode under "
        "JointVelocityMode. hold streams zeros; stale/track jog j%d by up to "
        "%.3f rad.\n",
        joint, jog_travel);
    raw.safe_shutdown();
    return 0;
  }

  // FeedbackTap snapshots the arm's state at the RT read point, so this thread
  // never reads the mode's RT-owned state.
  Seqlock<JointFeedback> snap;
  FeedbackTap tap(raw, snap);
  tap.connect();
  JointFeedback entry;
  tap.receive(entry);
  print_vec("\n[vhold] entry q: ", entry.q);
  std::printf("[vhold] starting in 3s: e-stop in reach. Ctrl-C aborts.\n");
  std::this_thread::sleep_for(std::chrono::seconds(3));
  if (g_stop.load()) {
    tap.safe_shutdown();
    return 1;
  }

  tap.set_servoing_low_level();

  JointVelocityMode mode(dyn);
  mode.set_command_timeout(timeout_s);
  SampleRing ring(8192);
  RtExecutor ex(tap, ring, {rate_hz, pacing, {rt_priority, cpu, true, true}});
  ex.request_mode(&mode);

  std::atomic<bool> stop{false};
  std::thread rt([&] { ex.run(stop); });

  // The non-RT command loop: for `dur` seconds, publish target(t) on the jogged
  // joint (unless silent) at --cmd-rate and sample the tap.
  const auto period = std::chrono::duration<double>(1.0 / cmd_rate_hz);
  auto loop = [&](double dur, bool silent, auto target, std::vector<Sample>& out) {
    const auto t0 = std::chrono::steady_clock::now();
    double t = 0.0;
    while (!g_stop.load() && t < dur) {
      const double c = target(t);
      if (!silent) {
        JointVec v = JointVec::Zero();
        v[joint] = c;
        mode.set_velocity_target(v);
      }
      JointFeedback fb;
      if (snap.load(fb)) out.push_back({t, fb.q, fb.qd, silent ? 0.0 : c});
      std::this_thread::sleep_for(period);
      t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    }
  };
  auto zero = [](double) { return 0.0; };
  auto wrap_delta = [](double a, double b) { return wrap_to_pi(a - b); };

  const char* verdict = "ABORTED: Ctrl-C";
  if (phase == "hold") {
    std::vector<Sample> s;
    loop(hold_s, false, zero, s);
    if (s.size() > 2) {
      const JointVec q0 = s.front().q;
      JointVec drift, peak = JointVec::Zero();
      for (const auto& x : s)
        for (int i = 0; i < kNumJoints; ++i)
          peak[i] = std::max(peak[i], std::abs(wrap_delta(x.q[i], q0[i])));
      for (int i = 0; i < kNumJoints; ++i) drift[i] = wrap_delta(s.back().q[i], q0[i]);
      print_vec("[vhold] drift over the hold (rad):     ", drift);
      print_vec("[vhold] peak |excursion| (rad):        ", peak);
      std::printf("[vhold] j1 (the joint that crept) rate: %+.5f rad/s over %.1fs\n",
                  drift[1] / s.back().t, s.back().t);
      verdict = peak.maxCoeff() < 0.005
                    ? "HOLDS: every joint stayed within 5 mrad"
                    : "CREEPS: a joint moved more than 5 mrad; read the numbers";
      if (g_stop.load()) verdict = "ABORTED: Ctrl-C";
    }
  } else if (phase == "stale") {
    std::vector<Sample> jog, silent;
    loop(jog_s, false, [&](double t) { return ramp_then_flat(t, qd, ramp_s); }, jog);
    // Stop publishing mid-motion. The watchdog fires after --timeout; the arm is
    // expected to stop where it IS, not finish a leash's worth of travel.
    loop(timeout_s + 1.0, true, zero, silent);
    if (!jog.empty() && !silent.empty()) {
      const double q_entry = jog.front().q[joint], q_stop = jog.back().q[joint];
      const double q_after = silent.back().q[joint];
      const double moved_jog = wrap_delta(q_stop, q_entry);
      const double after = wrap_delta(q_after, q_stop);
      double qd_at_stop = jog.back().qd[joint];
      std::printf("[vhold] jogged j%d %+.4f rad, measured qd at the cut %+.4f rad/s\n", joint,
                  moved_jog, qd_at_stop);
      std::printf("[vhold] travel AFTER the last setpoint: %+.4f rad (%.1f mrad)\n", after,
                  std::abs(after) * 1e3);
      verdict = std::abs(after) < 0.02
                    ? "FREEZES: stopped within 20 mrad of where the stream died"
                    : "COASTS: kept going after the stream died; read the numbers";
      if (std::abs(moved_jog) < 0.2 * jog_travel)
        verdict = "INCONCLUSIVE: the jog itself barely moved; the arm was not tracking";
      if (g_stop.load()) verdict = "ABORTED: Ctrl-C";
    }
  } else {  // track
    std::vector<Sample> out, back, settle;
    loop(jog_s, false, [&](double t) { return trapezoid(t, qd, jog_s, ramp_s); }, out);
    loop(jog_s, false, [&](double t) { return trapezoid(t, -qd, jog_s, ramp_s); }, back);
    loop(0.5, false, zero, settle);
    if (out.size() > 2 && back.size() > 2 && !settle.empty()) {
      // Flat-part tracking: mean measured qd against the command.
      auto flat_mean = [&](const std::vector<Sample>& s) {
        double sum = 0.0;
        int n = 0;
        for (const auto& x : s)
          if (x.t >= ramp_s + 0.1 && x.t < jog_s - ramp_s) {
            sum += x.qd[joint];
            ++n;
          }
        return n ? sum / n : 0.0;
      };
      // Lead estimate: integral of the command minus the travel, at the end of
      // the flat part. If the leash bites this saturates near 0.1 rad.
      auto lead_at_flat_end = [&](const std::vector<Sample>& s) {
        double integral = 0.0, lead = 0.0;
        for (size_t i = 1; i < s.size(); ++i) {
          integral += 0.5 * (s[i].cmd + s[i - 1].cmd) * (s[i].t - s[i - 1].t);
          if (s[i].t < jog_s - ramp_s) lead = integral - wrap_delta(s[i].q[joint], s[0].q[joint]);
        }
        return lead;
      };
      const double m_out = flat_mean(out), m_back = flat_mean(back);
      const double lead_out = lead_at_flat_end(out), lead_back = lead_at_flat_end(back);
      const double moved_out = wrap_delta(out.back().q[joint], out.front().q[joint]);
      const double net = wrap_delta(settle.back().q[joint], out.front().q[joint]);
      std::printf(
          "[vhold] commanded flat qd %+.4f: measured mean %+.4f out (%.0f%%), %+.4f back "
          "(%.0f%%)\n",
          qd, m_out, 100.0 * m_out / qd, m_back, 100.0 * m_back / -qd);
      std::printf(
          "[vhold] estimated lead at end of flat: %.4f rad out, %.4f rad back (leash 0.1)\n",
          std::abs(lead_out), std::abs(lead_back));
      std::printf(
          "[vhold] j%d travelled %+.4f rad out (expected %+.4f), net after return %+.4f rad\n",
          joint, moved_out, jog_travel * (qd < 0 ? -1 : 1), net);
      const double ratio = std::min(m_out / qd, m_back / -qd);
      const double lead = std::max(std::abs(lead_out), std::abs(lead_back));
      if (ratio > 0.9 && lead < 0.08)
        verdict = "TRACKS: measured qd follows the command and the leash stayed dormant";
      else if (lead >= 0.08)
        verdict = "LEASHED: the reference ran up against the 0.1 rad leash while tracking";
      else
        verdict = "LAGS: measured qd fell short of the command without the leash biting";
      if (g_stop.load()) verdict = "ABORTED: Ctrl-C";
    }
  }

  // Leave the mode holding (zero velocity), then tear down to a position hold.
  mode.set_velocity_target(JointVec::Zero());
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  stop = true;
  rt.join();
  tap.safe_shutdown();

  JointFeedback after;
  if (snap.load(after)) print_vec("[vhold] exit q:  ", after.q);
  std::printf("\n[vhold] VERDICT (%s): %s\n", phase.c_str(), verdict);
  if (use_sim)
    std::printf("[vhold] (--sim has no plant; nothing above is a result about the arm.)\n");
  return 0;
}
