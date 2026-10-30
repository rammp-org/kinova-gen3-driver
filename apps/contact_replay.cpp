// contact_replay — replays a recorded joint-state CSV through the contact monitor
// (observe mode) so thresholds can be tuned offline. Pure computation: no arm, no
// transport. See docs/guide/contact-detection.md.
//
//   contact_replay --urdf F --ee-frame E --csv in.csv --out r.csv
//                  [--gain 25] [--debounce 5 | --debounce-ms 5] [--thresholds a,b,c,d,e,f,g]
//                  [--tau-sign -1] [--impulse a,b,c,d,e,f,g] [--slow-tau 0.2]
//                  [--grasp BEGIN,END ...] [--grasp-com x,y,z]
#include "kinova_lowlevel/contact_replay.h"

#include <cstdio>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace kinova;

namespace {
void usage() {
  std::fprintf(
      stderr,
      "usage: contact_replay --urdf F --ee-frame E --csv in.csv --out r.csv\n"
      "       [--gain 25] [--debounce ROWS | --debounce-ms MS] [--thresholds a,b,c,d,e,f,g]\n"
      "       [--tau-sign -1] [--impulse a,b,c,d,e,f,g] [--slow-tau 0.2]\n"
      "       [--grasp BEGIN,END ...] [--grasp-com x,y,z]\n"
      "  --tau-sign -1  raw core v1.2.0 recording (default 1: command convention)\n"
      "  --impulse      enable the impulse trip with these thresholds [N·m]\n"
      "  --grasp        a grasp or release: tare from BEGIN s, weigh from END s, each\n"
      "                 window 0.3 s with the arm still; repeatable\n"
      "  --grasp-com    the held object's centre of mass, EE frame [m] (0,0,0.10)\n");
}

std::vector<double> parse_list(const std::string& flag, const std::string& s, size_t count) {
  std::vector<double> v;
  std::stringstream ss(s);
  std::string cell;
  while (std::getline(ss, cell, ',')) v.push_back(std::stod(cell));
  if (v.size() != count)
    throw std::runtime_error(flag + " needs exactly " + std::to_string(count) + " values");
  return v;
}

JointVec parse_thresholds(const std::string& flag, const std::string& s) {
  const std::vector<double> v = parse_list(flag, s, kNumJoints);
  return Eigen::Map<const JointVec>(v.data());
}
}  // namespace

int main(int argc, char** argv) {
  ReplayOptions opt;
  std::string csv, out;
  double tau_sign = 1.0, debounce_ms = 0.0;
  bool have_rows = false;
  try {
    for (int i = 1; i < argc; ++i) {
      const std::string a = argv[i];
      auto value = [&]() -> std::string {
        if (i + 1 >= argc) throw std::runtime_error(a + " needs a value");
        return argv[++i];
      };
      if (a == "--urdf")
        opt.urdf_path = value();
      else if (a == "--ee-frame")
        opt.ee_frame = value();
      else if (a == "--csv")
        csv = value();
      else if (a == "--out")
        out = value();
      else if (a == "--gain")
        opt.gain = std::stod(value());
      else if (a == "--debounce") {
        opt.debounce_cycles = std::stoi(value());
        have_rows = true;
      } else if (a == "--debounce-ms")
        debounce_ms = std::stod(value());
      else if (a == "--thresholds")
        opt.theta = parse_thresholds(a, value());
      else if (a == "--impulse") {
        opt.impulse = true;
        opt.impulse_theta = parse_thresholds(a, value());
      } else if (a == "--slow-tau")
        opt.slow_tau_s = std::stod(value());
      else if (a == "--grasp") {
        const std::vector<double> g = parse_list(a, value(), 2);
        opt.grasps.push_back({g[0], g[1]});
      } else if (a == "--grasp-com") {
        const std::vector<double> c = parse_list(a, value(), 3);
        opt.grasp_com = Eigen::Vector3d(c[0], c[1], c[2]);
      } else if (a == "--tau-sign")
        tau_sign = std::stod(value());
      else
        throw std::runtime_error("unknown argument " + a);
    }
    if (opt.urdf_path.empty() || opt.ee_frame.empty() || csv.empty() || out.empty())
      throw std::runtime_error("--urdf, --ee-frame, --csv and --out are required");

    if (have_rows && debounce_ms > 0.0)
      throw std::runtime_error("--debounce and --debounce-ms are mutually exclusive");
    const ReplayInput in = load_replay_csv(csv, tau_sign);
    if (debounce_ms > 0.0) opt.debounce_cycles = debounce_rows_from_ms(in.t, debounce_ms);
    const double dt_med = median_dt(in.t);
    const ReplayStats s = replay(in, opt);

    std::ofstream f(out);
    if (!f) throw std::runtime_error("cannot write " + out);
    f << "t,r1,r2,r3,r4,r5,r6,r7\n";
    f.precision(9);
    for (size_t k = 0; k < in.t.size(); ++k) {
      f << in.t[k];
      for (int j = 0; j < kNumJoints; ++j) f << ',' << s.r[k][j];
      f << '\n';
    }

    std::printf("%zu rows, gain %g, %zu moving samples after the %g s settle\n", in.t.size(),
                opt.gain, s.moving_samples, 5.0 / opt.gain);
    std::printf("debounce: %d rows = %.1f ms (median dt %.2f ms)\n", opt.debounce_cycles,
                opt.debounce_cycles * dt_med * 1000.0, dt_med * 1000.0);
    if (dt_med > 0.002)
      std::printf(
          "WARNING: median dt > 2 ms: the offline debounce cannot match the arm's 5 ms "
          "(1 kHz x 5 cycles); would_trips is not what the arm would count.\n");
    std::printf("joint  theta    rms    peak   first trip\n");
    for (int j = 0; j < kNumJoints; ++j) {
      std::printf("  %d   %6.2f  %6.2f  %6.2f   ", j + 1, opt.theta[j], s.rms[j], s.peak[j]);
      if (s.first_trip[j].valid)
        std::printf("t=%.3f s  r=%.2f%s\n", s.first_trip[j].t, s.first_trip[j].r,
                    s.first_trip[j].impulse ? "  (impulse)" : "");
      else
        std::printf("-\n");
    }
    std::printf("would-be trips: %zu\n", s.would_trips);
    for (size_t i = 0; i < s.trips.size(); ++i)
      std::printf("  t=%.3f s  joint %d%s\n", s.trips[i].t, s.trip_joints[i] + 1,
                  s.trips[i].impulse ? "  impulse" : "");
    static const char* const kOutcome[] = {"none",           "ok",          "not still at tare",
                                           "not still",      "unexplained", "out of range",
                                           "no load change", "aborted"};
    for (const ReplayWeigh& w : s.weighs) {
      const WeighResult& r = w.result;
      std::printf("weigh at t=%.3f s: %s, change %+.3f kg, fit %.2f N·m -> payload %.3f kg\n", w.t,
                  kOutcome[static_cast<int>(r.outcome)], r.delta_kg, r.fit_rms, r.payload.mass);
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "contact_replay: %s\n\n", e.what());
    usage();
    return 2;
  }
  return 0;
}
