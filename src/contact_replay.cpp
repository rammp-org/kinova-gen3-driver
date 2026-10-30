#include "kinova_lowlevel/contact_replay.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>

#include "kinova_lowlevel/contact_monitor.h"
#include "kinova_lowlevel/dynamics.h"

namespace kinova {

namespace {
std::vector<std::string> split(const std::string& line) {
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string cell;
  while (std::getline(ss, cell, ',')) out.push_back(cell);
  return out;
}
}  // namespace

ReplayInput load_replay_csv(const std::string& path, double tau_sign) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("cannot open " + path);
  std::string line;
  if (!std::getline(f, line)) throw std::runtime_error(path + ": empty file");
  std::map<std::string, size_t> col;
  const std::vector<std::string> header = split(line);
  for (size_t i = 0; i < header.size(); ++i) col[header[i]] = i;

  auto need = [&](const std::string& name) {
    const auto it = col.find(name);
    if (it == col.end()) throw std::runtime_error(path + ": missing column " + name);
    return it->second;
  };
  const size_t ct = need("t");
  size_t cq[kNumJoints], cqd[kNumJoints], ctau[kNumJoints];
  for (int j = 0; j < kNumJoints; ++j) {
    cq[j] = need("q" + std::to_string(j + 1));
    cqd[j] = need("qd" + std::to_string(j + 1));
    ctau[j] = need("tau" + std::to_string(j + 1));
  }

  ReplayInput in;
  for (size_t row = 2; std::getline(f, line); ++row) {
    if (line.empty()) continue;
    const std::vector<std::string> c = split(line);
    if (c.size() < header.size())
      throw std::runtime_error(path + ":" + std::to_string(row) + ": expected " +
                               std::to_string(header.size()) + " columns, got " +
                               std::to_string(c.size()));
    // The whole field must be a number (surrounding whitespace, a CRLF '\r', is fine):
    // plain strtod reads "abc" as 0 and "1.5x" as 1.5.
    auto num = [&](size_t i) {
      const char* s = c[i].c_str();
      char* end = nullptr;
      const double v = std::strtod(s, &end);
      while (end != s && std::isspace(static_cast<unsigned char>(*end))) ++end;
      if (end == s || *end != '\0')
        throw std::runtime_error(path + ":" + std::to_string(row) + ": column " + header[i] +
                                 ": not a number: '" + c[i] + "'");
      return v;
    };
    JointVec q, qd, tau;
    for (int j = 0; j < kNumJoints; ++j) {
      q[j] = num(cq[j]);
      qd[j] = num(cqd[j]);
      tau[j] = tau_sign * num(ctau[j]);
    }
    in.t.push_back(num(ct));
    in.q.push_back(q);
    in.qd.push_back(qd);
    in.tau.push_back(tau);
  }
  return in;
}

double median_dt(const std::vector<double>& t) {
  if (t.size() < 2) return 0.0;
  std::vector<double> d(t.size() - 1);
  for (size_t k = 1; k < t.size(); ++k) d[k - 1] = t[k] - t[k - 1];
  std::nth_element(d.begin(), d.begin() + d.size() / 2, d.end());
  return d[d.size() / 2];
}

int debounce_rows_from_ms(const std::vector<double>& t, double ms) {
  const double dt = median_dt(t);
  if (!(dt > 0.0) || !(ms > 0.0))
    throw std::invalid_argument("debounce_rows_from_ms: need dt > 0 and ms > 0");
  return std::max(1, static_cast<int>(std::lround(ms / 1000.0 / dt)));
}

ReplayStats replay(const ReplayInput& in, const ReplayOptions& opt) {
  const size_t n = in.t.size();
  if (in.q.size() != n || in.qd.size() != n || in.tau.size() != n)
    throw std::invalid_argument("replay: t, q, qd, tau differ in length");

  Dynamics dyn(opt.urdf_path, opt.ee_frame);
  ContactMonitorConfig cfg;
  cfg.detection = ContactDetection::kObserve;
  cfg.gain = opt.gain;
  cfg.debounce_cycles = opt.debounce_cycles;
  cfg.initial_theta = opt.theta;
  cfg.ceiling = opt.theta;  // replay has no profile ceiling; the thresholds are the ceiling
  ContactMonitor mon(dyn, cfg);

  ReplayStats s;
  s.r.reserve(n);
  JointVec sumsq = JointVec::Zero();
  uint64_t seen_trips = 0;
  for (size_t k = 0; k < n; ++k) {
    const double dt = k > 0 ? in.t[k] - in.t[k - 1] : 0.0;
    JointFeedback fb;
    fb.q = in.q[k];
    fb.qd = in.qd[k];
    fb.tau = in.tau[k];
    mon.step(fb, dt, false);
    ContactStatus st;
    if (!mon.status(st)) throw std::logic_error("replay: torn status read on a single thread");
    s.r.push_back(st.r);

    if (st.would_trips > seen_trips) {
      seen_trips = st.would_trips;
      ReplayTrip& ft = s.first_trip[st.last.joint];
      if (!ft.valid) ft = {true, in.t[k], st.last.r[st.last.joint]};
    }
    if (st.phase == ContactPhase::kArmed && in.qd[k].cwiseAbs().maxCoeff() > kReplayMovingQd) {
      ++s.moving_samples;
      sumsq += st.r.cwiseProduct(st.r);
      s.peak = s.peak.cwiseMax(st.r.cwiseAbs());
    }
  }
  s.would_trips = seen_trips;
  if (s.moving_samples > 0) s.rms = (sumsq / static_cast<double>(s.moving_samples)).cwiseSqrt();
  return s;
}

}  // namespace kinova
