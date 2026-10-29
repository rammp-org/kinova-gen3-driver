#pragma once
#include <string>
#include <vector>

#include "kinova_lowlevel/joint_types.h"

namespace kinova {

// Offline replay of recorded joint states through the same ContactMonitor the RT loop
// runs (observe mode), so thresholds can be tuned between arm sessions.

struct ReplayInput {
  std::vector<double> t;             // [s], strictly increasing
  std::vector<JointVec> q, qd, tau;  // tau in command convention (rest: tau == g(q))
};

// CSV with a header row naming at least t, q1..q7, qd1..qd7, tau1..tau7 (other columns
// are ignored). tau_sign = -1 turns a raw core v1.2.0 recording into command convention.
// Throws std::runtime_error on an unreadable file, a missing column or a malformed row.
ReplayInput load_replay_csv(const std::string& path, double tau_sign = 1.0);

struct ReplayOptions {
  std::string urdf_path;
  std::string ee_frame;
  double gain = 25.0;
  int debounce_cycles = 5;
  JointVec theta = (JointVec() << 8, 8, 6, 6, 2, 2, 2).finished();
};

struct ReplayTrip {
  bool valid = false;
  double t = 0.0;  // time of the row that completed the debounce
  double r = 0.0;  // that joint's residual at the trip [N·m]
};

struct ReplayStats {
  std::vector<JointVec> r;           // residual per input row
  JointVec rms = JointVec::Zero();   // over moving samples [N·m]
  JointVec peak = JointVec::Zero();  // max |r| over moving samples [N·m]
  size_t moving_samples = 0;         // armed rows with max|qd| > kReplayMovingQd
  size_t would_trips = 0;            // what the RT monitor would count
  ReplayTrip first_trip[kNumJoints];
};

constexpr double kReplayMovingQd = 0.02;  // [rad/s]

// Row k steps the monitor with dt = t[k] - t[k-1] (0 for the first row). Statistics skip
// the 5/gain settle. Throws std::invalid_argument on bad options or ragged input.
ReplayStats replay(const ReplayInput& in, const ReplayOptions& opt);

}  // namespace kinova
