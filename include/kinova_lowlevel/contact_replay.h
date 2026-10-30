#pragma once
#include <Eigen/Core>
#include <string>
#include <utility>
#include <vector>

#include "kinova_lowlevel/contact_monitor.h"
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

// Median row spacing [s] (0 for fewer than two rows), and a time debounce as whole rows:
// round(ms / median dt), at least 1. The RT monitor debounces cycles at 1 kHz, so a
// slower recording needs this to count the same crossings.
double median_dt(const std::vector<double>& t);
int debounce_rows_from_ms(const std::vector<double>& t, double ms);

struct ReplayOptions {
  std::string urdf_path;
  std::string ee_frame;
  double gain = 25.0;
  int debounce_cycles = 5;
  JointVec theta = (JointVec() << 8, 8, 6, 6, 2, 2, 2).finished();
  // The impulse trip (ContactMonitorConfig::impulse), off unless set.
  bool impulse = false;
  JointVec impulse_theta = JointVec::Zero();
  double slow_tau_s = 0.2;
  // Grasps and releases, each (begin, end) in recording time [s]: the tare window
  // starts at `begin`, the weigh window at `end`, both weigh_window_s long, so pick
  // the start of a still stretch before the gripper moves and after the lift.
  std::vector<std::pair<double, double>> grasps;
  Eigen::Vector3d grasp_com = Eigen::Vector3d(0.0, 0.0, 0.10);  // EE frame [m]
  double weigh_window_s = 0.3;
};

struct ReplayTrip {
  bool valid = false;
  double t = 0.0;        // time of the row that completed the debounce
  double r = 0.0;        // that joint's residual at the trip [N·m]
  bool impulse = false;  // reached on the fast band alone
};

struct ReplayWeigh {
  double t = 0.0;  // time of the row that produced the outcome
  WeighResult result;
};

struct ReplayStats {
  std::vector<JointVec> r;           // residual per input row
  JointVec rms = JointVec::Zero();   // over moving samples [N·m]
  JointVec peak = JointVec::Zero();  // max |r| over moving samples [N·m]
  size_t moving_samples = 0;         // armed rows with max|qd| > kReplayMovingQd
  size_t would_trips = 0;            // what the RT monitor would count
  ReplayTrip first_trip[kNumJoints];
  std::vector<ReplayTrip> trips;  // every would-trip in order, with its joint
  std::vector<int> trip_joints;
  std::vector<ReplayWeigh> weighs;  // every weigh outcome in order
};

constexpr double kReplayMovingQd = 0.02;  // [rad/s]

// Row k steps the monitor with dt = t[k] - t[k-1] (0 for the first row). Statistics skip
// the 5/gain settle. Throws std::invalid_argument on bad options or ragged input.
ReplayStats replay(const ReplayInput& in, const ReplayOptions& opt);

}  // namespace kinova
