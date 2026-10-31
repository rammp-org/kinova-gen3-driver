#include "kinova_lowlevel/contact_monitor.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include "fixtures.h"
#include "kinova_lowlevel/momentum_observer.h"
using namespace kinova;
using test_fixtures::Sinusoid;
using test_fixtures::synth_tau;

namespace {
constexpr double kDt = 1e-3;
constexpr double kGain = 25.0;
constexpr int kDebounce = 5;
constexpr int kSettleCycles = 200;  // 5/K at 1 kHz
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// Test thresholds (N·m), not the threshold profiles: a 6 N·m push on j2 is over
// kThetaLow and under kThetaHigh.
const JointVec kThetaLow = (JointVec() << 4, 4, 3, 3, 1.2, 1.2, 1.2).finished();
const JointVec kThetaHigh = (JointVec() << 8, 8, 6, 6, 2, 2, 2).finished();
const JointVec kCeiling = (JointVec() << 30, 30, 22, 22, 7, 7, 7).finished();

ContactMonitorConfig config(ContactDetection d = ContactDetection::kEnforce,
                            const JointVec& theta = kThetaHigh) {
  ContactMonitorConfig c;
  c.detection = d;
  c.gain = kGain;
  c.debounce_cycles = kDebounce;
  c.initial_theta = theta;
  c.ceiling = kCeiling;
  return c;
}

Sinusoid rest() {
  Sinusoid s;
  s.A.setZero();
  return s;
}

JointVec push(int joint, double n_m) {
  JointVec e = JointVec::Zero();
  e[joint] = n_m;
  return e;
}

// A monitor fed the synthesized feedback of `s`, one 1 ms cycle per call.
class Rig {
 public:
  explicit Rig(ContactMonitorConfig cfg, Sinusoid s = rest())
      : dyn_(URDF_PATH), mon_(dyn_, cfg), s_(s) {}

  // The feedback of the next cycle under the external torque tau_e.
  JointFeedback feedback(const JointVec& tau_e = JointVec::Zero()) {
    const double t = k_ * kDt;
    JointFeedback fb;
    fb.q = s_.q(t);
    fb.qd = s_.qd(t);
    fb.tau = synth_tau(dyn_, s_, t, tau_e);
    return fb;
  }
  bool step_fb(const JointFeedback& fb, bool missed = false) {
    ++k_;
    return mon_.step(fb, kDt, missed);
  }
  bool step(const JointVec& tau_e = JointVec::Zero()) { return step_fb(feedback(tau_e)); }
  // Steps n cycles; returns the index within them of the first that returned true, or -1.
  int run(int n, const JointVec& tau_e = JointVec::Zero()) {
    int first = -1;
    for (int i = 0; i < n; ++i)
      if (step(tau_e) && first < 0) first = i;
    return first;
  }
  ContactStatus status() const {
    ContactStatus st;
    EXPECT_TRUE(mon_.status(st));
    return st;
  }
  Dynamics& dyn() { return dyn_; }
  ContactMonitor& mon() { return mon_; }
  int cycles() const { return k_; }

 private:
  Dynamics dyn_;
  ContactMonitor mon_;
  Sinusoid s_;
  int k_ = 0;
};
}  // namespace

TEST(ContactMonitor, NoTripDuringSettling) {
  Rig rig(config());
  const JointVec press = push(1, 20.0);  // 2.5x the 8 N·m threshold, from t = 0
  for (int k = 0; k < kSettleCycles - 1; ++k) {
    EXPECT_FALSE(rig.step(press)) << k;
    const ContactStatus st = rig.status();
    EXPECT_EQ(st.phase, ContactPhase::kSettling) << k;
    EXPECT_EQ(st.would_trips, 0u) << k;
  }
  // r crossed θ ~20 ms in (40 ms time constant); only the window held it off.
  EXPECT_GT(rig.status().r[1], 8.0);
  EXPECT_FALSE(rig.mon().latched());
  const int first = rig.run(kDebounce + 5, press);
  EXPECT_GE(first, kDebounce);
  EXPECT_LE(first, kDebounce + 2);
  EXPECT_TRUE(rig.mon().latched());
}

TEST(ContactMonitor, EnforceLatchesOnSustainedPush) {
  Rig rig(config(ContactDetection::kEnforce, kThetaLow));
  const int first = rig.run(kSettleCycles + kDebounce + 2, push(1, 6.0));
  ASSERT_GE(first, 0);
  EXPECT_GE(first, kSettleCycles + kDebounce - 1);
  EXPECT_TRUE(rig.mon().latched());
  EXPECT_TRUE(rig.step(push(1, 6.0)));

  const ContactStatus st = rig.status();
  EXPECT_EQ(st.detection, ContactDetection::kEnforce);
  EXPECT_EQ(st.phase, ContactPhase::kLatched);
  EXPECT_EQ(st.would_trips, 1u);
  ASSERT_TRUE(st.has_trip);
  EXPECT_EQ(st.last.joint, 1);
  EXPECT_EQ(st.last.cycle, static_cast<uint64_t>(first));
  EXPECT_GT(st.last.r[1], 4.0);
  EXPECT_NEAR(st.last.r[1], 6.0, 0.1);
  for (int j = 0; j < kNumJoints; ++j)
    if (j != 1) EXPECT_LT(std::abs(st.last.r[j]), kThetaLow[j]) << j;
  EXPECT_EQ(st.last.theta, kThetaLow);
  EXPECT_EQ(st.last.q, rest().q0);
  EXPECT_EQ(st.last.qd, JointVec::Zero());
}

TEST(ContactMonitor, DebounceRejectsShortSpike) {
  // A qd glitch on the measured momentum: p jumps by (R/K) e_2, so r_2 jumps to ~R for
  // exactly the glitch cycles and nothing else moves. R = 20 N·m against θ = 8.
  auto spike = [](int cycles) {
    Rig rig(config());
    EXPECT_EQ(rig.run(300), -1);
    JointMat M;
    rig.dyn().mass_matrix(rest().q0, M);
    const JointVec glitch = M.inverse() * push(1, 20.0 / kGain);
    int first = -1;
    for (int i = 0; i < cycles; ++i) {
      JointFeedback fb = rig.feedback();
      fb.qd = glitch;
      if (rig.step_fb(fb) && first < 0) first = i;
      EXPECT_GT(rig.status().r[1], 8.0) << i;
    }
    const int after = rig.run(200);
    if (first < 0) first = after < 0 ? -1 : cycles + after;
    return first;
  };
  EXPECT_EQ(spike(3), -1);
  EXPECT_EQ(spike(kDebounce - 1), -1);
  EXPECT_EQ(spike(kDebounce), kDebounce - 1);  // the same glitch held for the count trips
}

TEST(ContactMonitor, ObserveCountsButNeverLatches) {
  Rig rig(config(ContactDetection::kObserve, kThetaLow));
  const JointVec press = push(1, 6.0);
  EXPECT_EQ(rig.run(1000, press), -1);  // a sustained push is one crossing
  ContactStatus st = rig.status();
  EXPECT_EQ(st.detection, ContactDetection::kObserve);
  EXPECT_EQ(st.phase, ContactPhase::kArmed);
  EXPECT_EQ(st.would_trips, 1u);
  ASSERT_TRUE(st.has_trip);
  EXPECT_EQ(st.last.joint, 1);
  EXPECT_FALSE(rig.mon().latched());

  EXPECT_EQ(rig.run(300), -1);  // release: r falls back under θ and the joint re-arms
  EXPECT_EQ(rig.status().would_trips, 1u);
  EXPECT_EQ(rig.run(300, press), -1);  // a second push is a second crossing
  st = rig.status();
  EXPECT_EQ(st.would_trips, 2u);
  EXPECT_GT(st.last.cycle, 1000u);
  EXPECT_FALSE(rig.mon().latched());
}

TEST(ContactMonitor, LatchHoldsWithoutClear) {
  Rig rig(config(ContactDetection::kEnforce, kThetaLow));
  ASSERT_GE(rig.run(kSettleCycles + kDebounce + 2, push(1, 6.0)), 0);
  int released = 0;
  for (int k = 0; k < 10000; ++k)
    if (!rig.step()) ++released;  // the push is gone for 10 s
  EXPECT_EQ(released, 0);
  ContactStatus st = rig.status();
  EXPECT_TRUE(rig.mon().latched());
  EXPECT_EQ(st.phase, ContactPhase::kLatched);
  EXPECT_LT(st.r.cwiseAbs().maxCoeff(), 0.1);  // the observer still runs: r shows it is gone
  EXPECT_EQ(st.would_trips, 1u);

  // The clear is consumed by the next step(); with nobody pressing it stays clear.
  rig.mon().request_clear();
  EXPECT_TRUE(rig.mon().latched());
  EXPECT_FALSE(rig.step());
  EXPECT_FALSE(rig.mon().latched());
  EXPECT_EQ(rig.status().phase, ContactPhase::kSettling);
  EXPECT_EQ(rig.run(1000), -1);
  st = rig.status();
  EXPECT_EQ(st.phase, ContactPhase::kArmed);
  EXPECT_EQ(st.would_trips, 1u);
  EXPECT_TRUE(st.has_trip);  // the last trip stays on record
}

// Review Focus 3: blind for 5/K after the clear, then a new trip.
TEST(ContactMonitor, ClearWhilePressedRetripsAfterSettle) {
  Rig rig(config(ContactDetection::kEnforce, kThetaLow));
  const JointVec press = push(1, 6.0);
  ASSERT_GE(rig.run(kSettleCycles + kDebounce + 2, press), 0);
  EXPECT_TRUE(rig.step(press));
  const uint64_t first_trip = rig.status().last.cycle;

  rig.mon().request_clear();
  for (int k = 0; k < kSettleCycles - 1; ++k) {
    EXPECT_FALSE(rig.step(press)) << k;
    EXPECT_EQ(rig.status().phase, ContactPhase::kSettling) << k;
  }
  EXPECT_FALSE(rig.mon().latched());
  const int again = rig.run(kDebounce + 5, press);
  EXPECT_GE(again, kDebounce);
  EXPECT_LE(again, kDebounce + 2);
  EXPECT_TRUE(rig.mon().latched());
  const ContactStatus st = rig.status();
  EXPECT_EQ(st.phase, ContactPhase::kLatched);
  EXPECT_EQ(st.would_trips, 2u);
  EXPECT_EQ(st.last.joint, 1);
  EXPECT_GT(st.last.cycle, first_trip);
}

// Review Focus 4: the sample is skipped, the integral is not poisoned, no trip on NaN,
// and the count is reported.
TEST(ContactMonitor, NonFiniteSampleSkipped) {
  {
    // Against a reference observer fed only the good samples, with the skipped dt
    // carried into the next good one.
    Dynamics dyn(URDF_PATH);
    ContactMonitor mon(dyn, config());
    MomentumObserver ref(dyn, kGain);
    const Sinusoid s;
    double pending = 0.0;
    for (int k = 0; k < 1000; ++k) {
      const double t = k * kDt;
      JointFeedback fb;
      fb.q = s.q(t);
      fb.qd = s.qd(t);
      fb.tau = synth_tau(dyn, s, t, JointVec::Zero());
      const bool bad = k == 500 || k == 600 || k == 700;
      if (k == 500) fb.tau[3] = kNaN;
      if (k == 600) fb.qd[0] = std::numeric_limits<double>::infinity();
      if (k == 700) fb.q[6] = kNaN;
      EXPECT_FALSE(mon.step(fb, kDt, false)) << k;
      if (bad) {
        pending += kDt;
      } else {
        ref.step(fb.q, fb.qd, fb.tau, pending + kDt);
        pending = 0.0;
      }
      ContactStatus st;
      ASSERT_TRUE(mon.status(st));
      ASSERT_TRUE(st.r.allFinite()) << k;
      EXPECT_LT((st.r - ref.residual()).cwiseAbs().maxCoeff(), 1e-12) << k;
    }
    ContactStatus st;
    ASSERT_TRUE(mon.status(st));
    EXPECT_EQ(st.bad_samples, 3u);
    EXPECT_EQ(st.would_trips, 0u);
    EXPECT_FALSE(mon.latched());
  }
  {
    // A bad sample leaves the debounce count where it was.
    Rig rig(config());
    const JointVec press = push(1, 6.0);  // under kThetaHigh's 8
    ASSERT_EQ(rig.run(500, press), -1);
    rig.mon().set_thresholds(kThetaLow);  // now over 4
    EXPECT_EQ(rig.run(kDebounce - 2, press), -1);
    JointFeedback fb = rig.feedback(press);
    fb.tau[1] = kNaN;
    EXPECT_FALSE(rig.step_fb(fb));
    EXPECT_FALSE(rig.step(press));
    EXPECT_TRUE(rig.step(press));
    EXPECT_EQ(rig.status().bad_samples, 1u);
  }
}

TEST(ContactMonitor, MissedFramesDoNotStepObserver) {
  {
    // Ten missed cycles, fed garbage to prove the monitor does not read it. The next
    // good sample integrates over the whole gap.
    Dynamics dyn(URDF_PATH);
    ContactMonitor mon(dyn, config());
    MomentumObserver ref(dyn, kGain);
    const Sinusoid s;
    JointFeedback garbage;
    garbage.q.setConstant(kNaN);
    garbage.qd.setConstant(kNaN);
    garbage.tau.setConstant(kNaN);
    double pending = 0.0;
    for (int k = 0; k < 1000; ++k) {
      const double t = k * kDt;
      const bool missed = k >= 400 && k < 410;
      JointFeedback fb;
      fb.q = s.q(t);
      fb.qd = s.qd(t);
      fb.tau = synth_tau(dyn, s, t, JointVec::Zero());
      EXPECT_FALSE(mon.step(missed ? garbage : fb, kDt, missed)) << k;
      if (missed) {
        pending += kDt;
      } else {
        ref.step(fb.q, fb.qd, fb.tau, pending + kDt);
        pending = 0.0;
      }
      ContactStatus st;
      ASSERT_TRUE(mon.status(st));
      EXPECT_LT((st.r - ref.residual()).cwiseAbs().maxCoeff(), 1e-12) << k;
    }
    ContactStatus st;
    ASSERT_TRUE(mon.status(st));
    EXPECT_EQ(st.bad_samples, 0u);  // a miss is not a bad sample
    EXPECT_EQ(st.would_trips, 0u);
  }
  {
    // Missed cycles leave the debounce count where it was.
    Rig rig(config());
    const JointVec press = push(1, 6.0);
    ASSERT_EQ(rig.run(500, press), -1);
    rig.mon().set_thresholds(kThetaLow);
    EXPECT_EQ(rig.run(kDebounce - 2, press), -1);
    for (int i = 0; i < 3; ++i) EXPECT_FALSE(rig.step_fb(rig.feedback(press), true));
    EXPECT_FALSE(rig.step(press));
    EXPECT_TRUE(rig.step(press));
  }
}

// A step longer than 1/K is stepped as 1/K, so a still arm reads the push exactly
// (to 1e-6) after a 0.2 s step, still armed. The same when the gap is made of missed
// cycles.
TEST(ContactMonitor, LongGapStepCappedAtOneOverK) {
  Rig rig(config(ContactDetection::kObserve));
  const JointVec press = push(1, 6.0);
  ASSERT_EQ(rig.run(500), -1);
  ASSERT_EQ(rig.status().phase, ContactPhase::kArmed);
  rig.mon().step(rig.feedback(press), 0.2, false);
  ContactStatus st = rig.status();
  EXPECT_EQ(st.phase, ContactPhase::kArmed);
  EXPECT_NEAR(st.r[1], 6.0, 1e-6);
  // The same through missed cycles: 30 ms missed + 30 ms good = 60 ms > 1/K.
  ASSERT_EQ(rig.run(500), -1);  // released (observe: step never returns true)
  ASSERT_LT(rig.status().r.cwiseAbs().maxCoeff(), 1e-3);
  rig.mon().step(rig.feedback(press), 0.03, true);
  rig.mon().step(rig.feedback(press), 0.03, false);
  st = rig.status();
  EXPECT_NEAR(st.r[1], 6.0, 1e-6);
}

TEST(ContactMonitor, MotionWithoutContactNoTrip) {
  Rig rig(config(), Sinusoid{});
  EXPECT_EQ(rig.run(4000), -1);
  const ContactStatus st = rig.status();
  EXPECT_EQ(st.would_trips, 0u);
  EXPECT_FALSE(st.has_trip);
  EXPECT_EQ(st.phase, ContactPhase::kArmed);
}

TEST(ContactMonitor, ThresholdChangeTakesEffectNextCycle) {
  Rig rig(config());
  const JointVec press = push(1, 6.0);
  ASSERT_EQ(rig.run(500, press), -1);
  EXPECT_EQ(rig.status().theta, kThetaHigh);
  rig.mon().set_thresholds(kThetaLow);
  EXPECT_EQ(rig.status().theta, kThetaHigh);  // published by the RT side, not by the setter
  EXPECT_FALSE(rig.step(press));
  EXPECT_EQ(rig.status().theta, kThetaLow);
  EXPECT_EQ(rig.run(kDebounce - 2, press), -1);
  EXPECT_TRUE(rig.step(press));  // the count started on the cycle after the change
  EXPECT_EQ(rig.status().last.theta, kThetaLow);
}

TEST(ContactMonitor, OnRunStartResettlesButKeepsLatch) {
  Rig latched(config(ContactDetection::kEnforce, kThetaLow));
  ASSERT_GE(latched.run(kSettleCycles + kDebounce + 2, push(1, 6.0)), 0);
  latched.mon().on_run_start();
  EXPECT_TRUE(latched.mon().latched());
  EXPECT_EQ(latched.status().phase, ContactPhase::kLatched);
  EXPECT_TRUE(latched.step());
  EXPECT_EQ(latched.status().phase, ContactPhase::kLatched);

  Rig armed(config());
  ASSERT_EQ(armed.run(500), -1);
  EXPECT_EQ(armed.status().phase, ContactPhase::kArmed);
  armed.mon().on_run_start();
  EXPECT_EQ(armed.status().phase, ContactPhase::kSettling);
  EXPECT_EQ(armed.status().r, JointVec::Zero());
  EXPECT_FALSE(armed.step());
  EXPECT_EQ(armed.status().phase, ContactPhase::kSettling);
}

// settled (r trusted) follows the observer's re-inits, not the phase: false from
// construction, false again the moment a run restarts latched or a clear is
// consumed on a skipped sample, true only after 5/K of good steps.
TEST(ContactMonitor, SettledFollowsTheObserverNotThePhase) {
  Rig rig(config(ContactDetection::kEnforce, kThetaLow));
  EXPECT_FALSE(rig.status().settled);
  ASSERT_GE(rig.run(kSettleCycles + kDebounce + 2, push(1, 6.0)), 0);
  EXPECT_TRUE(rig.status().settled);

  rig.mon().on_run_start();  // restarts latched
  EXPECT_EQ(rig.status().phase, ContactPhase::kLatched);
  EXPECT_FALSE(rig.status().settled);
  rig.step_fb(rig.feedback(), /*missed=*/true);
  EXPECT_FALSE(rig.status().settled);
  rig.run(kSettleCycles + 2);
  EXPECT_EQ(rig.status().phase, ContactPhase::kLatched);
  EXPECT_TRUE(rig.status().settled);

  rig.mon().request_clear();
  rig.step_fb(rig.feedback(), /*missed=*/true);  // the clear lands on a skipped sample
  EXPECT_FALSE(rig.status().settled);
}

// A clear requested while no run is going (the link down, latched) is consumed by
// the next run start, so that run starts unlatched and settling.
TEST(ContactMonitor, OnRunStartConsumesAPendingClear) {
  Rig rig(config(ContactDetection::kEnforce, kThetaLow));
  ASSERT_GE(rig.run(kSettleCycles + kDebounce + 2, push(1, 6.0)), 0);
  ASSERT_TRUE(rig.mon().latched());
  rig.mon().request_clear();
  rig.mon().on_run_start();
  EXPECT_FALSE(rig.mon().latched());
  EXPECT_EQ(rig.status().phase, ContactPhase::kSettling);
  EXPECT_FALSE(rig.step());
  EXPECT_EQ(rig.status().phase, ContactPhase::kSettling);
}

TEST(ContactThresholds, ValidateRefusesAndNamesJoint) {
  EXPECT_TRUE(validate_contact_thresholds(kThetaHigh, kCeiling).ok);
  EXPECT_TRUE(validate_contact_thresholds(kCeiling, kCeiling).ok);  // at the ceiling is allowed
  for (const double bad :
       {kNaN, std::numeric_limits<double>::infinity(), 0.0, -1.0, kCeiling[4] + 0.1}) {
    JointVec theta = kThetaHigh;
    theta[4] = bad;
    const ThresholdCheck c = validate_contact_thresholds(theta, kCeiling);
    EXPECT_FALSE(c.ok) << bad;
    EXPECT_NE(c.message.find("joint 5"), std::string::npos) << c.message;
    EXPECT_NE(c.message.find("ceiling 7 N"), std::string::npos) << c.message;
    EXPECT_NE(c.message.find("> 0"), std::string::npos) << c.message;
  }
  JointVec ceiling = kCeiling;
  ceiling[2] = kNaN;  // a NaN ceiling must not let everything through
  const ThresholdCheck c = validate_contact_thresholds(kThetaHigh, ceiling);
  EXPECT_FALSE(c.ok);
  EXPECT_NE(c.message.find("joint 3"), std::string::npos) << c.message;
}

TEST(ContactMonitor, CtorRejectsBadConfig) {
  Dynamics dyn(URDF_PATH);
  auto throws_naming = [&](const ContactMonitorConfig& c, const std::string& field) {
    try {
      ContactMonitor m(dyn, c);
      ADD_FAILURE() << "no throw, expected one naming " << field;
    } catch (const std::invalid_argument& e) {
      EXPECT_NE(std::string(e.what()).find(field), std::string::npos) << e.what();
    }
  };
  for (const double g : {0.0, -1.0, kNaN}) {
    ContactMonitorConfig c = config();
    c.gain = g;
    throws_naming(c, "gain");
  }
  for (const int n : {0, -5}) {
    ContactMonitorConfig c = config();
    c.debounce_cycles = n;
    throws_naming(c, "debounce_cycles");
  }
  {
    ContactMonitorConfig c = config();
    c.initial_theta[2] = 0.0;
    throws_naming(c, "initial_theta");
    c = config();
    c.initial_theta[0] = kCeiling[0] + 0.1;
    throws_naming(c, "initial_theta");
  }
  for (const double v : {kNaN, 0.0, -1.0}) {
    ContactMonitorConfig c = config();
    c.ceiling[6] = v;
    throws_naming(c, "ceiling");
  }
  throws_naming(ContactMonitorConfig{}, "ceiling");  // thresholds never set

  ContactMonitor ok(dyn, config(ContactDetection::kObserve));
  EXPECT_EQ(ok.ceiling(), kCeiling);
  EXPECT_EQ(ok.detection(), ContactDetection::kObserve);
  EXPECT_FALSE(ok.latched());
  ContactStatus st;
  ASSERT_TRUE(ok.status(st));  // published from construction
  EXPECT_EQ(st.theta, kThetaHigh);
  EXPECT_EQ(st.phase, ContactPhase::kSettling);
  EXPECT_EQ(st.detection, ContactDetection::kObserve);
}
