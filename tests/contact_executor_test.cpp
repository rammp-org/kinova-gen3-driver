// RtExecutor with a ContactMonitor attached: a trip switches the loop to the
// on-contact mode (FloatMode) and holds it there while latched, whatever the
// Supervisor requests. Runs the real loop on SimTransport at 1 kHz; the observer
// sees external torque through SimTransport::set_external_torque.
#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "fixtures.h"
#include "kinova_lowlevel/contact_monitor.h"
#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/float_mode.h"
#include "kinova_lowlevel/joint_position_mode.h"
#include "kinova_lowlevel/joint_torque_mode.h"
#include "kinova_lowlevel/rt_executor.h"
#include "kinova_lowlevel/sim_transport.h"
#include "kinova_lowlevel/telemetry.h"
using namespace kinova;

namespace {

// Exchange call that applies the push: the seed read is call 0 and cycle k makes
// call k+1, so the observer has settled (5/K = 200 cycles at 1 kHz) well before.
constexpr uint64_t kPushAt = 300;
// Low enough that a 6 N·m push trips in ~20 cycles.
const JointVec kTheta = (JointVec() << 2, 2, 2, 2, 1, 1, 1).finished();
const JointVec kCeiling = (JointVec() << 30, 30, 22, 22, 7, 7, 7).finished();

JointVec push_j2(double n_m) {
  JointVec e = JointVec::Zero();
  e[1] = n_m;
  return e;
}

ActuatorModes all(ActuatorMode m) {
  ActuatorModes modes;
  modes.fill(m);
  return modes;
}

// Forwards to a SimTransport. Records every command sent and every actuator-mode
// switch, runs on_call(i) on the RT thread at the top of exchange call i, and
// stops the loop itself once stop_at calls have been made: no wall-clock timing.
//
// A real arm under position (or velocity) control reports the torque holding it,
// g(q) at rest. The sim instead keeps the last commanded torque: the seed's
// benign zero-torque hold, which the observer would read as an external g(q). So
// the sim is handed a torque command of `hold` in place of a non-torque command;
// `sent` records what the executor actually sent.
class Bench : public Transport {
 public:
  Bench(SimTransport& sim, std::atomic<bool>& stop, const JointVec& hold)
      : sim_(sim), stop_(stop), hold_(hold) {
    sent.reserve(8192);
  }
  void connect() override { sim_.connect(); }
  void set_servoing_low_level() override { sim_.set_servoing_low_level(); }
  void set_actuator_modes(const ActuatorModes& m) override {
    switches.emplace_back(calls, m);
    sim_.set_actuator_modes(m);
  }
  void exchange(const JointCommand& c, JointFeedback& fb) override {
    const uint64_t i = calls++;
    sent.push_back(c);
    if (on_call) on_call(i);
    if (calls >= stop_at) stop_.store(true);
    if (c.mode == ActuatorMode::kTorque) {
      sim_.exchange(c, fb);
    } else {
      JointCommand held = c;
      held.mode = ActuatorMode::kTorque;
      held.torque = hold_;
      sim_.exchange(held, fb);
    }
  }
  void send(const JointCommand& c) override { sim_.send(c); }
  void receive(JointFeedback& fb) override { sim_.receive(fb); }
  void safe_shutdown() override { sim_.safe_shutdown(); }
  void release_low_level() override { sim_.release_low_level(); }
  bool arm_in_fault() override { return sim_.arm_in_fault(); }

  uint64_t calls = 0, stop_at = UINT64_MAX;
  std::vector<JointCommand> sent;                            // sent[i] = call i's command
  std::vector<std::pair<uint64_t, ActuatorModes>> switches;  // (calls made so far, modes)
  std::function<void(uint64_t)> on_call;

 private:
  SimTransport& sim_;
  std::atomic<bool>& stop_;
  const JointVec hold_;
};

JointVec gravity(Dynamics& dyn, const JointVec& q) {
  JointVec g;
  dyn.gravity(q, g);
  return g;
}

JointFeedback at_rest(const JointVec& q, const JointVec& g) {
  JointFeedback fb;
  fb.q = q;
  fb.qd.setZero();
  fb.tau = g;  // the observer reads ~0 at rest
  return fb;
}

// One arm: the sim at rest at q0 holding tau = g(q0), the executor, and the modes,
// all on one Dynamics (the RT thread is its only user while run() runs).
struct World {
  Dynamics dyn{URDF_PATH};
  JointVec q0 = test_fixtures::Sinusoid().q0;
  JointVec g0 = gravity(dyn, q0);
  SimTransport sim{at_rest(q0, g0)};
  std::atomic<bool> stop{false};
  Bench bench{sim, stop, g0};
  SampleRing ring{8192};
  RtExecutor ex{bench, ring, {1000.0, Pacing::kSleepSpin, {0, -1, false, false}}};
  FloatMode flt{dyn};
  JointPositionMode pos{dyn};
  std::unique_ptr<ContactMonitor> mon;

  void attach(ContactDetection d = ContactDetection::kEnforce) {
    ContactMonitorConfig c;
    c.detection = d;
    c.initial_theta = kTheta;
    c.ceiling = kCeiling;
    mon = std::make_unique<ContactMonitor>(dyn, c);
    ex.attach_contact(*mon, flt);
  }

  // Runs until `calls` exchange calls have been made in total.
  void run_until(uint64_t calls) {
    bench.stop_at = calls;
    stop.store(false);
    ex.run(stop);
  }

  // FloatMode's command at rest at q0: gravity, nothing else.
  bool is_float(const JointCommand& c) const {
    return c.mode == ActuatorMode::kTorque && (c.torque - g0).cwiseAbs().maxCoeff() < 1e-9;
  }

  ContactStatus status() const {
    ContactStatus st;
    EXPECT_TRUE(mon->status(st));
    return st;
  }
};

// The position mode walks its reference toward q0 + 0.2 rad on j1 from call 5, so
// its commands never equal the measured q (the sim does not move): a command whose
// position is exactly q0 is the executor's freeze, not the mode's.
void start_moving(World& w) {
  JointVec target = w.q0;
  target[0] += 0.2;
  auto prev = std::move(w.bench.on_call);
  w.bench.on_call = [&w, target, prev](uint64_t i) {
    if (i == 5) w.pos.set_target(target);
    if (prev) prev(i);
  };
}

}  // namespace

TEST(ContactExecutor, PositionModeTripAdoptsFloat) {
  World w;
  w.attach();
  w.bench.on_call = [&](uint64_t i) {
    if (i == kPushAt) w.sim.set_external_torque(push_j2(6.0));
  };
  start_moving(w);
  w.ex.request_mode(&w.pos);
  w.run_until(kPushAt + 100);

  ASSERT_TRUE(w.mon->latched());
  const ContactStatus st = w.status();
  ASSERT_TRUE(st.has_trip);
  EXPECT_EQ(st.last.joint, 1);
  // Trip cycle k computes the freeze, sent by call k+2; cycle k+1 adopts float,
  // whose first command goes out on call k+3.
  const uint64_t k = st.last.cycle;
  ASSERT_GE(k + 1, kPushAt);  // cycle kPushAt-1 is the first to see the push
  ASSERT_LT(k + 3, w.bench.sent.size());
  EXPECT_LE(k + 3 - kPushAt, 50u);

  for (uint64_t i = 10; i < k + 2; ++i) {
    ASSERT_EQ(w.bench.sent[i].mode, ActuatorMode::kPosition) << "call " << i;
    ASSERT_GT(w.bench.sent[i].position[0], w.q0[0]) << "call " << i;  // the mode is moving
  }
  const JointCommand& freeze = w.bench.sent[k + 2];
  EXPECT_EQ(freeze.mode, ActuatorMode::kPosition);
  EXPECT_EQ(freeze.position, w.q0);  // fb.q; compute was skipped
  EXPECT_TRUE(freeze.velocity.isZero());
  for (uint64_t i = k + 3; i < w.bench.sent.size(); ++i)
    ASSERT_TRUE(w.is_float(w.bench.sent[i])) << "call " << i;

  // Adopted through set_actuator_modes, at the boundary after the trip.
  ASSERT_EQ(w.bench.switches.size(), 2u);
  EXPECT_EQ(w.bench.switches[0].second, all(ActuatorMode::kPosition));
  EXPECT_EQ(w.bench.switches[1].first, k + 2);
  EXPECT_EQ(w.bench.switches[1].second, all(ActuatorMode::kTorque));
}

TEST(ContactExecutor, TorqueModeTripSwitchesSameCycle) {
  World w;
  w.attach();
  // Gravity scaled 1.02: tells its command from float's (exactly g), and the
  // 2% excess it reads back as external torque stays under the thresholds.
  JointTorqueParams p;
  p.scale = 1.02;
  JointTorqueMode grav(w.dyn, p);
  w.bench.on_call = [&](uint64_t i) {
    if (i == kPushAt) w.sim.set_external_torque(push_j2(6.0));
  };
  w.ex.request_mode(&grav);
  w.run_until(kPushAt + 100);

  ASSERT_TRUE(w.mon->latched());
  const uint64_t k = w.status().last.cycle;
  ASSERT_GE(k + 1, kPushAt);
  ASSERT_LT(k + 2, w.bench.sent.size());
  // Cycle k's own compute is float's: sent by call k+2. Call k+1 carried cycle
  // k-1's gravity-comp command.
  const JointCommand& before = w.bench.sent[k + 1];
  EXPECT_EQ(before.mode, ActuatorMode::kTorque);
  EXPECT_NEAR(before.torque[1], 1.02 * w.g0[1], 1e-9);
  for (uint64_t i = k + 2; i < w.bench.sent.size(); ++i)
    ASSERT_TRUE(w.is_float(w.bench.sent[i])) << "call " << i;
  // Same actuator modes: no pumping handshake for the switch.
  ASSERT_EQ(w.bench.switches.size(), 1u);
  EXPECT_EQ(w.bench.switches[0].second, all(ActuatorMode::kTorque));
}

// Review Focus 2: a mode request racing the trip, and requests while latched, are
// not adopted until the clear; then the latest one is, with no re-request.
TEST(ContactExecutor, RequestWhileLatchedDeferredUntilClear) {
  World w;
  w.attach();
  JointPositionMode hold(w.dyn);  // the Supervisor's position hold
  constexpr uint64_t kRequestsEnd = kPushAt + 150, kUnpush = kPushAt + 200, kClear = kPushAt + 250;
  w.bench.on_call = [&](uint64_t i) {
    if (i == kPushAt) w.sim.set_external_torque(push_j2(6.0));
    // A request every cycle from the push on: through the trip and the latch.
    if (i >= kPushAt && i < kRequestsEnd) w.ex.request_mode(&hold);
    if (i == kUnpush) w.sim.set_external_torque(JointVec::Zero());
    if (i == kClear) w.mon->request_clear();
  };
  w.ex.request_mode(&w.pos);
  w.run_until(kClear + 50);

  EXPECT_FALSE(w.mon->latched());
  const uint64_t k = w.status().last.cycle;
  ASSERT_GE(k + 1, kPushAt);
  ASSERT_LT(k + 3, kRequestsEnd);
  // From float's first command to the clear, float only.
  for (uint64_t i = k + 3; i <= kClear + 1; ++i)
    ASSERT_TRUE(w.is_float(w.bench.sent[i])) << "call " << i;
  // The clear is consumed in cycle kClear-1 (after call kClear's exchange); cycle
  // kClear adopts the pending request, whose first command goes out on kClear+2.
  for (uint64_t i = kClear + 2; i < w.bench.sent.size(); ++i)
    ASSERT_EQ(w.bench.sent[i].mode, ActuatorMode::kPosition) << "call " << i;
  // pos, hold (requested before the trip), float, then hold again after the clear.
  ASSERT_EQ(w.bench.switches.size(), 4u);
  EXPECT_EQ(w.bench.switches[2].second, all(ActuatorMode::kTorque));
  EXPECT_EQ(w.bench.switches[3].first, kClear + 1);
  EXPECT_EQ(w.bench.switches[3].second, all(ActuatorMode::kPosition));
}

// Review Focus 1: a run ending while latched (comms loss, an ArmLink fault) comes
// back in float, still latched, not in the Supervisor's position hold.
TEST(ContactExecutor, LatchSurvivesRunRestart) {
  World w;
  w.attach();
  w.bench.on_call = [&](uint64_t i) {
    if (i == kPushAt) w.sim.set_external_torque(push_j2(6.0));
  };
  w.ex.request_mode(&w.pos);
  w.run_until(kPushAt + 100);
  ASSERT_TRUE(w.mon->latched());
  ASSERT_GE(w.status().last.cycle + 1, kPushAt);

  // The push is gone; the Supervisor re-requests its hold between runs.
  w.sim.set_external_torque(JointVec::Zero());
  const size_t switches_before = w.bench.switches.size();
  const uint64_t first_call = w.bench.calls;
  w.ex.request_mode(&w.pos);
  w.run_until(first_call + 400);  // past the 200-cycle settle

  EXPECT_TRUE(w.mon->latched());
  EXPECT_EQ(w.status().phase, ContactPhase::kLatched);
  ASSERT_GT(w.bench.switches.size(), switches_before);
  EXPECT_EQ(w.bench.switches[switches_before].second, all(ActuatorMode::kTorque));
  EXPECT_EQ(w.bench.switches.size(), switches_before + 1);  // float only
  // first_call (the seed) and first_call+1 carry the run's benign default
  // command; from cycle 0's compute on, every command is float's.
  for (uint64_t i = first_call + 2; i < w.bench.sent.size(); ++i)
    ASSERT_TRUE(w.is_float(w.bench.sent[i])) << "call " << i;
}

TEST(ContactExecutor, ObserveNeverChangesCommand) {
  auto run = [](bool pushed, std::vector<JointCommand>& sent, uint64_t& would_trips) {
    World w;
    w.attach(ContactDetection::kObserve);
    w.bench.on_call = [&](uint64_t i) {
      if (pushed && i == kPushAt) w.sim.set_external_torque(push_j2(6.0));
    };
    start_moving(w);
    w.ex.request_mode(&w.pos);
    w.run_until(kPushAt + 200);
    EXPECT_FALSE(w.mon->latched());
    CycleSample s;
    while (w.ring.pop(s)) ASSERT_EQ(s.flags & 8u, 0u);
    sent = w.bench.sent;
    would_trips = w.status().would_trips;
  };
  std::vector<JointCommand> quiet, pushed;
  uint64_t trips_quiet = 0, trips_pushed = 0;
  run(false, quiet, trips_quiet);
  run(true, pushed, trips_pushed);
  EXPECT_EQ(trips_quiet, 0u);
  EXPECT_EQ(trips_pushed, 1u);  // it saw the push
  ASSERT_EQ(quiet.size(), pushed.size());
  for (size_t i = 0; i < quiet.size(); ++i) {
    ASSERT_EQ(quiet[i].mode, pushed[i].mode) << "call " << i;
    ASSERT_EQ(quiet[i].position, pushed[i].position) << "call " << i;
    ASSERT_EQ(quiet[i].velocity, pushed[i].velocity) << "call " << i;
    ASSERT_EQ(quiet[i].torque, pushed[i].torque) << "call " << i;
  }
}

TEST(ContactExecutor, NoMonitorUnchanged) {
  World w;  // no attach_contact
  w.bench.on_call = [&](uint64_t i) {
    if (i == kPushAt) w.sim.set_external_torque(push_j2(6.0));
  };
  start_moving(w);
  w.ex.request_mode(&w.pos);
  w.run_until(kPushAt + 100);

  for (uint64_t i = 10; i < w.bench.sent.size(); ++i) {
    ASSERT_EQ(w.bench.sent[i].mode, ActuatorMode::kPosition) << "call " << i;
    ASSERT_GT(w.bench.sent[i].position[0], w.q0[0]) << "call " << i;
  }
  ASSERT_EQ(w.bench.switches.size(), 1u);
  CycleSample s;
  while (w.ring.pop(s)) ASSERT_EQ(s.flags & 8u, 0u);

  // A request is still adopted at the next boundary.
  JointTorqueMode grav(w.dyn);
  const uint64_t at = w.bench.calls;
  w.bench.on_call = [&](uint64_t i) {
    if (i == at + 10) w.ex.request_mode(&grav);
  };
  w.run_until(at + 30);
  ASSERT_EQ(w.bench.switches.size(), 2u);
  EXPECT_EQ(w.bench.switches[1].first, at + 11);
  EXPECT_EQ(w.bench.sent.back().mode, ActuatorMode::kTorque);
}

TEST(ContactExecutor, LatchFlagInTelemetry) {
  World w;
  w.attach();
  w.bench.on_call = [&](uint64_t i) {
    if (i == kPushAt) w.sim.set_external_torque(push_j2(6.0));
  };
  w.ex.request_mode(&w.pos);
  w.run_until(kPushAt + 100);

  ASSERT_TRUE(w.mon->latched());
  const uint64_t k = w.status().last.cycle;
  ASSERT_GE(k + 1, kPushAt);
  CycleSample s;
  uint64_t n = 0;
  while (w.ring.pop(s)) {
    ++n;
    ASSERT_EQ((s.flags & 8u) != 0, s.cycle_index >= k) << "cycle " << s.cycle_index;
  }
  EXPECT_EQ(n, kPushAt + 99);  // every cycle sampled (calls 1..stop_at-1)
}

// on_run_start() at each run() start: the observer re-settles (never on a mode
// change, so a request inside the run leaves it armed).
TEST(ContactExecutor, RunStartResettlesObserver) {
  World w;
  w.attach();
  JointTorqueMode grav(w.dyn);
  w.bench.on_call = [&](uint64_t i) {
    if (i == kPushAt) w.ex.request_mode(&grav);
  };
  w.ex.request_mode(&w.pos);
  w.run_until(kPushAt + 50);
  ASSERT_EQ(w.bench.switches.size(), 2u);
  EXPECT_EQ(w.status().phase, ContactPhase::kArmed);

  const uint64_t first_call = w.bench.calls;
  ContactPhase early = ContactPhase::kArmed;
  w.bench.on_call = [&](uint64_t i) {
    if (i == first_call + 10) {
      ContactStatus st;
      if (w.mon->status(st)) early = st.phase;
    }
  };
  w.run_until(first_call + 20);
  EXPECT_EQ(early, ContactPhase::kSettling);
}
