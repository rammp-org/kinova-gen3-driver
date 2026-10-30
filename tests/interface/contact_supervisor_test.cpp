// The Supervisor's side of the unexpected-contact stop: a trip halts with
// HaltReason::kContact (result -11), work is refused while latched, the latch
// survives a link outage, clear_contact() hands the arm back to the position hold,
// and threshold profiles are validated and token-gated. Runs the real RtExecutor
// at 1 kHz on SimTransport with a real ContactMonitor; a push is
// SimTransport::set_external_torque.
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <string>
#include <thread>

#include "../fixtures.h"
#include "fake_backend.h"
#include "kinova_lowlevel/contact_monitor.h"
#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/feedback_tap.h"
#include "kinova_lowlevel/float_mode.h"
#include "kinova_lowlevel/interface/arbiter.h"
#include "kinova_lowlevel/interface/contact.h"
#include "kinova_lowlevel/interface/supervisor.h"
#include "kinova_lowlevel/rt_executor.h"
#include "kinova_lowlevel/sim_transport.h"
using namespace kinova;
using namespace kinova::interface;

namespace {

// The starting numbers from the design (N·m, j1..j7).
const JointVec kSensitive = (JointVec() << 4, 4, 3, 3, 1.2, 1.2, 1.2).finished();
const JointVec kNormal = (JointVec() << 8, 8, 6, 6, 2, 2, 2).finished();
const JointVec kLoose = (JointVec() << 20, 20, 15, 15, 5, 5, 5).finished();
const JointVec kCeiling = (JointVec() << 30, 30, 22, 22, 7, 7, 7).finished();

ContactProfiles profiles() { return {kSensitive, kNormal, kLoose, kCeiling}; }

ContactMonitorConfig monitor_config() {
  ContactMonitorConfig c;
  c.initial_theta = kNormal;
  c.ceiling = kCeiling;
  return c;
}

JointVec push_j2(double n_m) {
  JointVec e = JointVec::Zero();
  e[1] = n_m;
  return e;
}

bool wait_for(const std::function<bool()>& pred, double timeout_s = 3.0) {
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < end) {
    if (pred()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return pred();
}

void sleep_ms(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

// Sits between the sim and the rest of the chain. The sim never moves; once
// `moved` is set, feedback reports q_moved instead, as if the arm had floated
// there. Only j1 may differ from the sim's q: base yaw leaves g(q) unchanged on a
// level base, so the sim's gravity-model tau still matches and the observer reads
// ~0. While `watch` is set, counts position commands that are not exactly q_moved.
class Probe : public Transport {
 public:
  explicit Probe(SimTransport& s) : sim_(s) {}
  void connect() override { sim_.connect(); }
  void set_servoing_low_level() override { sim_.set_servoing_low_level(); }
  void set_actuator_modes(const ActuatorModes& m) override { sim_.set_actuator_modes(m); }
  void exchange(const JointCommand& c, JointFeedback& fb) override {
    if (watch.load() && c.mode == ActuatorMode::kPosition &&
        (c.position - q_moved).cwiseAbs().maxCoeff() > 1e-6)
      off_hold.fetch_add(1);
    sim_.exchange(c, fb);
    if (moved.load()) fb.q = q_moved;
  }
  void send(const JointCommand& c) override { sim_.send(c); }
  void receive(JointFeedback& fb) override {
    sim_.receive(fb);
    if (moved.load()) fb.q = q_moved;
  }
  void safe_shutdown() override { sim_.safe_shutdown(); }
  void release_low_level() override { sim_.release_low_level(); }
  bool arm_in_fault() override { return sim_.arm_in_fault(); }

  // Feedback q from `moved` on: q with j1 moved by dq1. Call while not moved.
  void move_j1(const JointVec& q, double dq1) {
    q_moved = q;
    q_moved[0] += dq1;
    moved.store(true);
  }

  JointVec q_moved = JointVec::Zero();  // written before `moved` is set
  std::atomic<bool> moved{false}, watch{false};
  std::atomic<int> off_hold{0};

 private:
  SimTransport& sim_;
};

// FakeBackend whose settle() blocks while `gate` is closed: holds the sampler
// still, mid-drain, for as long as a test needs.
class GatedBackend : public FakeBackend {
 public:
  void settle(const GoalId& id, const TrajectoryResult& r) override {
    while (closed.load()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    FakeBackend::settle(id, r);
  }
  std::atomic<bool> closed{false};
};

// SupFix (supervisor_test.cpp) plus contact detection: the sim at rest at q0
// holding tau = g(q0) under every mode (its gravity model, on its own Dynamics),
// a ContactMonitor and FloatMode attached to the executor, and the Supervisor
// told about the monitor. The modes and the monitor share `dyn` (the RT thread is
// its only user); the pump has its own.
struct ContactFix {
  Dynamics dyn{URDF_PATH}, pump_dyn{URDF_PATH}, sim_dyn{URDF_PATH};
  JointVec q0 = test_fixtures::Sinusoid().q0;
  SimTransport sim{test_fixtures::at_rest(q0, test_fixtures::gravity(dyn, q0))};
  Probe probe{sim};
  GripperController gc{probe};
  Seqlock<JointFeedback> snap;
  FeedbackTap tap{gc, snap};
  SampleRing ring{1u << 12};
  JointPositionMode pos{dyn};
  JointImpedanceMode imp{dyn};
  JointTorqueMode tau{dyn};
  JointVelocityMode vel{dyn};
  FloatMode flt{dyn};
  ContactMonitor mon{dyn, monitor_config()};
  RtExecutor exec{tap, ring, {1000.0, kinova::Pacing::kSleepSpin, {}}};
  GatedBackend be;
  double mode_settle_s;
  Supervisor sup{deps()};
  std::atomic<bool> stop{false};
  std::thread rt;

  explicit ContactFix(double settle_s = SupervisorConfig{}.mode_settle_s)
      : mode_settle_s(settle_s) {
    sim.set_gravity_model([this](const JointVec& q, JointVec& g) { sim_dyn.gravity(q, g); });
    exec.attach_contact(mon, flt);
  }
  ~ContactFix() {
    sup.stop();
    teardown();
  }

  SupervisorDeps deps() {
    SupervisorDeps d;
    d.pos = &pos;
    d.imp = &imp;
    d.tau = &tau;
    d.vel = &vel;
    d.exec = &exec;
    d.snap = &snap;
    d.pump_dyn = &pump_dyn;
    d.stream = &be;
    d.action = &be;
    d.grip = &gc;
    d.contact = &mon;
    d.contact_profiles = profiles();
    d.cfg.mode_settle_s = mode_settle_s;
    return d;
  }

  void run_rt() {
    stop = false;
    rt = std::thread([&] { exec.run(stop); });
  }
  void teardown() {
    stop = true;
    if (rt.joinable()) rt.join();
  }

  // Push on j2 (over the default θ of 8 N·m) until the monitor latches.
  void trip() {
    sim.set_external_torque(push_j2(12.0));
    ASSERT_TRUE(wait_for([&] { return mon.latched(); })) << "no trip";
  }

  // A slow position ramp from where the arm is, guard off (the sim does not move).
  TrajectoryGoal ramp_goal(double dur = 3.0) const {
    TrajectoryGoal g;
    JointVec b = q0;
    b[0] += 0.2;
    g.trajectory.points = {{q0, 0.0}, {b, dur}};
    g.control_mode = ControlModeKind::kPosition;
    g.path_tolerance = JointVec::Constant(-1.0);
    return g;
  }

  ContactStatus status() const {
    ContactStatus st;
    EXPECT_TRUE(mon.status(st));
    return st;
  }
};

GoalId id_of(uint8_t n) {
  GoalId id{};
  id[0] = n;
  return id;
}

}  // namespace

TEST(ContactSupervisor, InFlightGoalSettlesContactStop) {
  EXPECT_EQ(result_code::kContactStop, -11);
  ContactFix f;
  f.sup.start();
  f.run_rt();
  const TrajectoryGoal g = f.ramp_goal();
  ASSERT_EQ(f.sup.on_trajectory_goal(g), GoalResponse::kAccept);
  f.sup.on_trajectory_accepted(id_of(1), g);
  sleep_ms(400);  // moving, and the observer has settled
  ASSERT_EQ(f.be.result_count(), 0u);
  f.trip();
  ASSERT_TRUE(wait_for([&] { return f.be.result_count() == 1; }));
  sleep_ms(50);
  EXPECT_EQ(f.be.result_count(), 1u);  // one halt per rising edge
  EXPECT_EQ(f.be.last_result_id()[0], 1);
  EXPECT_EQ(f.be.last_result().error_code, result_code::kContactStop);
  EXPECT_EQ(f.be.last_result().error_string, "halted: unexpected contact");
}

TEST(ContactSupervisor, RefusesWhileLatched) {
  ContactFix f;
  f.sup.start();
  f.run_rt();
  sleep_ms(300);
  f.trip();
  sleep_ms(50);  // the sampler has seen the edge

  const TrajectoryGoal g = f.ramp_goal();
  EXPECT_EQ(f.sup.on_trajectory_goal(g), GoalResponse::kReject);

  StreamOpenRequest r;
  r.timeout_s = 1.0;
  const StreamOpenResult res = f.sup.on_stream_open(r);
  EXPECT_FALSE(res.accepted);
  EXPECT_EQ(res.error_code, result_code::kContactStop);
  EXPECT_EQ(res.message, "stopped on unexpected contact; call /clear_faults");
  EXPECT_FALSE(f.sup.stream_is_open());

  // A goal ACCEPTed anyway (the goal and accepted callbacks are separate) settles -11.
  f.sup.on_trajectory_accepted(id_of(2), g);
  ASSERT_TRUE(wait_for([&] { return f.be.result_count() == 1; }));
  EXPECT_EQ(f.be.last_result().error_code, result_code::kContactStop);
  EXPECT_EQ(f.be.last_result().error_string, "halted: unexpected contact");

  GripperSetpoint gs;
  gs.command.position = 1.0f;
  f.sup.on_gripper_setpoint(gs);

  // Threshold changes still work while latched.
  ContactThresholdsRequest tr;
  tr.profile = ContactProfile::kLoose;
  EXPECT_TRUE(f.sup.on_set_contact_thresholds(tr).accepted);

  sleep_ms(50);
  EXPECT_TRUE(f.mon.latched());
  f.sup.stop();
  f.teardown();
  EXPECT_EQ(f.sim.last_command().mode, ActuatorMode::kTorque);  // still floating
  EXPECT_FALSE(f.sim.last_command().gripper.active);            // the setpoint was dropped
}

TEST(ContactSupervisor, ClearRequestsPositionHold) {
  ContactFix f;
  f.sup.start();
  EXPECT_FALSE(f.sup.clear_contact());  // nothing latched
  f.run_rt();
  sleep_ms(300);
  f.trip();
  sleep_ms(50);
  f.probe.move_j1(f.q0, 0.3);                   // floated away from the pre-trip pose
  f.sim.set_external_torque(JointVec::Zero());  // the person lets go
  sleep_ms(20);
  EXPECT_TRUE(f.sup.clear_contact());
  ASSERT_TRUE(wait_for([&] { return !f.mon.latched(); }));
  EXPECT_FALSE(f.sup.clear_contact());  // already clear
  sleep_ms(300);                        // settle, armed, no re-trip
  EXPECT_FALSE(f.mon.latched());
  EXPECT_EQ(f.sup.on_trajectory_goal(f.ramp_goal()), GoalResponse::kAccept);
  f.sup.stop();
  f.teardown();
  // Adopted the position hold at the measured pose, not float or a stale target.
  const JointCommand c = f.sim.last_command();
  EXPECT_EQ(c.mode, ActuatorMode::kPosition);
  for (int i = 0; i < kNumJoints; ++i)
    EXPECT_NEAR(c.position[i], f.probe.q_moved[i], 1e-6) << "joint " << i;
}

// The sampler can be blind for a mode settle (250 ms). A clear landing before it
// sees the latch must still settle the goal the trip interrupted, never resume it.
TEST(ContactSupervisor, ClearBeforeTheSamplerSawTheLatchStillSettlesTheGoal) {
  ContactFix f;  // sampler not started: an accepted goal waits in the inbox
  f.exec.request_mode(&f.pos);
  const TrajectoryGoal g = f.ramp_goal();
  ASSERT_EQ(f.sup.on_trajectory_goal(g), GoalResponse::kAccept);
  f.sup.on_trajectory_accepted(id_of(3), g);
  f.run_rt();
  sleep_ms(300);
  f.trip();
  f.sim.set_external_torque(JointVec::Zero());
  ASSERT_TRUE(f.sup.clear_contact());
  ASSERT_TRUE(wait_for([&] { return !f.mon.latched(); }));
  f.sup.start();
  ASSERT_TRUE(wait_for([&] { return f.be.result_count() == 1; }));
  EXPECT_EQ(f.be.last_result().error_code, result_code::kContactStop);
}

// A position goal popped before the trip sits in its mode settle while the arm
// trips, floats away and is cleared. It must settle -11 without ever driving its
// first sample (the pre-trip pose) into the position hold.
TEST(ContactSupervisor, ClearDuringAModeSettleNeverDrivesAPreTripGoal) {
  ContactFix f(1.0);
  f.sup.start();
  f.run_rt();
  // Into impedance first, so the position goal below switches modes.
  StreamOpenRequest r;
  r.control_mode = ControlModeKind::kImpedance;
  r.timeout_s = 5.0;
  ASSERT_TRUE(f.sup.on_stream_open(r).accepted);
  f.sup.on_stream_close({});
  const TrajectoryGoal a = f.ramp_goal();
  ASSERT_EQ(f.sup.on_trajectory_goal(a), GoalResponse::kAccept);
  f.sup.on_trajectory_accepted(id_of(4), a);
  sleep_ms(100);  // the sampler is asleep in A's 1 s mode settle
  f.trip();
  f.probe.move_j1(f.q0, 0.3);
  f.sim.set_external_torque(JointVec::Zero());
  sleep_ms(20);
  ASSERT_TRUE(f.sup.clear_contact());
  ASSERT_TRUE(wait_for([&] { return !f.mon.latched(); }));
  f.probe.watch.store(true);  // from here every position command must be the hold
  ASSERT_TRUE(wait_for([&] { return f.be.result_count() == 1; }));
  sleep_ms(100);
  EXPECT_EQ(f.probe.off_hold.load(), 0);
  EXPECT_EQ(f.be.result_count(), 1u);
  EXPECT_EQ(f.be.last_result_id()[0], 4);
  EXPECT_EQ(f.be.last_result().error_code, result_code::kContactStop);
}

// The sampler is held mid-drain (a blocked settle) across a clear, a stream open
// and a re-trip, so it never sees the latch go down. The re-trip must still halt:
// the stream opened in between is closed.
TEST(ContactSupervisor, RetripTheSamplerNeverSawUnlatchStillHalts) {
  ContactFix f;
  f.sup.start();
  f.run_rt();
  sleep_ms(300);
  f.trip();
  sleep_ms(50);  // the sampler has seen the edge
  f.be.closed.store(true);
  f.sup.on_trajectory_accepted(id_of(5), f.ramp_goal());  // settled -11 in the drain: blocks
  sleep_ms(20);
  ASSERT_TRUE(f.sup.clear_contact());  // still pushing: re-trips after 5/K
  ASSERT_TRUE(wait_for([&] { return !f.mon.latched(); }));
  StreamOpenRequest r;
  r.timeout_s = 5.0;
  ASSERT_TRUE(f.sup.on_stream_open(r).accepted);
  ASSERT_TRUE(wait_for([&] { return f.mon.latched(); }));  // the re-trip
  f.be.closed.store(false);
  ASSERT_TRUE(wait_for([&] { return !f.sup.stream_is_open(); }, 0.5));
  EXPECT_EQ(f.sup.stream_close_cause(), StreamCloseCause::kContact);
}

// A stream open at the trip closes with cause kContact, not kHalted: the client's
// recovery is a clear, not a re-open. Any other halt still closes kHalted.
TEST(ContactSupervisor, StreamOpenAtTripClosesWithCauseContact) {
  ContactFix f;
  f.sup.start();
  f.run_rt();
  sleep_ms(300);
  StreamOpenRequest r;
  r.timeout_s = 5.0;
  ASSERT_TRUE(f.sup.on_stream_open(r).accepted);
  f.trip();
  ASSERT_TRUE(wait_for([&] { return !f.sup.stream_is_open(); }, 0.5));
  EXPECT_EQ(f.sup.stream_close_cause(), StreamCloseCause::kContact);

  ContactFix g;
  ASSERT_TRUE(g.sup.on_stream_open(r).accepted);
  g.sup.on_halt(HaltReason::kEmergencyStop);
  EXPECT_FALSE(g.sup.stream_is_open());
  EXPECT_EQ(g.sup.stream_close_cause(), StreamCloseCause::kHalted);
}

// Review Focus 1: a link outage while latched comes back in float, still latched,
// goals refused -- never in the Supervisor's position hold.
TEST(ContactSupervisor, LatchSurvivesLinkOutage) {
  ContactFix f;
  f.sup.start();
  f.run_rt();
  sleep_ms(300);
  f.trip();
  sleep_ms(50);
  f.sim.set_external_torque(JointVec::Zero());  // no push from here: no re-trip either
  const ContactStatus before = f.status();

  f.teardown();  // ArmLink's run() has returned before it tells the listener
  f.sup.on_link_state(LinkState::kConnecting);
  sleep_ms(50);
  f.sup.on_link_state(LinkState::kRunning);
  f.run_rt();
  sleep_ms(300);

  EXPECT_TRUE(f.mon.latched());
  const ContactStatus after = f.status();
  EXPECT_EQ(after.phase, ContactPhase::kLatched);
  EXPECT_EQ(after.last.cycle, before.last.cycle);  // the same latch, not a new trip
  EXPECT_EQ(f.sup.on_trajectory_goal(f.ramp_goal()), GoalResponse::kReject);
  f.sup.stop();
  f.teardown();
  EXPECT_EQ(f.sim.last_command().mode, ActuatorMode::kTorque);  // float, not the hold
}

TEST(ContactThresholds, NamedProfileApplied) {
  ContactFix f;
  EXPECT_EQ(f.sup.on_query_contact().profile, ContactProfile::kDefault);
  EXPECT_TRUE(f.sup.on_query_contact().enabled);
  ContactThresholdsRequest r;
  r.profile = ContactProfile::kSensitive;
  const ContactThresholdsResult res = f.sup.on_set_contact_thresholds(r);
  EXPECT_TRUE(res.accepted) << res.message;
  EXPECT_EQ(res.profile, ContactProfile::kSensitive);
  EXPECT_EQ(res.theta, kSensitive);
  EXPECT_EQ(f.sup.on_query_contact().profile, ContactProfile::kSensitive);

  r.profile = ContactProfile::kCustom;
  r.custom = (JointVec() << 9, 9, 7, 7, 3, 3, 3).finished();
  EXPECT_TRUE(f.sup.on_set_contact_thresholds(r).accepted);
  f.run_rt();
  ASSERT_TRUE(wait_for([&] { return f.status().theta == r.custom; }));  // the monitor adopted it
  EXPECT_EQ(f.sup.on_query_contact().profile, ContactProfile::kCustom);
  EXPECT_EQ(f.sup.on_query_contact().status.theta, r.custom);
}

// Review Focus 5: refused, the joint and the limit in the message, active θ unchanged.
TEST(ContactThresholds, CustomAboveCeilingRefused) {
  ContactFix f;
  f.run_rt();
  ContactThresholdsRequest r;
  r.profile = ContactProfile::kCustom;
  r.custom = kNormal;
  r.custom[4] = 7.5;  // j5 ceiling is 7
  const ContactThresholdsResult res = f.sup.on_set_contact_thresholds(r);
  EXPECT_FALSE(res.accepted);
  EXPECT_NE(res.message.find("joint 5"), std::string::npos) << res.message;
  EXPECT_NE(res.message.find("ceiling 7"), std::string::npos) << res.message;
  EXPECT_EQ(res.profile, ContactProfile::kDefault);  // what is still active
  EXPECT_EQ(res.theta, kNormal);
  sleep_ms(20);
  EXPECT_EQ(f.status().theta, kNormal);
  EXPECT_EQ(f.sup.on_query_contact().profile, ContactProfile::kDefault);
}

TEST(ContactThresholds, CustomNonFiniteRefused) {
  ContactFix f;
  ContactThresholdsRequest r;
  r.profile = ContactProfile::kCustom;
  r.custom = kNormal;
  r.custom[2] = std::numeric_limits<double>::quiet_NaN();
  ContactThresholdsResult res = f.sup.on_set_contact_thresholds(r);
  EXPECT_FALSE(res.accepted);
  EXPECT_NE(res.message.find("joint 3"), std::string::npos) << res.message;
  EXPECT_EQ(res.theta, kNormal);

  r.custom = kNormal;
  r.custom[6] = 0.0;  // <= 0
  res = f.sup.on_set_contact_thresholds(r);
  EXPECT_FALSE(res.accepted);
  EXPECT_NE(res.message.find("joint 7"), std::string::npos) << res.message;

  r.profile = static_cast<ContactProfile>(42);  // an out-of-enum byte
  res = f.sup.on_set_contact_thresholds(r);
  EXPECT_FALSE(res.accepted);
  EXPECT_NE(res.message.find("unknown contact profile"), std::string::npos) << res.message;
  EXPECT_EQ(f.sup.on_query_contact().profile, ContactProfile::kDefault);
}

TEST(ContactThresholds, UnauthorizedRefusedByArbiter) {
  ContactFix f;
  Arbiter arb{f.sup, f.sup, f.sup, ArbitrationMode::kEnforced, 1};
  ContactThresholdsRequest r;
  r.profile = ContactProfile::kLoose;
  ContactThresholdsResult res = arb.on_set_contact_thresholds(r);  // no grant, no token
  EXPECT_FALSE(res.accepted);
  EXPECT_EQ(res.message, "not authorized");
  EXPECT_EQ(res.profile, ContactProfile::kDefault);  // a refusal reports what is active
  EXPECT_EQ(res.theta, kNormal);
  EXPECT_EQ(arb.status().rejected_count, 1u);
  EXPECT_EQ(arb.on_query_contact().profile, ContactProfile::kDefault);  // reads are open

  r.token = arb.grant("tester").token;
  res = arb.on_set_contact_thresholds(r);
  EXPECT_TRUE(res.accepted) << res.message;
  EXPECT_EQ(arb.on_query_contact().profile, ContactProfile::kLoose);
}

TEST(ContactThresholds, CtorRejectsProfilesAboveCeiling) {
  ContactFix f;
  SupervisorDeps d = f.deps();
  d.contact_profiles.loose[0] = 31.0;  // j1 ceiling is 30
  try {
    Supervisor s{d};
    FAIL() << "constructed with a profile above the ceiling";
  } catch (const std::invalid_argument& e) {
    const std::string m = e.what();
    EXPECT_NE(m.find("loose"), std::string::npos) << m;
    EXPECT_NE(m.find("joint 1"), std::string::npos) << m;
    EXPECT_NE(m.find("ceiling 30"), std::string::npos) << m;
  }
  d = f.deps();
  d.contact_profiles.ceiling[3] = 25.0;  // not the monitor's ceiling
  EXPECT_THROW(Supervisor{d}, std::invalid_argument);
}

TEST(ContactThresholds, WithoutAMonitorRefused) {
  ContactFix f;
  SupervisorDeps d = f.deps();
  d.contact = nullptr;
  Supervisor s{d};
  const ContactThresholdsResult res = s.on_set_contact_thresholds({});
  EXPECT_FALSE(res.accepted);
  EXPECT_EQ(res.message, "contact detection not available");
  EXPECT_FALSE(s.on_query_contact().enabled);
  EXPECT_FALSE(s.clear_contact());
}
