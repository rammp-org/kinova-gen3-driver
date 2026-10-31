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
#include "kinova_lowlevel/cartesian.h"  // ee_wrench_from_residual
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

// The threshold profiles (N·m, j1..j7).
const JointVec kSensitive = (JointVec() << 8, 8, 6, 6, 2, 2, 2).finished();
const JointVec kNormal = (JointVec() << 16, 16, 12, 12, 4, 4, 4).finished();
const JointVec kLoose = (JointVec() << 24, 24, 18, 18, 6, 6, 6).finished();
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

  // Push on j2 (over the default θ of 16 N·m) until the monitor latches.
  void trip() {
    sim.set_external_torque(push_j2(24.0));
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

// The pump end of the /ee_state wrench, without the RT loop: the test thread is
// the monitor's RT thread (on_run_start + `cycles` 1 ms steps at rest under
// `tau_ext`), then stores `fb` for the pump and returns what it published. The
// observer saw tau_ext and the pump sees fb, so the two estimates differ and the
// published value shows which one was used.
ArmState publish_after(ContactFix& f, const JointVec& tau_ext, int cycles,
                       const JointFeedback& fb) {
  JointFeedback seen = test_fixtures::at_rest(f.q0, test_fixtures::gravity(f.sim_dyn, f.q0));
  seen.tau -= tau_ext;
  f.mon.on_run_start();
  for (int i = 0; i < cycles; ++i) f.mon.step(seen, 0.001, false);
  // Before start: the pump's first iteration must see both this sample and the
  // monitor's last status as new (nothing steps the monitor after this).
  f.snap.store(fb);
  f.sup.start();
  sleep_ms(80);
  const ArmState s = f.sup.on_query_state();
  f.sup.stop();
  EXPECT_GT(s.stamp_s, 0.0) << "no pump tick landed";
  return s;
}

// ee_wrench_from_residual on the pump's inputs: J and g at fb.q.
Vector6 wrench_of(const JointVec& q, const JointVec& residual) {
  Dynamics d{URDF_PATH};
  Jacobian6 J;
  d.jacobian(q, J);
  return ee_wrench_from_residual(J, residual);
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

// /ee_state's wrench (#112). Once the observer is trusted (armed or latched) the
// pump publishes ee_wrench_from_residual(J, r) from the monitor's status; before
// that, or with no monitor (supervisor_test.cpp), the quasi-static g(q) - tau.
// 600 cycles = the 5/K = 200 ms settle plus 400 ms (10/K) of convergence.
TEST(ContactWrench, ArmedPublishesTheObserverResidual) {
  ContactFix f;
  const JointFeedback fb = test_fixtures::at_rest(f.q0, test_fixtures::gravity(f.sim_dyn, f.q0));
  const ArmState s = publish_after(f, push_j2(8.0), 600, fb);  // under θ: armed, not latched
  const ContactStatus st = f.status();
  ASSERT_EQ(st.phase, ContactPhase::kArmed);
  EXPECT_TRUE(s.ee_wrench.isApprox(wrench_of(fb.q, st.r), 1e-12))
      << s.ee_wrench.transpose() << " != " << wrench_of(fb.q, st.r).transpose();
  // fb carries no push, so the quasi-static estimate would be ~0.
  EXPECT_GT(s.ee_wrench.norm(), 1.0);
}

TEST(ContactWrench, LatchedPublishesTheObserverResidual) {
  ContactFix f;
  const JointFeedback fb = test_fixtures::at_rest(f.q0, test_fixtures::gravity(f.sim_dyn, f.q0));
  const ArmState s = publish_after(f, push_j2(24.0), 600, fb);  // over θ: latches
  const ContactStatus st = f.status();
  ASSERT_EQ(st.phase, ContactPhase::kLatched);
  EXPECT_TRUE(s.ee_wrench.isApprox(wrench_of(fb.q, st.r), 1e-12));
  EXPECT_GT(s.ee_wrench.norm(), 1.0);
}

TEST(ContactWrench, SettlingPublishesQuasiStatic) {
  ContactFix f;
  JointFeedback fb = test_fixtures::at_rest(f.q0, test_fixtures::gravity(f.sim_dyn, f.q0));
  fb.tau -= push_j2(8.0);
  const ArmState s = publish_after(f, push_j2(8.0), 10, fb);  // 10 ms into the 200 ms settle
  ASSERT_EQ(f.status().phase, ContactPhase::kSettling);
  EXPECT_TRUE(s.ee_wrench.isApprox(wrench_of(fb.q, push_j2(8.0)), 1e-9))
      << s.ee_wrench.transpose() << " != " << wrench_of(fb.q, push_j2(8.0)).transpose();
}

// A run that restarts latched (no clear) re-settles its observer: r restarts
// from 0 and is not trusted for 5/K, though the phase stays kLatched.
TEST(ContactWrench, LatchedRestartPublishesQuasiStaticUntilSettled) {
  ContactFix f;
  JointFeedback pushed = test_fixtures::at_rest(f.q0, test_fixtures::gravity(f.sim_dyn, f.q0));
  pushed.tau -= push_j2(24.0);
  f.mon.on_run_start();
  for (int i = 0; i < 600; ++i) f.mon.step(pushed, 0.001, false);
  ASSERT_TRUE(f.mon.latched());
  const ArmState s = publish_after(f, push_j2(24.0), 10, pushed);  // 10 ms into the re-settle
  ASSERT_EQ(f.status().phase, ContactPhase::kLatched);
  EXPECT_TRUE(s.ee_wrench.isApprox(wrench_of(pushed.q, push_j2(24.0)), 1e-9));
}

// The RT loop stopped (ArmLink in kFaulted keeps feedback flowing, but nothing
// steps the monitor): its last status is frozen, so the pump falls back. The
// monitor's last step comes after the last publish, as when feedback stops
// before the loop does: that status is no fresher at the next publish.
TEST(ContactWrench, StalledMonitorPublishesQuasiStatic) {
  ContactFix f;
  JointFeedback seen = test_fixtures::at_rest(f.q0, test_fixtures::gravity(f.sim_dyn, f.q0));
  seen.tau -= push_j2(8.0);
  f.mon.on_run_start();
  for (int i = 0; i < 600; ++i) f.mon.step(seen, 0.001, false);
  // The pump sees no push, the observer a push: the value says which was used.
  const JointFeedback fb = test_fixtures::at_rest(f.q0, test_fixtures::gravity(f.sim_dyn, f.q0));
  f.snap.store(fb);  // before start, as in publish_after
  f.sup.start();
  sleep_ms(80);
  const ArmState live = f.sup.on_query_state();
  f.mon.step(seen, 0.001, false);  // the last step, with no feedback published after it
  sleep_ms(80);
  f.snap.store(fb);  // fresh feedback, no monitor step since
  sleep_ms(80);
  const ArmState stalled = f.sup.on_query_state();
  f.sup.stop();

  EXPECT_GT(live.ee_wrench.norm(), 1.0) << "live: not the observer";
  EXPECT_LT(stalled.ee_wrench.norm(), 1e-9) << stalled.ee_wrench.transpose();
}

// A fault is NaN whichever estimate is selected, as before #112.
TEST(ContactWrench, FaultPublishesNaN) {
  ContactFix f;
  JointFeedback fb = test_fixtures::at_rest(f.q0, test_fixtures::gravity(f.sim_dyn, f.q0));
  fb.fault = true;
  const ArmState s = publish_after(f, push_j2(8.0), 600, fb);
  ASSERT_EQ(f.status().phase, ContactPhase::kArmed);
  EXPECT_TRUE(s.fault);
  EXPECT_TRUE(s.ee_wrench.array().isNaN().all()) << s.ee_wrench.transpose();
}

// End to end on the RT loop: a static push read both ways. Both are "external
// torque, positive as applied by the environment": the sim reports
// tau = g - tau_ext, the quasi-static g - tau is +tau_ext, and the observer
// converges to the same +tau_ext.
TEST(ContactWrench, EstimatesAgreeOnAStaticPush) {
  ContactFix f;
  f.sim.set_external_torque(push_j2(8.0));  // under θ: no trip
  f.sup.start();
  f.run_rt();
  ASSERT_TRUE(wait_for([&] {
    const ContactStatus st = f.status();
    return st.phase == ContactPhase::kArmed && std::abs(st.r[1] - 8.0) < 1e-4;
  })) << "observer never converged";
  sleep_ms(50);  // a pump tick after convergence
  const ArmState s = f.sup.on_query_state();
  const ContactStatus st = f.status();
  f.sup.stop();
  f.teardown();

  ASSERT_EQ(st.phase, ContactPhase::kArmed);
  const JointVec quasi_static = test_fixtures::gravity(f.sim_dyn, s.q) - s.tau;
  EXPECT_NEAR(quasi_static[1], 8.0, 1e-6);
  EXPECT_NEAR(st.r[1], 8.0, 1e-3);  // same sign: the observer reads +push too
  const Vector6 qs_wrench = wrench_of(s.q, quasi_static);
  EXPECT_GT(qs_wrench.norm(), 1.0);
  EXPECT_LT((s.ee_wrench - qs_wrench).norm(), 1e-2)
      << "observer " << s.ee_wrench.transpose() << " vs quasi-static " << qs_wrench.transpose();
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

// The wire numbering (SetContactThresholds.PROFILE_*): a request whose profile was
// never set (0) gets the default profile, never the most sensitive one.
TEST(ContactThresholds, ProfileZeroIsDefault) {
  static_assert(static_cast<int>(ContactProfile::kDefault) == 0, "PROFILE_DEFAULT");
  static_assert(static_cast<int>(ContactProfile::kSensitive) == 1, "PROFILE_SENSITIVE");
  static_assert(static_cast<int>(ContactProfile::kLoose) == 2, "PROFILE_LOOSE");
  static_assert(static_cast<int>(ContactProfile::kCustom) == 3, "PROFILE_CUSTOM");
  ContactThresholdsRequest r;
  r.profile = static_cast<ContactProfile>(0);
  JointVec theta = JointVec::Zero();
  ASSERT_TRUE(resolve_contact(r, profiles(), theta).ok);
  EXPECT_EQ(theta, kNormal);
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
