// Payload compensation, grasp/release weighing and the impulse trip of ContactMonitor.
//
// The "measured" torque comes from a second Dynamics whose URDF carries the payload as a
// fixed link under the EE (Pinocchio lumps it into link 7, the way a held object is),
// plus a pose-dependent model error like the one measured on abra, minus any external
// torque. The monitor only ever sees the bare model. Mirrors test_contact.py on
// kinova-gen3-ros2 feat/collision-payload, which this is a port of.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include "fixtures.h"
#include "kinova_lowlevel/contact_monitor.h"
#include "kinova_lowlevel/dynamics.h"
using namespace kinova;
using test_fixtures::with_payload_link;

namespace {
constexpr double kDt = 1e-3;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr char kEe[] = "gen3_end_effector_link";

const JointVec kDefault = (JointVec() << 8, 8, 6, 6, 2, 2, 2).finished();
const JointVec kCeiling = (JointVec() << 30, 30, 22, 22, 7, 7, 7).finished();
const JointVec kImpulse = (JointVec() << 2, 2, 1.5, 1.5, 0.6, 0.6, 0.6).finished();
const JointVec kQ0 = (JointVec() << 0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57).finished();
const JointVec kLift = (JointVec() << 0.0, -0.06, 0.0, 0.08, 0.0, -0.02, 0.0).finished();
const Eigen::Vector3d kGrasp(0.0, 0.0, 0.10);  // between the 2F-85 fingertips, EE frame
constexpr double kBottle = 0.5;                // kg

// Observe counts every crossing and never latches; enforce latches on the first, so
// `last` is the first crossing (a bump crosses on several joints, and again as it ends).
ContactMonitorConfig config(bool impulse = true, ContactDetection d = ContactDetection::kObserve) {
  ContactMonitorConfig c;
  c.detection = d;
  c.initial_theta = kDefault;
  c.ceiling = kCeiling;
  c.impulse = impulse;
  c.impulse_theta = kImpulse;
  return c;
}

// q, qd, qdd at one instant.
struct State {
  JointVec q = kQ0, qd = JointVec::Zero(), qdd = JointVec::Zero();
};

// Min-jerk from a to b over [t0, t1], held outside it.
State minjerk(double t, double t0, double t1, const JointVec& a, const JointVec& b) {
  const double T = t1 - t0, s = std::clamp((t - t0) / T, 0.0, 1.0);
  const bool in = t >= t0 && t <= t1;
  State x;
  x.q = a + (b - a) * (10 * std::pow(s, 3) - 15 * std::pow(s, 4) + 6 * std::pow(s, 5));
  x.qd = in ? JointVec((b - a) * ((30 * s * s - 60 * std::pow(s, 3) + 30 * std::pow(s, 4)) / T))
            : JointVec::Zero();
  x.qdd = in ? JointVec((b - a) * ((60 * s - 180 * s * s + 120 * std::pow(s, 3)) / (T * T)))
             : JointVec::Zero();
  return x;
}

// A brisk sweep on 2, 4, 6 and the wrist roll from q_start, from rest at t0.
State carry(double t, double t0, const JointVec& q_start) {
  const JointVec A = (JointVec() << 0, 0.3, 0, 0.4, 0, 0.4, 0.8).finished();
  const double w = M_PI, s = std::max(0.0, t - t0);
  State x;
  x.q = q_start + A * ((1 - std::cos(w * s)) / 2);
  x.qd = t >= t0 ? JointVec(A * (w * std::sin(w * s) / 2)) : JointVec::Zero();
  x.qdd = t >= t0 ? JointVec(A * (w * w * std::cos(w * s) / 2)) : JointVec::Zero();
  return x;
}

// Model error the shape of abra's: 4 N·m offset on the shoulder, pose-dependent parts.
JointVec bias(const JointVec& q) {
  JointVec b = JointVec::Zero();
  b[1] = 1.5 * std::sin(q[3]) + 4.0;
  b[3] = 1.0 * std::sin(q[1]);
  return b;
}

class Rig {
 public:
  explicit Rig(ContactMonitorConfig cfg, double mass = kBottle)
      : dyn_(URDF_PATH, kEe),
        held_(UrdfXml{with_payload_link(mass, kGrasp)}, kEe),
        mon_(dyn_, cfg) {}

  // load in [0, 1] scales the held object in (lift-off); f is a world force on it;
  // tau_e is external joint torque applied directly.
  JointFeedback feedback(const State& x, double load, const Eigen::Vector3d& f,
                         const JointVec& tau_e = JointVec::Zero()) {
    const JointVec bare = id(dyn_, x), full = id(held_, x);
    JointFeedback fb;
    fb.q = x.q;
    fb.qd = x.qd;
    fb.tau = bare + load * (full - bare) + bias(x.q) - force_torque(x.q, f) - tau_e;
    return fb;
  }
  bool step(const State& x, double load = 0.0, const Eigen::Vector3d& f = Eigen::Vector3d::Zero(),
            const JointVec& tau_e = JointVec::Zero()) {
    ++k_;
    return mon_.step(feedback(x, load, f, tau_e), kDt, false);
  }
  double t() const { return k_ * kDt; }
  ContactStatus status() const {
    ContactStatus st;
    EXPECT_TRUE(mon_.status(st));
    return st;
  }
  ContactMonitor& mon() { return mon_; }
  Dynamics& dyn() { return dyn_; }

 private:
  static JointVec id(Dynamics& d, const State& x) {
    JointMat M, C;
    JointVec g;
    d.mass_matrix(x.q, M);
    d.coriolis(x.q, x.qd, C);
    d.gravity(x.q, g);
    return M * x.qdd + C * x.qd + g;
  }
  // Joint torque of world force f on the held object at kGrasp: Jc^T f.
  JointVec force_torque(const JointVec& q, const Eigen::Vector3d& f) {
    if (f.isZero()) return JointVec::Zero();
    Jacobian6 J;
    dyn_.jacobian(q, J);
    const Eigen::Vector3d rc = dyn_.fk(q).R * kGrasp;
    Eigen::Matrix3d rx;
    rx << 0, -rc.z(), rc.y(), rc.z(), 0, -rc.x(), -rc.y(), rc.x(), 0;
    const Eigen::Matrix<double, 3, kNumJoints> Jc = J.topRows<3>() - rx * J.bottomRows<3>();
    return Jc.transpose() * f;
  }

  Dynamics dyn_, held_;
  ContactMonitor mon_;
  int k_ = 0;
};

// The pick of test_contact.py: still at kQ0, the gripper closes, the lift (the object
// leaves the table over 0.1 s early in it), still, then the carry from 2.8 s.
State pick_state(double t) {
  if (t < 2.8) return minjerk(t, 1.5, 2.0, kQ0, kQ0 + kLift);
  return carry(t, 2.8, kQ0 + kLift);
}
double pick_load(double t) { return std::clamp((t - 1.55) / 0.1, 0.0, 1.0); }

// Runs the pick to t_end; begin at 0.9 s, end at 2.1 s if `declare`. A force `f` on the
// object in [f0, f1). Returns the would-trip count.
uint64_t run_pick(Rig& rig, double t_end, bool declare, double f0 = -1, double f1 = -1,
                  const Eigen::Vector3d& f = Eigen::Vector3d::Zero()) {
  while (rig.t() < t_end) {
    const double t = rig.t();
    if (declare && std::abs(t - 0.9) < kDt / 2) rig.mon().request_load_change_begin();
    if (declare && std::abs(t - 2.1) < kDt / 2) rig.mon().request_load_change_end(kGrasp);
    rig.step(pick_state(t), pick_load(t), (t >= f0 && t < f1) ? f : Eigen::Vector3d::Zero());
  }
  return rig.status().would_trips;
}
}  // namespace

TEST(TipPayloadRegressor, MatchesAPayloadLinkInTheUrdf) {
  const Eigen::Vector3d c(0.01, -0.02, 0.12);
  Dynamics bare(URDF_PATH, kEe), held(UrdfXml{with_payload_link(0.7, c)}, kEe);
  Eigen::Vector4d theta;
  theta << 0.7, 0.7 * c;
  for (int k = 0; k < 5; ++k) {
    const JointVec q =
        kQ0 + JointVec::Constant(0.3 * k - 0.6)
                  .cwiseProduct((JointVec() << 1, -1, 0.5, 1, -0.7, 1, 0.3).finished());
    JointVec gb, gh;
    bare.gravity(q, gb);
    held.gravity(q, gh);
    TipRegressor Y;
    bare.tip_payload_regressor(q, Y);
    EXPECT_LT((gh - gb - Y * theta).cwiseAbs().maxCoeff(), 1e-9) << k;
  }
}

TEST(ContactPayload, DeclaredPayloadReadsAsNoExternalTorque) {
  Rig rig(config(), 1.5);
  State still;
  for (int k = 0; k < 400; ++k) rig.step(still, 1.0);
  const ContactStatus before = rig.status();
  EXPECT_GT((before.r + bias(still.q)).cwiseAbs().maxCoeff(), 3.0);  // 1.5 kg reads as a push

  TipPayload p;
  p.mass = 1.5;
  p.com = kGrasp;
  ASSERT_TRUE(validate_tip_payload(p, 4.0).ok);
  rig.mon().set_payload(p);
  for (int k = 0; k < 100; ++k) rig.step(still, 1.0);
  const ContactStatus st = rig.status();
  EXPECT_NEAR(st.payload.mass, 1.5, 1e-12);
  // What is left is the model error alone, as with nothing held.
  EXPECT_LT((st.r + bias(still.q)).cwiseAbs().maxCoeff(), 0.01);
  // Adopting it moved the slow average too: the jump in r is not an impulse.
  EXPECT_EQ(st.would_trips, before.would_trips);
}

TEST(ContactPayload, WeighsTheBottleThroughTheModelError) {
  Rig rig(config());
  run_pick(rig, 2.6, true);
  const ContactStatus st = rig.status();
  EXPECT_EQ(st.weigh.outcome, WeighOutcome::kOk);
  EXPECT_EQ(st.load_change, LoadChange::kNone);
  // 4 N·m of shoulder bias, larger than the bottle's own torque; the tare cancels it.
  EXPECT_NEAR(st.payload.mass, kBottle, 0.05);
  EXPECT_NEAR(st.weigh.delta_kg, kBottle, 0.05);
  EXPECT_LT(st.weigh.fit_rms, 0.1);
  EXPECT_EQ(st.would_trips, 0u);
}

TEST(ContactPayload, PickCarryAndCatchABump) {
  // The headline: weigh a bottle, carry it at speed, someone bumps it from the side.
  Rig rig(config(true, ContactDetection::kEnforce));
  EXPECT_EQ(run_pick(rig, 6.0, true), 0u);  // the grasp and 3 s of carrying: quiet
  run_pick(rig, 6.3, true, 6.0, 6.15, Eigen::Vector3d(25, 0, 0));
  const ContactStatus st = rig.status();
  EXPECT_EQ(st.phase, ContactPhase::kLatched);
  ASSERT_TRUE(st.has_trip);
  EXPECT_GE(st.last.cycle, 6000u);
  EXPECT_LE(st.last.cycle, 6030u);  // within 30 ms of first contact
}

TEST(ContactPayload, UndeclaredGraspTripsTheImpulse) {
  // Why the load-change requests exist: the object leaving the table is a fast step.
  Rig rig(config());
  run_pick(rig, 2.6, false);
  const ContactStatus st = rig.status();
  EXPECT_GE(st.would_trips, 1u);
  EXPECT_TRUE(st.last.impulse);
}

TEST(ContactPayload, BumpDuringTheGraspStillTrips) {
  // The projection removes what a held load can do, not everything.
  Rig rig(config(true, ContactDetection::kEnforce));
  run_pick(rig, 2.05, true, 1.8, 1.95, Eigen::Vector3d(25, 0, 0));
  const ContactStatus st = rig.status();
  EXPECT_EQ(st.load_change, LoadChange::kHolding);
  EXPECT_EQ(st.phase, ContactPhase::kLatched);
  ASSERT_TRUE(st.has_trip);
  EXPECT_GE(st.last.cycle, 1800u);
  EXPECT_LE(st.last.cycle, 1830u);
}

TEST(ContactPayload, ReleaseWeighsBackToEmpty) {
  Rig rig(config());
  run_pick(rig, 2.6, true);
  ASSERT_NEAR(rig.status().payload.mass, kBottle, 0.05);
  // Put it down: lower onto the table, still, begin, the table takes it, end.
  const double t0 = rig.t();
  while (rig.t() < t0 + 0.8) rig.step(minjerk(rig.t(), t0, t0 + 0.5, kQ0 + kLift, kQ0), 1.0);
  const State down;  // at rest at kQ0
  rig.mon().request_load_change_begin();
  for (int k = 0; k < 400; ++k) rig.step(down, 1.0);
  ASSERT_EQ(rig.status().load_change, LoadChange::kHolding);
  for (int k = 0; k < 100; ++k) rig.step(down, 1.0 - k / 100.0);
  rig.mon().request_load_change_end(kGrasp);
  for (int k = 0; k < 400; ++k) rig.step(down, 0.0);
  const ContactStatus st = rig.status();
  EXPECT_EQ(st.weigh.outcome, WeighOutcome::kOk);
  EXPECT_EQ(st.payload.mass, 0.0);
  EXPECT_NEAR(st.weigh.delta_kg, -kBottle, 0.05);
  EXPECT_EQ(st.would_trips, 0u);
}

// A hand leans on the elbow over 2 s to 2.5 N·m and stays; at 4.5 s a sharp 3 N·m tap
// lands on top. The peak, 5.5 N·m, stays under θ = 6 on joint 4. The fast band shows
// about 2/3 of a 150 ms step at K = 25 and a 0.2 s average: ~2 N·m, over 1.5.
JointVec lean_and_tap(double t) {
  JointVec e = JointVec::Zero();
  e[3] = 2.5 * std::clamp((t - 1.0) / 2.0, 0.0, 1.0) + (t >= 4.5 && t < 4.65 ? 3.0 : 0.0);
  return e;
}

TEST(ContactPayload, LeanDoesNotTripTheImpulseATapDoes) {
  Rig rig(config());
  const State still;
  while (rig.t() < 4.5) rig.step(still, 0.0, Eigen::Vector3d::Zero(), lean_and_tap(rig.t()));
  EXPECT_EQ(rig.status().would_trips, 0u);
  while (rig.t() < 5.0) rig.step(still, 0.0, Eigen::Vector3d::Zero(), lean_and_tap(rig.t()));
  const ContactStatus st = rig.status();
  EXPECT_EQ(st.would_trips, 1u);
  EXPECT_EQ(st.last.joint, 3);
  EXPECT_TRUE(st.last.impulse);
  EXPECT_GE(st.last.cycle, 4500u);
  EXPECT_LE(st.last.cycle, 4540u);  // the observer's 40 ms time constant, then 5 ms debounce
}

TEST(ContactPayload, TheLevelTripAloneMissesTheSameTap) {
  Rig rig(config(false));
  const State still;
  while (rig.t() < 5.0) rig.step(still, 0.0, Eigen::Vector3d::Zero(), lean_and_tap(rig.t()));
  EXPECT_EQ(rig.status().would_trips, 0u);
}

TEST(ContactPayload, NotStillAtTareAndEndWithoutBegin) {
  Rig rig(config());
  for (int k = 0; k < 400; ++k) rig.step(State{});
  rig.mon().request_load_change_end(kGrasp);
  rig.step(State{});
  EXPECT_EQ(rig.status().weigh.outcome, WeighOutcome::kNoLoadChange);

  rig.mon().request_load_change_begin();
  const double t0 = rig.t();
  while (rig.t() < t0 + 0.4) rig.step(minjerk(rig.t(), t0, t0 + 0.4, kQ0, kQ0 + kLift));
  const ContactStatus st = rig.status();
  EXPECT_EQ(st.weigh.outcome, WeighOutcome::kNotStillAtTare);
  EXPECT_EQ(st.load_change, LoadChange::kNone);
}

TEST(ContactPayload, PressingDuringTheWeighIsNotALoad) {
  // Someone holds the forearm sideways while the arm "weighs": no payload explains it.
  Rig rig(config(false));
  const State still;
  for (int k = 0; k < 400; ++k) rig.step(still);
  rig.mon().request_load_change_begin();
  for (int k = 0; k < 400; ++k) rig.step(still);
  JointVec e = JointVec::Zero();
  e[0] = 3.0;  // joint 1 about the vertical: no gravity load can do this
  e[2] = 3.0;
  rig.mon().request_load_change_end(kGrasp);
  for (int k = 0; k < 400; ++k) rig.step(still, 0.0, Eigen::Vector3d::Zero(), e);
  const ContactStatus st = rig.status();
  EXPECT_EQ(st.weigh.outcome, WeighOutcome::kUnexplained);
  EXPECT_EQ(st.load_change, LoadChange::kHolding);
  EXPECT_EQ(st.payload.mass, 0.0);
}

TEST(ContactPayload, Validation) {
  TipPayload p;
  p.mass = 0.5;
  p.com = kGrasp;
  EXPECT_TRUE(validate_tip_payload(p, 4.0).ok);
  p.mass = -0.1;
  EXPECT_FALSE(validate_tip_payload(p, 4.0).ok);
  p.mass = 5.0;
  const ThresholdCheck heavy = validate_tip_payload(p, 4.0);
  EXPECT_FALSE(heavy.ok);
  EXPECT_NE(heavy.message.find("4 kg"), std::string::npos) << heavy.message;
  p.mass = 0.5;
  p.com = Eigen::Vector3d(0, 0, kNaN);
  EXPECT_FALSE(validate_tip_payload(p, 4.0).ok);

  Dynamics dyn(URDF_PATH, kEe);
  ContactMonitorConfig c = config();
  c.impulse_theta[4] = kNaN;
  EXPECT_THROW(ContactMonitor(dyn, c), std::invalid_argument);
  c.impulse = false;  // unused when off
  EXPECT_NO_THROW(ContactMonitor(dyn, c));
  c = config();
  c.slow_tau_s = 0.0;
  EXPECT_THROW(ContactMonitor(dyn, c), std::invalid_argument);
  c = config();
  c.weigh_window_cycles = 0;
  EXPECT_THROW(ContactMonitor(dyn, c), std::invalid_argument);
}
