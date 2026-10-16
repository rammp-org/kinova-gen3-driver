#include "kinova_lowlevel/interface/gains.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <utility>
#include <vector>

#include "../fixtures.h"
#include "kinova_lowlevel/dynamics.h"
using namespace kinova;
using namespace kinova::interface;

namespace {
JointImpedanceGainValues good() {
  JointImpedanceGainValues g;
  g.kq = (JointVec() << 80, 80, 80, 80, 30, 30, 30).finished();
  g.zeta = 0.5;
  g.torque_limit = (JointVec() << 39, 39, 39, 39, 9, 9, 9).finished();
  return g;
}
}  // namespace

TEST(GainsValidation, AcceptsTheModeDefaults) { EXPECT_TRUE(validate_custom(good()).ok); }

TEST(GainsValidation, RejectsZeroTorqueLimit) {
  // THE #64 shape: a zero-filled message default. The clamp would eat gravity
  // and the arm falls. Must be refused with a reason, never clamped.
  JointImpedanceGainValues g = good();
  g.torque_limit = JointVec::Zero();
  const GainsCheck c = validate_custom(g);
  EXPECT_FALSE(c.ok);
  EXPECT_FALSE(c.message.empty());
}

TEST(GainsValidation, RejectsTorqueLimitBelowGravityFloor) {
  JointImpedanceGainValues g = good();
  g.torque_limit = kTorqueLimitFloor * 0.5;
  EXPECT_FALSE(validate_custom(g).ok);
}

TEST(GainsValidation, RejectsNonFiniteAndNegativeFields) {
  JointImpedanceGainValues g = good();
  g.kq[2] = std::numeric_limits<double>::quiet_NaN();  // NaN survives std::clamp in compute()
  EXPECT_FALSE(validate_custom(g).ok);
  g = good();
  g.zeta = 0.0;  // undamped spring
  EXPECT_FALSE(validate_custom(g).ok);
  g = good();
  g.zeta = -0.3;  // anti-damped
  EXPECT_FALSE(validate_custom(g).ok);
  g = good();
  g.kq[0] = kKqMax * 2.0;
  EXPECT_FALSE(validate_custom(g).ok);
  g = good();
  g.kq[0] = 0.0;  // no spring at all is not impedance
  EXPECT_FALSE(validate_custom(g).ok);
}

TEST(GainsProfiles, EveryNamedEntryPassesItsOwnValidation) {
  for (GainsProfile p : {GainsProfile::kSoft, GainsProfile::kMedium, GainsProfile::kStiff}) {
    const JointImpedanceParams jp = profile_params(p);
    JointImpedanceGainValues g;
    g.kq = jp.Kq;
    g.zeta = jp.zeta;
    g.torque_limit = jp.torque_limit;
    EXPECT_TRUE(validate_custom(g).ok) << static_cast<int>(p);
  }
}

TEST(GainsProfiles, MediumMatchesTheModeDefaults) {
  // "Switch to impedance, change nothing else" must behave exactly like the
  // tuned constructor defaults everyone has been running.
  const JointImpedanceParams m = profile_params(GainsProfile::kMedium);
  const JointImpedanceParams d{};
  EXPECT_TRUE(m.Kq.isApprox(d.Kq));
  EXPECT_DOUBLE_EQ(m.zeta, d.zeta);
  EXPECT_TRUE(m.torque_limit.isApprox(d.torque_limit));
  EXPECT_DOUBLE_EQ(m.max_tracking_error, d.max_tracking_error);
}

TEST(GainsProfiles, NamelessKindsThrow) {
  EXPECT_THROW(profile_params(GainsProfile::kCustom), std::invalid_argument);
  EXPECT_THROW(profile_params(GainsProfile::kSessionDefault), std::invalid_argument);
}

TEST(GainsResolve, SessionDefaultReturnsTheSessionDefault) {
  const JointImpedanceParams def = profile_params(GainsProfile::kSoft);
  const JointImpedanceParams r = resolve_gains(ImpedanceGains{}, def);
  EXPECT_TRUE(r.Kq.isApprox(def.Kq));
  EXPECT_DOUBLE_EQ(r.max_tracking_error, def.max_tracking_error);
}

TEST(GainsResolve, CustomOverridesOnlyItsThreeFieldsOnTheSessionDefault) {
  // No-partial-reset guarantee: leash/ramp/ref-speed come from the session
  // default, NOT from a default-constructed params (the old mode-switch bug).
  const JointImpedanceParams def = profile_params(GainsProfile::kSoft);  // leash 0.45
  ImpedanceGains s;
  s.profile = GainsProfile::kCustom;
  s.custom = good();
  s.custom.zeta = 0.8;
  const JointImpedanceParams r = resolve_gains(s, def);
  EXPECT_DOUBLE_EQ(r.zeta, 0.8);
  EXPECT_TRUE(r.Kq.isApprox(s.custom.kq));
  EXPECT_TRUE(r.torque_limit.isApprox(s.custom.torque_limit));
  EXPECT_DOUBLE_EQ(r.max_tracking_error, def.max_tracking_error);
  EXPECT_DOUBLE_EQ(r.gain_ramp_s, def.gain_ramp_s);
}

namespace {
// max over q of |gravity(q)[i]| within the joint limits. A random sample only
// bounds a maximum from below -- 20k uniform samples once found 17.77 N*m where the
// true worst case is 17.90, enough to hide a floor below its stated margin -- so
// this samples to find good basins and then climbs each of the best ones,
// coordinate by coordinate, halving the step until it stalls. Fixed-seed
// mt19937, so every platform searches the same points.
double worst_gravity(Dynamics& dyn, int i, const JointVec& lo, const JointVec& hi) {
  JointVec g;
  auto f = [&](const JointVec& q) {
    dyn.gravity(q, g);
    return std::abs(g[i]);
  };
  std::mt19937 rng(42);
  std::vector<std::pair<double, JointVec>> starts;
  for (int n = 0; n < 2000; ++n) {
    JointVec q;
    for (int k = 0; k < kNumJoints; ++k)
      q[k] = std::uniform_real_distribution<double>(lo[k], hi[k])(rng);
    starts.emplace_back(f(q), q);
  }
  std::partial_sort(starts.begin(), starts.begin() + 8, starts.end(),
                    [](const auto& a, const auto& b) { return a.first > b.first; });
  double best = 0.0;
  for (int s = 0; s < 8; ++s) {
    auto [v, q] = starts[s];
    for (double step = 0.5; step > 1e-4; step /= 2) {
      for (bool improved = true; improved;) {
        improved = false;
        for (int k = 0; k < kNumJoints; ++k)
          for (double d : {step, -step}) {
            JointVec t = q;
            t[k] = std::clamp(t[k] + d, lo[k], hi[k]);
            if (const double w = f(t); w > v) v = w, q = t, improved = true;
          }
      }
    }
    best = std::max(best, v);
  }
  return best;
}
}  // namespace

TEST(GainsFloor, FloorsCoverWorstCaseGravityWithMargin) {
  // The floor constants are hard-coded; THIS test keeps them honest against the
  // URDF. Find the worst-case |g_i| on BOTH supported mounts -- level and the
  // 30 deg tilted plate (flatbot / gen_1_5) -- and require floor >= 1.05x the
  // larger and floor <= ceil. The tilt matters: on a level arm the base yaw
  // (index 0) has a vertical axis and gravity never loads it; tilted, it does.
  // (1.05, not more: index 1's worst case is 35.79 N*m against a 39 N*m effort
  // ceiling, so a fatter margin is physically unavailable.) On failure it prints
  // the worst case so the constant can be re-pinned after a model or mount change.
  Dynamics level(URDF_PATH);  // 2F-85 model: worst-case payload
  Dynamics tilted(UrdfXml{test_fixtures::tilted_fixture()}, "gen3_end_effector_link");
  JointVec lo, hi;
  level.joint_limits(lo, hi);
  for (int k = 0; k < kNumJoints; ++k) {  // continuous joints: one full turn
    if (!std::isfinite(lo[k])) lo[k] = -M_PI;
    if (!std::isfinite(hi[k])) hi[k] = M_PI;
  }
  JointVec max_g;
  for (int i = 0; i < kNumJoints; ++i)
    max_g[i] = std::max(worst_gravity(level, i, lo, hi), worst_gravity(tilted, i, lo, hi));
  // Pin the search itself: it must reach the independently found tilted worst
  // case at index 0 (17.904 N*m, Pinocchio hill-climb, 2026-10-16).
  EXPECT_GT(max_g[0], 17.9);
  for (int i = 0; i < kNumJoints; ++i) {
    EXPECT_GE(kTorqueLimitFloor[i], 1.05 * max_g[i])
        << "joint " << i << ": measured worst-case |gravity| = " << max_g[i];
    EXPECT_LE(kTorqueLimitFloor[i], kTorqueLimitCeil[i]);
  }
}

TEST(Gains, NamedProfileOverlaysGainsOntoTheSessionDefault) {
  // Profiles own the GAIN fields only. Returning profile_params raw from
  // resolve_gains swapped the whole struct, so the first profiled (or bare)
  // command silently reset every deployment-tuned non-gain field -- ik
  // limits, cmd_timeout_s, gain_ramp_s -- to struct defaults (review
  // finding).
  JointImpedanceParams base;  // stands in for a deployment-tuned default
  base.cmd_timeout_s = 0.777;
  base.gain_ramp_s = 1.25;
  base.max_ref_speed = JointVec::Constant(0.321);

  ImpedanceGains s;
  s.profile = GainsProfile::kSoft;
  const JointImpedanceParams out = resolve_gains(s, base);

  const JointImpedanceParams soft = profile_params(GainsProfile::kSoft);
  EXPECT_TRUE(out.Kq.isApprox(soft.Kq));
  EXPECT_DOUBLE_EQ(out.zeta, soft.zeta);
  EXPECT_DOUBLE_EQ(out.max_tracking_error, soft.max_tracking_error);
  EXPECT_TRUE(out.torque_limit.isApprox(soft.torque_limit));
  // ...and everything the deployment tuned survives.
  EXPECT_DOUBLE_EQ(out.cmd_timeout_s, 0.777);
  EXPECT_DOUBLE_EQ(out.gain_ramp_s, 1.25);
  EXPECT_TRUE(out.max_ref_speed.isApprox(JointVec::Constant(0.321)));
}
