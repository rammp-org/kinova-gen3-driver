#include "kinova_lowlevel/velocity_reference.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

#include "kinova_lowlevel/dynamics.h"
using namespace kinova;

namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();

struct Fix {
  JointVec q_lower = JointVec::Constant(-10.0);
  JointVec q_upper = JointVec::Constant(10.0);
  std::array<bool, kNumJoints> continuous{};  // all bounded by default
};
}  // namespace

TEST(VelocityReference, IntegratesTheVelocityIntoTheReference) {
  Fix f;
  JointVec q_ref = JointVec::Zero();
  const JointVec q_meas = JointVec::Zero();
  const JointVec qd = JointVec::Constant(0.5);
  const double dt = 0.001;
  for (int i = 0; i < 100; ++i)  // 100 ms at 0.5 rad/s -> 0.05 rad, well inside the leash
    integrate_leashed_reference(q_ref, qd, dt, q_meas, kVelocityRefMaxLead, f.continuous,
                                f.q_lower, f.q_upper);
  EXPECT_NEAR(q_ref[0], 0.05, 1e-9);
}

TEST(VelocityReference, LeashCapsTheLeadWhenTheMeasurementLags) {
  // The windup guard: measured q never moves (blocked joint), so the reference
  // may lead it by at most the leash, no matter how long the command persists.
  Fix f;
  JointVec q_ref = JointVec::Zero();
  const JointVec q_meas = JointVec::Zero();
  const JointVec qd = JointVec::Constant(1.0);
  for (int i = 0; i < 1000; ++i)  // 1 s at 1 rad/s would be 1.0 rad unleashed
    integrate_leashed_reference(q_ref, qd, 0.001, q_meas, kVelocityRefMaxLead, f.continuous,
                                f.q_lower, f.q_upper);
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(q_ref[i], kVelocityRefMaxLead, 1e-9);
}

TEST(VelocityReference, ClampsABoundedJointAtItsLimit) {
  Fix f;
  f.q_upper[0] = 0.03;  // tighter than the leash
  JointVec q_ref = JointVec::Zero();
  JointVec q_meas = JointVec::Zero();
  const JointVec qd = JointVec::Constant(1.0);
  for (int i = 0; i < 200; ++i) {
    integrate_leashed_reference(q_ref, qd, 0.001, q_meas, kVelocityRefMaxLead, f.continuous,
                                f.q_lower, f.q_upper);
    q_meas = q_ref;  // tracking perfectly, so only the limit can stop it
  }
  EXPECT_NEAR(q_ref[0], 0.03, 1e-9);
  EXPECT_GT(q_ref[1], 0.1);  // the unclamped joints kept going
}

TEST(VelocityReference, AContinuousJointTakesTheShortWayAcrossTheWrap) {
  // Reference just past +pi wraps to just above -pi; the lead against a
  // measurement at +3.1 must read as small, not as ~2*pi.
  Fix f;
  f.continuous[0] = true;
  f.q_lower[0] = -kInf;
  f.q_upper[0] = kInf;
  JointVec q_ref = JointVec::Constant(0.0);
  q_ref[0] = 3.14;
  JointVec q_meas = JointVec::Constant(0.0);
  q_meas[0] = 3.14;
  JointVec qd = JointVec::Zero();
  qd[0] = 0.5;
  for (int i = 0; i < 100; ++i) {  // +0.05 rad, crossing pi
    integrate_leashed_reference(q_ref, qd, 0.001, q_meas, kVelocityRefMaxLead, f.continuous,
                                f.q_lower, f.q_upper);
    q_meas[0] = q_ref[0];  // tracking
  }
  // Stays in (-pi, pi] and advanced ~0.05 rad around the circle.
  EXPECT_LE(q_ref[0], M_PI);
  EXPECT_NEAR(q_ref[0], wrap_to_pi(3.14 + 0.05), 1e-6);
}

TEST(TwistDls, ReproducesTheCommandedTwistAwayFromSingularities) {
  Dynamics dyn(URDF_PATH);
  const JointVec q = (JointVec() << 0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57).finished();
  Jacobian6 J;
  dyn.jacobian(q, J);
  Vector6 V = Vector6::Zero();
  V[0] = 0.05;  // 5 cm/s along x
  TwistDlsParams p;
  p.posture_gain = 0.0;  // task term alone for this check
  TwistDlsSolver s;
  JointVec qd = JointVec::Zero();
  std::array<bool, kNumJoints> cont{};
  s.solve(J, q, V, p, cont, qd);
  EXPECT_GT(qd.cwiseAbs().maxCoeff(), 0.0);
  EXPECT_LT((J * qd - V).norm(), 1e-3);  // achieved twist matches the command
  EXPECT_GT(s.last_manipulability(), 0.0);
}

TEST(TwistDls, PostureBiasActsOnlyInTheNullSpace) {
  Dynamics dyn(URDF_PATH);
  const JointVec q = (JointVec() << 0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57).finished();
  Jacobian6 J;
  dyn.jacobian(q, J);
  TwistDlsParams p;  // default posture_gain pulls toward q_rest
  p.q_rest = q + JointVec::Constant(0.3);
  TwistDlsSolver s;
  JointVec qd = JointVec::Zero();
  std::array<bool, kNumJoints> cont{};
  s.solve(J, q, Vector6::Zero(), p, cont, qd);
  EXPECT_GT(qd.cwiseAbs().maxCoeff(), 0.0);   // the bias is doing something...
  EXPECT_LT((J * qd).norm(), 1e-3);           // ...without moving the end effector
  EXPECT_GT(qd.dot(p.q_rest - q), 0.0);       // and it points toward the rest posture
}
