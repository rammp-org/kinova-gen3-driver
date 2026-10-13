#include "kinova_lowlevel/cartesian.h"

#include <gtest/gtest.h>

#include <cmath>

#include "kinova_lowlevel/dynamics.h"
using namespace kinova;

TEST(PoseError, IdenticalPosesGiveZero) {
  Pose a;
  a.p = Eigen::Vector3d(0.1, -0.2, 0.3);
  a.R = Eigen::Quaterniond(Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitY()));
  Vector6 e = pose_error(a, a);
  EXPECT_NEAR(e.norm(), 0.0, 1e-12);
}

TEST(PoseError, PureTranslationIsDesiredMinusCurrent) {
  Pose cur, des;
  cur.p = Eigen::Vector3d(0, 0, 0);
  des.p = Eigen::Vector3d(0.05, -0.10, 0.02);
  Vector6 e = pose_error(des, cur);
  EXPECT_NEAR((e.head<3>() - des.p).norm(), 0.0, 1e-12);
  EXPECT_NEAR(e.tail<3>().norm(), 0.0, 1e-12);
}

TEST(PoseError, PureRotationAboutZ) {
  Pose cur, des;
  const double ang = 0.3;
  des.R = Eigen::Quaterniond(Eigen::AngleAxisd(ang, Eigen::Vector3d::UnitZ()));
  Vector6 e = pose_error(des, cur);
  EXPECT_NEAR(e.head<3>().norm(), 0.0, 1e-12);
  EXPECT_NEAR(e[3], 0.0, 1e-9);
  EXPECT_NEAR(e[4], 0.0, 1e-9);
  EXPECT_NEAR(e[5], ang, 1e-9);
}

TEST(PoseError, LargeRotationTakesShortestPath) {
  // A 1.8*pi rotation about +X has a quaternion with w<0; pose_error must report
  // it as the equivalent short -0.2*pi arc (axis flips to -X), exercising the
  // shortest-geodesic branch. (At q=1.8pi: half-angle=0.9pi, w=cos(0.9pi)<0 ->
  // flip -> angle=0.2pi about -X -> rotvec[0] = -0.2pi.)
  // Rotation lives in e.tail<3>() = e[3..5]; e[3] is the X component.
  Pose cur, des;
  des.R = Eigen::Quaterniond(Eigen::AngleAxisd(1.8 * M_PI, Eigen::Vector3d::UnitX()));
  Vector6 e = pose_error(des, cur);
  EXPECT_NEAR(e.head<3>().norm(), 0.0, 1e-12);  // pure rotation, no translation
  EXPECT_NEAR(e[3], -0.2 * M_PI, 1e-9);         // short arc, not +1.8*pi
  EXPECT_NEAR(e[4], 0.0, 1e-9);
  EXPECT_NEAR(e[5], 0.0, 1e-9);
}

// ---- ee_wrench_from_residual ------------------------------------------------
// The map (J, tau_ext) -> F through the damped pseudoinverse of J^T. J comes from
// the real Gen3 model so the tests exercise the actual 6x7 geometry, including a
// genuinely singular pose -- a synthetic J would prove less.

namespace {
// Elbow-up home (DiffIkParams::q_rest): far from singular, the pose the arm
// actually works around.
JointVec well_conditioned_q() {
  return (JointVec() << 0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57).finished();
}
}  // namespace

TEST(EeWrench, RecoversAWrenchAppliedThroughJt) {
  Dynamics dyn{URDF_PATH};
  Jacobian6 J;
  dyn.jacobian(well_conditioned_q(), J);
  Vector6 F;  // environment-on-tool: push along +x/-y/+z plus a twist
  F << 5.0, -3.0, 8.0, 0.4, -0.2, 0.6;
  const JointVec tau_ext = J.transpose() * F;
  // Near-zero damping: this tests the SOLVE, the default's accuracy/robustness
  // trade is pinned by the two tests below.
  const Vector6 F_hat = ee_wrench_from_residual(J, tau_ext, 1e-6);
  EXPECT_TRUE(F_hat.isApprox(F, 1e-3))
      << "F_hat " << F_hat.transpose() << " != F " << F.transpose();
}

TEST(EeWrench, DefaultDampingBiasIsSmallAtAWorkingPose) {
  // The default lambda buys a hard singular-pose cap (test below) at the price
  // of a bias at working poses. This pins that price: a real contact must come
  // through within ~10% where the arm actually operates, or the default is too
  // heavy to be useful.
  Dynamics dyn{URDF_PATH};
  Jacobian6 J;
  dyn.jacobian(well_conditioned_q(), J);
  Vector6 F;
  F << 5.0, -3.0, 8.0, 0.4, -0.2, 0.6;
  const Vector6 F_hat = ee_wrench_from_residual(J, J.transpose() * F);
  EXPECT_TRUE(F_hat.isApprox(F, 0.1))
      << "F_hat " << F_hat.transpose() << " biased >10% from F " << F.transpose();
}

TEST(EeWrench, ZeroResidualGivesExactlyZero) {
  Dynamics dyn{URDF_PATH};
  Jacobian6 J;
  dyn.jacobian(well_conditioned_q(), J);
  const Vector6 F_hat = ee_wrench_from_residual(J, JointVec::Zero());
  EXPECT_EQ(F_hat.norm(), 0.0);
}

TEST(EeWrench, DampingBoundsTheEstimateAtASingularPose) {
  // q = 0 is the fully-stretched candle: J loses rank and an undamped
  // pinv(J^T) would turn a residual's null-space component into an enormous
  // phantom wrench. The damped per-direction gain peaks at 1/(2*lambda), so
  // the estimate must honour the documented bound ||F|| <= ||tau||/(2*lambda):
  // at the default lambda a noise-floor residual maps to tens of newtons, not
  // the hundreds-to-thousands a DiffIk-sized lambda would permit. More damping
  // must tighten the cap, never grow the estimate.
  Dynamics dyn{URDF_PATH};
  Jacobian6 J;
  dyn.jacobian(JointVec::Zero(), J);
  const JointVec tau_ext = JointVec::Constant(2.5);  // the proximal-joint noise floor
  const Vector6 at_default = ee_wrench_from_residual(J, tau_ext);
  const Vector6 heavier = ee_wrench_from_residual(J, tau_ext, 0.1);
  EXPECT_TRUE(at_default.allFinite());
  EXPECT_LE(at_default.norm(), tau_ext.norm() / (2 * 0.05))  // ~66 N for this residual
      << "default damping does not deliver the documented 1/(2*lambda) cap";
  EXPECT_LE(heavier.norm(), tau_ext.norm() / (2 * 0.1));
  EXPECT_LE(heavier.norm(), at_default.norm());
}
