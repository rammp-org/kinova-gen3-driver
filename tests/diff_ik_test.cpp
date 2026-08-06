#include <gtest/gtest.h>
#include <cmath>
#include "kinova_lowlevel/diff_ik.h"
using namespace kinova;

namespace {
JointVec sample_q() {
  JointVec q; q << 0.1, 0.3, -0.2, 0.8, 0.5, -0.4, 0.2; return q;
}
// Solver tuned for tests: converge fully rather than take one teleop-sized step.
DiffIkParams converging_params() {
  DiffIkParams p;
  p.max_iters = 200;
  p.posture_gain = 0.0;
  p.limit_gain = 0.0;
  p.max_pos_err = 1.0;
  p.max_rot_err = 1.0;
  p.max_joint_step = 0.2;
  return p;
}
}  // namespace

TEST(DiffIk, ConvergesToReachablePoseFromPerturbedSeed) {
  Dynamics dyn(URDF_PATH);
  const JointVec q_true = sample_q();
  const Pose target = dyn.fk(q_true);

  DiffIkSolver ik(dyn, converging_params());
  JointVec q = q_true;
  q[1] += 0.15; q[3] -= 0.20; q[5] += 0.10;      // perturbed warm start

  IkResult r = ik.solve(target, q);
  EXPECT_TRUE(r.converged);
  // The POSE must match. q itself need not -- the arm is redundant.
  Pose reached = dyn.fk(q);
  EXPECT_LT((reached.p - target.p).norm(), 1e-3);
  Vector6 e = pose_error(target, reached);
  EXPECT_LT(e.tail<3>().norm(), 1e-2);
}

TEST(DiffIk, WarmStartAtSolutionIsNoOp) {
  Dynamics dyn(URDF_PATH);
  const JointVec q_true = sample_q();
  DiffIkSolver ik(dyn, converging_params());
  JointVec q = q_true;
  IkResult r = ik.solve(dyn.fk(q_true), q);
  EXPECT_TRUE(r.converged);
  EXPECT_EQ(r.iters, 0);                          // returned before doing any work
  EXPECT_NEAR((q - q_true).norm(), 0.0, 1e-12);
}

TEST(DiffIk, RespectsHardJointLimits) {
  Dynamics dyn(URDF_PATH);
  DiffIkParams p = converging_params();
  dyn.joint_limits(p.q_lower, p.q_upper);
  p.limit_clamp_margin = 0.02;
  DiffIkSolver ik(dyn, p);

  // Drive at a target far outside the workspace so the solve pushes hard on the
  // limited joints (2/4/6) for many iterations.
  Pose target = dyn.fk(sample_q());
  target.p += Eigen::Vector3d(2.0, 2.0, -2.0);
  JointVec q = sample_q();
  ik.solve(target, q);

  for (int i = 0; i < kNumJoints; ++i) {
    EXPECT_TRUE(std::isfinite(q[i])) << "joint " << i;
    if (std::isfinite(p.q_lower[i])) {
      EXPECT_GE(q[i], p.q_lower[i] + p.limit_clamp_margin - 1e-9) << "joint " << i;
    }
    if (std::isfinite(p.q_upper[i])) {
      EXPECT_LE(q[i], p.q_upper[i] - p.limit_clamp_margin + 1e-9) << "joint " << i;
    }
  }
}

TEST(DiffIk, UnreachableTargetStaysBoundedAndFinite) {
  Dynamics dyn(URDF_PATH);
  DiffIkParams p;                                  // production-ish: 4 iters, clamps on
  dyn.joint_limits(p.q_lower, p.q_upper);
  DiffIkSolver ik(dyn, p);
  Pose target = dyn.fk(sample_q());
  target.p += Eigen::Vector3d(10.0, -10.0, 10.0);  // wildly unreachable

  JointVec q = sample_q();
  const JointVec q0 = q;
  ik.solve(target, q);
  EXPECT_FALSE(q.hasNaN());
  // Per-iteration joint clamp bounds total motion by max_iters * max_joint_step.
  EXPECT_LE((q - q0).lpNorm<Eigen::Infinity>(),
            p.max_iters * p.max_joint_step + 1e-9);
}

TEST(DiffIk, PostureBiasMovesConfigWithoutBreakingTheTask) {
  Dynamics dyn(URDF_PATH);
  const JointVec q_true = sample_q();
  const Pose target = dyn.fk(q_true);

  DiffIkParams p = converging_params();
  p.posture_gain = 0.3;
  p.q_rest = q_true;
  p.q_rest[2] += 0.5;                              // pull the redundant DOF
  DiffIkSolver ik(dyn, p);

  JointVec q = q_true;
  IkResult r = ik.solve(target, q);
  EXPECT_GT((q - q_true).norm(), 1e-3);            // the posture bias actually moved it
  EXPECT_LT(r.pos_err, 1e-3);                      // ...without breaking the task
  EXPECT_LT(r.rot_err, 1e-2);
}

TEST(DiffIk, PostureErrorWrapsForContinuousJoints) {
  // Joint 1 (index 0) is continuous. q_rest=+3.0 and q=-3.0 are 0.28 rad apart the
  // short way round, not 6.0. Without wrapping the bias would drive the long way.
  Dynamics dyn(URDF_PATH);
  DiffIkParams p = converging_params();
  dyn.joint_limits(p.q_lower, p.q_upper);
  p.max_iters = 1;
  p.posture_gain = 1.0;
  p.q_rest = sample_q();
  p.q_rest[0] = 3.0;
  DiffIkSolver ik(dyn, p);

  JointVec q = sample_q();
  q[0] = -3.0;
  ik.solve(dyn.fk(q), q);
  EXPECT_LT(q[0], -3.0);        // wrapped: moves NEGATIVE, toward -pi and around
}
