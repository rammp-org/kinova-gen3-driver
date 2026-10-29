#include "kinova_lowlevel/dynamics.h"

#include <gtest/gtest.h>

#include <cmath>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <string>
#include <vector>

#include "fixtures.h"
using namespace kinova;
TEST(Dynamics, LoadsModel) {
  Dynamics dyn(URDF_PATH);
  EXPECT_EQ(dyn.nv(), kNumJoints);
}
TEST(Dynamics, ContinuousJointsInflateNq) {
  // The Gen3 7-DOF has continuous joints, which Pinocchio represents as (cos,sin)
  // pairs -> nq > nv. This guards the faithful packing path (no wide-limit hack):
  // if a build ever used bounded revolute joints instead, nq would equal nv.
  Dynamics dyn(URDF_PATH);
  EXPECT_GT(dyn.nq(), dyn.nv());
}
TEST(Dynamics, GravityFiniteAndLoadsJointOffAxis) {
  Dynamics dyn(URDF_PATH);
  JointVec q = JointVec::Zero(), tau;
  dyn.gravity(q, tau);
  EXPECT_TRUE(tau.allFinite());
  JointVec q2 = JointVec::Zero();
  q2[1] = M_PI / 2.0;  // arm horizontal
  JointVec tau2;
  dyn.gravity(q2, tau2);
  EXPECT_GT(tau2.cwiseAbs().maxCoeff(), 1.0);
}
TEST(Dynamics, DefaultEeFrameResolvesOn2f85) {
  Dynamics dyn(URDF_PATH);          // default frame "gen3_end_effector_link"
  EXPECT_EQ(dyn.nv(), kNumJoints);  // ctor did not throw
}

TEST(Dynamics, UnknownFrameThrows) {
  EXPECT_THROW(Dynamics(URDF_PATH, "no_such_frame_xyz"), std::runtime_error);
}

TEST(DynamicsFk, NeutralPoseIsFiniteUnitQuatInReach) {
  Dynamics dyn(URDF_PATH);
  JointVec q = JointVec::Zero();
  Pose x = dyn.fk(q);
  EXPECT_TRUE(x.p.allFinite());
  EXPECT_NEAR(x.R.norm(), 1.0, 1e-9);  // unit quaternion
  EXPECT_GT(x.p.norm(), 0.05);         // tip is away from base origin
  EXPECT_LT(x.p.norm(), 1.5);          // within physical reach
}

TEST(DynamicsFk, BaseYawRotatesTipAboutVerticalAxis) {
  // Joint 0 is the base yaw about a vertical (world-Z) axis. Rotating it must
  // leave the tip HEIGHT unchanged while MOVING it horizontally. We deliberately
  // do NOT assert a rotation direction or an exact origin-radius: the base axis
  // is offset from the world origin, so origin-radius is not exactly conserved.
  // Height-invariance + horizontal motion is the robust, physically-honest check;
  // fk's full numeric correctness is pinned by the finite-difference Jacobian
  // test (Task 4). Independent of exact link lengths.
  Dynamics dyn(URDF_PATH);
  JointVec q = JointVec::Zero();
  Pose a = dyn.fk(q);
  q[0] = M_PI / 2.0;
  Pose b = dyn.fk(q);
  EXPECT_NEAR(b.p.z(), a.p.z(), 1e-6);                      // vertical axis: height held
  EXPECT_GT((b.p.head<2>() - a.p.head<2>()).norm(), 0.01);  // tip moved horizontally
}

namespace {
// Angular part of a small rotation R_b * R_a^{-1} as a rotation vector.
Eigen::Vector3d rotvec(const Eigen::Quaterniond& Ra, const Eigen::Quaterniond& Rb) {
  Eigen::Quaterniond qe = Rb * Ra.inverse();
  if (qe.w() < 0) qe.coeffs() *= -1.0;  // shortest path
  Eigen::AngleAxisd aa(qe.normalized());
  return aa.angle() * aa.axis();
}
}  // namespace

TEST(DynamicsJacobian, MatchesFiniteDifferenceOfFk) {
  Dynamics dyn(URDF_PATH);
  JointVec q;
  q << 0.1, 0.3, -0.2, 0.8, 0.5, -0.4, 0.2;  // a non-singular pose
  Jacobian6 J;
  dyn.jacobian(q, J);

  const double eps = 1e-6;
  for (int i = 0; i < kNumJoints; ++i) {
    JointVec qp = q;
    qp[i] += eps;
    Pose a = dyn.fk(q), b = dyn.fk(qp);
    Eigen::Vector3d dlin = (b.p - a.p) / eps;       // world-frame linear vel
    Eigen::Vector3d dang = rotvec(a.R, b.R) / eps;  // world-frame angular vel
    EXPECT_NEAR((J.block<3, 1>(0, i) - dlin).norm(), 0.0, 1e-4) << "lin col " << i;
    EXPECT_NEAR((J.block<3, 1>(3, i) - dang).norm(), 0.0, 1e-4) << "ang col " << i;
  }
}
TEST(Dynamics, JointLimitsMatchUrdf) {
  Dynamics dyn(URDF_PATH);
  JointVec lo, hi;
  dyn.joint_limits(lo, hi);
  // Gen3 joints 1,3,5,7 (indices 0,2,4,6) are `continuous` in the URDF. Pinocchio
  // packs those as (cos,sin) and reports a meaningless [-1,1] config-space limit,
  // so they must come back as infinite and never be clamped.
  for (int i : {0, 2, 4, 6}) {
    EXPECT_TRUE(std::isinf(lo[i])) << "joint index " << i;
    EXPECT_TRUE(std::isinf(hi[i])) << "joint index " << i;
    EXPECT_LT(lo[i], 0.0);
    EXPECT_GT(hi[i], 0.0);
  }
  EXPECT_NEAR(lo[1], -2.41, 1e-6);
  EXPECT_NEAR(hi[1], 2.41, 1e-6);
  EXPECT_NEAR(lo[3], -2.66, 1e-6);
  EXPECT_NEAR(hi[3], 2.66, 1e-6);
  EXPECT_NEAR(lo[5], -2.23, 1e-6);
  EXPECT_NEAR(hi[5], 2.23, 1e-6);
}
TEST(Dynamics, VelocityLimitsMatchUrdf) {
  Dynamics dyn(URDF_PATH);
  JointVec v;
  dyn.velocity_limits(v);
  for (int i : {0, 1, 2, 3}) EXPECT_NEAR(v[i], 1.3963, 1e-4) << "joint index " << i;
  for (int i : {4, 5, 6}) EXPECT_NEAR(v[i], 1.2218, 1e-4) << "joint index " << i;
}

namespace {
using kinova::test_fixtures::articulate_gripper;
using kinova::test_fixtures::kTilt;
using kinova::test_fixtures::read_file;
using kinova::test_fixtures::replace_once;
using kinova::test_fixtures::Ry;
using kinova::test_fixtures::tilted_fixture;
constexpr const char* kEe = "gen3_end_effector_link";
std::string articulated_fixture() { return articulate_gripper(read_file(URDF_PATH)); }
const std::vector<JointVec> kPoses = [] {
  std::vector<JointVec> v(3, JointVec::Zero());
  v[1] << 0.3, 0.6, -1.0, 1.2, 0.4, -0.7, 1.1;
  v[2] << -1.4, -0.9, 2.5, -2.0, 1.6, 0.9, -2.2;
  return v;
}();
}  // namespace

TEST(DynamicsXml, MatchesPathConstructor) {
  Dynamics from_path(URDF_PATH);
  Dynamics from_xml(UrdfXml{read_file(URDF_PATH)}, kEe);
  for (const auto& q : kPoses) {
    JointVec a, b;
    from_path.gravity(q, a);
    from_xml.gravity(q, b);
    EXPECT_LT((a - b).cwiseAbs().maxCoeff(), 1e-12);
    EXPECT_LT((from_path.fk(q).p - from_xml.fk(q).p).norm(), 1e-12);
  }
  EXPECT_GT(from_xml.nq(), from_xml.nv());  // continuous packing survives the XML path
}

TEST(DynamicsXml, EmptyOrMalformedXmlThrows) {
  EXPECT_ANY_THROW(Dynamics(UrdfXml{""}, kEe));
  EXPECT_ANY_THROW(Dynamics(UrdfXml{"<robot name=\"x\"><link"}, kEe));
}

TEST(DynamicsReduce, ArticulatedGripperIsLockedToTheFrozenModel) {
  Dynamics frozen(URDF_PATH);
  Dynamics reduced(UrdfXml{articulated_fixture()}, kEe);
  EXPECT_EQ(reduced.nv(), kNumJoints);
  for (const auto& q : kPoses) {
    JointVec a, b;
    frozen.gravity(q, a);
    reduced.gravity(q, b);
    EXPECT_LT((a - b).cwiseAbs().maxCoeff(), 1e-12);
    Jacobian6 Ja, Jb;
    frozen.jacobian(q, Ja);
    reduced.jacobian(q, Jb);
    EXPECT_LT((Ja - Jb).cwiseAbs().maxCoeff(), 1e-12);
    EXPECT_LT((frozen.fk(q).p - reduced.fk(q).p).norm(), 1e-12);
    JointMat Ma, Mb;
    frozen.mass_matrix(q, Ma);
    reduced.mass_matrix(q, Mb);
    EXPECT_LT((Ma - Mb).cwiseAbs().maxCoeff(), 1e-12);
  }
  JointVec lo_a, hi_a, lo_b, hi_b, v_a, v_b;
  frozen.joint_limits(lo_a, hi_a);
  reduced.joint_limits(lo_b, hi_b);
  EXPECT_EQ(lo_a, lo_b);  // exact: infinities for the continuous joints compare equal
  EXPECT_EQ(hi_a, hi_b);
  frozen.velocity_limits(v_a);
  reduced.velocity_limits(v_b);
  EXPECT_EQ(v_a, v_b);
}

TEST(DynamicsReduce, EeFrameBeyondAGripperJointKeepsItAndThrows) {
  // A frame past the articulated knuckle keeps that joint on the chain: nv = 8.
  EXPECT_THROW(Dynamics(UrdfXml{articulated_fixture()}, "gen3_robotiq_85_left_finger_link"),
               std::runtime_error);
}

TEST(DynamicsReduce, JointNamesAreTheArmChainInOrder) {
  Dynamics dyn(UrdfXml{articulated_fixture()}, kEe);
  const std::vector<std::string> want = {"gen3_joint_1", "gen3_joint_2", "gen3_joint_3",
                                         "gen3_joint_4", "gen3_joint_5", "gen3_joint_6",
                                         "gen3_joint_7"};
  EXPECT_EQ(dyn.joint_names(), want);
}

TEST(DynamicsMount, TiltedRootRotatesFkAndJacobianIntoTheRootFrame) {
  Dynamics flat(URDF_PATH), tilted(UrdfXml{tilted_fixture()}, kEe);
  const Eigen::Matrix3d R = Ry(kTilt);
  for (const auto& q : kPoses) {
    const Pose a = flat.fk(q), b = tilted.fk(q);
    EXPECT_LT((R * a.p - b.p).norm(), 1e-12);
    EXPECT_LT((R * a.R.toRotationMatrix() - b.R.toRotationMatrix()).cwiseAbs().maxCoeff(), 1e-12);
    Jacobian6 Ja, Jb;
    flat.jacobian(q, Ja);
    tilted.jacobian(q, Jb);
    EXPECT_LT((R * Ja.topRows<3>() - Jb.topRows<3>()).cwiseAbs().maxCoeff(), 1e-12);
    EXPECT_LT((R * Ja.bottomRows<3>() - Jb.bottomRows<3>()).cwiseAbs().maxCoeff(), 1e-12);
  }
}

TEST(DynamicsMount, TiltedRootGravityEqualsLevelModelWithGravityRotated) {
  // Ground truth built independently: the LEVEL model, with Pinocchio's gravity
  // vector expressed in the arm's base frame, R^T * (0, 0, -g).
  pinocchio::Model level;
  pinocchio::urdf::buildModel(URDF_PATH, level);
  level.gravity.linear(Ry(kTilt).transpose() * Eigen::Vector3d(0, 0, -9.81));
  pinocchio::Data data(level);
  Dynamics tilted(UrdfXml{tilted_fixture()}, kEe);
  for (const auto& q : kPoses) {
    Eigen::VectorXd cfg = pinocchio::neutral(level);
    for (int i = 0; i < level.nv; ++i) {  // same (cos, sin) packing as Dynamics
      const auto jid = level.getJointId(level.names[i + 1]);
      const int k = level.idx_qs[jid];
      if (level.nqs[jid] == 2) {
        cfg[k] = std::cos(q[i]);
        cfg[k + 1] = std::sin(q[i]);
      } else {
        cfg[k] = q[i];
      }
    }
    const Eigen::VectorXd want = pinocchio::computeGeneralizedGravity(level, data, cfg);
    JointVec got;
    tilted.gravity(q, got);
    EXPECT_LT((want - got).cwiseAbs().maxCoeff(), 1e-9);
  }
}

TEST(DynamicsMount, TiltLoadsTheBaseYawJointThatLevelLeavesFree) {
  // Physical sanity: on a level arm joint 1's axis is vertical, so gravity never
  // loads it. Tilted 30 deg, it does whenever the arm's CoM is off that axis.
  Dynamics flat(URDF_PATH), tilted(UrdfXml{tilted_fixture()}, kEe);
  JointVec q = JointVec::Zero(), tf, tt;
  q[1] = M_PI / 2.0;  // arm horizontal
  q[0] = M_PI / 2.0;  // pointing along +Y, sideways to the tilt
  flat.gravity(q, tf);
  tilted.gravity(q, tt);
  EXPECT_NEAR(tf[0], 0.0, 1e-4);  // URDF rounding leaves ~9e-6 N*m; tilted is > 1
  EXPECT_GT(std::abs(tt[0]), 1.0);
}

TEST(DynamicsReduce, TiltedAndArticulatedReducesToTheTiltedFrozenModel) {
  // The production shape: a rotated root AND an articulated gripper, so
  // buildReducedModel runs with the mount in the model.
  Dynamics frozen(UrdfXml{tilted_fixture()}, kEe);
  Dynamics reduced(UrdfXml{articulate_gripper(tilted_fixture())}, kEe);
  for (const auto& q : kPoses) {
    JointVec a, b;
    frozen.gravity(q, a);
    reduced.gravity(q, b);
    EXPECT_LT((a - b).cwiseAbs().maxCoeff(), 1e-12);
    Jacobian6 Ja, Jb;
    frozen.jacobian(q, Ja);
    reduced.jacobian(q, Jb);
    EXPECT_LT((Ja - Jb).cwiseAbs().maxCoeff(), 1e-12);
    EXPECT_LT((frozen.fk(q).p - reduced.fk(q).p).norm(), 1e-12);
  }
  EXPECT_NEAR(reduced.mount_tilt_deg(), 30.0, 1e-3);
}

TEST(DynamicsChain, APrismaticArmJointIsRefused) {
  // nv stays 7, so a count check alone would accept it and pack a length as an angle.
  std::string xml = read_file(URDF_PATH);
  replace_once(xml, "<joint name=\"gen3_joint_7\" type=\"continuous\">",
               "<joint name=\"gen3_joint_7\" type=\"prismatic\">");
  try {
    Dynamics dyn(UrdfXml{xml}, kEe);
    FAIL() << "a prismatic arm joint was accepted";
  } catch (const std::runtime_error& e) {
    EXPECT_NE(std::string(e.what()).find("1-DOF revolute"), std::string::npos) << e.what();
  }
}

TEST(DynamicsMount, TiltIsReadFromTheModel) {
  // Joint 1's axis against the root's vertical. The level fixture's joint_1 rpy is
  // 3.1416, not pi, which leaves ~4e-4 deg.
  EXPECT_NEAR(Dynamics(URDF_PATH).mount_tilt_deg(), 0.0, 1e-2);
  EXPECT_NEAR(Dynamics(UrdfXml{tilted_fixture()}, kEe).mount_tilt_deg(), 30.0, 1e-2);
}

namespace {
// Same (cos, sin) packing as Dynamics, for tests that drive Pinocchio directly.
Eigen::VectorXd pack_cfg(const pinocchio::Model& m, const JointVec& q) {
  Eigen::VectorXd cfg = pinocchio::neutral(m);
  for (int i = 0; i < m.nv; ++i) {
    const auto jid = m.getJointId(m.names[i + 1]);
    const int k = m.idx_qs[jid];
    if (m.nqs[jid] == 2) {
      cfg[k] = std::cos(q[i]);
      cfg[k + 1] = std::sin(q[i]);
    } else {
      cfg[k] = q[i];
    }
  }
  return cfg;
}
}  // namespace

TEST(Dynamics, CoriolisTimesQdMatchesNonlinearMinusGravity) {
  // C(q,qd) qd == rnea(q,qd,0) - g(q), against Pinocchio driven directly.
  Dynamics dyn(URDF_PATH);
  pinocchio::Model model;
  pinocchio::urdf::buildModel(URDF_PATH, model);
  pinocchio::Data data(model);
  for (int k = 0; k < 20; ++k) {
    const JointVec q = JointVec::Random();
    const JointVec qd = 1.5 * JointVec::Random();
    const Eigen::VectorXd cfg = pack_cfg(model, q);
    const Eigen::VectorXd nle = pinocchio::nonLinearEffects(model, data, cfg, qd);
    const Eigen::VectorXd g = pinocchio::computeGeneralizedGravity(model, data, cfg);
    JointMat C;
    dyn.coriolis(q, qd, C);
    EXPECT_LT((C * qd - (nle - g)).cwiseAbs().maxCoeff(), 1e-9);
  }
}

TEST(Dynamics, CoriolisIsZeroAtRest) {
  Dynamics dyn(URDF_PATH);
  JointMat C;
  dyn.coriolis(JointVec::Random(), JointVec::Zero(), C);
  EXPECT_EQ(C.cwiseAbs().maxCoeff(), 0.0);
}

TEST(Dynamics, MdotMinusTwoCIsSkewSymmetric) {
  Dynamics dyn(URDF_PATH);
  constexpr double h = 1e-6;
  for (int k = 0; k < 10; ++k) {
    const JointVec q = JointVec::Random();
    const JointVec qd = 1.5 * JointVec::Random();
    const JointVec x = JointVec::Random();
    JointMat Mp, Mm, C;
    dyn.mass_matrix(JointVec(q + h * qd), Mp);
    dyn.mass_matrix(JointVec(q - h * qd), Mm);
    dyn.coriolis(q, qd, C);
    const JointMat Mdot = (Mp - Mm) / (2 * h);
    EXPECT_NEAR(x.dot((Mdot - 2 * C) * x), 0.0, 1e-5);
  }
}
