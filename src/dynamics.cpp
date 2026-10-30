#include "kinova_lowlevel/dynamics.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <stdexcept>
#include <string>
#include <vector>
namespace kinova {
struct Dynamics::Impl {
  pinocchio::Model model;
  pinocchio::Data data;
  Eigen::VectorXd qcfg;
  pinocchio::FrameIndex frame_id = 0;
  double mount_tilt_deg = 0.0;
  // `full` is the model as parsed; keep only the chain root -> ee_frame.
  Impl(const pinocchio::Model& full, const std::string& ee_frame) {
    // Footgun guard: a typo'd/missing EE frame must fail loudly at startup, never
    // silently control the wrong point.
    if (!full.existFrame(ee_frame))
      throw std::runtime_error("Dynamics: EE frame '" + ee_frame + "' not in URDF");
    const auto& chain = full.supports[full.frames[full.getFrameId(ee_frame)].parentJoint];
    std::vector<pinocchio::JointIndex> lock;
    for (pinocchio::JointIndex j = 1; j < pinocchio::JointIndex(full.njoints); ++j)
      if (std::find(chain.begin(), chain.end(), j) == chain.end()) lock.push_back(j);
    model =
        lock.empty() ? full : pinocchio::buildReducedModel(full, lock, pinocchio::neutral(full));
    data = pinocchio::Data(model);
    qcfg = pinocchio::neutral(model);
    frame_id = model.getFrameId(ee_frame);
  }
  // Flat 7-vector of joint angles -> Pinocchio config vector, packing each
  // continuous joint as (cos, sin) (nqs==2). The single source of truth used by
  // gravity/fk/jacobian so they can never disagree about the configuration.
  void pack(const JointVec& q) {
    for (int i = 0; i < model.nv; ++i) {
      int jid = model.getJointId(model.names[i + 1]);
      int qidx = model.idx_qs[jid];
      if (model.nqs[jid] == 2) {
        qcfg[qidx] = std::cos(q[i]);
        qcfg[qidx + 1] = std::sin(q[i]);
      } else {
        qcfg[qidx] = q[i];
      }
    }
  }
};
namespace {
pinocchio::Model parse_path(const std::string& path) {
  pinocchio::Model m;
  pinocchio::urdf::buildModel(path, m);
  return m;
}
pinocchio::Model parse_xml(const std::string& xml) {
  if (xml.empty()) throw std::runtime_error("Dynamics: URDF XML is empty");
  pinocchio::Model m;
  pinocchio::urdf::buildModelFromXML(xml, m);
  return m;
}
// Guard against a wrong/mismatched URDF silently corrupting the fixed-size
// JointVec. Hard throw (not assert) so it fires in Release too. See no-silent-footgun.
// nv == 7 alone is not enough: a multi-DOF joint (planar, spherical) can make the
// count while leaving fewer than 7 names, which pack() and the limit readers index
// past, and a prismatic joint would be packed as an angle. So: exactly seven joints,
// each 1-DOF and revolute (Pinocchio's "R..." / "Revolute..." joint models).
void check_arm_chain(const pinocchio::Model& m) {
  if (m.nv != kNumJoints || m.njoints != kNumJoints + 1)
    throw std::runtime_error("Dynamics: URDF nv=" + std::to_string(m.nv) + " in " +
                             std::to_string(m.njoints - 1) + " joints, expected " +
                             std::to_string(kNumJoints) +
                             " after reducing to the EE chain (wrong URDF or EE frame)");
  for (pinocchio::JointIndex j = 1; j < pinocchio::JointIndex(m.njoints); ++j) {
    const std::string kind = m.joints[j].shortname();
    if (m.nvs[j] != 1 || kind.rfind("JointModelR", 0) != 0)
      throw std::runtime_error("Dynamics: joint '" + m.names[j] + "' is " + kind +
                               "; every arm joint must be a 1-DOF revolute joint");
  }
}
// Joint 1's axis in the root frame is independent of q (nothing moves before it),
// so the neutral configuration is as good as any. |cos| because the Gen3's joint 1
// axis points DOWN (rpy pi on joint_1): level reads 0, not 180.
double tilt_deg(const pinocchio::Model& m, pinocchio::Data& d, const Eigen::VectorXd& q) {
  pinocchio::computeJointJacobians(m, d, q);
  const Eigen::Vector3d axis = d.J.col(0).tail<3>().normalized();
  return std::acos(std::min(1.0, std::abs(axis.z()))) * 180.0 / M_PI;
}
}  // namespace

Dynamics::Dynamics(const std::string& urdf_path, const std::string& ee_frame)
    : impl_(std::make_unique<Impl>(parse_path(urdf_path), ee_frame)) {
  check_arm_chain(impl_->model);
  impl_->mount_tilt_deg = tilt_deg(impl_->model, impl_->data, impl_->qcfg);
}
Dynamics::Dynamics(const UrdfXml& urdf, const std::string& ee_frame)
    : impl_(std::make_unique<Impl>(parse_xml(urdf.xml), ee_frame)) {
  check_arm_chain(impl_->model);
  impl_->mount_tilt_deg = tilt_deg(impl_->model, impl_->data, impl_->qcfg);
}
double Dynamics::mount_tilt_deg() const { return impl_->mount_tilt_deg; }
std::vector<std::string> Dynamics::joint_names() const {
  // names[0] is "universe"; the reduced model holds exactly the arm chain.
  return {impl_->model.names.begin() + 1, impl_->model.names.end()};
}
Dynamics::~Dynamics() = default;
int Dynamics::nv() const { return impl_->model.nv; }
int Dynamics::nq() const { return impl_->model.nq; }
void Dynamics::gravity(const JointVec& q, JointVec& tau_out) {
  impl_->pack(q);
  const Eigen::VectorXd& g =
      pinocchio::computeGeneralizedGravity(impl_->model, impl_->data, impl_->qcfg);
  for (int i = 0; i < impl_->model.nv; ++i) tau_out[i] = g[i];
}
Pose Dynamics::fk(const JointVec& q) {
  impl_->pack(q);
  pinocchio::forwardKinematics(impl_->model, impl_->data, impl_->qcfg);
  pinocchio::updateFramePlacement(impl_->model, impl_->data, impl_->frame_id);
  const pinocchio::SE3& M = impl_->data.oMf[impl_->frame_id];
  Pose x;
  x.p = M.translation();
  x.R = Eigen::Quaterniond(M.rotation());
  x.R.normalize();
  return x;
}
void Dynamics::mass_matrix(const JointVec& q, JointMat& M_out) {
  impl_->pack(q);
  pinocchio::crba(impl_->model, impl_->data, impl_->qcfg);
  // CRBA populates only the upper triangle; mirror it so callers never read
  // uninitialised garbage below the diagonal.
  impl_->data.M.triangularView<Eigen::StrictlyLower>() =
      impl_->data.M.transpose().triangularView<Eigen::StrictlyLower>();
  for (int i = 0; i < impl_->model.nv; ++i)
    for (int j = 0; j < impl_->model.nv; ++j) M_out(i, j) = impl_->data.M(i, j);
}
void Dynamics::coriolis(const JointVec& q, const JointVec& qd, JointMat& C_out) {
  impl_->pack(q);
  // qd is in the tangent space (nv), so it needs no packing.
  pinocchio::computeCoriolisMatrix(impl_->model, impl_->data, impl_->qcfg, qd);
  for (int i = 0; i < impl_->model.nv; ++i)
    for (int j = 0; j < impl_->model.nv; ++j) C_out(i, j) = impl_->data.C(i, j);
}
void Dynamics::velocity_limits(JointVec& v_max) const {
  const pinocchio::Model& m = impl_->model;
  for (int i = 0; i < m.nv; ++i) {
    int jid = m.getJointId(m.names[i + 1]);
    v_max[i] = m.velocityLimit[m.idx_vs[jid]];
  }
}
void Dynamics::joint_limits(JointVec& lower, JointVec& upper) const {
  const pinocchio::Model& m = impl_->model;
  constexpr double kInf = std::numeric_limits<double>::infinity();
  // Same name->joint->config-index walk as Impl::pack, so the limits can never
  // disagree with the configuration packing about which joint is which.
  for (int i = 0; i < m.nv; ++i) {
    int jid = m.getJointId(m.names[i + 1]);
    int qidx = m.idx_qs[jid];
    if (m.nqs[jid] == 2) {  // continuous: (cos,sin) packing, no meaningful bound
      lower[i] = -kInf;
      upper[i] = kInf;
    } else {
      lower[i] = m.lowerPositionLimit[qidx];
      upper[i] = m.upperPositionLimit[qidx];
    }
  }
}
void Dynamics::jacobian(const JointVec& q, Jacobian6& J_out) {
  impl_->pack(q);
  J_out.setZero();
  // LOCAL_WORLD_ALIGNED: spatial velocity expressed in world-aligned axes at the
  // EE origin — the same frame the Cartesian stiffness gains live in.
  pinocchio::computeFrameJacobian(impl_->model, impl_->data, impl_->qcfg, impl_->frame_id,
                                  pinocchio::LOCAL_WORLD_ALIGNED, J_out);
}
}  // namespace kinova
