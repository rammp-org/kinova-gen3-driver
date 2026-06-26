#include "kinova_lowlevel/dynamics.h"
#include <cmath>
#include <stdexcept>
#include <string>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
namespace kinova {
struct Dynamics::Impl {
  pinocchio::Model model;
  pinocchio::Data data;
  Eigen::VectorXd qcfg;
  pinocchio::FrameIndex frame_id = 0;
  explicit Impl(const std::string& urdf) {
    pinocchio::urdf::buildModel(urdf, model);
    data = pinocchio::Data(model);
    qcfg = pinocchio::neutral(model);
  }
  // Flat 7-vector of joint angles -> Pinocchio config vector, packing each
  // continuous joint as (cos, sin) (nqs==2). The single source of truth used by
  // gravity/fk/jacobian so they can never disagree about the configuration.
  void pack(const JointVec& q) {
    for (int i = 0; i < model.nv; ++i) {
      int jid = model.getJointId(model.names[i + 1]);
      int qidx = model.idx_qs[jid];
      if (model.nqs[jid] == 2) { qcfg[qidx] = std::cos(q[i]); qcfg[qidx + 1] = std::sin(q[i]); }
      else { qcfg[qidx] = q[i]; }
    }
  }
};
Dynamics::Dynamics(const std::string& urdf_path, const std::string& ee_frame)
    : impl_(std::make_unique<Impl>(urdf_path)) {
  // Guard against a wrong/mismatched URDF silently corrupting the fixed-size
  // JointVec. Hard throw (not assert) so it fires in Release too. See no-silent-footgun.
  if (impl_->model.nv != kNumJoints) {
    throw std::runtime_error(
        "Dynamics: URDF nv=" + std::to_string(impl_->model.nv) +
        " != kNumJoints=" + std::to_string(kNumJoints) + " (wrong URDF for this build)");
  }
  // Footgun guard: a typo'd/missing EE frame must fail loudly at startup, never
  // silently control the wrong point.
  if (!impl_->model.existFrame(ee_frame)) {
    throw std::runtime_error("Dynamics: EE frame '" + ee_frame + "' not in URDF");
  }
  impl_->frame_id = impl_->model.getFrameId(ee_frame);
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
void Dynamics::jacobian(const JointVec& q, Jacobian6& J_out) {
  impl_->pack(q);
  J_out.setZero();
  // LOCAL_WORLD_ALIGNED: spatial velocity expressed in world-aligned axes at the
  // EE origin — the same frame the Cartesian stiffness gains live in.
  pinocchio::computeFrameJacobian(impl_->model, impl_->data, impl_->qcfg,
                                  impl_->frame_id, pinocchio::LOCAL_WORLD_ALIGNED,
                                  J_out);
}
}  // namespace kinova
