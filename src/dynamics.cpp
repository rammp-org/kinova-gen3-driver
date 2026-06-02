#include "kinova_lowlevel/dynamics.h"
#include <cmath>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
namespace kinova {
struct Dynamics::Impl {
  pinocchio::Model model;
  pinocchio::Data data;
  Eigen::VectorXd qcfg;
  explicit Impl(const std::string& urdf) {
    pinocchio::urdf::buildModel(urdf, model);
    data = pinocchio::Data(model);
    qcfg = pinocchio::neutral(model);
  }
};
Dynamics::Dynamics(const std::string& urdf_path) : impl_(new Impl(urdf_path)) {}
Dynamics::~Dynamics() { delete impl_; }
int Dynamics::nv() const { return impl_->model.nv; }
int Dynamics::nq() const { return impl_->model.nq; }
void Dynamics::gravity(const JointVec& q, JointVec& tau_out) {
  auto& m = impl_->model; auto& cfg = impl_->qcfg;
  for (int i = 0; i < m.nv; ++i) {
    int jid = m.getJointId(m.names[i + 1]);
    int qidx = m.idx_qs[jid];
    if (m.nqs[jid] == 2) { cfg[qidx] = std::cos(q[i]); cfg[qidx + 1] = std::sin(q[i]); }
    else { cfg[qidx] = q[i]; }
  }
  const Eigen::VectorXd& g = pinocchio::computeGeneralizedGravity(m, impl_->data, cfg);
  for (int i = 0; i < m.nv; ++i) tau_out[i] = g[i];
}
}  // namespace kinova
