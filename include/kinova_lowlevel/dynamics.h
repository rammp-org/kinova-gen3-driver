#pragma once
#include <string>
#include "kinova_lowlevel/joint_types.h"
namespace kinova {
class Dynamics {
 public:
  explicit Dynamics(const std::string& urdf_path);
  ~Dynamics();
  void gravity(const JointVec& q, JointVec& tau_out);  // RT-safe: no alloc after ctor
  int nv() const;
  int nq() const;
 private:
  struct Impl;
  Impl* impl_;
};
}  // namespace kinova
