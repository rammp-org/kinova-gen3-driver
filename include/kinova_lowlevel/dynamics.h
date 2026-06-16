#pragma once
#include <memory>
#include <string>
#include "kinova_lowlevel/joint_types.h"
#include "kinova_lowlevel/cartesian_types.h"
namespace kinova {
class Dynamics {
 public:
  explicit Dynamics(const std::string& urdf_path,
                    const std::string& ee_frame = "gen3_end_effector_link");
  ~Dynamics();
  void gravity(const JointVec& q, JointVec& tau_out);   // RT-safe: no alloc after ctor
  Pose fk(const JointVec& q);                           // RT-safe: pose of ee_frame
  void jacobian(const JointVec& q, Jacobian6& J_out);   // RT-safe: 6x7, LOCAL_WORLD_ALIGNED
  int nv() const;
  int nq() const;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace kinova
