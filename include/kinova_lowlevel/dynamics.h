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
  // Per-joint position limits [rad] from the URDF. Continuous joints report
  // +/-infinity: Pinocchio packs them (cos,sin) and their config-space limits are
  // [-1,1], which is meaningless as a joint angle. Reads the model only (never
  // `data`), so it is safe to call while another thread uses fk/jacobian.
  void joint_limits(JointVec& lower, JointVec& upper) const;
  int nv() const;
  int nq() const;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace kinova
