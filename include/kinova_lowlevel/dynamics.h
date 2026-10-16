#pragma once
#include <memory>
#include <string>
#include <vector>

#include "kinova_lowlevel/cartesian_types.h"
#include "kinova_lowlevel/joint_types.h"
namespace kinova {
// A URDF given as its XML text (e.g. read from /arm/robot_description), as
// opposed to a path. A distinct type so the two constructors cannot be confused.
struct UrdfXml {
  std::string xml;
};
class Dynamics {
 public:
  explicit Dynamics(const std::string& urdf_path,
                    const std::string& ee_frame = "gen3_end_effector_link");
  // Same model, from XML text. Both constructors REDUCE the model to the joints on
  // the chain from the root to ee_frame, locking every other joint at q = 0 -- the
  // same as type="fixed" at its URDF origin -- so an articulated gripper's mass
  // stays lumped at the wrist. nv must be 7 after reduction or construction throws.
  explicit Dynamics(const UrdfXml& urdf,
                    const std::string& ee_frame = "gen3_end_effector_link");
  // The arm's joint names in chain order (root to tip), as named in the URDF.
  std::vector<std::string> joint_names() const;
  ~Dynamics();
  void gravity(const JointVec& q, JointVec& tau_out);  // RT-safe: no alloc after ctor
  Pose fk(const JointVec& q);                          // RT-safe: pose of ee_frame
  void jacobian(const JointVec& q, Jacobian6& J_out);  // RT-safe: 6x7, LOCAL_WORLD_ALIGNED
  // Joint-space mass matrix M(q) via CRBA. RT-safe: no alloc after ctor. Returned
  // fully symmetric (CRBA fills only the upper triangle; this mirrors it).
  void mass_matrix(const JointVec& q, JointMat& M_out);
  // Per-joint position limits [rad] from the URDF. Continuous joints report
  // +/-infinity: Pinocchio packs them (cos,sin) and their config-space limits are
  // [-1,1], which is meaningless as a joint angle. Reads the model only (never
  // `data`), so it is safe to call while another thread uses fk/jacobian.
  void joint_limits(JointVec& lower, JointVec& upper) const;
  // Per-joint velocity limits [rad/s] from the URDF. Anything the driver rate-
  // limits should be derived from these rather than a hand-picked constant, which
  // is how you silently ship a cap below what the hardware can do.
  void velocity_limits(JointVec& v_max) const;
  int nv() const;
  int nq() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace kinova
