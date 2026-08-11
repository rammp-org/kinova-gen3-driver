#pragma once
#include "kinova_lowlevel/joint_types.h"
namespace kinova {

// Non-RT seam for publishing a joint-space target to a control mode. The
// joint-space counterpart of PoseTargetSink: a caller that already knows the
// configuration it wants does not have to go through a Cartesian pose and an IK
// solve to ask for it.
//
// Contract: callable from ONE non-RT supervisor thread concurrently with the RT
// thread's compute(). Implementations publish via a double-buffer + release-store
// so the RT reader always observes a whole target, never a torn one.
class JointTargetSink {
 public:
  virtual ~JointTargetSink() = default;
  virtual void set_target(const JointVec& q_d) noexcept = 0;
};

}  // namespace kinova
