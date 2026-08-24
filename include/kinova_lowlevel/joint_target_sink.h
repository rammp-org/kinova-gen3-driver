#pragma once
#include "kinova_lowlevel/joint_types.h"
namespace kinova {

// A joint-space reference, optionally carrying the derivatives a planner
// computed. qd/qdd are meaningful only when the matching has_* flag is set;
// a zero qd is NOT the same as "no velocity" (see JointImpedanceMode, where
// feeding a zero reference velocity is exactly the behaviour we are fixing).
struct JointTarget {
  JointVec q   = JointVec::Zero();
  JointVec qd  = JointVec::Zero();
  JointVec qdd = JointVec::Zero();
  bool has_velocity = false;
  bool has_acceleration = false;   // only honoured together with has_velocity
};

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

  // Full reference, derivatives included. This is what the trajectory executor
  // calls; a mode uses the derivatives as feedforward if it can.
  virtual void set_joint_target(const JointTarget& t) noexcept = 0;

  // Position-only convenience for callers with no profile (teleop, tests).
  // Deliberately non-virtual: it is the same operation with absent derivatives,
  // not a second thing a mode has to implement.
  void set_target(const JointVec& q_d) noexcept { set_joint_target(JointTarget{q_d}); }
};

}  // namespace kinova
