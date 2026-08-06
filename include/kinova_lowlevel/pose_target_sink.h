#pragma once
#include "kinova_lowlevel/cartesian_types.h"
namespace kinova {

// Non-RT seam for publishing a Cartesian target to a control mode. Every mode the
// teleop server can drive implements it, so the server holds a single pointer and
// does not care which impedance law is running underneath.
//
// Contract: callable from ONE non-RT supervisor thread concurrently with the RT
// thread's compute(). Implementations publish via a double-buffer + release-store
// so the RT reader always observes a whole target, never a torn one.
class PoseTargetSink {
 public:
  virtual ~PoseTargetSink() = default;
  virtual void set_target(const Pose& x_d) noexcept = 0;
};

}  // namespace kinova
