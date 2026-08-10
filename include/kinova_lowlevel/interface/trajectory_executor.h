#pragma once
#include <vector>
#include "kinova_lowlevel/joint_types.h"   // kinova::JointVec, kNumJoints
namespace kinova::interface {

struct JointWaypoint { kinova::JointVec q; double t_s; };
struct Trajectory {
  std::vector<JointWaypoint> points;
  double duration_s() const { return points.empty() ? 0.0 : points.back().t_s; }
};

kinova::JointVec sample(const Trajectory& tr, double t_s);

}  // namespace kinova::interface
