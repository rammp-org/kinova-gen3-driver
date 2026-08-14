#pragma once
#include <array>
#include <cstdint>
#include <string>
#include "kinova_lowlevel/joint_types.h"
#include "kinova_lowlevel/cartesian_types.h"
#include "kinova_lowlevel/interface/trajectory_executor.h"  // Trajectory, Preemption, ControlModeKind
namespace kinova::interface {

using GoalId = std::array<uint8_t, 16>;          // mirrors a ROS2 action UUID

struct JointImpedanceGains { JointVec kq = JointVec::Zero(); double zeta = 0.5;
                             JointVec torque_limit = JointVec::Zero(); };

struct TrajectoryGoal {
  Trajectory trajectory;
  JointVec path_tolerance = JointVec::Constant(-1.0);   // <0 disables (matches executor)
  JointVec goal_tolerance = JointVec::Constant(-1.0);
  double   goal_time_tolerance_s = 0.0;
  ControlModeKind control_mode = ControlModeKind::kPosition;
  Preemption      preemption   = Preemption::kLatestWins;
  JointImpedanceGains gains{};
  bool has_gains = false;
  std::string sender_id;
};
struct TrajectoryFeedback { JointVec desired=JointVec::Zero(), actual=JointVec::Zero(), error=JointVec::Zero();
                            double fraction_complete = 0.0; };
struct TrajectoryResult   { int error_code = 0; std::string error_string; JointVec final_error = JointVec::Zero(); };
struct ArmState { JointVec q=JointVec::Zero(), qd=JointVec::Zero(), tau=JointVec::Zero();
                  Pose ee_pose; bool fault=false; double stamp_s=0.0; };
struct GainsRequest { JointImpedanceGains gains{}; };
struct GainsResult  { bool accepted=false; std::string message; };

enum class GoalResponse   { kAccept, kReject };
enum class CancelResponse { kAccept, kReject };

namespace result_code {
  constexpr int kSuccessful = 0, kInvalidGoal = -1, kPathToleranceViolated = -4,
                kGoalToleranceViolated = -5, kPreempted = -6, kPlanningFailed = -7;
}
}  // namespace kinova::interface
