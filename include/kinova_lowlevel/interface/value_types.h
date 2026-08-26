#pragma once
#include <array>
#include <cstdint>
#include <string>
#include "kinova_lowlevel/joint_types.h"
#include "kinova_lowlevel/cartesian_types.h"
#include "kinova_lowlevel/interface/trajectory_executor.h"  // Trajectory, Preemption, ControlModeKind
namespace kinova::interface {

using GoalId = std::array<uint8_t, 16>;          // mirrors a ROS2 action UUID
using Token  = std::array<uint8_t, 16>;          // 128-bit capability token; POD, alloc-free

enum class ArbitrationMode { kEnforced, kDisabled };
// The caller declares WHY the arm must stop; the supervisor decides HOW.
enum class HaltReason      { kOwnershipRevoked, kEmergencyStop, kOperatorRequest };

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
  Token token{};                                  // every command carries its own authority
};
struct TrajectoryFeedback { JointVec desired=JointVec::Zero(), actual=JointVec::Zero(), error=JointVec::Zero();
                            double fraction_complete = 0.0; };
struct TrajectoryResult   { int error_code = 0; std::string error_string; JointVec final_error = JointVec::Zero(); };
struct ArmState { JointVec q=JointVec::Zero(), qd=JointVec::Zero(), tau=JointVec::Zero();
                  Pose ee_pose; bool fault=false; double stamp_s=0.0; };
struct GainsRequest { JointImpedanceGains gains{}; Token token{}; };
struct GainsResult  { bool accepted=false; std::string message; };
// Cancel had no struct to carry a token; it needs one, or any stranger can stop your motion.
struct CancelRequest { GoalId id{}; Token token{}; };

struct GrantResult       { bool accepted=false; Token token{}; uint64_t generation=0; std::string message; };
struct ArbitrationStatus { ArbitrationMode mode = ArbitrationMode::kEnforced; bool estopped=false;
                           bool owned=false; std::string owner_id; uint64_t generation=0;
                           uint64_t rejected_count=0; };

enum class GoalResponse   { kAccept, kReject, kRejectUnauthorized };
enum class CancelResponse { kAccept, kReject };

namespace result_code {
  constexpr int kSuccessful = 0, kInvalidGoal = -1, kPathToleranceViolated = -4,
                kGoalToleranceViolated = -5, kPreempted = -6, kPlanningFailed = -7,
                kNotAuthorized = -8, kHalted = -9;
}
}  // namespace kinova::interface
