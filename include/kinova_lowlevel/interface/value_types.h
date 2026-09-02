#pragma once
#include <array>
#include <cstdint>
#include <string>
#include "kinova_lowlevel/joint_types.h"
#include "kinova_lowlevel/cartesian_types.h"
#include "kinova_lowlevel/gripper_types.h"
#include "kinova_lowlevel/interface/trajectory_executor.h"  // Trajectory, Preemption, ControlModeKind
namespace kinova::interface {

using GoalId = std::array<uint8_t, 16>;          // mirrors a ROS2 action UUID
using Token  = std::array<uint8_t, 16>;          // 128-bit capability token; POD, alloc-free

enum class ArbitrationMode { kEnforced, kDisabled };
// The caller declares WHY the arm must stop; the supervisor decides HOW.
enum class HaltReason      { kOwnershipRevoked, kEmergencyStop, kOperatorRequest };
// Why the last streaming session ended. A client that was streaming happily and
// suddenly has its setpoints refused needs to tell "you went quiet" apart from
// "the driver could not solve for the pose you asked for" -- the second is a
// tracking failure and re-opening the same session will just reproduce it.
enum class StreamCloseCause { kNone, kClientRequest, kDeadlineExpired, kHalted, kIkFault };

// What a streaming client sends. The METHOD on StreamSink disambiguates which
// struct applies -- there is deliberately no tag field on the setpoint itself,
// so "kind says pose, pose field is garbage" is not representable.
enum class SetpointKind { kJointPosition, kEePose, kJointVelocity, kEeTwist, kJointTorque };

struct StreamOpenRequest {
  SetpointKind    kind         = SetpointKind::kJointPosition;
  ControlModeKind control_mode = ControlModeKind::kPosition;
  double          timeout_s    = 0.1;   // <= 0 is REJECTED at open: no deadline, no safe-stop
  Token           token{};
};
struct StreamOpenResult   { bool accepted=false; int error_code=0; std::string message; };
struct StreamCloseRequest { Token token{}; };

// One struct, three meanings -- units are per-method: rad (position), rad/s
// (velocity), N*m (feedforward torque).
struct JointSetpoint { JointVec values = JointVec::Zero(); Token token{}; };
struct PoseSetpoint  { Pose     pose{};                    Token token{}; };
struct TwistSetpoint { Vector6  twist = Vector6::Zero();   Token token{}; };  // [linear; angular], base frame

// The gripper's command, carrying its own authority like every other setpoint. The
// gripper rides the ARM's token (spec decision 1): one physical machine, one holder.
struct GripperSetpoint { kinova::GripperCommand command{}; Token token{}; };

// What the gripper reports. Mirrors GripperFeedback plus a stamp.
//
// There is deliberately NO velocity. MotorFeedback has one, but it was measured on the
// arm to be the commanded speed echoed back rather than a measurement, so core removed
// the field; see gripper_types.h. `effort` is a 0..1 fraction of maximum derived from
// motor current, never Newtons -- and note a SUSTAINED grasp reports a SMALL effort
// (~0.05), because the gripper spikes on contact and then settles to a low holding
// current.
struct GripperState {
  float  position = 0.0f;
  float  effort   = 0.0f;
  float  current  = 0.0f;   // amps
  bool   present  = false;
  double stamp_s  = 0.0;
};

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
                kNotAuthorized = -8, kHalted = -9, kStreamRejected = -10;
}
}  // namespace kinova::interface
