#pragma once
#include <array>
#include <cstdint>
#include <string>

#include "kinova_lowlevel/cartesian_types.h"
#include "kinova_lowlevel/gripper_types.h"
#include "kinova_lowlevel/interface/trajectory_executor.h"  // Trajectory, Preemption, ControlModeKind
#include "kinova_lowlevel/joint_types.h"
namespace kinova::interface {

using GoalId = std::array<uint8_t, 16>;  // mirrors a ROS2 action UUID
using Token = std::array<uint8_t, 16>;   // 128-bit capability token; POD, alloc-free

enum class ArbitrationMode { kEnforced, kDisabled };
// The caller declares WHY the arm must stop; the supervisor decides HOW.
enum class HaltReason { kOwnershipRevoked, kEmergencyStop, kOperatorRequest };
// Why the last streaming session ended. A client that was streaming happily and
// suddenly has its setpoints refused needs to tell "you went quiet" apart from
// "the driver could not solve for the pose you asked for" -- the second is a
// tracking failure and re-opening the same session will just reproduce it.
enum class StreamCloseCause { kNone, kClientRequest, kDeadlineExpired, kHalted, kIkFault };

struct JointImpedanceGainValues {
  JointVec kq = JointVec::Zero();
  double zeta = 0.5;
  JointVec torque_limit = JointVec::Zero();
};

// How a command names its compliance. kSessionDefault = "whatever the session
// default points at" (initially the kMedium profile); named profiles are
// complete, core-owned parameter sets; kCustom overrides kq/zeta/torque_limit
// on top of the session default and MUST pass validate_custom at accept time.
enum class GainsProfile { kSessionDefault, kSoft, kMedium, kStiff, kCustom };
struct ImpedanceGains {
  GainsProfile profile = GainsProfile::kSessionDefault;
  JointImpedanceGainValues custom{};  // read iff profile == kCustom
};

// What a streaming client sends. The METHOD on StreamSink disambiguates which
// struct applies -- there is deliberately no tag field on the setpoint itself,
// so "kind says pose, pose field is garbage" is not representable.
enum class SetpointKind { kJointPosition, kEePose, kJointVelocity, kEeTwist, kJointTorque };

struct StreamOpenRequest {
  SetpointKind kind = SetpointKind::kJointPosition;
  ControlModeKind control_mode = ControlModeKind::kPosition;
  double timeout_s = 0.1;  // <= 0 is REJECTED at open: no deadline, no safe-stop
  // Compliance for an impedance session, resolved and applied AT OPEN. A
  // non-default spec on a non-impedance open is REJECTED. Mid-session gain
  // changes are deliberately out of scope for v1.3.0: close and re-open.
  ImpedanceGains gains{};
  Token token{};
};
struct StreamOpenResult {
  bool accepted = false;
  int error_code = 0;
  std::string message;
};
struct StreamCloseRequest {
  Token token{};
};

// One struct, three meanings -- units are per-method: rad (position), rad/s
// (velocity), N*m (feedforward torque).
struct JointSetpoint {
  JointVec values = JointVec::Zero();
  Token token{};
};
struct PoseSetpoint {
  Pose pose{};
  Token token{};
};
struct TwistSetpoint {
  Vector6 twist = Vector6::Zero();
  Token token{};
};  // [linear; angular], base frame

// The gripper's command, carrying its own authority like every other setpoint. The
// gripper rides the ARM's token (spec decision 1): one physical machine, one holder.
//
// `command.active` has NO EFFECT here. Supervisor::on_gripper_setpoint always calls
// GripperController::set_target, which arms stamping unconditionally -- stamp()
// overwrites the outgoing `active` from GripperController's own internal `stamping_`
// flag regardless of what this struct carries. `active` is a wire-level flag owned
// by GripperController: sending a GripperSetpoint always arms stamping, and
// GripperController::release() -- the halt path -- is the ONLY way to disarm it.
// A caller setting `command.active = false` expecting "stop commanding the gripper"
// gets the gripper commanded anyway; it is a documented no-op, not a second release
// path (see GripperController::release()).
struct GripperSetpoint {
  kinova::GripperCommand command{};
  Token token{};
};

// What the gripper reports. Mirrors GripperFeedback plus a stamp.
//
// There is deliberately NO velocity. MotorFeedback has one, but it was measured on the
// arm to be the commanded speed echoed back rather than a measurement, so core removed
// the field; see gripper_types.h. `effort` is a 0..1 fraction of maximum derived from
// motor current, never Newtons -- and note a SUSTAINED grasp reports a SMALL effort
// (~0.05), because the gripper spikes on contact and then settles to a low holding
// current.
struct GripperState {
  float position = 0.0f;
  float effort = 0.0f;
  float current = 0.0f;  // amps
  bool present = false;
  double stamp_s = 0.0;
};

struct TrajectoryGoal {
  Trajectory trajectory;
  JointVec path_tolerance = JointVec::Constant(-1.0);  // <0 disables (matches executor)
  JointVec goal_tolerance = JointVec::Constant(-1.0);
  double goal_time_tolerance_s = 0.0;
  // Execute this goal slower: (0, 1], 1.0 = as planned. Dilates the executor's
  // clock, so the path is unchanged and the commanded velocity scales with it.
  double speed_scale = 1.0;
  ControlModeKind control_mode = ControlModeKind::kPosition;
  Preemption preemption = Preemption::kLatestWins;
  // Which compliance this goal runs under when control_mode == kImpedance.
  // Defaults to the session default (initially the kMedium profile). A
  // position goal carrying a non-default spec is REJECTED: gains that cannot
  // act are a caller bug, surfaced loudly, not ignored.
  ImpedanceGains gains{};
  std::string sender_id;
  Token token{};  // every command carries its own authority
};
struct TrajectoryFeedback {
  JointVec desired = JointVec::Zero(), actual = JointVec::Zero(), error = JointVec::Zero();
  double fraction_complete = 0.0;
};
struct TrajectoryResult {
  int error_code = 0;
  std::string error_string;
  JointVec final_error = JointVec::Zero();
};
// ee_twist is J(q)*qd, so it is consistent WITH ee_pose by construction -- both come
// from the same URDF model and the same feedback sample. It is LOCAL_WORLD_ALIGNED,
// matching Dynamics::jacobian: a world-aligned frame at the tool, NOT the body frame.
// Deliberately not read from the arm's own tool_twist: that would be a different frame
// and a different model from the ee_pose beside it, and the two would silently disagree.
struct ArmState {
  JointVec q = JointVec::Zero(), qd = JointVec::Zero(), tau = JointVec::Zero();
  Pose ee_pose;
  Vector6 ee_twist = Vector6::Zero();
  bool fault = false;
  double stamp_s = 0.0;
  // The runtime speed override currently in force. Populated by the
  // Supervisor's pump loop from the same atomic the sampler reads (NOT the
  // authoritative value itself -- see SpeedOverrideRequest::may_raise for why
  // gating no longer reads this). An operator-visible answer to "did my
  // slow-down get undone?".
  double speed_override = 1.0;
};
struct GainsRequest {
  ImpedanceGains spec{};
  Token token{};
};
struct GainsResult {
  bool accepted = false;
  std::string message;
};
// Mirrors GainsRequest's shape: a value carried alongside the capability
// token that authorizes it, so the Arbiter can gate on the token without
// widening CommandSink's on_set_speed_override signature into two arguments.
struct SpeedOverrideRequest {
  double scale = 1.0;
  std::string sender_id;
  Token token{};
  // Set by the arbitration layer (Arbiter::on_set_speed_override) from
  // admit(token) -- which already covers the e-stop latch -- and ALWAYS
  // OVERWRITTEN before the request is forwarded downstream; a caller setting
  // this field itself has no effect. True means "this caller is authorised
  // to raise the effective override, not just lower it." The Arbiter decides
  // AUTHORISATION only; the Supervisor decides DIRECTION, atomically against
  // its own speed_override_, because it is the only place that value is
  // authoritative -- see Supervisor::set_speed_override.
  bool may_raise = false;
};
struct SpeedResult {
  bool accepted = false;
  std::string message;
};
// Cancel had no struct to carry a token; it needs one, or any stranger can stop your motion.
struct CancelRequest {
  GoalId id{};
  Token token{};
};

// What the streaming tier is currently doing. Every field mirrors a StreamingSession
// accessor, all of which are atomic reads -- this is a snapshot, not a lock.
struct StreamStatus {
  bool open = false;
  SetpointKind kind = SetpointKind::kJointPosition;
  ControlModeKind control_mode = ControlModeKind::kPosition;
  double timeout_s = 0.0;
  uint64_t rejected_count = 0;  // refused by the SESSION, not the Arbiter
};

struct GrantResult {
  bool accepted = false;
  Token token{};
  uint64_t generation = 0;
  std::string message;
};
struct ArbitrationStatus {
  ArbitrationMode mode = ArbitrationMode::kEnforced;
  bool estopped = false;
  bool owned = false;
  std::string owner_id;
  uint64_t generation = 0;
  uint64_t rejected_count = 0;
};

enum class GoalResponse { kAccept, kReject, kRejectUnauthorized };
enum class CancelResponse { kAccept, kReject };

namespace result_code {
constexpr int kSuccessful = 0, kInvalidGoal = -1, kPathToleranceViolated = -4,
              kGoalToleranceViolated = -5, kPreempted = -6, kPlanningFailed = -7,
              kNotAuthorized = -8, kHalted = -9, kStreamRejected = -10;
}
}  // namespace kinova::interface
