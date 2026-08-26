#pragma once
#include <string>
#include "kinova_lowlevel/interface/value_types.h"
namespace kinova::interface {
// Driven ports — the supervisor CALLS these to push data out.
class StreamPort { public: virtual ~StreamPort() = default;
  virtual void publish_state(const ArmState&) = 0; };
class ActionServerPort { public: virtual ~ActionServerPort() = default;
  virtual void publish_feedback(const GoalId&, const TrajectoryFeedback&) = 0;
  virtual void settle(const GoalId&, const TrajectoryResult&) = 0; };
// Driving port — the supervisor IMPLEMENTS this; the backend calls it on inbound messages.
class CommandSink { public: virtual ~CommandSink() = default;
  virtual GoalResponse   on_trajectory_goal(const TrajectoryGoal&) = 0;
  virtual void           on_trajectory_accepted(const GoalId&, const TrajectoryGoal&) = 0;
  virtual CancelResponse on_trajectory_cancel(const CancelRequest&) = 0;
  virtual GainsResult    on_set_gains(const GainsRequest&) = 0;
  virtual ArmState       on_query_state() = 0;
  // Stop the arm now. General primitive: ownership revocation and /estop both use it.
  virtual void           on_halt(HaltReason) = 0; };

// Driving port for ownership. Separate from CommandSink: "who may command" is not
// "command the arm", and a harness that ignores ownership implements only CommandSink.
class ArbitrationSink { public: virtual ~ArbitrationSink() = default;
  virtual GrantResult       grant(const std::string& owner_id) = 0;
  virtual void              revoke() = 0;
  virtual void              estop() = 0;
  virtual void              estop_clear() = 0;
  virtual ArbitrationStatus status() const = 0; };
}  // namespace kinova::interface
