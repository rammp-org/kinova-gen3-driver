#pragma once
#include <mutex>
#include <random>
#include <string>
#include "kinova_lowlevel/interface/ports.h"
namespace kinova::interface {

// Decides WHO may command the arm, and nothing else.
//
// Decorates a CommandSink: every inbound command carries a Token; the Arbiter
// compares it against the live grant and either delegates downstream or rejects.
// It includes nothing from controls -- no ControlMode, no RtExecutor, no Dynamics,
// no Transport -- so the safety-critical admission logic is unit-testable against a
// fake sink with no robot, no URDF and no threads. Same idiom as FeedbackTap
// decorating Transport.
//
// Lock discipline (spec: "Thread safety"): the mutex IS held across command
// delegation, so admit-and-deliver is atomic against revoke()/estop() -- otherwise a
// command admitted a moment before a revoke could reach the Supervisor AFTER the halt
// and restart a stopped arm. It is NEVER held across on_halt().
class Arbiter : public CommandSink, public ArbitrationSink {
 public:
  // seed == 0 -> seed the token RNG from std::random_device.
  Arbiter(CommandSink& downstream, ArbitrationMode mode, uint64_t seed = 0);

  // ArbitrationSink
  GrantResult       grant(const std::string& owner_id) override;
  void              revoke() override;
  void              estop() override;
  void              estop_clear() override;
  ArbitrationStatus status() const override;

  // CommandSink
  GoalResponse   on_trajectory_goal(const TrajectoryGoal&) override;
  void           on_trajectory_accepted(const GoalId&, const TrajectoryGoal&) override;
  CancelResponse on_trajectory_cancel(const CancelRequest&) override;
  GainsResult    on_set_gains(const GainsRequest&) override;
  ArmState       on_query_state() override;       // never gated -- reads are always open
  void           on_halt(HaltReason) override;    // pass-through

 private:
  bool  admit(const Token&) const;   // caller holds m_
  Token mint();                      // caller holds m_

  CommandSink&    down_;
  ArbitrationMode mode_;
  mutable std::mutex m_;
  std::mt19937_64 rng_;
  bool        owned_ = false;
  bool        estopped_ = false;
  Token       token_{};
  std::string owner_id_;
  uint64_t    generation_ = 0;
  uint64_t    rejected_ = 0;
};
}  // namespace kinova::interface
