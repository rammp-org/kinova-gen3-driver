#include "kinova_lowlevel/interface/arbiter.h"
#include <cstring>
namespace kinova::interface {

Arbiter::Arbiter(CommandSink& downstream, StreamSink& downstream_stream, ArbitrationMode mode,
                 uint64_t seed)
  : down_(downstream), down_stream_(downstream_stream), mode_(mode),
    rng_(seed ? seed : std::random_device{}()) {}

Token Arbiter::mint() {
  Token t{}; const uint64_t a = rng_(), b = rng_();
  std::memcpy(t.data(), &a, sizeof(a)); std::memcpy(t.data() + 8, &b, sizeof(b));
  return t;
}

// Admitted iff the bypass is on, or the command carries the live token.
// E-stop latches over BOTH -- it is the one thing kDisabled does not bypass.
bool Arbiter::admit(const Token& t) const {
  if (estopped_.load(std::memory_order_acquire)) return false;
  if (mode_ == ArbitrationMode::kDisabled) return true;
  return owned_ && t == token_;
}

GrantResult Arbiter::grant(const std::string& owner_id) {
  bool need_halt = false;
  { std::lock_guard<std::mutex> l(m_);
    if (estopped_.load(std::memory_order_acquire)) return {false, Token{}, generation_, "e-stopped"};
    // A re-grant is revoke-then-grant, never a silent swap under a moving arm.
    if (owned_) { owned_ = false; token_ = Token{}; need_halt = true; } }
  if (need_halt) down_.on_halt(HaltReason::kOwnershipRevoked);   // lock NOT held
  std::lock_guard<std::mutex> l(m_);
  if (estopped_.load(std::memory_order_acquire))
    return {false, Token{}, generation_, "e-stopped"};                // raced an estop in the halt window
  token_ = mint(); owned_ = true; owner_id_ = owner_id; ++generation_;
  return {true, token_, generation_, ""};
}

void Arbiter::revoke() {
  bool need_halt = false;
  { std::lock_guard<std::mutex> l(m_);
    if (owned_) { owned_ = false; token_ = Token{}; owner_id_.clear(); need_halt = true; } }
  if (need_halt) down_.on_halt(HaltReason::kOwnershipRevoked);
}

// The ONE path that never waits for m_. Delegated calls run under m_, and
// Supervisor::on_stream_open sleeps mode_settle_s (250 ms) inside one of them, so an
// estop() that acquired m_ first could sit a quarter of a second before latching --
// and another quarter before the halt reached the arm. Instead:
//   1. latch estopped_ (atomic, release) -- admit() refuses from this instant, so
//      nothing can slip through the window below no matter who holds m_;
//   2. deliver the halt immediately, still without m_ (on_halt is never called
//      under it anyway);
//   3. only then take m_ for the ownership bookkeeping, which is not
//      safety-critical because step 1 already refuses every command.
void Arbiter::estop() {
  estopped_.store(true, std::memory_order_release);
  down_.on_halt(HaltReason::kEmergencyStop);   // unconditional: e-stop always halts
  std::lock_guard<std::mutex> l(m_);
  owned_ = false; token_ = Token{}; owner_id_.clear();
}

void Arbiter::estop_clear() {
  std::lock_guard<std::mutex> l(m_);
  // Cleared UNDER m_ (unlike the latch): re-enabling the arm may wait its turn.
  estopped_.store(false, std::memory_order_release);   // exits to no-owner, never straight back to owned
}

ArbitrationStatus Arbiter::status() const {
  std::lock_guard<std::mutex> l(m_);
  return {mode_, estopped_.load(std::memory_order_acquire), owned_, owner_id_, generation_, rejected_};
}

GoalResponse Arbiter::on_trajectory_goal(const TrajectoryGoal& g) {
  std::lock_guard<std::mutex> l(m_);
  if (!admit(g.token)) { ++rejected_; return GoalResponse::kRejectUnauthorized; }
  return down_.on_trajectory_goal(g);
}
void Arbiter::on_trajectory_accepted(const GoalId& id, const TrajectoryGoal& g) {
  std::lock_guard<std::mutex> l(m_);
  // Re-checked: never trust that a matching on_trajectory_goal preceded this call.
  if (!admit(g.token)) { ++rejected_; return; }
  down_.on_trajectory_accepted(id, g);
}
CancelResponse Arbiter::on_trajectory_cancel(const CancelRequest& c) {
  std::lock_guard<std::mutex> l(m_);
  // Cancel is gated: a stranger must not be able to stop your motion. The emergency
  // path is estop(), not cancel.
  if (!admit(c.token)) { ++rejected_; return CancelResponse::kReject; }
  return down_.on_trajectory_cancel(c);
}
GainsResult Arbiter::on_set_gains(const GainsRequest& r) {
  std::lock_guard<std::mutex> l(m_);
  if (!admit(r.token)) { ++rejected_; return {false, "not authorized"}; }
  return down_.on_set_gains(r);
}
ArmState Arbiter::on_query_state() { return down_.on_query_state(); }
void     Arbiter::on_halt(HaltReason r) { down_.on_halt(r); }

StreamOpenResult Arbiter::on_stream_open(const StreamOpenRequest& r) {
  std::lock_guard<std::mutex> l(m_);
  if (!admit(r.token)) { ++rejected_; return {false, result_code::kNotAuthorized, "not authorized"}; }
  return down_stream_.on_stream_open(r);
}
void Arbiter::on_stream_close(const StreamCloseRequest& r) {
  std::lock_guard<std::mutex> l(m_);
  if (!admit(r.token)) { ++rejected_; return; }
  down_stream_.on_stream_close(r);
}
void Arbiter::on_setpoint_joint_position(const JointSetpoint& s) {
  std::lock_guard<std::mutex> l(m_);
  if (!admit(s.token)) { ++rejected_; return; }
  down_stream_.on_setpoint_joint_position(s);
}
void Arbiter::on_setpoint_joint_velocity(const JointSetpoint& s) {
  std::lock_guard<std::mutex> l(m_);
  if (!admit(s.token)) { ++rejected_; return; }
  down_stream_.on_setpoint_joint_velocity(s);
}
void Arbiter::on_setpoint_joint_torque(const JointSetpoint& s) {
  std::lock_guard<std::mutex> l(m_);
  if (!admit(s.token)) { ++rejected_; return; }
  down_stream_.on_setpoint_joint_torque(s);
}
void Arbiter::on_setpoint_pose(const PoseSetpoint& s) {
  std::lock_guard<std::mutex> l(m_);
  if (!admit(s.token)) { ++rejected_; return; }
  down_stream_.on_setpoint_pose(s);
}
void Arbiter::on_setpoint_twist(const TwistSetpoint& s) {
  std::lock_guard<std::mutex> l(m_);
  if (!admit(s.token)) { ++rejected_; return; }
  down_stream_.on_setpoint_twist(s);
}
}  // namespace kinova::interface
