#include "kinova_lowlevel/interface/supervisor.h"
#include <chrono>
namespace kinova::interface {
using clock = std::chrono::steady_clock;
static double secs_since(clock::time_point t0){ return std::chrono::duration<double>(clock::now()-t0).count(); }

static const char* halt_reason_string(HaltReason r) {
  switch (r) {
    case HaltReason::kOwnershipRevoked: return "halted: ownership revoked";
    case HaltReason::kEmergencyStop:    return "halted: emergency stop";
    case HaltReason::kOperatorRequest:  return "halted: operator request";
  }
  return "halted";
}

Supervisor::Supervisor(JointPositionMode& pos, JointImpedanceMode& imp, RtExecutor& exec,
                       Seqlock<JointFeedback>& snap, Dynamics& pump_dyn,
                       StreamPort& stream, ActionServerPort& action, SupervisorConfig cfg)
  : pos_(pos), imp_(imp), exec_(exec), snap_(snap), pump_dyn_(pump_dyn),
    stream_(stream), action_(action), cfg_(cfg) {}
Supervisor::~Supervisor(){ stop(); }

void Supervisor::start() {
  exec_.request_mode(&pos_);                       // initial mode = position
  traj_.emplace(pos_);                             // executor bound to the active mode's sink
  active_mode_kind_ = ControlModeKind::kPosition;  atomic_mode_.store(0);
  running_.store(true);
  sampler_ = std::thread([this]{ sampler_loop(); });
  pump_    = std::thread([this]{ pump_loop(); });
}
void Supervisor::stop() {
  if (!running_.exchange(false)) return;
  if (sampler_.joinable()) sampler_.join();
  if (pump_.joinable())    pump_.join();
}

void Supervisor::pump_loop() {
  const auto period = std::chrono::duration<double>(1.0/cfg_.pump_hz);
  const auto t0 = clock::now();
  while (running_.load(std::memory_order_acquire)) {
    JointFeedback fb;
    if (snap_.load(fb)) {
      ArmState s; s.q=fb.q; s.qd=fb.qd; s.tau=fb.tau; s.fault=fb.fault; s.stamp_s=secs_since(t0);
      s.ee_pose = pump_dyn_.fk(fb.q);
      state_snap_.store(s);
      stream_.publish_state(s);
    }
    std::this_thread::sleep_for(std::chrono::duration_cast<clock::duration>(period));
  }
}

void Supervisor::sampler_loop() {                 // fleshed out in Tasks 6-9
  const auto period = std::chrono::duration<double>(1.0/cfg_.sampler_hz);
  const auto t0 = clock::now();
  GoalId active_id{}; bool have_active=false;
  JointVec q_meas = JointVec::Zero();   // last-good measured q; reused when a snapshot read fails
  GoalId queued_id{}; bool have_queued=false;
  while (running_.load(std::memory_order_acquire)) {
    // 0) a halt jumps the queue: settle everything ACCEPTed, then hold where the arm IS.
    bool halt = false; HaltReason hr = HaltReason::kOwnershipRevoked;
    { std::lock_guard<std::mutex> l(q_mtx_);
      if (halt_pending_) { halt = true; hr = halt_reason_; halt_pending_ = false; } }
    if (halt) {
      TrajectoryResult r; r.error_code = result_code::kHalted; r.error_string = halt_reason_string(hr);
      if (have_active) action_.settle(active_id, r);
      if (have_queued) action_.settle(queued_id, r);   // ACCEPTed already; dropping it orphans the client
      traj_.emplace(active_sink());
      have_active = false; have_queued = false; in_flight_.store(false);
      JointFeedback fb; const bool ok = snap_.load(fb);
      q_meas = sampled_q(ok, fb.q, q_meas);            // never inject a phantom zero here of all places
      active_sink().set_target(q_meas);                // hold at MEASURED q, not the last reference
    }
    // 1) drain inbox (only this thread touches traj_)
    for (;;) {
      Inbound in; { std::lock_guard<std::mutex> l(q_mtx_); if (inbox_.empty()) break; in=inbox_.front(); inbox_.pop_front(); }
      if (in.cancel) {
        // Abort the whole chain and reset the executor to idle; the mode keeps
        // commanding its last reference, so the arm holds where it was.
        if (have_active) { TrajectoryResult r; r.error_code=result_code::kPreempted; action_.settle(active_id, r); }
        if (have_queued) { TrajectoryResult r; r.error_code=result_code::kPreempted; action_.settle(queued_id, r); }
        traj_.emplace(active_mode_kind_==ControlModeKind::kImpedance
                      ? static_cast<kinova::JointTargetSink&>(imp_)
                      : static_cast<kinova::JointTargetSink&>(pos_));
        have_active=false; have_queued=false; in_flight_.store(false);
        continue;
      }
      if (in.goal.control_mode != active_mode_kind_) {
        if (have_active) {   // cross-mode goal slipped past the accept-time pre-check (in_flight_ lag); a mode change requires the arm at rest
          TrajectoryResult r; r.error_code = result_code::kInvalidGoal;
          r.error_string = "mode change while a trajectory is in flight";
          action_.settle(in.id, r); continue;
        }
        if (in.goal.control_mode == ControlModeKind::kImpedance) {
          if (in.goal.has_gains) { JointImpedanceParams p; p.Kq=in.goal.gains.kq; p.zeta=in.goal.gains.zeta;
                                   p.torque_limit=in.goal.gains.torque_limit; imp_.set_gains(p); }
          exec_.request_mode(&imp_); traj_.emplace(imp_);
          active_mode_kind_=ControlModeKind::kImpedance; atomic_mode_.store(1);
        } else {
          exec_.request_mode(&pos_); traj_.emplace(pos_);
          active_mode_kind_=ControlModeKind::kPosition; atomic_mode_.store(0);
        }
        std::this_thread::sleep_for(                                    // let the RT loop adopt + on_enter settle
            std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(cfg_.mode_settle_s)));
      }
      const SubmitResult sr = traj_->submit(in.goal.trajectory, in.goal.control_mode,
                                            in.goal.preemption, in.goal.path_tolerance);
      if (sr != SubmitResult::kAccepted) {
        TrajectoryResult r; r.error_code=result_code::kInvalidGoal; r.error_string="rejected by executor";
        action_.settle(in.id, r); continue;
      }
      if (!have_active) {
        // idle -> active: the executor adopts immediately regardless of preemption.
        active_id = in.id; have_active = true; in_flight_.store(true);
      } else if (in.goal.preemption == Preemption::kLatestWins) {
        // Preempt the active goal; the executor also drops any queued follow-on.
        { TrajectoryResult r; r.error_code=result_code::kPreempted; action_.settle(active_id, r); }
        if (have_queued) { TrajectoryResult r; r.error_code=result_code::kPreempted; action_.settle(queued_id, r); have_queued=false; }
        active_id = in.id;
      } else {
        // kQueue: this goal waits behind the active one. The executor overwrites any
        // prior queued goal, so settle the displaced one as preempted before overwriting.
        if (have_queued) { TrajectoryResult r; r.error_code=result_code::kPreempted; action_.settle(queued_id, r); }
        queued_id = in.id; have_queued = true;   // active_id / in_flight_ untouched
      }
    }
    // 2) tick the active trajectory
    if (traj_->is_active()) {
      JointFeedback fb; const bool ok = snap_.load(fb);   // sequence the read; don't rely on arg eval order
      q_meas = sampled_q(ok, fb.q, q_meas);               // failed read -> reuse last-good q (no phantom zero)
      const ExecStatus st = traj_->tick(secs_since(t0), q_meas);
      TrajectoryFeedback fbk; fbk.actual=q_meas; fbk.fraction_complete=st.fraction; action_.publish_feedback(active_id, fbk);
      if (st.promoted) {
        // The active goal finished successfully and the queued goal took over gaplessly.
        { TrajectoryResult r; r.error_code=result_code::kSuccessful; action_.settle(active_id, r); }
        active_id = queued_id; have_queued = false;   // promoted goal is now active; have_active/in_flight_ stay true
      }
      if (st.completed && have_active) {
        TrajectoryResult r;
        r.error_code = (st.error_code==ExecStatus::kPathToleranceViolated)
                       ? result_code::kPathToleranceViolated : result_code::kSuccessful;
        action_.settle(active_id, r); have_active=false; in_flight_.store(false);
        // A divergence abort drops any queued follow-on in the executor — settle it too.
        if (st.error_code==ExecStatus::kPathToleranceViolated && have_queued) {
          TrajectoryResult rq; rq.error_code=result_code::kPreempted; action_.settle(queued_id, rq); have_queued=false;
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::duration_cast<clock::duration>(period));
  }
}

// on_trajectory_goal (backend thread): fast pre-check only, no executor mutation.
GoalResponse Supervisor::on_trajectory_goal(const TrajectoryGoal& g){
  if (g.trajectory.points.empty()) return GoalResponse::kReject;                 // INVALID_GOAL
  const uint8_t want = (g.control_mode==ControlModeKind::kImpedance)?1:0;
  if (in_flight_.load() && want != atomic_mode_.load()) return GoalResponse::kReject; // mode-change-while-moving
  return GoalResponse::kAccept;
}
void Supervisor::on_trajectory_accepted(const GoalId& id, const TrajectoryGoal& g){
  std::lock_guard<std::mutex> l(q_mtx_); inbox_.push_back({id, g, false});
}
CancelResponse Supervisor::on_trajectory_cancel(const CancelRequest& c){
  std::lock_guard<std::mutex> l(q_mtx_); inbox_.push_back({c.id, {}, true}); return CancelResponse::kAccept;
}
// on_halt (backend thread): latch + flush the queue, nothing else. The sampler owns
// traj_ and settle(), so the control action happens there -- which keeps
// settle-exactly-once true by construction rather than by careful reasoning.
void Supervisor::on_halt(HaltReason r) {
  std::lock_guard<std::mutex> l(q_mtx_);
  inbox_.clear();                 // a halt must never sit behind queued trajectories
  halt_reason_ = r;
  halt_pending_ = true;
}

kinova::JointTargetSink& Supervisor::active_sink() {
  return active_mode_kind_ == ControlModeKind::kImpedance
         ? static_cast<kinova::JointTargetSink&>(imp_)
         : static_cast<kinova::JointTargetSink&>(pos_);
}
GainsResult    Supervisor::on_set_gains(const GainsRequest&){ return {}; }
ArmState       Supervisor::on_query_state(){ ArmState s; state_snap_.load(s); return s; }
}  // namespace kinova::interface
