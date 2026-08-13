#include "kinova_lowlevel/interface/supervisor.h"
#include <chrono>
namespace kinova::interface {
using clock = std::chrono::steady_clock;
static double secs_since(clock::time_point t0){ return std::chrono::duration<double>(clock::now()-t0).count(); }

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
  while (running_.load(std::memory_order_acquire)) {
    // 1) drain inbox (only this thread touches traj_)
    for (;;) {
      Inbound in; { std::lock_guard<std::mutex> l(q_mtx_); if (inbox_.empty()) break; in=inbox_.front(); inbox_.pop_front(); }
      if (in.cancel) continue;                                   // cancel handled in Task 9
      // (mode switching handled in Task 8; Task 6 assumes same/position mode)
      const SubmitResult sr = traj_->submit(in.goal.trajectory, in.goal.control_mode,
                                            in.goal.preemption, in.goal.path_tolerance);
      if (sr != SubmitResult::kAccepted) {
        TrajectoryResult r; r.error_code=result_code::kInvalidGoal; r.error_string="rejected by executor";
        action_.settle(in.id, r); continue;
      }
      if (have_active && in.goal.preemption==Preemption::kLatestWins) {          // preempted the old goal
        TrajectoryResult r; r.error_code=result_code::kPreempted; action_.settle(active_id, r);
      }
      active_id = in.id; have_active = true;
      in_flight_.store(true);
    }
    // 2) tick the active trajectory
    if (traj_->is_active()) {
      JointFeedback fb; JointVec q = JointVec::Zero(); if (snap_.load(fb)) q = fb.q;
      const ExecStatus st = traj_->tick(secs_since(t0), q);
      TrajectoryFeedback fbk; fbk.actual=q; fbk.fraction_complete=st.fraction; action_.publish_feedback(active_id, fbk);
      if (st.promoted) { /* Task 9 */ }
      if (st.completed && have_active) {
        TrajectoryResult r;
        r.error_code = (st.error_code==ExecStatus::kPathToleranceViolated)
                       ? result_code::kPathToleranceViolated : result_code::kSuccessful;
        action_.settle(active_id, r); have_active=false; in_flight_.store(false);
      }
    }
    std::this_thread::sleep_for(std::chrono::duration_cast<clock::duration>(period));
  }
}

// on_trajectory_goal (backend thread): fast pre-check only, no executor mutation.
GoalResponse Supervisor::on_trajectory_goal(const TrajectoryGoal& g){
  if (g.trajectory.points.empty()) return GoalResponse::kReject;                 // INVALID_GOAL
  if (g.preemption == Preemption::kQueue) return GoalResponse::kReject;  // TODO(Task 9): queued-goal result tracking not yet implemented
  const uint8_t want = (g.control_mode==ControlModeKind::kImpedance)?1:0;
  if (in_flight_.load() && want != atomic_mode_.load()) return GoalResponse::kReject; // mode-change-while-moving
  return GoalResponse::kAccept;
}
void Supervisor::on_trajectory_accepted(const GoalId& id, const TrajectoryGoal& g){
  std::lock_guard<std::mutex> l(q_mtx_); inbox_.push_back({id, g, false});
}
CancelResponse Supervisor::on_trajectory_cancel(const GoalId&){ return CancelResponse::kReject; }
GainsResult    Supervisor::on_set_gains(const GainsRequest&){ return {}; }
ArmState       Supervisor::on_query_state(){ ArmState s; state_snap_.load(s); return s; }
}  // namespace kinova::interface
