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
  while (running_.load(std::memory_order_acquire)) {
    std::this_thread::sleep_for(std::chrono::duration_cast<clock::duration>(period));
  }
}

// Stubs — filled in by later tasks.
GoalResponse   Supervisor::on_trajectory_goal(const TrajectoryGoal&){ return GoalResponse::kReject; }
void           Supervisor::on_trajectory_accepted(const GoalId&, const TrajectoryGoal&){}
CancelResponse Supervisor::on_trajectory_cancel(const GoalId&){ return CancelResponse::kReject; }
GainsResult    Supervisor::on_set_gains(const GainsRequest&){ return {}; }
ArmState       Supervisor::on_query_state(){ ArmState s; state_snap_.load(s); return s; }
}  // namespace kinova::interface
