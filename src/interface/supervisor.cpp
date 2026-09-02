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

Supervisor::Supervisor(JointPositionMode& pos, JointImpedanceMode& imp, JointTorqueMode& tau, JointVelocityMode& vel,
                       RtExecutor& exec, Seqlock<JointFeedback>& snap, Dynamics& pump_dyn,
                       StreamPort& stream, ActionServerPort& action, SupervisorConfig cfg)
  : pos_(pos), imp_(imp), tau_(tau), vel_(vel), exec_(exec), snap_(snap), pump_dyn_(pump_dyn),
    stream_(stream), action_(action), cfg_(cfg) {}
Supervisor::~Supervisor(){ stop(); }

void Supervisor::start() {
  exec_.request_mode(&pos_);                       // initial mode = position
  traj_.emplace(pos_);                             // executor bound to the active mode's sink
  active_mode_kind_.store(ControlModeKind::kPosition);
  traj_bound_kind_  = ControlModeKind::kPosition;
  t0_ = clock::now();                              // origin for every session/expiry stamp
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
      // ONE load of the running mode: a backend on_stream_open landing between two
      // reads would bind traj_ to one sink and record the OTHER kind, which is the
      // silent mis-mapping traj_bound_kind_ exists to prevent.
      const ControlModeKind k = active_mode_kind_.load();
      kinova::JointTargetSink* sink = sink_for(k);
      // A kind with no joint sink (kTorque) still needs traj_ reset to idle; pos_ is
      // an inert placeholder there, because traj_bound_kind_ = k forces a rebind
      // before any position/impedance goal can be driven through it.
      traj_.emplace(sink ? *sink : static_cast<kinova::JointTargetSink&>(pos_));
      traj_bound_kind_ = k;
      have_active = false; have_queued = false; in_flight_.store(false);
      JointFeedback fb; const bool ok = snap_.load(fb);
      q_meas = sampled_q(ok, fb.q, q_meas);            // never inject a phantom zero here of all places
      // Torque's safe-stop is gravity-comp hold, already restored by close_stream()
      // above -- it has no joint target to latch, so there is nothing to write.
      if (sink) sink->set_target(q_meas);              // hold at MEASURED q, not the last reference
    }
    // 0b) lifecycle half of the streaming deadline. The mode has already made the
    //     OUTPUT safe at 1 kHz (its own watchdog); this closes the session, latches
    //     the hold and lets trajectory goals back in. Stamped against t0_, the same
    //     origin session_.open()/admit() use -- a different origin would make the
    //     deadline meaningless.
    if (stream_open_.load() && session_.expired(secs_since(t0_)))
      close_stream(StreamCloseCause::kDeadlineExpired);
    // 0c) the OTHER way a streaming session must end: the pose path stopped
    //     converging. JointPositionMode has already frozen the reference at
    //     measured q at 1 kHz; without this the session stays OPEN, the deadline
    //     keeps refreshing off the client's own setpoints, and the client goes on
    //     streaming poses believing it is tracking -- the exact silent divergence
    //     ik_faulted() exists to prevent. Guarded on the RUNNING mode, not on the
    //     session's, because pos_ is the only mode that owns this flag.
    if (stream_open_.load() && active_mode_kind_.load() == ControlModeKind::kPosition &&
        pos_.ik_faulted())
      close_stream(StreamCloseCause::kIkFault);
    // 1) drain inbox (only this thread touches traj_)
    for (;;) {
      Inbound in; { std::lock_guard<std::mutex> l(q_mtx_); if (inbox_.empty()) break; in=inbox_.front(); inbox_.pop_front(); }
      if (in.cancel) {
        // Abort the whole chain and reset the executor to idle; the mode keeps
        // commanding its last reference, so the arm holds where it was.
        if (have_active) { TrajectoryResult r; r.error_code=result_code::kPreempted; action_.settle(active_id, r); }
        if (have_queued) { TrajectoryResult r; r.error_code=result_code::kPreempted; action_.settle(queued_id, r); }
        const ControlModeKind k = active_mode_kind_.load();   // ONE load: see the halt path
        kinova::JointTargetSink* sink = sink_for(k);
        traj_.emplace(sink ? *sink : static_cast<kinova::JointTargetSink&>(pos_));
        traj_bound_kind_ = k;
        have_active=false; have_queued=false; in_flight_.store(false);
        continue;
      }
      // A goal ACCEPTed before a stream opened, drained after. in_flight_ is set
      // HERE, not at accept time, so on_stream_open's in_flight_ check cannot see a
      // goal still sitting in inbox_ -- up to one sampler period (4 ms at 250 Hz)
      // wide. Executing it now would tick the trajectory into the very sink the
      // backend thread is streaming into: two writers, one double buffer, which is
      // exactly what the direct-write design says can never happen. Refuse instead.
      if (stream_open_.load()) {
        TrajectoryResult r; r.error_code = result_code::kInvalidGoal;
        r.error_string = "a streaming session opened before this goal could start";
        action_.settle(in.id, r); continue;
      }
      // Second-layer guard on the mode enum. on_trajectory_goal refuses kVelocity
      // and kTorque, but a backend may call on_trajectory_accepted without a
      // preceding accepted goal -- and the mode-switch branch below is binary, so
      // anything that is neither position nor impedance would be driven AS
      // position. Fail loud instead of silently mis-mapping.
      if (in.goal.control_mode != ControlModeKind::kPosition &&
          in.goal.control_mode != ControlModeKind::kImpedance) {
        TrajectoryResult r; r.error_code = result_code::kInvalidGoal;
        r.error_string = "trajectory execution supports position and impedance only";
        action_.settle(in.id, r); continue;
      }
      // Rebind unless BOTH agree with the goal. A streaming session moves
      // active_mode_kind_ from the backend thread without touching traj_ (which
      // only this thread may rebuild), so the two can disagree in either
      // direction and each one alone is a silent mis-mapping:
      //   * != traj_bound_kind_ only: a kPosition goal after a kImpedance stream
      //     skips the rebind, so traj_ drives pos_ while the executor runs imp_.
      //   * != active_mode_kind_ only: a goal in the mode the executor already
      //     runs skips the rebind, so traj_ keeps writing the previous sink.
      // Either way the arm sits still and the goal settles SUCCESSFUL. Testing
      // both makes the rebind a no-op at worst.
      if (in.goal.control_mode != traj_bound_kind_ ||
          in.goal.control_mode != active_mode_kind_.load()) {
        if (have_active) {   // cross-mode goal slipped past the accept-time pre-check (in_flight_ lag); a mode change requires the arm at rest
          TrajectoryResult r; r.error_code = result_code::kInvalidGoal;
          r.error_string = "mode change while a trajectory is in flight";
          action_.settle(in.id, r); continue;
        }
        if (in.goal.control_mode == ControlModeKind::kImpedance) {
          if (in.goal.has_gains) { JointImpedanceParams p; p.Kq=in.goal.gains.kq; p.zeta=in.goal.gains.zeta;
                                   p.torque_limit=in.goal.gains.torque_limit; imp_.set_gains(p); }
          exec_.request_mode(&imp_); traj_.emplace(imp_);   // no-op in the executor if already active
          active_mode_kind_.store(ControlModeKind::kImpedance);
        } else {
          exec_.request_mode(&pos_); traj_.emplace(pos_);
          active_mode_kind_.store(ControlModeKind::kPosition);
        }
        traj_bound_kind_ = in.goal.control_mode;
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
  if (stream_open_.load()) return GoalResponse::kReject;   // a stream owns the arm
  if (g.trajectory.points.empty()) return GoalResponse::kReject;                 // INVALID_GOAL
  if (g.control_mode == ControlModeKind::kVelocity ||
      g.control_mode == ControlModeKind::kTorque) {
    return GoalResponse::kReject;    // trajectory execution is position/impedance only
  }
  // in_flight_ implies a goal is running, so a stream cannot be open and
  // active_mode_kind_ is one of the same two kinds g.control_mode was just
  // filtered to. Reading it directly keeps ONE record of the running mode.
  if (in_flight_.load() && g.control_mode != active_mode_kind_.load())
    return GoalResponse::kReject;                                    // mode-change-while-moving
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
  close_stream(StreamCloseCause::kHalted);   // stop admitting setpoints BEFORE the hold is latched
  std::lock_guard<std::mutex> l(q_mtx_);
  inbox_.clear();                 // a halt must never sit behind queued trajectories
  halt_reason_ = r;
  halt_pending_ = true;
}

// EXPLICIT over all four kinds rather than a binary "impedance or else". The old
// fall-through mapped kTorque and kVelocity onto pos_, so a halt during a torque
// stream wrote its hold into a mode the executor is not running -- silently a
// no-op. nullptr says "this kind has no joint target" out loud:
//   * kTorque   -- its safe-stop IS gravity-comp hold (spec decision 6), which
//                  close_stream() produces by restoring the mode's own timeout.
//   * kVelocity -- JointVelocityMode is deliberately not a JointTargetSink: it has
//                  no position target, only a velocity one. Its own safe-stop is
//                  zero velocity, latched in close_stream() directly, not through
//                  this map.
// A switch with no default makes adding a kind a compile error, not a silent case.
kinova::JointTargetSink* Supervisor::sink_for(ControlModeKind k) {
  switch (k) {
    case ControlModeKind::kPosition:  return &pos_;
    case ControlModeKind::kImpedance: return &imp_;
    case ControlModeKind::kTorque:    return nullptr;
    case ControlModeKind::kVelocity:  return nullptr;
  }
  return nullptr;
}
// The pose-target sink a control mode kind owns, EXPLICIT for the same reason
// sink_for is: an inline "impedance or else" ternary here would recreate exactly
// the binary mapping sink_for's own comment records as having been wrong before.
kinova::PoseTargetSink* Supervisor::pose_sink_for(ControlModeKind k) {
  switch (k) {
    case ControlModeKind::kImpedance: return &imp_;
    case ControlModeKind::kPosition:  return &pos_;   // Plan 2: position gained IK
    case ControlModeKind::kVelocity:
    case ControlModeKind::kTorque:    return nullptr;
  }
  return nullptr;
}
GainsResult    Supervisor::on_set_gains(const GainsRequest&){ return {}; }
ArmState       Supervisor::on_query_state(){ ArmState s; state_snap_.load(s); return s; }

StreamOpenResult Supervisor::on_stream_open(const StreamOpenRequest& r) {
  if (in_flight_.load())
    return {false, result_code::kStreamRejected, "a trajectory goal is in flight"};
  // The next three checks are DELIBERATELY duplicated in StreamingSession::open.
  // Here they refuse a bad request BEFORE a pointless mode switch and before a
  // watchdog is re-armed on the strength of it; there they run again because the
  // session is the authority on its own lifecycle and must not depend on its caller
  // having filtered. Not an oversight -- do not delete either copy.
  if (stream_open_.load())
    return {false, result_code::kStreamRejected, "a session is already open; close it first"};
  if (r.timeout_s <= 0.0)
    return {false, result_code::kStreamRejected,
            "timeout_s must be > 0: an unbounded stream has no safe-stop"};
  if (!pair_supported(r.kind, r.control_mode))
    return {false, result_code::kStreamRejected, "unsupported (setpoint kind, control mode) pair"};

  // Switch modes BEFORE the session is marked open, so no setpoint can land mid-switch.
  const ControlModeKind want = r.control_mode;
  if (want != active_mode_kind_.load()) {
    if      (want == ControlModeKind::kImpedance) exec_.request_mode(&imp_);
    else if (want == ControlModeKind::kTorque)    exec_.request_mode(&tau_);
    else if (want == ControlModeKind::kVelocity)  exec_.request_mode(&vel_);
    else                                          exec_.request_mode(&pos_);
    active_mode_kind_.store(want);
    std::this_thread::sleep_for(std::chrono::duration_cast<clock::duration>(
        std::chrono::duration<double>(cfg_.mode_settle_s)));
  }
  // Serialised against close_stream()'s teardown, and taken AFTER the settle sleep
  // so an e-stop's halt never waits on it. On the designed hiccup-recovery path
  // (expiry closes the session, the client immediately re-opens) an in-flight close
  // would otherwise land its set_command_timeout(-1.0) on the FRESHLY opened
  // session, silently downgrading its watchdog to the mode default.
  std::lock_guard<std::mutex> l(stream_mtx_);
  // One deadline, pushed into the mode so it can make the OUTPUT safe at 1 kHz
  // while the session handles lifecycle at sampler rate. set_command_timeout is
  // CommandWatchdog::arm under the hood -- see Task 4.
  if      (want == ControlModeKind::kPosition)  pos_.set_command_timeout(r.timeout_s);
  else if (want == ControlModeKind::kImpedance) imp_.set_command_timeout(r.timeout_s);
  else if (want == ControlModeKind::kTorque)    tau_.set_command_timeout(r.timeout_s);
  else if (want == ControlModeKind::kVelocity)  vel_.set_command_timeout(r.timeout_s);

  const StreamOpenResult res = session_.open(r, secs_since(t0_));
  if (!res.accepted) {
    // Refused AFTER the mode switch and the re-arm: hand the mode straight back to
    // its own supervision rather than leaving an armed watchdog with no session
    // behind it. (-1.0 restores the configured default; 0.0 would disable it.)
    if      (want == ControlModeKind::kPosition)  pos_.set_command_timeout(-1.0);
    else if (want == ControlModeKind::kImpedance) imp_.set_command_timeout(-1.0);
    else if (want == ControlModeKind::kTorque)    tau_.set_command_timeout(-1.0);
    else if (want == ControlModeKind::kVelocity)  vel_.set_command_timeout(-1.0);
    return res;
  }
  stream_open_.store(true);                        // marked LAST
  return res;
}

void Supervisor::on_stream_close(const StreamCloseRequest&) {
  close_stream(StreamCloseCause::kClientRequest);
}

// One teardown, four callers: graceful close, deadline expiry, IK fault, on_halt.
void Supervisor::close_stream(StreamCloseCause cause) {
  // Taken BEFORE the exchange so the whole teardown is atomic against a re-open:
  // otherwise a close that had already marked the session shut could still be
  // running its disarm when on_stream_open re-armed the watchdog, and would then
  // overwrite it. Also keeps in-flight setpoints out of the hold latch.
  std::lock_guard<std::mutex> l(stream_mtx_);
  if (!stream_open_.exchange(false)) return;       // marked FIRST: setpoints are refused from here
  close_cause_.store(cause);
  session_.close();
  // Latch the safe state EXPLICITLY rather than relying on what each mode happens
  // to do when its watchdog is disarmed: impedance stays frozen at measured q,
  // position would resume slewing toward the last streamed target. The spec gives
  // the session the lifecycle, so the teardown owns the hold. Written before the
  // disarm below so the mode never sees an un-held cycle.
  const ControlModeKind running = active_mode_kind_.load();
  if (kinova::JointTargetSink* sink = sink_for(running)) {
    JointFeedback fb; const bool ok = snap_.load(fb);
    if (!ok && !have_hold_q_) {
      // FIRST close and a failed Seqlock read: there is no last-good q yet. Skipping
      // the hold here is not an option -- position mode would disarm, un-stale, and
      // slew back toward the last streamed setpoint AFTER the session closed. The
      // running mode's own reference is always valid, so hold there instead.
      stream_hold_q_ = (running == ControlModeKind::kImpedance) ? imp_.reference()
                                                                : pos_.reference();
    }
    stream_hold_q_ = sampled_q(ok, fb.q, stream_hold_q_);   // no phantom zero, ever
    have_hold_q_ = true;
    sink->set_target(stream_hold_q_);              // hold at MEASURED q
  }
  // Velocity mode has no joint target to hold either, but unlike torque its
  // safe-stop is not "restore the default and let it ramp" -- it is zero velocity,
  // commanded directly, so a closed session can never leave the arm coasting on
  // its last streamed velocity.
  if (running == ControlModeKind::kVelocity) vel_.set_velocity_target(JointVec::Zero());
  // Hand the mode back to its OWN supervision: a negative argument restores the
  // mode's configured cmd_timeout_s. Passing 0.0 would disable the watchdog
  // outright and silently destroy a timeout somebody set at construction.
  // Torque mode has no joint target to hold; restoring its default is what makes
  // it safe, by ramping the feedforward to zero, i.e. gravity-comp hold.
  if (running == ControlModeKind::kPosition)  pos_.set_command_timeout(-1.0);
  if (running == ControlModeKind::kImpedance) imp_.set_command_timeout(-1.0);
  if (running == ControlModeKind::kTorque)    tau_.set_command_timeout(-1.0);
  if (running == ControlModeKind::kVelocity)  vel_.set_command_timeout(-1.0);
  // Re-arm the IK latch. on_enter is otherwise its only reset, and on_stream_open
  // re-enters a mode only when the KIND changes -- so without this, one IK fault
  // would make every future kEePose/kPosition session close on its first sampler
  // tick, for the life of the process. Written after the hold above, by which point
  // the mode's target source is a joint target and no further solve can re-latch
  // it. Unconditional: pos_ owns the flag whichever mode was running.
  pos_.clear_ik_fault();
}

// Each setpoint admits through the session, then writes the sink DIRECTLY from this
// (backend) thread. Sound because sessions and trajectory goals are mutually
// exclusive, so the sampler writes no targets while a session is open.
void Supervisor::on_setpoint_joint_position(const JointSetpoint& s) {
  std::lock_guard<std::mutex> l(stream_mtx_);
  if (!session_.admit(SetpointKind::kJointPosition, secs_since(t0_))) return;
  // sink_for() is the ONE map from mode kind to joint sink -- the same one the halt
  // and close paths use. The old binary "impedance or else" was correct only because
  // pair_supported (another translation unit) confines kJointPosition to position and
  // impedance; safety must not depend on a table somewhere else. nullptr here means
  // the running mode has no joint target, so the setpoint is dropped rather than
  // written into a mode the executor is not running.
  if (kinova::JointTargetSink* sink = sink_for(session_.control_mode())) sink->set_target(s.values);
}
void Supervisor::on_setpoint_pose(const PoseSetpoint& s) {
  std::lock_guard<std::mutex> l(stream_mtx_);
  if (!session_.admit(SetpointKind::kEePose, secs_since(t0_))) return;
  // nullptr means the running mode has no pose sink, so the setpoint is dropped
  // rather than written into a mode the executor is not running.
  if (kinova::PoseTargetSink* sink = pose_sink_for(session_.control_mode()))
    sink->set_target(s.pose);
}
void Supervisor::on_setpoint_joint_torque(const JointSetpoint& s) {
  std::lock_guard<std::mutex> l(stream_mtx_);
  if (!session_.admit(SetpointKind::kJointTorque, secs_since(t0_))) return;
  tau_.set_torque(s.values);
}
void Supervisor::on_setpoint_joint_velocity(const JointSetpoint& s) {
  std::lock_guard<std::mutex> l(stream_mtx_);
  if (!session_.admit(SetpointKind::kJointVelocity, secs_since(t0_))) return;
  if (session_.control_mode() == ControlModeKind::kVelocity) vel_.set_velocity_target(s.values);
}
void Supervisor::on_setpoint_twist(const TwistSetpoint& s) {
  std::lock_guard<std::mutex> l(stream_mtx_);
  if (!session_.admit(SetpointKind::kEeTwist, secs_since(t0_))) return;
  if (session_.control_mode() == ControlModeKind::kVelocity) vel_.set_twist_target(s.twist);
}
}  // namespace kinova::interface
