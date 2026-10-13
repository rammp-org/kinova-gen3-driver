#include "kinova_lowlevel/interface/supervisor.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <string>

#include "kinova_lowlevel/cartesian.h"  // ee_wrench_from_residual
namespace kinova::interface {
using clock = std::chrono::steady_clock;
static double secs_since(clock::time_point t0) {
  return std::chrono::duration<double>(clock::now() - t0).count();
}

static const char* halt_reason_string(HaltReason r) {
  switch (r) {
    case HaltReason::kOwnershipRevoked:
      return "halted: ownership revoked";
    case HaltReason::kEmergencyStop:
      return "halted: emergency stop";
    case HaltReason::kOperatorRequest:
      return "halted: operator request";
  }
  return "halted";
}

namespace {
// Named so the throw says which field, not merely that something was null.
template <typename T>
T& require(T* p, const char* field) {
  if (!p)
    throw std::invalid_argument(std::string("SupervisorDeps::") + field +
                                " is required and was null");
  return *p;
}
}  // namespace

Supervisor::Supervisor(const SupervisorDeps& d)
    : pos_(require(d.pos, "pos")),
      imp_(require(d.imp, "imp")),
      tau_(require(d.tau, "tau")),
      vel_(require(d.vel, "vel")),
      exec_(require(d.exec, "exec")),
      snap_(require(d.snap, "snap")),
      pump_dyn_(require(d.pump_dyn, "pump_dyn")),
      stream_(require(d.stream, "stream")),
      action_(require(d.action, "action")),
      grip_(d.grip),
      cfg_(d.cfg) {
  // Cache which joints wrap (and the limits themselves -- the sampler's
  // compliant-velocity integrate step clamps against them), once, from the same
  // URDF source JointPositionMode reads. The executor's divergence guard needs
  // the wrap flags; see TrajectoryExecutor::tick. Dynamics is not thread-safe
  // against the RT loop, so this is read here in the constructor rather than
  // per mode switch.
  pump_dyn_.joint_limits(q_lower_, q_upper_);
  pump_dyn_.velocity_limits(v_max_);
  for (int i = 0; i < kinova::kNumJoints; ++i)
    continuous_[i] = !std::isfinite(q_lower_[i]) && !std::isfinite(q_upper_[i]);
  // The session default = MEDIUM GAINS over the mode's CONSTRUCTED params.
  // Seeding from profile_params alone made the first kSessionDefault
  // resolution replace the whole struct, silently resetting every
  // deployment-tuned non-gain field -- ik limits, cmd_timeout_s, ramp -- to
  // struct defaults (review finding).
  session_default_params_ =
      overlay_profile_gains(imp_.params(), profile_params(GainsProfile::kMedium));
}
Supervisor::~Supervisor() { stop(); }

void Supervisor::start() {
  exec_.request_mode(&pos_);         // initial mode = position
  traj_.emplace(pos_, continuous_);  // executor bound to the active mode's sink
  active_mode_kind_.store(ControlModeKind::kPosition);
  traj_bound_kind_ = ControlModeKind::kPosition;
  t0_ = clock::now();  // origin for every session/expiry stamp
  running_.store(true);
  sampler_ = std::thread([this] { sampler_loop(); });
  pump_ = std::thread([this] { pump_loop(); });
}
void Supervisor::stop() {
  if (!running_.exchange(false)) return;
  if (sampler_.joinable()) sampler_.join();
  if (pump_.joinable()) pump_.join();
}

void Supervisor::pump_loop() {
  const auto period = std::chrono::duration<double>(1.0 / cfg_.pump_hz);
  const auto t0 = clock::now();
  while (running_.load(std::memory_order_acquire)) {
    JointFeedback fb;
    if (snap_.load(fb)) {
      ArmState s;
      s.q = fb.q;
      s.qd = fb.qd;
      s.tau = fb.tau;
      s.fault = fb.fault;
      s.stamp_s = secs_since(t0);
      {
        // Shared with the sampler's twist resolution; see dyn_mtx_ in the header.
        std::lock_guard<std::mutex> dl(dyn_mtx_);
        s.ee_pose = pump_dyn_.fk(fb.q);
        pump_dyn_.jacobian(fb.q, pump_J_);
        // Under dyn_mtx_ like fk/jacobian: the sampler shares pump_dyn_ (and
        // its Pinocchio Data scratch), so gravity() must not race its ticks.
        pump_dyn_.gravity(fb.q, pump_g_);
      }
      s.ee_twist = pump_J_ * fb.qd;
      // Quasi-statics in the one normalized convention (JointFeedback::tau):
      // motor + external = gravity, so the externally applied joint torque is
      // g(q) - tau. See ArmState::ee_wrench. Under a fault the torque feedback
      // may be stale or zeroed, so publish NaN -- a runtime-checkable "no
      // measurement", not a confident garbage number beside a fault flag.
      if (fb.fault)
        s.ee_wrench.setConstant(std::numeric_limits<double>::quiet_NaN());
      else
        s.ee_wrench = kinova::ee_wrench_from_residual(pump_J_, pump_g_ - fb.tau);
      s.speed_override = speed_override_.load();
      state_snap_.store(s);
      stream_.publish_state(s);
    }
    std::this_thread::sleep_for(std::chrono::duration_cast<clock::duration>(period));
  }
}

void Supervisor::sampler_loop() {  // fleshed out in Tasks 6-9
  const auto period = std::chrono::duration<double>(1.0 / cfg_.sampler_hz);
  const auto t0 = clock::now();
  GoalId active_id{};
  bool have_active = false;
  JointVec q_meas = JointVec::Zero();  // last-good measured q; reused when a snapshot read fails
  bool have_q_meas = false;            // false until the FIRST successful snapshot read:
                                       // q_meas is still the Zero initializer before that,
                                       // and nothing downstream may treat it as a posture
  GoalId queued_id{};
  bool have_queued = false;
  ImpedanceGains queued_gains{};  // the queued goal's spec, resolved and applied at promotion
  auto last_tick = clock::now();  // for the compliant-velocity integration dt
  while (running_.load(std::memory_order_acquire)) {
    // Measured wall dt, one per iteration. The loop paces with sleep_for, so the
    // real period jitters above 1/sampler_hz under load; integrating with the
    // NOMINAL period would make the commanded velocity rate-inaccurate by exactly
    // that jitter. A stall cannot produce a reference jump: the leash bounds the
    // lead regardless of dt.
    const auto tick_now = clock::now();
    const double tick_dt = std::chrono::duration<double>(tick_now - last_tick).count();
    last_tick = tick_now;
    // 0) a halt jumps the queue: settle everything ACCEPTed, then hold where the arm IS.
    bool halt = false;
    HaltReason hr = HaltReason::kOwnershipRevoked;
    {
      std::lock_guard<std::mutex> l(q_mtx_);
      if (halt_pending_) {
        halt = true;
        hr = halt_reason_;
        halt_pending_ = false;
      }
    }
    if (halt) {
      TrajectoryResult r;
      r.error_code = result_code::kHalted;
      r.error_string = halt_reason_string(hr);
      if (have_active) action_.settle(active_id, r);
      if (have_queued)
        action_.settle(queued_id, r);  // ACCEPTed already; dropping it orphans the client
      // ONE load of the running mode: a backend on_stream_open landing between two
      // reads would bind traj_ to one sink and record the OTHER kind, which is the
      // silent mis-mapping traj_bound_kind_ exists to prevent.
      const ControlModeKind k = active_mode_kind_.load();
      kinova::JointTargetSink* sink = sink_for(k);
      // A kind with no joint sink (kTorque) still needs traj_ reset to idle; pos_ is
      // an inert placeholder there, because traj_bound_kind_ = k forces a rebind
      // before any position/impedance goal can be driven through it.
      traj_.emplace(sink ? *sink : static_cast<kinova::JointTargetSink&>(pos_), continuous_);
      traj_bound_kind_ = k;
      have_active = false;
      have_queued = false;
      in_flight_.store(false);
      JointFeedback fb;
      const bool ok = snap_.load(fb);
      q_meas = sampled_q(ok, fb.q, q_meas);  // never inject a phantom zero here of all places
      // Torque's safe-stop is gravity-comp hold, already restored by close_stream()
      // above -- it has no joint target to latch, so there is nothing to write.
      if (sink) sink->set_target(q_meas);  // hold at MEASURED q, not the last reference
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
    // 0d) compliant velocity/twist (#63): while a velocity-kind session runs in
    //     impedance, integrate the stored command into a leashed position
    //     reference and drive it into the mode -- the same mechanism
    //     JointVelocityMode runs inside compute() at 1 kHz, executed HERE
    //     because the client's send rate is irregular and the integration must
    //     not be. These writes also keep imp_'s own watchdog fresh, which is by
    //     design: the session deadline (0b above) is the staleness authority
    //     for these pairs, and its teardown latches the hold at measured q.
    if (stream_open_.load() && session_.control_mode() == ControlModeKind::kImpedance &&
        (session_.kind() == SetpointKind::kJointVelocity ||
         session_.kind() == SetpointKind::kEeTwist)) {
      JointFeedback fb;
      const bool ok = snap_.load(fb);
      have_q_meas = have_q_meas || ok;
      q_meas = sampled_q(ok, fb.q, q_meas);  // the leash needs MEASURED q, never a phantom zero
      std::lock_guard<std::mutex> l(stream_mtx_);
      // Re-checked under the lock: a close that landed since the test above has
      // already latched the hold, and one more target would overwrite it. The
      // KIND and MODE are re-read too: a close-plus-reopen that both landed in
      // the gap would otherwise have this tick integrate the dead session's
      // stale command into a different-kind session (review finding).
      // have_q_meas: before the FIRST good read, q_meas is the Zero
      // initializer and the leash would drag the reference toward the zero
      // posture (review finding) -- skip the tick, the mode keeps holding.
      if (stream_open_.load() && have_q_meas &&
          session_.control_mode() == ControlModeKind::kImpedance &&
          (session_.kind() == SetpointKind::kJointVelocity ||
           session_.kind() == SetpointKind::kEeTwist)) {
        JointVec qd = JointVec::Zero();
        if (session_.kind() == SetpointKind::kEeTwist) {
          {
            std::lock_guard<std::mutex> dl(dyn_mtx_);  // stream_mtx_ -> dyn_mtx_, never reversed
            pump_dyn_.jacobian(q_meas, sampler_J_);
          }
          stream_dls_.solve(sampler_J_, q_meas, stream_twist_cmd_, stream_dls_params_, continuous_,
                            qd);
        } else {
          qd = stream_qd_cmd_;
        }
        // Same cap the stiff velocity path runs: a wrong-units command gets
        // scaled to the URDF rating, loudly bounded instead of silently
        // integrated at whatever rate the leash lets it drag the arm.
        kinova::limit_joint_velocity(v_max_, qd);
        const JointVec q_ref_prev = stream_q_ref_;
        integrate_leashed_reference(stream_q_ref_, qd, tick_dt, q_meas, kinova::kVelocityRefMaxLead,
                                    continuous_, q_lower_, q_upper_);
        kinova::JointTarget t;
        t.q = stream_q_ref_;
        // Feed forward the rate the reference ACTUALLY advanced at, not the
        // commanded one. In free motion they are equal (no pulsing: the mode
        // holds this rate steady between writes). When the leash pins against
        // a blocked arm the achieved rate goes to zero, and the damper term
        // dies with it -- the commanded rate would keep a standing D*qd push
        // on top of the leash-bounded spring for as long as the client
        // streams, breaking the leash's "this bounds how hard the spring
        // pushes" contract (review finding).
        if (tick_dt > 0.0) {
          for (int i = 0; i < kinova::kNumJoints; ++i) {
            double step = stream_q_ref_[i] - q_ref_prev[i];
            if (continuous_[i]) step = kinova::wrap_to_pi(step);
            t.qd[i] = step / tick_dt;
          }
        }
        t.has_velocity = true;  // free feedforward: damp toward the achieved rate, not zero
        imp_.set_joint_target(t);
      }
    }
    // 1) drain inbox (only this thread touches traj_)
    for (;;) {
      Inbound in;
      {
        std::lock_guard<std::mutex> l(q_mtx_);
        if (inbox_.empty()) break;
        in = inbox_.front();
        inbox_.pop_front();
      }
      if (in.cancel) {
        // Abort the whole chain and reset the executor to idle; the mode keeps
        // commanding its last reference, so the arm holds where it was.
        if (have_active) {
          TrajectoryResult r;
          r.error_code = result_code::kPreempted;
          action_.settle(active_id, r);
        }
        if (have_queued) {
          TrajectoryResult r;
          r.error_code = result_code::kPreempted;
          action_.settle(queued_id, r);
        }
        const ControlModeKind k = active_mode_kind_.load();  // ONE load: see the halt path
        kinova::JointTargetSink* sink = sink_for(k);
        traj_.emplace(sink ? *sink : static_cast<kinova::JointTargetSink&>(pos_), continuous_);
        traj_bound_kind_ = k;
        have_active = false;
        have_queued = false;
        in_flight_.store(false);
        continue;
      }
      // A goal ACCEPTed before a stream opened, drained after. in_flight_ is set
      // HERE, not at accept time, so on_stream_open's in_flight_ check cannot see a
      // goal still sitting in inbox_ -- up to one sampler period (4 ms at 250 Hz)
      // wide. Executing it now would tick the trajectory into the very sink the
      // backend thread is streaming into: two writers, one double buffer, which is
      // exactly what the direct-write design says can never happen. Refuse instead.
      if (stream_open_.load()) {
        TrajectoryResult r;
        r.error_code = result_code::kInvalidGoal;
        r.error_string = "a streaming session opened before this goal could start";
        action_.settle(in.id, r);
        continue;
      }
      // Second-layer guard on the mode enum. on_trajectory_goal refuses kVelocity
      // and kTorque, but a backend may call on_trajectory_accepted without a
      // preceding accepted goal -- and the mode-switch branch below is binary, so
      // anything that is neither position nor impedance would be driven AS
      // position. Fail loud instead of silently mis-mapping.
      if (in.goal.control_mode != ControlModeKind::kPosition &&
          in.goal.control_mode != ControlModeKind::kImpedance) {
        TrajectoryResult r;
        r.error_code = result_code::kInvalidGoal;
        r.error_string = "trajectory execution supports position and impedance only";
        action_.settle(in.id, r);
        continue;
      }
      // Same second layer for the gains: this path is reachable without
      // on_trajectory_goal's accept-time checks (see the mode check above).
      // An out-of-enum profile would throw out of resolve_gains on THIS
      // thread, and unvalidated custom gains below the gravity floor would
      // reach the arm -- the exact #64 failure (review finding).
      if (in.goal.control_mode == ControlModeKind::kImpedance &&
          (!known_profile(in.goal.gains.profile) ||
           (in.goal.gains.profile == GainsProfile::kCustom &&
            !validate_custom(in.goal.gains.custom).ok))) {
        TrajectoryResult r;
        r.error_code = result_code::kInvalidGoal;
        r.error_string = "gains failed validation at execution";
        action_.settle(in.id, r);
        continue;
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
      // NOTE: gains are applied when the goal STARTS RUNNING, after submit
      // accepts (below) -- not here at drain. Applying at drain mutated the
      // RUNNING goal's compliance for a queued goal, and even for one rejected
      // two checks later (review finding). A queued goal's spec is stashed and
      // resolved at promotion, so kSessionDefault still means "the default
      // when the goal runs".
      if (in.goal.control_mode != traj_bound_kind_ ||
          in.goal.control_mode != active_mode_kind_.load()) {
        if (have_active) {  // cross-mode goal slipped past the accept-time pre-check (in_flight_
                            // lag); a mode change requires the arm at rest
          TrajectoryResult r;
          r.error_code = result_code::kInvalidGoal;
          r.error_string = "mode change while a trajectory is in flight";
          action_.settle(in.id, r);
          continue;
        }
        if (in.goal.control_mode == ControlModeKind::kImpedance) {
          exec_.request_mode(&imp_);
          traj_.emplace(imp_, continuous_);  // no-op in the executor if already active
          active_mode_kind_.store(ControlModeKind::kImpedance);
        } else {
          exec_.request_mode(&pos_);
          traj_.emplace(pos_, continuous_);
          active_mode_kind_.store(ControlModeKind::kPosition);
        }
        traj_bound_kind_ = in.goal.control_mode;
        std::this_thread::sleep_for(  // let the RT loop adopt + on_enter settle
            std::chrono::duration_cast<clock::duration>(
                std::chrono::duration<double>(cfg_.mode_settle_s)));
      }
      const SubmitResult sr =
          traj_->submit(in.goal.trajectory, in.goal.control_mode, in.goal.preemption,
                        in.goal.path_tolerance, in.goal.speed_scale);
      if (sr != SubmitResult::kAccepted) {
        TrajectoryResult r;
        r.error_code = result_code::kInvalidGoal;
        r.error_string = sr == SubmitResult::kRejectedSpeedScale
                             ? "speed_scale outside [kMinSpeedScale, 1.0]: refused, not clamped"
                             : "rejected by executor";
        action_.settle(in.id, r);
        continue;
      }
      if (!have_active) {
        // idle -> active: the executor adopts immediately regardless of preemption.
        // EVERY impedance goal applies its resolved gains as it starts -- mode
        // switch or not. No goal's gains outlive it: the next bare goal
        // resolves kSessionDefault and overwrites them.
        if (in.goal.control_mode == ControlModeKind::kImpedance)
          apply_impedance_gains(in.goal.gains);
        active_id = in.id;
        have_active = true;
        in_flight_.store(true);
      } else if (in.goal.preemption == Preemption::kLatestWins) {
        // Preempt the active goal; the executor also drops any queued follow-on.
        {
          TrajectoryResult r;
          r.error_code = result_code::kPreempted;
          action_.settle(active_id, r);
        }
        if (have_queued) {
          TrajectoryResult r;
          r.error_code = result_code::kPreempted;
          action_.settle(queued_id, r);
          have_queued = false;
        }
        // The latest-wins goal takes over at the next cycle boundary: its
        // gains are the running tuning from here.
        if (in.goal.control_mode == ControlModeKind::kImpedance)
          apply_impedance_gains(in.goal.gains);
        active_id = in.id;
      } else {
        // kQueue: this goal waits behind the active one. The executor overwrites any
        // prior queued goal, so settle the displaced one as preempted before overwriting.
        if (have_queued) {
          TrajectoryResult r;
          r.error_code = result_code::kPreempted;
          action_.settle(queued_id, r);
        }
        queued_id = in.id;
        queued_gains = in.goal.gains;  // applied at PROMOTION, not now: the
                                       // active goal keeps its own tuning
        have_queued = true;            // active_id / in_flight_ untouched
      }
    }
    // 2) tick the active trajectory
    if (traj_->is_active()) {
      JointFeedback fb;
      const bool ok = snap_.load(fb);  // sequence the read; don't rely on arg eval order
      have_q_meas = have_q_meas || ok;
      q_meas = sampled_q(ok, fb.q, q_meas);  // failed read -> reuse last-good q (no phantom zero)
      const ExecStatus st = traj_->tick(secs_since(t0), q_meas, speed_override_.load());
      TrajectoryFeedback fbk;
      fbk.actual = q_meas;
      fbk.fraction_complete = st.fraction;
      action_.publish_feedback(active_id, fbk);
      if (st.promoted) {
        // The active goal finished successfully and the queued goal took over gaplessly.
        {
          TrajectoryResult r;
          r.error_code = result_code::kSuccessful;
          action_.settle(active_id, r);
        }
        // The queued goal runs NOW, so its gains resolve now -- kSessionDefault
        // means the default at promotion, and the finished goal's tuning dies
        // with it. Queue promotion cannot cross modes (rejected at drain), so
        // the running mode is the stashed goal's mode.
        // Known window: st.promoted is reported AFTER the tick that executed
        // the promoted goal's first sample, so that one sampler period runs
        // under the finished goal's gains (the mode's gain ramp spans it).
        // Closing it means promoting outside the executor's tick -- not worth
        // the restructure for ~one period at a goal boundary.
        if (active_mode_kind_.load() == ControlModeKind::kImpedance)
          apply_impedance_gains(queued_gains);
        active_id = queued_id;
        have_queued = false;  // promoted goal is now active; have_active/in_flight_ stay true
      }
      if (st.completed && have_active) {
        TrajectoryResult r;
        r.error_code = (st.error_code == ExecStatus::kPathToleranceViolated)
                           ? result_code::kPathToleranceViolated
                           : result_code::kSuccessful;
        action_.settle(active_id, r);
        have_active = false;
        in_flight_.store(false);
        // A divergence abort drops any queued follow-on in the executor — settle it too.
        if (st.error_code == ExecStatus::kPathToleranceViolated && have_queued) {
          TrajectoryResult rq;
          rq.error_code = result_code::kPreempted;
          action_.settle(queued_id, rq);
          have_queued = false;
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::duration_cast<clock::duration>(period));
  }
}

// on_trajectory_goal (backend thread): fast pre-check only, no executor mutation.
GoalResponse Supervisor::on_trajectory_goal(const TrajectoryGoal& g) {
  if (stream_open_.load()) return GoalResponse::kReject;          // a stream owns the arm
  if (g.trajectory.points.empty()) return GoalResponse::kReject;  // INVALID_GOAL
  // Refused below kMinSpeedScale, not just at/below zero: effective_scale()
  // floors there internally, and accepting a request slower than the floor
  // would silently RUN it faster than asked -- the clamp this feature's
  // posture forbids. The floor must be unreachable from outside.
  if (!std::isfinite(g.speed_scale) || g.speed_scale < kMinSpeedScale || g.speed_scale > 1.0)
    return GoalResponse::kReject;  // out-of-range scale ([kMinSpeedScale, 1.0]); reason surfaces at
                                   // the ROS boundary
  if (g.control_mode == ControlModeKind::kVelocity || g.control_mode == ControlModeKind::kTorque) {
    return GoalResponse::kReject;  // trajectory execution is position/impedance only
  }
  // An out-of-enum profile byte must die here: resolve_gains on it throws on
  // the sampler thread, where nothing catches (std::terminate mid-motion).
  if (!known_profile(g.gains.profile)) return GoalResponse::kReject;
  // Gains that cannot act are a caller bug -- reject loudly, don't ignore.
  if (g.control_mode == ControlModeKind::kPosition &&
      g.gains.profile != GainsProfile::kSessionDefault)
    return GoalResponse::kReject;
  // Custom gains are bounds-checked at ACCEPT, so a bad request dies with the
  // goal response instead of reaching the arm (#64).
  if (g.control_mode == ControlModeKind::kImpedance && g.gains.profile == GainsProfile::kCustom &&
      !validate_custom(g.gains.custom).ok)
    return GoalResponse::kReject;
  // in_flight_ implies a goal is running, so a stream cannot be open and
  // active_mode_kind_ is one of the same two kinds g.control_mode was just
  // filtered to. Reading it directly keeps ONE record of the running mode.
  if (in_flight_.load() && g.control_mode != active_mode_kind_.load())
    return GoalResponse::kReject;  // mode-change-while-moving
  return GoalResponse::kAccept;
}
void Supervisor::on_trajectory_accepted(const GoalId& id, const TrajectoryGoal& g) {
  std::lock_guard<std::mutex> l(q_mtx_);
  inbox_.push_back({id, g, false});
}
CancelResponse Supervisor::on_trajectory_cancel(const CancelRequest& c) {
  std::lock_guard<std::mutex> l(q_mtx_);
  inbox_.push_back({c.id, {}, true});
  return CancelResponse::kAccept;
}
// on_halt (backend thread): latch + flush the queue, nothing else. The sampler owns
// traj_ and settle(), so the control action happens there -- which keeps
// settle-exactly-once true by construction rather than by careful reasoning.
void Supervisor::on_halt(HaltReason r) {
  close_stream(StreamCloseCause::kHalted);  // stop admitting setpoints BEFORE the hold is latched
  // Spec decision 5: stop stamping, do NOT command open. Opening is a motion, and
  // e-stop means stop moving; the 2F-85 self-locks, so ceasing to command holds the
  // grip. Deliberately not a "safe" open -- anything held stays held rather than
  // being dropped from wherever the arm happened to be.
  if (grip_) grip_->release();
  std::lock_guard<std::mutex> l(q_mtx_);
  inbox_.clear();  // a halt must never sit behind queued trajectories
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
    case ControlModeKind::kPosition:
      return &pos_;
    case ControlModeKind::kImpedance:
      return &imp_;
    case ControlModeKind::kTorque:
      return nullptr;
    case ControlModeKind::kVelocity:
      return nullptr;
  }
  return nullptr;
}
// The pose-target sink a control mode kind owns, EXPLICIT for the same reason
// sink_for is: an inline "impedance or else" ternary here would recreate exactly
// the binary mapping sink_for's own comment records as having been wrong before.
kinova::PoseTargetSink* Supervisor::pose_sink_for(ControlModeKind k) {
  switch (k) {
    case ControlModeKind::kImpedance:
      return &imp_;
    case ControlModeKind::kPosition:
      return &pos_;  // Plan 2: position gained IK
    case ControlModeKind::kVelocity:
    case ControlModeKind::kTorque:
      return nullptr;
  }
  return nullptr;
}
void Supervisor::apply_impedance_gains(const ImpedanceGains& s) {
  std::lock_guard<std::mutex> l(gains_mtx_);
  imp_.set_gains(resolve_gains(s, session_default_params_));
}
// Sets the SESSION DEFAULT -- what kSessionDefault resolves to from now on. It
// deliberately touches no live mode: a running impedance session keeps the
// tuning it opened with (open-time-only semantics, spec open item resolved);
// the next bare command picks the new default up.
GainsResult Supervisor::on_set_gains(const GainsRequest& r) {
  if (!known_profile(r.spec.profile))
    return {false, "unknown gains profile: expected soft, medium, stiff or custom"};
  if (r.spec.profile == GainsProfile::kSessionDefault)
    return {false, "set_gains needs a named profile or custom gains"};
  if (r.spec.profile == GainsProfile::kCustom) {
    const GainsCheck c = validate_custom(r.spec.custom);
    if (!c.ok) return {false, c.message};
  }
  std::lock_guard<std::mutex> l(gains_mtx_);
  session_default_params_ = resolve_gains(r.spec, session_default_params_);
  return {true, ""};
}
SpeedResult Supervisor::on_set_speed_override(const SpeedOverrideRequest& r) {
  return set_speed_override(r.scale, r.may_raise);
}
SpeedResult Supervisor::set_speed_override(double s, bool may_raise) {
  if (!std::isfinite(s)) return {false, "speed override must be finite"};
  // Same floor as on_trajectory_goal, and for the same reason: below
  // kMinSpeedScale, effective_scale()'s internal clamp would silently run the
  // arm FASTER than the caller asked for. Refuse it here too, so the floor is
  // unreachable from outside at either accept site. Ahead of the direction
  // check below: a below-floor or above-1.0 request is invalid regardless of
  // who is asking.
  if (s < kMinSpeedScale || s > 1.0)
    return {false, "speed override must be in [" + std::to_string(kMinSpeedScale) + ", 1.0]; got " +
                       std::to_string(s)};
  // Compare-and-store as ONE atomic operation against speed_override_, not a
  // load, then a separate compare, then a separate store: that would leave a
  // window between reading "current" and writing "new" for a concurrent
  // caller's store to land in, making the comparison stale by the time the
  // store happens -- which is exactly the race a caller-side snapshot
  // comparison reopened (see Arbiter::on_set_speed_override). Do not
  // "simplify" this back into a plain load-then-store.
  double cur = speed_override_.load(std::memory_order_acquire);
  do {
    if (!may_raise && s >= cur)
      return {false, "raising the speed override requires the current token"};
  } while (!speed_override_.compare_exchange_weak(cur, s));
  return {true, ""};
}
ArmState Supervisor::on_query_state() {
  ArmState s;
  state_snap_.load(s);
  return s;
}

void Supervisor::on_gripper_setpoint(const GripperSetpoint& s) {
  if (!grip_) return;  // no gripper on this robot: a no-op, not an error
  // Matches the six arm-setpoint siblings: GripperController::set_target is
  // documented as belonging to ONE non-RT thread (its double-buffer write is not
  // itself thread-safe against a second concurrent writer), and nothing upstream
  // of Supervisor guarantees that today -- an Arbiter happens to serialise via its
  // own mutex, but two of the three in-tree wirings have no Arbiter, and a
  // multi-threaded ROS2 executor could call in from two callback threads at once.
  // stream_mtx_ is reused rather than adding a second lock; on_halt's grip_->release()
  // is a single relaxed-enough atomic store with no lock of its own (see supervisor.h),
  // so this does not change its relationship to on_halt or introduce any new nesting.
  std::lock_guard<std::mutex> l(stream_mtx_);
  grip_->set_target(s.command);
}

GripperState Supervisor::on_query_gripper() {
  GripperState g;
  // present stays false. This is "no controller wired" (a robot built without a
  // gripper), not "no gripper attached" (present's other, hardware-detected meaning
  // from KortexTransport/SimTransport) -- the two are conflated here deliberately:
  // both cases mean there is nothing to report, and a caller has no use for telling
  // "unwired" apart from "unattached".
  if (!grip_) return g;
  JointFeedback fb;
  if (!snap_.load(fb)) return g;  // a torn read reports absent rather than garbage
  g.position = fb.gripper.position;
  g.effort = fb.gripper.effort;
  g.current = fb.gripper.current;
  g.present = fb.gripper.present;
  // NOTE the asymmetry with ArmState::stamp_s: that one is SAMPLE time, set once
  // inside the pump when fb was captured. This is QUERY time, computed here, on
  // whatever fb the last pump cycle happened to leave in snap_ -- same field name,
  // different meaning on two adjacent structs. Also: if called before start(), t0_
  // is still default-constructed, so this returns time-since-boot, not 0.
  g.stamp_s = secs_since(t0_);
  return g;
}

// Every accessor below is an atomic load, so this needs no lock and is safe to call
// from any thread -- which is the point: it is the only way a backend can tell an
// expired session from a live one.
StreamStatus Supervisor::on_query_stream() {
  return {session_.is_open(), session_.kind(), session_.control_mode(), session_.timeout_s(),
          session_.rejected_count()};
}

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
  if (!known_profile(r.gains.profile))
    return {false, result_code::kStreamRejected,
            "unknown gains profile: expected session-default, soft, medium, stiff or custom"};
  // Gains that cannot act are a caller bug -- reject loudly, don't ignore.
  if (r.control_mode != ControlModeKind::kImpedance &&
      r.gains.profile != GainsProfile::kSessionDefault)
    return {false, result_code::kStreamRejected, "gains supplied for a non-impedance stream"};
  if (r.control_mode == ControlModeKind::kImpedance && r.gains.profile == GainsProfile::kCustom) {
    const GainsCheck c = validate_custom(r.gains.custom);
    if (!c.ok) return {false, result_code::kStreamRejected, c.message};
  }
  // NOTE: the gains are applied AFTER session_.open accepts, below. Applying
  // here looked harmless but mutated a live impedance HOLD on a refused open
  // (review finding): the hold keeps running under gains of a session that
  // never existed. Setpoints only land once stream_open_ is true, so applying
  // after acceptance still precedes the first setpoint-driven cycle.

  // Switch modes BEFORE the session is marked open, so no setpoint can land mid-switch.
  const ControlModeKind want = r.control_mode;
  if (want != active_mode_kind_.load()) {
    if (want == ControlModeKind::kImpedance)
      exec_.request_mode(&imp_);
    else if (want == ControlModeKind::kTorque)
      exec_.request_mode(&tau_);
    else if (want == ControlModeKind::kVelocity)
      exec_.request_mode(&vel_);
    else
      exec_.request_mode(&pos_);
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
  if (want == ControlModeKind::kPosition)
    pos_.set_command_timeout(r.timeout_s);
  else if (want == ControlModeKind::kImpedance)
    imp_.set_command_timeout(r.timeout_s);
  else if (want == ControlModeKind::kTorque)
    tau_.set_command_timeout(r.timeout_s);
  else if (want == ControlModeKind::kVelocity)
    vel_.set_command_timeout(r.timeout_s);

  // Compliant velocity/twist (#63): the SAMPLER integrates, so seed its state
  // here, before the session becomes visible -- reference at measured q (the
  // first tick holds) and no stored command (a session that opens and says
  // nothing must not resume a velocity someone streamed last session). On a
  // torn first snapshot read, the mode's own reference is the honest fallback,
  // never a phantom zero. Harmless if session_.open refuses below: the sampler
  // only reads this state while a session is open.
  if (want == ControlModeKind::kImpedance &&
      (r.kind == SetpointKind::kJointVelocity || r.kind == SetpointKind::kEeTwist)) {
    JointFeedback fb;
    stream_q_ref_ = snap_.load(fb) ? fb.q : imp_.reference();
    stream_qd_cmd_.setZero();
    stream_twist_cmd_.setZero();
  }

  const StreamOpenResult res = session_.open(r, secs_since(t0_));
  if (!res.accepted) {
    // Refused AFTER the mode switch and the re-arm: hand the mode straight back to
    // its own supervision rather than leaving an armed watchdog with no session
    // behind it. (-1.0 restores the configured default; 0.0 would disable it.)
    if (want == ControlModeKind::kPosition)
      pos_.set_command_timeout(-1.0);
    else if (want == ControlModeKind::kImpedance)
      imp_.set_command_timeout(-1.0);
    else if (want == ControlModeKind::kTorque)
      tau_.set_command_timeout(-1.0);
    else if (want == ControlModeKind::kVelocity)
      vel_.set_command_timeout(-1.0);
    return res;
  }
  // The session exists from here on: its gains become the live tuning now,
  // before any setpoint can land (stream_open_ below is what admits them).
  if (r.control_mode == ControlModeKind::kImpedance) apply_impedance_gains(r.gains);
  stream_open_.store(true);  // marked LAST
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
  if (!stream_open_.exchange(false)) return;  // marked FIRST: setpoints are refused from here
  close_cause_.store(cause);
  session_.close();
  // Latch the safe state EXPLICITLY rather than relying on what each mode happens
  // to do when its watchdog is disarmed: impedance stays frozen at measured q,
  // position would resume slewing toward the last streamed target. The spec gives
  // the session the lifecycle, so the teardown owns the hold. Written before the
  // disarm below so the mode never sees an un-held cycle.
  const ControlModeKind running = active_mode_kind_.load();
  if (kinova::JointTargetSink* sink = sink_for(running)) {
    JointFeedback fb;
    const bool ok = snap_.load(fb);
    if (!ok && !have_hold_q_) {
      // FIRST close and a failed Seqlock read: there is no last-good q yet. Skipping
      // the hold here is not an option -- position mode would disarm, un-stale, and
      // slew back toward the last streamed setpoint AFTER the session closed. The
      // running mode's own reference is always valid, so hold there instead.
      stream_hold_q_ =
          (running == ControlModeKind::kImpedance) ? imp_.reference() : pos_.reference();
    }
    stream_hold_q_ = sampled_q(ok, fb.q, stream_hold_q_);  // no phantom zero, ever
    have_hold_q_ = true;
    sink->set_target(stream_hold_q_);  // hold at MEASURED q
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
  if (running == ControlModeKind::kPosition) pos_.set_command_timeout(-1.0);
  if (running == ControlModeKind::kImpedance) imp_.set_command_timeout(-1.0);
  if (running == ControlModeKind::kTorque) tau_.set_command_timeout(-1.0);
  if (running == ControlModeKind::kVelocity) vel_.set_command_timeout(-1.0);
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
// The two velocity kinds have TWO homes (streaming_session.cpp's pair table):
// kVelocity writes the mode's sink directly, exactly as before. kImpedance only
// STORES the command -- the sampler is the one writer of imp_'s joint target
// while such a session is open (it integrates at its own fixed rate; writing
// here too would put two writers on one single-writer double buffer).
void Supervisor::on_setpoint_joint_velocity(const JointSetpoint& s) {
  std::lock_guard<std::mutex> l(stream_mtx_);
  if (!session_.admit(SetpointKind::kJointVelocity, secs_since(t0_))) return;
  if (session_.control_mode() == ControlModeKind::kVelocity)
    vel_.set_velocity_target(s.values);
  else if (session_.control_mode() == ControlModeKind::kImpedance)
    stream_qd_cmd_ = s.values;
}
void Supervisor::on_setpoint_twist(const TwistSetpoint& s) {
  std::lock_guard<std::mutex> l(stream_mtx_);
  if (!session_.admit(SetpointKind::kEeTwist, secs_since(t0_))) return;
  if (session_.control_mode() == ControlModeKind::kVelocity)
    vel_.set_twist_target(s.twist);
  else if (session_.control_mode() == ControlModeKind::kImpedance)
    stream_twist_cmd_ = s.twist;
}
}  // namespace kinova::interface
