# Arm Driver Interface — Design Spec (v1: ROS2 Joint-Trajectory Execution)

**Date:** 2026-08-10
**Status:** Approved for planning (design locked; consolidates brainstorm decisions 1–9)
**Scope:** The **interface layer** that sits on top of the low-level driver so
external consumers can command the arm and observe its state over a real
transport. v1 delivers **joint-space trajectory execution driven by a motion
planner (cuRobo), over ROS2**, plus a live telemetry stream. The transport is
abstracted behind thin ports so ATOS/socket can follow later, but **only the
ROS2 backend is built in v1**.

Everything the interface adds lives in **non-RT** threads and touches the 1 kHz
RT loop only through the driver's existing lock-free seams (`ControlMode`
setters in; `SampleRing` / a feedback tap out). **No change to `RtExecutor`, the
RT cycle, or the RT-safety contract.**

The full context, alternatives, and rationale for every choice below live in the
running decision log: `SHARED - RAMMP/ARM test environment/Arm Driver Interface -
Design Decisions.md` (Obsidian). This spec is the consolidated, buildable design.

## Goal

Make the driver **usable and testable by real downstream consumers** without each
one hand-rolling its own bridge (as `teleop_socket_server` did). Concretely, v1
lets a motion planner send a planned joint trajectory and have the arm execute it
smoothly, report progress/result, and stream state back — over ROS2, using
conventions the ROS ecosystem (and our ATOS middleware) already understand.

This is the foundational interface every higher-level stack will build on, so it
holds the driver's bar: **stability and deterministic 1 kHz timing are not
negotiable**, and the transport boundary is kept clean so the stack can evolve.

## Out of scope (explicitly deferred)

- **Control-mode designs** — `JointImpedanceMode` and `JointPositionMode` are a
  **dependency**, specified as separate control-mode work (see *Dependencies*).
  This spec consumes them; it does not design them.
- **Multi-sender arbitration** — a real, near-term, cross-cutting need (multiple
  users/surfaces may command the arm), but its own follow-on spec. v1 leaves the
  **hooks** (sender identity + a pluggable admit seam) and nothing more.
- **Socket and ATOS backends** — the port abstraction keeps them possible; v1
  does not build them and does not contort the design to prove them.
- **Streaming-setpoint input** (the low-latency reactive tier: teleop, IMU
  closed-loop, learned policies) — a separate command input behind the same
  ports, designed later.
- **Task-space trajectories** — feeding Cartesian paths to the Cartesian
  impedance controller. Later variant.
- **Host-side tooling** (jog UI, live plots, record/replay) — the eventual eval
  experience; a consumer of this interface, designed later.

## Dependencies (referenced, specified elsewhere)

The interface layer requires two joint-space `ControlMode` implementations. Their
internals are out of scope; the interface depends only on this contract:

- **`JointImpedanceMode`** — torque control (`required_modes()` → `kTorque`):
  gravity comp + joint-space PD toward a commanded joint target. Compliant.
  Natural sibling of the `JointTorqueMode` currently being hardened.
- **`JointPositionMode`** — actuator position servoing (`required_modes()` →
  `kPosition`): tracks a commanded joint target stiffly.
- **Contract both must expose:** a non-RT, single-writer setter to publish the
  current joint target — the joint-space analogue of
  `CartesianImpedanceMode::set_target(Pose)`, e.g. `set_joint_target(const
  JointVec&)`, RT-safe on the read side (double-buffer/seqlock snapshot read
  once per `compute`). The interface's sampler calls this each tick.

## Approved design decisions (condensed)

1. **v1 consumer = motion planner (cuRobo).** Teleop generalized to "streaming EE
   command from a human-driven device" (deferred). *(Decision 1)*
2. **Driver executes trajectories** — a non-RT sampler feeds the mode's setter;
   fire-and-forget/action semantics; RT loop untouched. *(Decision 2)*
3. **Joint-space execution on a consumer-selected controller** — position or
   impedance. Task-space is a later variant. *(Decision 3)*
4. **Completion = time-based** (final timestamp) with a **divergence guard**
   (path-tolerance violation → error); **preemption is per-trajectory**
   (queue | latest-wins); **queue is gapless** (planner owns seam continuity).
   *(Decision 4)*
5. **Telemetry = two interface channels** (action feedback + live state stream)
   **plus a separate local full-rate log** (SysID/benchmark; not the interface).
   Keeping 1 kHz off the network is a performance default, not a hard rule.
   *(Decision 5)*
6. **Thin Action/Stream/Service ports + DI (hexagonal).** Driver core links no
   transport. *(Decision 6)*
7. **Follow ROS2 action semantics fully; ROS2-first backend.** Reuse the standard
   `control_msgs/FollowJointTrajectory` messages. *(Decision 7)*
8. **Custom action wrapping the standard FJT messages; self-contained** — mode
   selection lives *in the goal*, not in stateful controller switching.
   *(Decision 8)*
9. **Arbitration deferred to its own spec**; v1 leaves identity + a policy hook.
   *(Decision 9)*

---

## Architecture — four layers, hexagonal

```
        consumer (cuRobo / any ROS2 client)
                    │  ROS2 action / topics / services
        ┌───────────┴────────────┐
        │  B: Ros2Backend        │   adapter — the ONLY unit that includes ROS2
        └───────────┬────────────┘
     driven ports ▲ │ ▼ driving ports        (A: thin transport-agnostic ports)
        ┌───────────┴────────────┐
        │  C: Supervisor         │   owns threads; binds commands ↔ RT seams
        │   - trajectory sampler │
        │   - telemetry pump     │
        └───────────┬────────────┘
        set_target ▲│▼ FeedbackTap/Seqlock    (existing lock-free seams)
        ┌───────────┴────────────┐
        │  RtExecutor (1 kHz RT) │   UNCHANGED
        │  ControlMode, Dynamics │
        │  Transport (Kortex/Sim)│
        └────────────────────────┘
```

- **A — Semantic Control API (ports).** Three pure-virtual C++ port interfaces
  (`Action`, `Stream`, `Service`) plus the plain-C++ command/telemetry value
  types. No transport types. The seam is **bidirectional** (Component 1).
- **B — Transport backend.** `Ros2Backend` implements the ports using rclcpp
  action server / publishers / services. The **only** unit that includes ROS2;
  its own translation unit, conditionally compiled (mirrors how
  `kortex_transport.cpp` is only built under `KINOVA_ENABLE_KORTEX`).
- **C — Supervisor (driver binding).** Owns the non-RT threads; translates
  inbound commands into `ControlMode` setter calls (single-writer) and pumps
  telemetry out. Generalizes what `teleop_socket_server` prototyped.
- **D — Host tooling.** Out of scope; a future consumer of B.

**Governing invariant (whole layer):** the driver library (`kinova_lowlevel`)
and the supervisor depend only on Layer A. No ROS/ATOS/socket type crosses into
the core. Backends are injected at the composition root (`main`).

---

## Component 1 — Semantic Control API (Layer A: the ports)

Ports & adapters (hexagonal). Two directions, both dependency-injected:

- **Driven ports** (driver → transport, outbound). The supervisor *calls* these
  to push data out:
  - `StreamPort::publish_state(const ArmState&)` — the live state stream.
  - `ActionServerPort::publish_feedback(GoalId, const TrajectoryFeedback&)` and
    `::settle(GoalId, const TrajectoryResult&)` — action feedback + result.
- **Driving ports** (transport → driver, inbound). The supervisor *implements*
  these; the backend *calls* them when a message arrives:
  ```cpp
  class CommandSink {                       // implemented by the supervisor
   public:
    // Action lifecycle (mirrors ROS2 action server callbacks):
    virtual GoalResponse  on_trajectory_goal(const TrajectoryGoal&) = 0;   // accept/reject
    virtual void          on_trajectory_accepted(GoalId) = 0;              // begin executing
    virtual CancelResponse on_trajectory_cancel(GoalId) = 0;
    // Services (request/reply):
    virtual GainsResult   on_set_gains(const GainsRequest&) = 0;
    virtual ArmState      on_query_state() = 0;
  };
  ```

**Value types** are plain, transport-free structs mirroring the ROS2 message
*content* (so the mapping in B is a field copy, not a translation): `TrajectoryGoal`
(waypoints, tolerances, `control_mode`, `preemption`, `gains`, `sender_id`),
`TrajectoryFeedback` (desired/actual/error + `fraction_complete`),
`TrajectoryResult` (error code + string + final error), `ArmState` (q, q̇, EE
pose, measured torque, fault flags, timestamp).

**DI wiring** (in `main`): construct `Ros2Backend`, construct the `Supervisor`
against the backend's driven ports, register the supervisor's `CommandSink` with
the backend. Neither side names a concrete transport.

---

## Component 2 — The `ExecuteJointTrajectory` action (the command)

**Governing decision:** a **custom action composed from the standard FJT
messages** — maximally compliant at the message level, custom only at the action
type, and **self-contained** (no stateful controller switch). ROS2 actions can't
nest, but they are built from messages, and messages compose.

```
# ExecuteJointTrajectory.action
# ---------- Goal ----------
trajectory_msgs/JointTrajectory       trajectory          # reused verbatim from FJT
control_msgs/JointTolerance[]         path_tolerance      # reused from FJT
control_msgs/JointTolerance[]         goal_tolerance      # reused from FJT
builtin_interfaces/Duration           goal_time_tolerance # reused from FJT
uint8   control_mode        # 0 = POSITION, 1 = IMPEDANCE      (our addition)
uint8   preemption          # 0 = QUEUE,    1 = LATEST_WINS    (our addition)
JointImpedanceGains gains   # used iff control_mode == IMPEDANCE (our addition)
string  sender_id           # arbitration hook (policy deferred) (our addition)
---
# ---------- Result ----------   (identical shape to FJT)
int32   error_code          # SUCCESSFUL=0, INVALID_GOAL=-1, PATH_TOLERANCE_VIOLATED=-4,
                            #  GOAL_TOLERANCE_VIOLATED=-5, PREEMPTED, ... (mirror FJT + our extras)
string  error_string
trajectory_msgs/JointTrajectoryPoint final_error
---
# ---------- Feedback ----------  (identical shape to FJT)
std_msgs/Header                        header
string[]                               joint_names
trajectory_msgs/JointTrajectoryPoint   desired
trajectory_msgs/JointTrajectoryPoint   actual
trajectory_msgs/JointTrajectoryPoint   error
float32                                fraction_complete
```

**Self-contained mode selection.** `control_mode` is read from the goal; the
supervisor activates the matching `ControlMode` internally via
`RtExecutor::request_mode` (atomic pointer swap, adopted at a cycle boundary).
There is no external, stateful "switch controller then send trajectory" step —
the command fully describes its own intent. (A stock generic-FJT client cannot
talk to this action without a thin shim; acceptable — cuRobo is our client and we
own it.)

**Gains authority (resolves a two-paths ambiguity).** For v1, the gains in the
**goal are authoritative** for that trajectory — consistent with the
self-contained rule. The set-gains *service* (Component 1) sets only the
**defaults** applied when a goal omits gains, and exists mainly as the seam the
future streaming-setpoint tier will use; it is intentionally thin in v1.

---

## Component 3 — Trajectory execution (the sampler)

The supervisor runs a **non-RT sampler thread** per active goal.

- **Sampling:** interpolate `trajectory` at the current time → a joint target
  `q_d(t)` (+ `q̇_d` for the impedance feedforward if useful); call the active
  mode's `set_joint_target(q_d)` each tick. Sampler rate is decoupled from and
  well below 1 kHz (e.g. 200–500 Hz); the RT loop reads the latest snapshot.
- **Mode activation:** on `on_trajectory_accepted`, map `control_mode` →
  `JointPositionMode` | `JointImpedanceMode`, push its gains (impedance), then
  `request_mode`. The actuator-mode handshake (`kPosition`↔`kTorque`) happens
  once, at the cycle boundary on mode adoption — never at 1 kHz.
- **Completion = time-based:** the goal succeeds when the sampler passes the
  trajectory's final timestamp. No settle-wait (that was the source of the
  jerky/slow end-of-motion delay).
- **Divergence guard:** while sampling, if joint error exceeds `path_tolerance`,
  abort with `PATH_TOLERANCE_VIOLATED`. Optionally check `goal_tolerance` at the
  end and report `GOAL_TOLERANCE_VIOLATED` in the result (informational; still
  time-completed).
- **Preemption (per-goal flag):**
  - `LATEST_WINS` — a new accepted goal preempts the current (result
    `PREEMPTED`), and its sampler takes over. Matches reactive replanning.
  - `QUEUE` — the new goal is admitted but deferred; on the current goal's final
    timestamp the next begins **with zero gap** (gapless). The arm never
    decelerates to a stop at an interior boundary.
- **Mode changes require the arm to be at rest.** A goal whose `control_mode`
  differs from the currently active mode is **rejected** at `on_trajectory_goal`
  while any trajectory is in flight (executing or queued) — regardless of
  preemption flag. This avoids an actuator-mode handshake (`kPosition`↔`kTorque`)
  mid-motion. The consumer must let motion finish (or send a stop) before
  switching modes; a same-mode goal is unaffected.
- **Seam continuity is the planner's job:** cuRobo plans each segment from the
  current end state, so velocities match across a queued seam. The executor only
  guarantees *no dead-time*; it does not re-time or blend.

---

## Component 4 — Telemetry

Three paths; **only the first two are on the interface**:

1. **Action feedback** (driven `ActionServerPort`) — per active goal:
   desired/actual/error joint points + `fraction_complete`, at the sampler rate
   or a throttled subset.
2. **Live state stream** (driven `StreamPort`) — free-running **standard
   `sensor_msgs/JointState`** (q, q̇, measured effort) at a **configurable rate
   (~100–200 Hz default)**. EE pose and fault flags are **deferred** (expand
   later); in v1 a fault surfaces by aborting the active goal, and EE pose is
   available on demand via the query-state service. An opt-in **diagnostic**
   topic (loop-timing percentiles, per-cycle detail) is available for
   eval/tuning. Lossy-tolerant: the pump reads the latest `FeedbackTap`/`Seqlock`
   snapshot; a slow consumer drops frames, it never back-pressures the RT loop.
3. **Full-rate local logging** (NOT the interface) — 1 kHz, lossless, to local
   disk, via the existing `SampleRing` → non-RT drain. For SysID/benchmark.
   Extended with the signal channels SysID needs (q, q̇, τ_cmd, τ_meas). A future
   high-rate *network* path is not precluded, just not the default.

The state pump and each feedback source that computes FK gets its **own
`Dynamics`** instance (Pinocchio FK is not thread-safe — see Component 6).

---

## Component 5 — `Ros2Backend` (the transport adapter)

The only unit that includes ROS2. Implements Layer A over rclcpp:

- **Action port** → an rclcpp **action server** for `ExecuteJointTrajectory`. Its
  `handle_goal`/`handle_accepted`/`handle_cancel` callbacks call the supervisor's
  `CommandSink`; `publish_feedback`/goal `succeed`/`abort`/`canceled` are driven
  by the supervisor via `ActionServerPort`. Goal UUIDs supply the identity the
  arbitration hook will use.
- **Stream port** → an rclcpp **publisher** of standard `sensor_msgs/JointState`
  (v1); EE pose / faults deferred; diagnostic on a second topic.
- **Service port** → rclcpp **services** for set-gains and query-state.
- **Composition:** built in its own translation unit; a
  `KINOVA_ENABLE_ROS2`-style CMake option pulls in `ament`/`rclcpp` only when
  building the backend, exactly as `KINOVA_ENABLE_KORTEX` gates KORTEX. The
  sim/default build does not require ROS2.

Message mapping is a field copy (value types were defined to mirror FJT content),
so the adapter stays thin.

---

## Component 6 — Driver binding & threading (Layer C supervisor)

Owns all non-RT threads and enforces the seam rules the RT core relies on.

- **Single-writer command seam.** Exactly one thread calls a given mode's
  `set_joint_target` (the active sampler). Mode activation via `request_mode`.
  This preserves the existing single-writer contract on the mode setters.
- **Per-thread `Dynamics`.** Every thread that calls FK/Jacobian owns its own
  `Dynamics` (Pinocchio mutates internal state; it is not thread-safe). At least:
  the state pump. The RT thread keeps its own, as today.
- **Telemetry tap.** A `FeedbackTap`-style Transport decorator snapshots `fb`
  where the RT loop reads it, published through a `Seqlock` to the state pump —
  no driver-core change (proven by `teleop_socket_server`).
- **Lifecycle & safety.** Connect/servo/shutdown as today; on fault
  (`fb.fault`) the active goal aborts and the supervisor stops sampling. Clean
  shutdown joins sampler/pump threads before `safe_shutdown`.

---

## RT-safety invariants (the bar)

- **Nothing in this layer runs on the RT thread.** Sampler, pump, backend, and
  ROS executor are all non-RT. The RT cycle is byte-for-byte unchanged.
- **Seams are lock-free and bounded.** Commands cross in via the existing
  double-buffer/atomic mode setters (single writer); telemetry crosses out via
  `Seqlock`/`SampleRing` (drop-don't-block). No new allocation, lock, or blocking
  call on the RT path.
- **Verification gate.** `rt_safety_test` (zero major page faults, zero dropped
  samples in steady state) must still pass with the supervisor running against
  `SimTransport`; add a supervisor-in-the-loop variant. Any timing-relevant
  change is benchmarked before/after per the driver's standing rule.

---

## Arbitration hooks (policy deferred — Decision 9)

v1 builds no arbitration policy but must not design it out:

- **Identity in every command** — `sender_id` in the goal (+ the ROS2 goal UUID).
- **A single admit seam** — `CommandSink::on_trajectory_goal` is the one place
  goals are accepted/rejected; v1's policy is "single active commander"
  (the queue/latest-wins rule). A future arbitration policy object is consulted
  here without restructuring.

---

## Testing strategy

- **Unit (no robot):** value-type ↔ ROS2 message mapping round-trips; sampler
  interpolation + completion timing; divergence-abort logic; preemption
  (queue gapless vs latest-wins) as pure logic over a fake clock.
- **Integration (SimTransport, no robot):** supervisor + sampler + mode +
  `RtExecutor` + `SimTransport`, driven by a real ROS2 action client sending an
  `ExecuteJointTrajectory` goal; assert feedback stream, time-based completion,
  and a forced-divergence abort.
- **RT-safety:** supervisor-in-the-loop variant of `rt_safety_test` — zero major
  faults / zero drops in steady state with the sampler + pump active.
- **On-robot (attended only):** documented, not run unattended, per the
  integration runbook.

---

## Resolved during review

- **Mid-motion mode switch → resolved: reject.** A goal that changes
  `control_mode` is rejected while any trajectory is in flight; mode changes
  require the arm at rest (Component 3).
- **State-stream message → resolved: standard `sensor_msgs/JointState` for v1**;
  EE pose / faults deferred, expand later (Component 4).

## Open questions / risks

- **ROS2 build integration.** The driver builds via plain CMake today; the
  backend needs `ament`/`rclcpp`. Settled direction: isolate it behind a CMake
  option (mirroring `KINOVA_ENABLE_KORTEX`) so the default sim build stays
  ROS-free — to be confirmed against the actual `ament`/CMake interop when the
  backend is built.

---

## Reference

Running decision log (context + alternatives + rationale, Decisions 1–9):
`SHARED - RAMMP/ARM test environment/Arm Driver Interface - Design Decisions.md`
(Obsidian, RAMMP vault).
