# Streaming Setpoints — Design Spec (the reactive tier)

**Date:** 2026-08-26
**Status:** Approved for planning
**Scope:** The low-latency streaming command tier in the driver core — the
"reactive" input the v1 interface spec deferred. First client is **visual
servoing**; teleop and learned policies are the obvious next ones. The ROS2
frontend is **out of scope**: this spec defines the core seams a transport binds
to.

This fills the second of the two deferrals in
[`2026-08-10-arm-driver-interface-design.md`](2026-08-10-arm-driver-interface-design.md):

> **Streaming-setpoint input** (the low-latency reactive tier: teleop, IMU
> closed-loop, learned policies) — a separate command input behind the same
> ports, designed later.

## Goal

Let an owner stream commands at its own rate — position, velocity, or torque, in
joint space or task space — into a running control mode, with a bounded and
explicit safety story when the stream stops.

## Dependencies

- **Arbitration (PR #25)** — every streaming call carries a `Token` and is gated
  by the `Arbiter`. Streaming is the surface that made arbitration urgent; it
  assumes that layer exists.
- **`JointTorqueMode` (PR #3, 97 commits behind main)** — torque streaming needs
  it. Rebased and landed **first, as its own change** (see *Decomposition*).

## Out of scope

- **The ROS2 frontend.** Topics, QoS, message types, node wiring.
- **Cartesian wrench streaming** (`tau = gravity(q) + Jᵀ F`). A deliberate
  future sibling, tracked separately; PR #3's own non-goals already identify the
  seam, so it slots into the same clamp-and-watchdog path without redesign.
- **`CartesianImpedanceMode`.** It implements `PoseTargetSink` and could accept
  streamed poses, but the joint-space modes are the v1 targets; adding it is
  wiring, not design.
- **Client-side smoothing.** The caller owns command smoothness. The driver
  owns rate limiting and the safe-stop, nothing more.

## Approved design decisions

### 1. Two axes: what you send × how it executes

A streaming session is a pair — a **setpoint shape** and a **control mode**.
Valid pairs are a table; anything off it is refused at open.

| setpoint shape | permitted `control_mode` | mechanism |
| --- | --- | --- |
| joint position | `kPosition`, `kImpedance` | existing `JointTargetSink` |
| EE pose | `kPosition`, `kImpedance` | `PoseTargetSink` + in-loop IK |
| joint velocity | `kVelocity` | native input of the new mode |
| EE twist | `kVelocity` | `J(q) qd = V`, damped least squares, in-loop |
| joint torque | `kTorque` | `JointTorqueMode::set_torque` (feedforward) |

`control_mode` is required even where it is implied, and validated. A client
that believes it is streaming twist into impedance mode must be told, not
quietly corrected.

**`ControlModeKind` must be extended.** It carries `kPosition` and `kImpedance`
today; this work adds `kVelocity` and `kTorque`. That enum is also read by
`TrajectoryGoal`, so the trajectory tier must explicitly **reject** the two new
values — a planned trajectory into a velocity or torque mode is not a supported
combination, and it must fail loud rather than fall through to a default.

### 2. Sessions are explicit, and mutually exclusive with goals

**Decision.** An owner must explicitly `on_stream_open` before setpoints are
accepted. Open is refused while a trajectory goal is in flight; a goal is refused
while a session is open; a second open is refused (close first — no implicit
replace).

**Rejected.** *A setpoint implicitly preempts an in-flight goal* — convenient,
but one stray packet from a half-dead client silently kills a planned move, and
there is no moment where the client declares "I am driving now."
*Let the sink resolve it latest-setter-wins* — a trap: the sampler re-sets the
target at 250 Hz from the executor while setpoints arrive independently, so the
two alternate rather than one winning, and the arm judders between references.

**Dividend.** Mutual exclusion is what makes decision 4 sound.

### 3. All kinematics live in the modes, in-loop; the session is a conduit

**Decision.** Pose → `q` (IK) and twist → `qd` (damped least squares) are
evaluated **inside `compute()` at 1 kHz**, not once per setpoint. The
`StreamingSession` forwards the raw target and touches no `Dynamics` at all.

**Why.** The arm moves between setpoints. A Jacobian evaluated when the setpoint
*arrived* is stale by the time it is used, so re-solving each cycle with the same
twist is not redundant work — it is the closed loop. `JointImpedanceMode`
already runs its IK in-loop for exactly this reason; this is symmetry with a
proven pattern, and it keeps `Dynamics` behind the mode boundary.

### 4. Setpoints are written to the sink directly, by the backend thread

**Decision.** A setpoint reaches the mode's target sink the moment it arrives, on
the caller's thread. No sampler quantisation; the client streams as fast as it
likes.

**Why it is safe.** The `*TargetSink` contract is *single writer*, not *specific
thread*. Because sessions and goals are mutually exclusive (decision 2), the
sampler writes no targets while a session is open. So the invariant becomes
**exactly one writer at a time, and which thread that is depends on session
state** — sampler when closed, backend when open.

**Rejected.** *Route setpoints through the sampler tick* — one writer forever,
trivially safe, but imposes a latency floor of one sampler period (≤4 ms at
`sampler_hz = 250`) and caps useful streaming at the sampler rate.
*Raise `sampler_hz` during a session* — keeps the simple invariant but burns a
non-RT thread spinning near RT rates on the same CPU as the 1 kHz loop, which is
exactly the shape of thing that shows up as jitter.

**Cost.** A subtler invariant, whose handoff must be right at three moments
(§ Component 6) and which requires a genuine concurrency test.

### 5. Velocity mode is stiff, and says so

**Decision.** `JointVelocityMode` commands `ActuatorMode::kVelocity` and tracks
the commanded velocity as accurately as the actuator servo allows. It is **not**
compliant.

**Why.** A mode's name is its contract. `JointPositionMode` does not apologise
for being stiff. A velocity mode that quietly delivered compliance via a torque
law would be a silent semantic difference of exactly the kind this repo rejects.
Compliant velocity tracking is a *different promise* and belongs in a different
mode, added later if wanted.

**Consequence — we inherit the safety envelope.** In a torque law a bad `qd`
near a singularity produces a *bounded* torque, and `torque_limit` is a free
backstop. In velocity mode whatever we compute goes to the actuators. So a
per-joint velocity clamp (seeded from the URDF, as `JointPositionParams` already
does) and damped least squares that stiffens near singularities are **required
parts of the design**, not refinements.

### 6. One deadline, two enforcement levels, per-mode safe-stop

**Decision.** The client declares one `timeout_s` at open. It is pushed into the
active mode's params and enforced twice:

- **Mode level (RT, 1 kHz) — make the output safe.** Only the mode knows what
  safe means for its actuator contract, and only the RT thread can ramp.
- **Session level (sampler) — lifecycle.** Close the session, stop admitting
  setpoints, report why, **retain the ownership grant** so a client that hiccuped
  reopens rather than renegotiating with the orchestrator.

| mode | safe-on-stale |
| --- | --- |
| torque | ramp `tau_ff → 0` over `cmd_decay_s`, revert to gravity comp |
| velocity | command zero velocity |
| position / impedance | freeze the reference at measured `q` |

**Why per-mode.** "Hold at measured q" is not universal — it is meaningless in
torque mode, which has no position loop. Making the safe-stop part of each
mode's contract is decision 5 applied to failure.

**Mechanism, reused not reinvented.** `JointTorqueMode` already tracks staleness
by summing `dt_s` in `compute()` and comparing a monotonic `write_count_` bumped
by the setter — **no clock call on the RT thread**. That mechanism becomes the
pattern for the other streaming-capable modes.

### 7. Streaming gets its own port

**Decision.** A new `StreamSink` driving port, not more `CommandSink` methods.

**Why.** `CommandSink` is already six methods after `on_halt`; adding open,
close and five setpoint shapes would make it a twelve-method god-interface.
`ArbitrationSink` set the precedent — a distinct concern gets a distinct port,
and a backend implements only what it supports.

### 8. Typed setpoint methods, not a tagged union

**Decision.** One method per setpoint shape.

**Why.** A single `Setpoint{kind, q, pose, qd, twist, tau}` makes an invalid
state representable — "kind says pose, pose field is garbage." Typed methods make
it impossible to send a twist labelled a pose, and the session rejects a method
that does not match the kind declared at open.

### 9. Twist is `Vector6` in the base frame

**Decision.** `[linear; angular]`, base frame, no frame field in v1.

**Why.** A wrist-camera client already knows its own extrinsics and can
transform. Carrying a frame enum would require the driver to know the camera
transform, which is not its business.

### 10. `GravityCompTorqueMode` is removed; `benchmark_grav_comp` is retargeted

**Decision.** The **mode** is deleted as part of the PR #3 rebase. The
**benchmark binary is kept** and retargeted to `JointTorqueMode` with zero
feedforward.

**Why the mode goes.** `GravityCompParams` is `{scale, damping, torque_limit}`;
`JointTorqueParams` is that plus the watchdog fields, over the same law. So
`JointTorqueMode` with `tau_ff = 0` **is** `GravityCompTorqueMode` — a degenerate
configuration wearing a mode's clothes. Modes are named for their actuator
contract, not for one output they happen to produce.

**Why the binary stays.** It is not only a benchmark: it backs the on-robot
procedure in `docs/integration/grav_comp_static_check.md` (put the arm in torque
mode with zero feedforward, confirm gravity compensation holds it up), and is
referenced from `scripts/rt_setup.sh`, `docs/rt-tuning.md`,
`docs/getting-started.md` and `docs/integration-runbook.md`. Nothing else
provides that check. Retargeted, it also becomes the **torque-path benchmark**,
sitting alongside `benchmark_cartesian_impedance` for the Cartesian torque path.
Its name stays accurate: with zero feedforward, gravity compensation is exactly
what it measures.

**Consequence.** **Issue #18** (the documented invocation throws — the default EE
frame is absent from the bare-arm URDF) becomes something to **fix**, not to
close as moot.

## Component 1 — `StreamSink` and the value types

```cpp
enum class SetpointKind { kJointPosition, kEePose, kJointVelocity, kEeTwist, kJointTorque };

struct StreamOpenRequest {
  SetpointKind    kind         = SetpointKind::kJointPosition;
  ControlModeKind control_mode = ControlModeKind::kPosition;
  double          timeout_s    = 0.1;   // <= 0 is REJECTED; the deadline is mandatory
  Token           token{};
};
struct StreamOpenResult  { bool accepted=false; int error_code=0; std::string message; };
struct StreamCloseRequest{ Token token{}; };

// One struct, three meanings -- the METHOD disambiguates, never a tag field.
// Units are per-method: rad (position), rad/s (velocity), N*m (feedforward torque).
struct JointSetpoint  { JointVec values = JointVec::Zero(); Token token{}; };
struct PoseSetpoint   { Pose     pose{};               Token token{}; };
struct TwistSetpoint  { Vector6  twist = Vector6::Zero(); Token token{}; };  // [linear; angular], base frame

class StreamSink {
  virtual StreamOpenResult on_stream_open(const StreamOpenRequest&) = 0;
  virtual void             on_stream_close(const StreamCloseRequest&) = 0;
  virtual void             on_setpoint_joint_position(const JointSetpoint&) = 0;
  virtual void             on_setpoint_joint_velocity(const JointSetpoint&) = 0;
  virtual void             on_setpoint_joint_torque(const JointSetpoint&) = 0;
  virtual void             on_setpoint_pose(const PoseSetpoint&) = 0;
  virtual void             on_setpoint_twist(const TwistSetpoint&) = 0;
};
```

`Vector6` and `Jacobian6` already exist in `cartesian_types.h`.

A setpoint whose method does not match the kind declared at open is **rejected**,
counted, and does not reach the sink. `timeout_s <= 0` is refused at open: an
unbounded stream has no safe-stop, so the deadline is not optional.

**Arbitration.** The `Arbiter` decorates `StreamSink` as it decorates
`CommandSink` — a token compare per setpoint, nanoseconds, and the
lock-across-delegation property already guarantees no setpoint lands after a
revoke.

## Component 2 — `StreamingSession`

A new interface-layer unit. Holds the declared `(kind, control_mode, timeout_s)`,
the owner's last-write counter, and nothing else. It touches no `Dynamics`, no
`Transport`, and no Pinocchio; it is unit-testable over injected time against a
fake sink.

States are **closed** and **open**. Transitions:

| event | from | to | effect |
| --- | --- | --- | --- |
| `on_stream_open` | closed | open | validate pair; refuse if a goal is in flight; mode switch; push `timeout_s` into mode params; mark open **last** |
| `on_stream_open` | open | open | refused — close first |
| `on_stream_close` | open | closed | mark closed **first**, then mode safe-stop |
| watchdog expiry | open | closed | mark closed **first**, then mode safe-stop; report; **grant retained** |
| `on_halt` (revoke/e-stop) | open | closed | mark closed **first**, then the Supervisor's halt path runs |

`Supervisor::on_trajectory_goal` gains one atomic check (`stream_open_`) in its
existing fast pre-check.

## Component 3 — `JointPositionMode` gains a pose path

Mirrors `JointImpedanceMode` rather than inventing anything:

1. Hold a `DiffIkSolver ik_`; add `DiffIkParams ik` to `JointPositionParams`.
2. Implement `PoseTargetSink::set_target(const Pose&)` with a double-buffer.
3. Widen the target source from the current bool (`has_ext_target_`) to the
   three-way `TargetSource{kEntryPose, kPose, kJoint}` impedance already uses, so
   **IK runs only when a pose target is live** — a joint target or a trajectory
   pays nothing.
4. In `compute()`, when the source is `kPose`: `q_d = ik_.solve(target, q_ref_)`,
   warm-started from the current reference. The output then feeds the mode's
   **existing** pipeline unchanged — `rate_limit → leash → wrap → clamp` — so the
   whole safety envelope comes along for free.

**IK failure policy.** `IkResult` reports `{pos_err, rot_err, iters, converged}`.
A single non-converged solve is **not** a fault: a momentarily unreachable pose
is normal while a client servos toward something. **Sustained non-convergence
is.** The threshold is expressed in **time, not cycles** — `ik_fault_s`, default
`0.1` — and accumulated the same clock-free way as the staleness watchdog: sum
`dt_s` while `!converged`, reset on any converged solve. (Cycles would have been
the wrong unit: IK runs in `compute()` at 1 kHz per decision 3, so a
cycle-count threshold silently means something different at a different loop
rate.)

**How the fault reaches the session.** The mode cannot end a session — modes know
nothing about the interface layer. So `JointPositionMode` publishes an atomic
`ik_faulted_` flag (RT writer, non-RT reader, same discipline as the existing
`reference()`/`last_ik()` accessors but synchronised because it crosses threads).
The sampler observes it on its next tick and tears the session down with a
distinct reason. The mode meanwhile does the safe thing immediately at 1 kHz —
it freezes the reference — so the arm is already safe before the lifecycle
teardown catches up. Same fast-path/slow-path split as decision 6.

Position mode is stiff; holding a stale reference while its client believes it is
tracking is precisely the silent divergence this repo exists to fail loud on.

**Docstring debt.** The mode currently advertises *"runs no dynamics at all — no
gravity term, no mass matrix, no IK… the cheapest control path in the driver."*
That becomes conditional and the comment must say so: with a joint target it is
still the cheapest path; only a live pose target pulls in IK.

## Component 4 — `JointVelocityMode` (new)

`ActuatorMode::kVelocity`. Accepts a joint-velocity target (native) and an EE
twist through a new `TwistTargetSink`.

Per RT cycle, when the source is a twist:

1. `J = dyn_.jacobian(q)` (already RT-safe).
2. Solve `J qd = V` by **damped least squares**, damping rising near
   singularities. This is a single 6×7 solve — **not** `DiffIkSolver`, which is
   iterative Gauss-Newton position IK and the wrong tool.
3. Add a **null-space posture term**, or the redundant DOF drifts and the elbow
   wanders while the tool tracks fine.
4. Clamp per joint to `max_qd`, seeded from the URDF velocity limits exactly as
   `JointPositionParams::seed_limits()` does — non-finite entries filled from the
   URDF, finite entries clamped down to it.
5. Staleness per decision 6 → command zero velocity.

Singularity behaviour is a **required test**, not an aspiration: at a
straight-arm configuration the commanded `qd` must stay bounded.

## Component 5 — `JointTorqueMode` (rebased, PR #3)

Lands **before** this work, as its own change. Beyond the rebase itself:

- **`torque_limit` becomes per-joint.** It is currently a scalar `39.0`, and
  `JointImpedanceParams` already documents why that is wrong: *"The URDF gives
  joints 5-7 an effort limit of 9 N·m; the single scalar … would overrun the
  wrist by 4x."* Torque mode has the bug the impedance mode was written to avoid.
- **Remove `GravityCompTorqueMode`** and `benchmark_grav_comp` (decision 10).
  `rt_safety_test.cpp` lines 32 and 242 retarget to `JointTorqueMode`.
- Streaming then wires `set_torque` to a setpoint path; the mode's existing
  watchdog is already the right shape.

## Component 6 — the write handoff

**Invariant:** exactly one thread writes targets at any instant. Sampler when the
session is closed; backend thread when it is open.

All three transitions use the same **mark-then-act** ordering the `Arbiter`
already uses for revoke:

- **Open** — refuse if a goal is in flight, do the mode switch, then mark open
  **last**, so no setpoint can land mid-switch.
- **Close** — mark closed **first**; further setpoints are rejected from that
  instant; then the safe-stop runs.
- **Timeout** — the watchdog marks closed, then acts. This is the sharp one:
  setpoints may still be arriving from a client that is not actually dead, and
  the mark is what makes the subsequent write exclusive.

An arbitration halt marks the session closed before the Supervisor's halt path
does anything else, so streaming stops admitting before the hold is latched —
symmetric with how goals stop being admitted.

## RT-safety invariants

- No clock call, allocation, lock or blocking call in any `compute()`. Staleness
  is tracked by summing `dt_s` against a monotonic write counter.
- `JointVelocityMode` adds per-cycle work (Jacobian + a 6×7 DLS solve). This is
  **new RT cost** and must be measured, not assumed.
- Setpoint publication uses the established double-buffer + atomic-index pattern;
  the RT reader always observes a whole target, never a torn one.

## Testing strategy

**Tier 1 — units** (no robot, no URDF, no threads). `StreamingSession` over
injected time against a fake sink: the valid-pair table, refusal while a goal is
in flight, goal refusal while open, second-open refusal, watchdog firing at the
deadline, identical teardown from all three callers. Per-mode safe-stop driven by
hand-called `compute()` cycles, including a monotone-decay case for torque.
`JointVelocityMode`: twist → `qd` correctness, null-space posture holding, the
per-joint clamp biting, and **bounded `qd` at a singular configuration**.
`JointPositionMode`: IK output feeding the existing pipeline, and N consecutive
non-converged solves faulting the session.

**Tier 2 — concurrency.** The handoff cannot be proven behaviourally. A writer
thread streaming setpoints against a thread closing the session or tripping the
watchdog, asserting no setpoint write lands after the closed-mark and that the
safe-stop is the final write, over many iterations. Plus a one-off build under
`-fsanitize=thread` rather than trusting iteration count alone.

**Tier 3 — sim integration.** `SimTransport` is a static echo, so velocity mode
produces no motion as-is. Add an **ideal velocity plant** (`q += qd_cmd * dt`) —
the same trick `execution_integration_test.cpp` already uses for position — which
makes twist → motion testable end to end with no hardware.

**Tier 4 — RtSafety.** New `RtSafety.<Mode>NoMajorFaultsSteadyState` entries for
`JointVelocityMode` and `JointTorqueMode`, plus a supervisor-in-loop variant with
a session open. Zero major page faults, zero dropped samples.

**Tier 5 — attended hardware. A gate, not a check.**
`ActuatorMode::kVelocity` has **never been exercised on this arm**, and
`JointTorqueMode` has not run since June. Probe first: actuators into `kVelocity`,
command a small constant joint velocity on one joint, observe whether it tracks,
ignores, or faults. If it faults or no-ops, the twist path needs rethinking — so
this runs before the velocity work is called done.

## Open decisions

- **How to measure `JointVelocityMode`'s per-cycle cost.** Retargeting
  `benchmark_grav_comp` (decision 10) covers the joint-torque path and
  `benchmark_cartesian_impedance` covers the Cartesian torque path, but **neither
  exercises velocity mode**, whose per-cycle Jacobian and DLS solve is the one
  genuinely new RT cost in this work. Either add a mode flag to an existing
  benchmark or write a small dedicated one. The project bar requires RT changes
  to be measured, so this must be resolved in Plan 2, not left open.

## Accepted gaps and risks

- **`kVelocity` is unproven firmware.** `KortexTransport` maps it, but no
  `ControlMode` has ever used it, and `joint_position_mode.h` already records
  that Kinova's own driver computes a velocity command and declines to send it
  (*"Velocity command interface not implemented properly in the kortex api"*).
  That comment is about the velocity field in **position** servoing, a different
  path — but it is the reason Tier 5 is a gate.
- **Two enforcement points for one deadline.** Deliberate (decision 6), but it is
  two places to get wrong; the mode-level and session-level values must be sourced
  from the same number.
- **Streaming has no rate audit.** The driver does not measure or report the
  client's actual stream rate. A client silently streaming at 5 Hz will simply
  feel sluggish. Worth revisiting once there is a real consumer.

## Decomposition — three plans, in order

1. **`JointTorqueMode` rebase** — PR #3 onto main, per-joint `torque_limit`,
   remove `GravityCompTorqueMode`, retarget `benchmark_grav_comp` and the
   RtSafety tests to `JointTorqueMode`, fix issue #18. Stands alone; no
   streaming content.
2. **Mode work** — `JointVelocityMode` (with the DLS twist map, null-space term
   and URDF-seeded clamp) and `JointPositionMode`'s pose path. Both testable
   without any streaming surface.
3. **The streaming tier** — `StreamSink`, `StreamingSession`, the write handoff,
   per-mode safe-stop wiring, and the concurrency test.

## Reference

- v1 interface spec (the deferral this fills):
  `2026-08-10-arm-driver-interface-design.md`
- Arbitration (dependency): `2026-08-25-arm-arbitration-design.md`
- `JointTorqueMode` design (PR #3): `2026-06-17-joint-torque-mode-design.md`
