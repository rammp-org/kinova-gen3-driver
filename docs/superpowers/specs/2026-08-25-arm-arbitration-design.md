# Arm Arbitration — Design Spec (capability-token ownership)

**Date:** 2026-08-25
**Status:** Approved for planning
**Scope:** A transport-agnostic ownership layer in the driver core that decides
**who may command the arm**, plus a general halt primitive that `/estop` and
ownership revocation both use. One new unit, two port changes, no control-law
changes.

This is the arbitration follow-on that
[`2026-08-10-arm-driver-interface-design.md`](2026-08-10-arm-driver-interface-design.md)
deferred: *"Multi-sender arbitration — a real, near-term, cross-cutting need …
but its own follow-on spec. v1 leaves the **hooks** (sender identity + a
pluggable admit seam) and nothing more."* This spec fills that seam.

## Goal

A task orchestrator hands ownership of the arm to one system at a time. It asks
the driver for a grant; the driver mints a **capability token** and returns it;
the orchestrator passes that token to the owning module. The driver then accepts
commands **only** when they carry the current token, and rejects everything else
loudly and diagnosably.

The same mechanism expresses the degenerate case — *nobody* may command — which
is what an emergency stop is.

## Out of scope (explicitly deferred)

- **The streaming-setpoint tier.** Arbitration is designed to be command-shape
  agnostic so the reactive/visual-servoing tier slots in behind it without
  redesign, but the streaming commands themselves get their own brainstorm and
  spec **after** this lands.
- **Orchestrator liveness / heartbeat.** Deliberately dropped (see *Accepted
  gaps*). External supervision tooling is expected to cover process death.
- **Multi-owner policy** — priority levels, preemption between owners, partial
  ownership (arm vs gripper). v1 is exactly one owner at a time, whole arm.
- **Security.** See *Accepted gaps*: this is a mistake boundary, not an access
  control boundary.
- **Control-law changes.** No `ControlMode` is modified by this work.

## Approved design decisions

### 1. Arbitration is a `CommandSink` decorator, not part of the Supervisor

**Decision.** A new unit `kinova::interface::Arbiter` implements `CommandSink`,
holds a `CommandSink&` downstream, and is wired between the transport backend
and the `Supervisor`. It includes **nothing** from controls: no `ControlMode`,
no `RtExecutor`, no `Dynamics`, no `Transport`, no Pinocchio. Only value types.

**Why.** `Supervisor` already owns `JointPositionMode&`, `JointImpedanceMode&`,
`RtExecutor&` and `Dynamics&` — putting admission there entangles "who may
command" with the control code, and makes the safety-critical logic untestable
without standing up modes, a URDF and threads. As a decorator, the Arbiter is
unit-testable against a fake sink with no robot, and every transport (ROS2,
socket, ATOS) inherits arbitration for free. It follows the repo's existing
decorator idiom — `FeedbackTap` decorates `Transport` for the same reason.

**Rejected.** *Injected `AdmissionPolicy` the Supervisor calls* — an object that
only answers yes/no at command time cannot **push**: hard revocation is an event
that must reach into a running system and stop a trajectory. It would need a
back-reference to the Supervisor, which is this decorator with worse ergonomics.
*Arbitration inside `Supervisor`* — the entanglement above. *Arbitration in each
transport backend* — re-implemented per backend, wrong in at least one, and
invisible to the sim test suite.

### 2. The driver mints tokens

**Decision.** `grant(owner_id) -> token` is request/response. The driver
generates a fresh random 128-bit token per grant (`std::mt19937_64`, seeded once
at startup, non-RT).

**Why.** It makes the grant-ordering race structurally impossible. If the
orchestrator minted tokens, it could hand one to a module before the driver knew
about it, and that module's first commands would be rejected for reasons it
cannot diagnose. When the driver mints, the orchestrator has nothing to hand over
until the driver has already issued it — correct ordering becomes the only
possible ordering rather than documentation someone must obey. Secondarily, the
comparing party is the only one that can guarantee a new token never collides
with a stale one held by a zombie node.

**Consequence.** A driver restart voids every outstanding token and the
orchestrator must re-grant. This is correct — after a restart the arm's state is
unknown and nothing should hold ownership across it — but the orchestrator has
to handle it.

**Note.** With one owner at a time and a fresh token per grant, a `generation`
counter is *not* needed for rejection (a stale token simply does not match). It
is kept for observability only, so a reject log can say "token from generation 7,
current is 9" instead of "denied".

### 3. Hard revoke: cancel and hold

**Decision.** Revoking ownership immediately halts the arm — the in-flight goal
is cancelled and the arm latches hold-at-measured-q. The next owner always starts
from a stationary arm in a known state.

**Rejected.** *Graceful revoke* (let the current goal finish) — handover latency
becomes unbounded and "who owns the arm right now" acquires a fuzzy middle state.
*A per-call hard/graceful flag* — two code paths through the safety-critical
transition. The orchestrator controls timing by choosing **when** to revoke; if
it wants a clean handover it waits for the goal to finish first.

### 4. E-stop is the degenerate case of arbitration, and it latches

**Decision.** `/estop` drops all grants, halts, and sets a **latched inhibit**
that rejects every command — including in `disabled` arbitration mode — until an
explicit `estop_clear()`. Clear exits to *no owner*, never straight back to
owned, and works in any arbitration mode (otherwise you can strand yourself).

**Why.** E-stop and revoke differ in exactly one property: latching. Expressing
e-stop through the arbitration state machine means it needs no new machinery.

### 5. `on_halt(HaltReason)` — the caller says why, the Supervisor decides how

**Decision.** One general halt primitive on `CommandSink`, not a
revocation-specific method. The caller declares a reason; the `Supervisor` maps
reason to control action. v1 behaviour is **identical for every reason**:
cancel and hold.

**Why.** Keeps controls knowledge out of the Arbiter — it never learns what
"hold" means in torque control — and lets new callers (e-stop, an operator
button, a future watchdog) reuse the path without touching control code. One
stop path means one set of tests; the enum earns its keep later if e-stop ever
needs to diverge (engaging brakes, dropping to gravity comp).

### 6. Two arbitration modes, not a boolean bypass

**Decision.** `ArbitrationMode { kEnforced, kDisabled }`, selected at
construction. `kDisabled` admits any command regardless of token, and is
reported in `ArbitrationStatus` so a client can *see* the driver is unarbitrated
rather than trusting that someone read a startup log. E-stop still latches in
`kDisabled`.

**Accepted risk (user's call, 2026-08-25).** A bypass is a second code path and
tends to become the one that actually runs. Accepted deliberately for
experimentation without an orchestrator.

## Component 1 — the Arbiter state machine

Three states. The Arbiter holds a token, an owner id, a generation, a rejection
counter and two enums — no goal state, no clock, no threads.

- **`kNoOwner`** — no valid grant. Commands rejected in `kEnforced`.
- **`kOwned`** — exactly one owner: `token`, `owner_id`, `generation`.
- **`kEstopped`** — latched; everything rejected, `kDisabled` included.

| Event | From | To | Side effect |
|---|---|---|---|
| `grant(owner_id)` | `kNoOwner` | `kOwned` | mint token, `generation++` |
| `grant(owner_id)` | `kOwned` | `kOwned` | `on_halt(kOwnershipRevoked)` **first** — a re-grant is revoke-then-grant, never a silent swap under a moving arm |
| `grant(owner_id)` | `kEstopped` | `kEstopped` | refused, `accepted=false` |
| `revoke()` | `kOwned` | `kNoOwner` | `on_halt(kOwnershipRevoked)` |
| `estop()` | any | `kEstopped` | drop grant, `on_halt(kEmergencyStop)` |
| `estop_clear()` | `kEstopped` | `kNoOwner` | — |

**Admission rule.** A command is admitted iff
`(mode == kDisabled && !estopped) || (state == kOwned && token == current_)`.

- `on_query_state()` **bypasses arbitration entirely** in every state, e-stopped
  included. Reads are always open.
- `on_trajectory_cancel()` **requires the owner's token** — a stranger must not
  be able to stop your motion. The emergency path is e-stop, not cancel.
- **Every other `CommandSink` method is gated**, including
  `on_trajectory_accepted()`, which re-checks the token carried on the goal
  struct rather than trusting that a matching `on_trajectory_goal()` preceded it.
- Rejection is per-method and never silent: `on_trajectory_goal` returns
  `kRejectUnauthorized`, `on_trajectory_cancel` returns `CancelResponse::kReject`,
  `on_set_gains` returns `{accepted=false}` with a message, and
  `on_trajectory_accepted` (which returns `void`) simply does not delegate. All
  four bump `rejected_count` and emit a rate-limited log naming the sender.

**Thread safety.** The ROS2 bring-up runs a real `MultiThreadedExecutor`, so
`CommandSink` and `ArbitrationSink` calls can arrive concurrently on different
threads. The Arbiter guards its state with a plain mutex, off the RT path and off
the sampler tick.

The lock discipline is asymmetric, and both halves matter:

- **Held across command delegation.** Admit-and-deliver must be atomic against
  `revoke()`/`estop()`, or a command admitted a moment before a revoke can reach
  the Supervisor *after* the halt has been processed — restarting a stopped arm.
  This is safe because the downstream command handlers are non-blocking by
  construction (an atomic pre-check, or a `push_back` under a briefly-held
  `q_mtx_`) and never call back into the Arbiter, so there is no lock inversion.
- **Never held across `on_halt`.** State is mutated under the lock, the lock is
  released, and only then is `on_halt` invoked. Because the state change lands
  first, no command can be admitted in the window between release and the halt
  reaching the Supervisor.

## Component 2 — ports and value types

The token is a fixed-size POD matching `GoalId`; these structs are copied on the
command path and this repo's value types are allocation-free.

```cpp
using Token = std::array<uint8_t, 16>;
```

**Every inbound command struct carries its own authority** — consistent with the
v1 decision that a command fully describes its own intent:

```cpp
struct TrajectoryGoal { /* ...existing... */ std::string sender_id; Token token{}; };
struct GainsRequest   { JointImpedanceGains gains{};                Token token{}; };
struct CancelRequest  { GoalId id{};                                Token token{}; };  // new
```

`CancelRequest` is new because `on_trajectory_cancel(const GoalId&)` had no
struct to carry a token.

```cpp
class CommandSink {
  virtual GoalResponse   on_trajectory_goal(const TrajectoryGoal&) = 0;
  virtual void           on_trajectory_accepted(const GoalId&, const TrajectoryGoal&) = 0;
  virtual CancelResponse on_trajectory_cancel(const CancelRequest&) = 0;   // was GoalId
  virtual GainsResult    on_set_gains(const GainsRequest&) = 0;
  virtual ArmState       on_query_state() = 0;                             // never gated
  virtual void           on_halt(HaltReason) = 0;                          // new
};

enum class HaltReason { kOwnershipRevoked, kEmergencyStop, kOperatorRequest };

class ArbitrationSink {          // new driving port, implemented by the Arbiter
  virtual GrantResult       grant(const std::string& owner_id) = 0;
  virtual void              revoke() = 0;
  virtual void              estop() = 0;
  virtual void              estop_clear() = 0;
  virtual ArbitrationStatus status() const = 0;
};

struct GrantResult       { bool accepted=false; Token token{}; uint64_t generation=0; std::string message; };
struct ArbitrationStatus { ArbitrationMode mode; bool estopped=false; bool owned=false;
                           std::string owner_id; uint64_t generation=0; uint64_t rejected_count=0; };
```

Arbitration is a **separate port** from `CommandSink` because "who may command"
is not "command the arm": a test harness that does not care about ownership
implements only `CommandSink`.

`status()` is the observability answer and stays **out of `ArmState`**, so
telemetry does not become coupled to arbitration.

**Rejection reasons.** Add `GoalResponse::kRejectUnauthorized` and
`result_code::kNotAuthorized = -8` (following `kPlanningFailed = -7`).
*Known limitation:* rclcpp_action gives a client only "rejected" with no reason,
so an unauthorized goal is indistinguishable from a malformed one client-side.
`rejected_count` in the status topic plus a reject log at the **backend** are
what make this debuggable — without them, a rejected command stream is
indistinguishable from a dead subscriber or a QoS mismatch.

**The Arbiter itself does not log.** The core library has no logging facility by
design — `std::cerr` appears only under `apps/`, and the library's only `fprintf`
is CSV telemetry. So the Arbiter *counts* (`rejected_count`) and returns a
distinguishable rejection; the transport backend, which already holds the
offending message and its sender, does the rate-limited logging in its own
framework (`RCLCPP_WARN` on the ROS2 side). This keeps the core ROS-free and
log-framework-free.

## Component 3 — the halt path

`on_halt` is called on the backend thread and does exactly two things there:
latch an atomic `halt_pending_` with the reason, and — under the existing
`q_mtx_` — clear `inbox_` of queued goals, so a halt can never sit behind queued
trajectories. `sampler_loop` observes the latch on its next tick and performs the
control action.

Halt therefore takes effect within one sampler period plus one RT cycle:
**≤5 ms** at `sampler_hz = 250`.

**Why not an atomic the modes read in `compute()`** (which would give ≤1 ms): it
adds a branch to the RT path (new `RtSafety` + benchmark obligation) and spreads
halt logic into both mode classes. The decisive argument is **settle-exactly-once**
— `sampler_loop` is currently the only thread that ever calls `action_.settle()`,
and halt introduces a new way to reach settle concurrently with normal
completion. That is precisely the race behind ros2 issue #7 and the
`is_canceling()` TOCTOU already fixed once. Routing halt through the sampler
preserves the single-settler invariant *by construction*. If 5 ms is ever too
slow, the answer is a hardware E-stop in the safety chain, not a faster sampler.

**On a latched halt the sampler:**

1. Drops `traj_` so nothing feeds new targets.
2. Settles the in-flight goal **and every queued goal it dropped**, each exactly
   once, with `result_code::kHalted = -9` and the reason in `error_string`. A
   queued goal was already ACCEPTed by the backend; dropping it silently orphans
   a client that waits forever (the bug class fixed in `a835bf5` / `d0791df`).
3. Latches hold: sets the active mode's target to the **last-good measured q**
   via the existing `sampled_q()` pattern — a failed Seqlock read must not inject
   zeros here of all places.
4. **Zeroes the reference velocity.** With the feedforward law
   (`- Dq*(qd - qd_ref) + M(q)*qdd_d`, core PR #15), a stale `qd_ref` from the
   aborted trajectory would keep commanding motion at exactly the moment we are
   trying to stop. With `qd_ref = 0` and target = measured q, the damping term
   becomes `-Dq*qd`, which actively brakes. **Whichever of this spec and PR #15
   lands second must honour this.**
5. Clears `in_flight_`.

**Hold differs by mode and needs no code change:** `JointPositionMode` servos the
actuators to where they are; `JointImpedanceMode` holds with configured
stiffness, so the arm stops but stays compliant to contact.

**Re-arming is free.** After a halt the arm is stationary, `traj_` is empty and
`in_flight_` is clear — so a newly granted owner's first goal may select a
*different* control mode, because the mode-switch-at-rest precondition is
trivially satisfied.

## Component 4 — transport mapping (ROS2)

Grant and revoke are **not on the control path** — they fire once per ownership
handover. Even a multi-millisecond service round trip is irrelevant. What must be
fast is the per-command token check, which is an in-process comparison inside the
Arbiter with no ROS in the path.

| Call | ROS2 shape | Why |
|---|---|---|
| `grant(owner_id) -> token` | service | needs a return value (the minted token) |
| `revoke()` | service | orchestrator gets confirmation the arm halted before handing it on |
| `estop()` | **topic** | must never block and must never depend on a responder being alive or discovered |
| `estop_clear()` | service | deliberate, wants confirmation, must work in any mode |
| `status()` | topic | diagnostic publication, incl. visible `kDisabled` mode |

The driver only ever **serves**; it never calls a service out. The rclcpp
callback-group deadlock (calling a service from inside a callback) therefore does
not apply. Handlers must stay non-blocking: `grant` is an RNG call and a few
stores.

## Component 5 — DI wiring

```cpp
Supervisor  sup(pos, imp, exec, snap, dyn, stream, action);   // unchanged construction
Arbiter     arb(sup, ArbitrationMode::kEnforced);             // decorates it
Ros2Backend backend(arb, arb);                                // CommandSink& and ArbitrationSink&
```

Neither side names a concrete transport; the Supervisor does not know the Arbiter
exists.

## RT-safety invariants

- The Arbiter is **not on the RT path**. `compute()` and the executor cycle are
  unmodified by this work.
- `on_halt` is non-blocking on the caller's thread: one atomic store plus a
  short mutex-guarded inbox clear, both already the established pattern for
  backend→sampler handoff.
- No allocation is introduced on the sampler tick: `Token` is a fixed-size POD
  and the Arbiter allocates only in `grant()` (the `owner_id` string), which is
  off the command path.

## Testing strategy

**Tier 1 — Arbiter unit tests** (no robot, no threads, no URDF, no ROS, no
clock). A `RecordingSink` fake plus ~15 tests covering the full transition table.
The ones that matter most: a wrong/stale token is rejected **and downstream is
never called** (assert the absence); the zombie case (hold a token, re-grant, old
token now rejected); a re-grant fires `on_halt` **before** the new grant goes
live; e-stop latches and refuses a fresh `grant()`; e-stop rejects **even in
`kDisabled`**; `estop_clear()` exits to `kNoOwner`; `on_query_state` passes in
every state including e-stopped; cancel requires the owner's token. Zero timing
dependence means zero flakiness.

**Tier 2 — sim integration**, extending
`tests/interface/execution_integration_test.cpp` (real Supervisor, real modes,
`SimTransport`): revoke mid-motion settles the goal **exactly once** (count the
calls) with `kHalted`, latches the target at measured q, and writes no further
targets; a halt with goals queued settles all of them with none orphaned;
after a halt a new grant + a goal in a *different* control mode succeeds; halt
takes effect within a bounded number of sampler periods.

**Tier 3 — RT safety.** `rt_safety_test` (zero major page faults, zero dropped
samples) with a halt exercised in steady state. Run it and read the result; do
not assume it from the structural argument.

**Tier 4 — benchmark.** Before/after p50 / p99 / p99.9 / max, overruns and
faults on record. Use the impedance benchmark with `gen3_7dof_2f85.urdf` — the
documented `benchmark_grav_comp` invocation currently throws (core issue #18,
the default EE frame is absent from the bare-arm URDF).

**Tier 5 — attended hardware**, per `docs/integration-runbook.md`: grant → small
single-joint move → revoke mid-motion → confirm the arm stopped by **re-reading
measured q**, not by trusting the reported result (completion is time-based and
`final_ref` is the commanded reference — neither proves the arm did anything).
Then `/estop` during motion → stops, latches, next goal refused until clear.

There is **no CI on either repo**. The Jetson is the only gate.

## Accepted gaps and risks

- **Orchestrator death freezes ownership.** With the heartbeat deliberately
  dropped, if the orchestrator dies holding a grant, ownership persists until
  someone calls `revoke()` or `estop()`. Deferred to external supervision
  tooling.
- **This is a software e-stop**, not a hardware safety chain. It depends on the
  driver process being alive and scheduled. It is not a substitute for a
  hardware E-stop.
- **The token is a mistake boundary, not a security boundary.** There is no
  SROS2 on this bus — anything that can publish can observe or replay a token.
  The design prevents two well-behaved systems from fighting over the arm; it
  does not defend against a hostile one. Do not build a safety argument on it.
- **The `kDisabled` bypass is a second code path** and tends to become the one
  that runs (accepted, decision 6).

## Blast radius and sequencing

The `CommandSink` change breaks every implementor: `Ros2Backend` plus the fake
supervisors in the goto integration tests — the same set of fakes across four
files that ros2 PR #5's freshness gate already forced a touch of. Each needs an
`on_halt` stub and the `CancelRequest` signature update.

Core lands first; `kinova_arm_ros2` cannot build until it does — the same
sequencing as `kPlanningFailed` / `has_velocities`.

## Reference

- v1 interface spec (the deferral this fills):
  `2026-08-10-arm-driver-interface-design.md`
- Decision log (human record): `SHARED - RAMMP/ARM test environment/Arm Driver
  Interface - Design Decisions.md` — edit only via the `obsidian-vault` skill.
- Next: the streaming-setpoint tier gets its own brainstorm and spec once this
  is implemented.
