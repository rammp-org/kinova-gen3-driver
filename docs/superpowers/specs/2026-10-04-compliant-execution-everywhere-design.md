# Compliant Execution Everywhere — Design Spec (v1.3.0)

**Date:** 2026-10-04
**Status:** Approved for planning
**Scope:** Joint impedance as a first-class option on every interface surface —
trajectories, the GoTo actions, and every streaming controller where compliance
makes sense — behind one gains contract, with defaults good enough that
switching to impedance requires no further configuration. Realizes GitHub
issue **#63** (milestone v1.3.0); its prerequisite bug is **#64**.

## Goal

A caller on any surface picks `position` or `impedance` per command. Picking
`impedance` and saying nothing else gets tuned, validated default gains and
behaves well — identically on every surface, every time, regardless of what the
previous caller did. Callers who *want* to tune can, from a small named menu or
with raw numbers, and nothing they send can put the arm into a dangerous
configuration silently.

**The guiding principle (the release's acceptance test):** *switch to
impedance, change nothing else, and it works.*

## Dependencies

- **v1.2.0** (speed scale, `56e7c61`) — the base. All work branches from it.
- **Issue #64** — gain validation. Lands first; everything else builds on a
  core that refuses bad gains.
- **`feat/target-feedforward`** (3 commits, based on an Aug 14 commit) — ported,
  not cherry-picked blind; it predates the 1.1.1 velocity rework.
- **`rammp_arm_interfaces`** — the GoTo and trajectory message changes live
  there; `kinova-gen3-ros2` follows. Same cross-repo pattern as the
  arbitration round.

## Out of scope

- **`CartesianImpedanceMode` in the Supervisor**, the `kEeWrench` setpoint
  kind, and the `cartesian_impedance` stream controller. Cartesian impedance
  stays teleop-only this release. (Per #63; the streaming spec already calls
  the wiring mechanical.)
- **Impedance as the default.** Every surface defaults to position, exactly as
  today. Revisit after the gains plumbing has soaked on the real arm.
- **`query_state`.** Stays stubbed.
- **Gripper.** `GripperController` decorates the Transport and is orthogonal.

## Approved design decisions

### 1. The gains contract: `ImpedanceGains`

Every command that can run impedance carries one optional `ImpedanceGains`:

- **A profile name** — `soft`, `medium`, `stiff` (exact menu fixed during
  planning; small). **Names describe the gains** — there is no profile named
  `default`. "Default" is a *pointer* to a profile (initially `medium`), never
  a profile itself, so a caller reading back the session state always sees
  which gains they are actually getting. The table lives **in the driver
  core**, each entry a *complete* `JointImpedanceParams` — not just
  `{kq, zeta, torque_limit}` but leash, ramp, and ref-speed too. One menu, one
  meaning, every front-end.
- **`custom` + raw gains** — the escape hatch for teleop and learned-policy
  clients that genuinely tune. Same validation as everything else.
- **Absent** — whatever the session default points at (initially `medium`).

Three core-enforced guarantees:

1. **Validated at accept time (#64).** Non-finite values, out-of-range
   stiffness or damping ratio, and torque limits too low to carry gravity are
   **rejected with a reason** — never clamped into something that drops the
   arm. Validation lives in the core, below every surface, so it protects raw
   and profile callers alike. The ROS mapping bug that stamps
   `has_gains = true` with message-default zeros on every impedance goal dies
   with the old fields.
2. **No leakage.** Each command and each stream session resolves its own
   `ImpedanceGains` at accept/open. A goal or stream without gains gets the session
   default — *not* whatever the previous caller left on the shared
   `JointImpedanceMode`. (Today a trajectory goal's gains silently persist
   into later `joint_impedance` streams.)
3. **No partial resets, no switch-gating.** Applying gains never resets the
   params it doesn't name (today the mode-switch branch rebuilds
   `JointImpedanceParams` from scratch, discarding leash/ramp/ref-speed), and
   gains apply even when the mode doesn't change (today a second impedance
   goal's gains are ignored).

**`on_set_gains` un-stubs** into "set the session default": profile or raw,
same validation, already Arbiter-gated. A `SetGains` ROS service sits on top.
This matches the original interface spec's intent (goal gains authoritative;
set-gains sets the defaults).

### 2. Surfaces

| Surface | Change |
| --- | --- |
| `TrajectoryGoal` / `ExecuteJointTrajectory` | `has_gains + gains` replaced by `ImpedanceGains`. Mode choice unchanged. |
| `StreamOpenRequest` / `open_stream` | Gains at open: `ImpedanceGains` added. Impedance streams stop inheriting leftovers. |
| GoTo ee-pose / joint-config / preset | Grow `control_mode` + `ImpedanceGains`. Default stays position — pure opt-in, no behavior change for existing clients. |
| `set_gains` | Real `CommandSink::on_set_gains` + new ROS service, token-gated. |

The `CommandSink`/`StreamSink` signature changes break `kinova-gen3-ros2`;
message changes land in `rammp_arm_interfaces`. Both follow in lockstep.

### 3. Compliant velocity and twist

`JointVelocityMode` stays stiff by contract and untouched. The 1.1.1 mechanism
— integrate the velocity command into a held, leashed position reference — is
**factored out of `JointVelocityMode` into a shared integrator** and reused in
front of `JointImpedanceMode`'s `JointTargetSink`. Twist goes through the same
damped-least-squares resolution it uses today, then the same integrator.

New valid pairs: `joint velocity × kImpedance`, `EE twist × kImpedance`. New
ROS controllers: `joint_velocity_impedance`, `ee_twist_impedance`. The pair
table and `pair_supported()` keep refusing everything else at open.

### 4. Target feedforward

Port `feat/target-feedforward`: joint-space targets become `{q, qd, qdd}`, and
`JointImpedanceMode` damps toward the reference velocity
(`tau = Kq·e + Dq·(qd_ref − qd) + …`) instead of toward zero. This removes the
steady-state tracking lag (`≈ 2ζ·qd·√(M/Kq)`) that would otherwise force
default stiffness up — it is what makes "default gains are good enough"
physically achievable. The trajectory executor forwards the sampled `qd` (and
`qdd` where the profile has it) instead of discarding them.

Per-cycle cost is **measured** before and after (p50/p99/p99.9/max, overruns,
faults), per the RT mandate. No benchmark binary exercises
`JointImpedanceMode` today — adding `benchmark_joint_impedance` (sibling of
`benchmark_cartesian_impedance`) is part of this work, and it lands *before*
the feedforward change so the baseline is real. No RT-path allocation, locking, or blocking is introduced;
`rt_safety_test` gates as usual.

### 5. Defaults as a deliverable

The `medium` profile — what the session default ships pointing at — is tuned
on the real arm and shipped with written acceptance criteria:

- **Hold:** no visible droop or drift holding a commanded posture (the known
  torque-mode droop is the benchmark of what *not* to ship).
- **Track:** documented tracking-error bound on a standard test trajectory at
  default speed scale.
- **Give:** visible, bounded compliance on contact — the point of the mode.

Conformance checks (ROS repo) assert the no-leak guarantee: an impedance goal
with no gains behaves identically before and after a custom-gains goal.

### 6. Execution semantics and hygiene

- **Impedance-aware completion/abort (#17):** divergence thresholds and
  completion checks account for the spring leash instead of inheriting
  position-mode semantics. Exact thresholds set during planning, validated on
  the arm.
- **Mode-switch cost (#6/#59):** measured during this work; a blocking switch
  cost is a finding, not a silent regression.
- **Docs in the same change:** `guide/control-modes.md`, `guide/streaming.md`,
  `docs/interface.md`, `reference/api.md`, deep-dive where the feedforward
  math changes, CHANGELOG. The stale `stream_check.cpp` error strings get
  fixed in passing.

## Compatibility and versioning

This **is** a breaking change, in two places:

- **Driver C++ API:** `TrajectoryGoal` drops `has_gains`/`gains` for
  `ImpedanceGains`, `StreamOpenRequest` grows a field, and `on_set_gains` goes from
  stub to real. Every consumer recompiles; code touching the old fields edits.
- **ROS messages (`rammp_arm_interfaces`):** `ExecuteJointTrajectory` swaps
  its gains fields; the GoTo actions grow fields. Any `.msg` change breaks
  ROS 2 type compatibility, so old and new nodes cannot talk regardless of
  how additive the change looks — it is a coordinated-rebuild event for
  every publisher of these actions, not just `kinova-gen3-ros2`.

The consumer set is closed and version-pinned (`kinova_gen3.repos` pins exact
driver tags; the deployment chain pins images), and precedent exists: the
arbitration round broke `CommandSink` and shipped inside a minor, with
`kinova-gen3-ros2` following in lockstep. **Decided (2026-10-04): ships as
driver v1.3.0** under that same practice, with a coordinated chain release
(interfaces → driver → ros2 → deployments) exactly like lock/via/speed on
2026-10-02 — acknowledged breaking, accepted because the pinned chain absorbs
it. (Standing reminder for future projects: stay on 0.x until truly stable.)

We deliberately do **not** keep the old gains fields alongside `ImpedanceGains` for
compatibility: ROS 2's type hashing gives no wire-compat reward for it, and
keeping `has_gains` alive preserves exactly the zero-filled-gains footgun
(#64) this release exists to kill.

## Testing

- **Unit (core):** gain validation (rejects, reasons), profile resolution,
  no-leak across goal→stream and goal→goal, gains-without-mode-switch, no
  partial resets, shared integrator parity with `JointVelocityMode` behavior,
  feedforward math.
- **RT:** `rt_safety_test` unchanged and green; benchmark before/after
  percentiles recorded in the PR.
- **Conformance (ROS repo):** new-controller availability, gains-at-open,
  no-leak, GoTo opt-in impedance, `SetGains` service gating.
- **Real arm:** the §5 acceptance criteria, run before release.

## Decomposition (for planning)

1. **#64 validation** — standalone, lands first.
2. **Core gains contract** — `ImpedanceGains`, profiles, no-leak/no-reset
   semantics, `on_set_gains`.
3. **Target feedforward port** — with before/after benchmarks.
4. **Shared integrator + impedance velocity/twist pairs.**
5. **Interfaces + ROS** — messages, GoTo fields, stream controllers,
   `SetGains` service, conformance.
6. **Default-profile tuning on the arm** — last, gated on 2–4.

## Open items (resolved during planning, not blockers)

- Exact profile menu and each profile's full parameter set.
- Validation bounds (per-joint torque-limit floors vs. a gravity-headroom
  check).
- Whether `ImpedanceGains` on `StreamOpenRequest` warrants a mid-session
  `set_gains` path too, or open-time only is enough for v1.3.0.
