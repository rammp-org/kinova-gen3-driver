# JointTorqueMode — design

**Date:** 2026-06-17
**Branch:** `feat/joint-torque-mode` (off `main`)
**Status:** approved design, ready for implementation plan

## Summary

Add `JointTorqueMode`: a real-time control mode that commands joint torques as a
**feedforward term on top of gravity compensation**. An external (non-RT) source
publishes a joint-torque setpoint `tau_ff`; each RT cycle the mode outputs

```
tau = scale * gravity(q) - damping * qd + tau_ff_applied
tau = clamp(tau, ±torque_limit)        # clamp on the TOTAL output, per joint
```

When no feedforward has been set (or after the staleness watchdog fires),
`tau_ff_applied = 0` and the mode is **identical to gravity compensation**.
Because of this, `JointTorqueMode` *replaces* the existing
`GravityCompTorqueMode` — gravity comp is simply `JointTorqueMode` with a
permanently-zero feedforward. The old class is deleted and all call sites are
rewired (behavior-preserving).

## Motivation

We have gravity comp working and want the next core controller. A joint-torque
interface is the natural substrate: it lets a teleop loop, a learned policy, or a
planner command joint torques while the driver keeps the arm gravity-compensated,
limited, and safe. It is also the base for a future end-effector wrench mode
(`tau_ff = Jᵀ F_ee`, see Non-goals).

### On drift (explicitly out of scope)

Gravity comp drifts slowly even with a correct URDF: with no position/velocity
*stiffness* term, any residual between `gravity(q)` and the true gravity load
(model error, unmodeled joint friction, payload mismatch) integrates into motion.
A feedforward torque mode inherits this unchanged at `tau_ff = 0` — a pure torque
controller fundamentally cannot hold position; only a stiffness term (the
Cartesian impedance mode) or a closed position loop can. **This PR does not try
to fix drift.** The shipped knob is `damping`, which bounds drift *speed* but does
not null it. Root-causing the drift (model error vs. friction vs. torque-sensor
bias, using the `--dry-run` residuals) is filed as separate future work.

## Design

### New class `JointTorqueMode`

Files: `include/kinova_lowlevel/joint_torque_mode.h`,
`src/joint_torque_mode.cpp`. Implements the existing `ControlMode` interface and
mirrors the structure of the current `GravityCompTorqueMode`, plus a
double-buffered feedforward setter and a staleness watchdog.

```cpp
struct JointTorqueParams {
  double scale        = 1.0;    // gravity scale
  double damping      = 0.0;    // joint velocity damping (N·m·s/rad)
  double torque_limit = 39.0;   // per-joint clamp on TOTAL output (N·m)
  double cmd_timeout_s = 0.1;   // watchdog: stale tau_ff after this; <=0 disables
  double cmd_decay_s   = 0.05;  // on staleness, linearly ramp tau_ff -> 0 over
                                // this window (0 => hard zero, avoids a torque step)
};

class JointTorqueMode : public ControlMode {
 public:
  JointTorqueMode(Dynamics& dyn, JointTorqueParams p = {});
  ActuatorModes required_modes() const override;   // all kTorque
  void on_enter(const JointFeedback& fb) override;  // zero tau_ff, reset watchdog
  void compute(const JointFeedback& fb, double dt_s, JointCommand& out) override;
  void on_exit() override {}

  // Non-RT setter (call from ONE supervisor thread).
  void set_torque(const JointVec& tau_ff) noexcept;

 private:
  Dynamics& dyn_;
  JointTorqueParams p_;
  // tau_ff double-buffer (single non-RT writer -> RT reader), same pattern as
  // CartesianImpedanceMode::set_target. A monotonically increasing write counter
  // lets compute() detect "a fresh command arrived this cycle".
  JointVec tau_ff_buf_[2];
  std::atomic<int> tau_ff_active_{0};
  std::atomic<uint64_t> write_count_{0};
  // RT-thread watchdog state (not shared).
  uint64_t last_seen_write_ = 0;
  JointVec tau_ff_target_ = JointVec::Zero();   // latest adopted command
  JointVec tau_ff_applied_ = JointVec::Zero();  // value actually summed in (post-decay)
  double stale_s_ = 0.0;
  // Preallocated RT scratch.
  JointVec g_;
  JointVec tau_;
};
```

### `compute()` (RT-safe, no allocation, no clock calls)

1. **Read the published feedforward** (acquire): load `write_count_`, then read the
   active `tau_ff_buf_` snapshot.
2. **Detect freshness / advance the watchdog** using accumulated `dt_s` (no
   `steady_clock` in the RT path):
   - if `write_count_ != last_seen_write_`: a new command arrived →
     `tau_ff_target_ = snapshot`, `last_seen_write_ = write_count_`, `stale_s_ = 0`.
   - else: `stale_s_ += dt_s`.
3. **Resolve the applied feedforward**:
   - if `cmd_timeout_s > 0` and `stale_s_ >= cmd_timeout_s`: linearly ramp
     `tau_ff_applied_` toward `0` over `cmd_decay_s` (hard zero if `cmd_decay_s <= 0`).
   - else: `tau_ff_applied_ = tau_ff_target_` (applied immediately — the caller owns
     command smoothness; the decay ramp exists only to soften the watchdog drop).
4. **Compose and clamp**:
   `dyn_.gravity(fb.q, g_)`;
   `tau_ = p_.scale * g_ - p_.damping * fb.qd + tau_ff_applied_`;
   clamp each `tau_[i]` to `±p_.torque_limit`.
5. **Emit**: `out.mode = kTorque`, `out.torque = tau_`, `out.position = fb.q`
   (position passthrough, matching the current grav-comp mode).

A later `set_torque` advances `write_count_`, so the mode re-engages cleanly after
a watchdog timeout.

### Concurrency / RT-safety

- Single non-RT writer (`set_torque`) → single RT reader (`compute`), exactly the
  two-buffer + atomic-index publication pattern already used by
  `CartesianImpedanceMode::set_target`. The write counter is an
  `atomic<uint64_t>` (release on write, acquire on read).
- All buffers and scratch are members allocated at construction; `compute()`
  performs no allocation and calls no clock — staleness is tracked by summing the
  `dt_s` the executor already passes in. This keeps it consistent with the
  existing `rt_safety_test` no-allocation guarantees.

## Consolidation: remove `GravityCompTorqueMode`

`GravityCompTorqueMode` is `JointTorqueMode` with a permanently-zero feedforward,
so it is removed and its uses rewired. Blast radius on this branch:

**Delete**
- `include/kinova_lowlevel/gravity_comp_mode.h`
- `src/gravity_comp_mode.cpp`

**Rewire (behavior-preserving)**
- `apps/benchmark_grav_comp.cpp`: construct `JointTorqueMode` with default params,
  never call `set_torque`. **Keep the app and its `--dry-run` read-only gravity
  validation** — that residual check (`tau_meas - gravity(q)` across poses) is a
  genuinely gravity-comp-specific diagnostic and the URDF-validation entry point
  documented in `docs/integration/grav_comp_static_check.md`. The app name and
  flags stay; only the mode construction changes.
- `tests/rt_safety_test.cpp` (2 usages) → `JointTorqueMode`.
- `tests/gravity_comp_mode_test.cpp` → renamed/rewritten as
  `tests/joint_torque_mode_test.cpp` (see Testing).
- `CMakeLists.txt`: swap `src/gravity_comp_mode.cpp` → `src/joint_torque_mode.cpp`
  in `KINOVA_LIB_SOURCES`; swap the test file in the `unit_tests` target.
- Docs prose reframing "grav comp mode" as "`JointTorqueMode` with zero
  feedforward": `README.md`, `docs/integration-runbook.md`,
  `docs/integration/grav_comp_static_check.md`, `docs/rt-tuning.md`. Historical
  specs/plans under `docs/superpowers/` are left as-is.

**Untouched**
- `src/rt_executor.cpp` / `src/rt_system.cpp` mention `grav_comp_test.cpp` only in
  comments referencing an external prototype file — not our class.
- The Cartesian impedance mode (open PR, not on `main`) computes gravity directly
  via `Dynamics` and never referenced `GravityCompTorqueMode`, so this
  consolidation does not disturb it.

## Testing (TDD)

`tests/joint_torque_mode_test.cpp`, mirroring the current grav-comp tests plus new
cases:

1. **Grav-comp equivalence** — no `set_torque`: output equals
   `clamp(scale*gravity(q) - damping*qd)`, `mode == kTorque`, all `required_modes`
   are `kTorque`, `position` passthrough. (Carries over the existing
   `OutputsClampedGravityAndPassthrough` and `DampingSubtractsVelocityTerm` cases.)
2. **Feedforward adds in** — after `set_torque(tau_ff)`, output equals
   `clamp(scale*gravity(q) - damping*qd + tau_ff)`; with a generous `torque_limit`,
   exactly `gravity + tau_ff` (no clamp).
3. **Total clamp** — a large `tau_ff` is clamped to `±torque_limit` on the total.
4. **Watchdog zeros stale feedforward** — set a feedforward, then call `compute`
   enough cycles (`> cmd_timeout_s`) without re-setting: applied feedforward decays
   to 0 and output returns to the grav-comp value. (Use `cmd_decay_s = 0` for a
   crisp assertion, and a separate case asserting the decay path is monotone.)
5. **Watchdog resets on fresh command** — re-calling `set_torque` before timeout
   keeps the feedforward applied; re-calling after a timeout re-engages it.
6. **`on_enter` resets** — entering zeros any prior feedforward and the watchdog.

RT no-allocation behavior is covered by extending the existing `rt_safety_test`
(which already drives the mode) to the renamed class.

## Non-goals / future work

- **End-effector wrench mode** (`EndEffectorWrenchMode` or similar, keeping the
  `*Mode` naming): commands a 6-DOF EE wrench `F`, outputs
  `tau = gravity(q) + Jᵀ F` through the same clamp + watchdog path. It is a thin
  layer over `JointTorqueMode` (`tau_ff = Jᵀ F`, `J` from `dyn_.jacobian`,
  `LOCAL_WORLD_ALIGNED`, the same Jacobian impedance uses). Own spec/PR.
- **Drift root-cause + correction** (residual-gravity calibration, friction
  identification). Own investigation.
- **Benchmark/demo app for `JointTorqueMode`** (e.g. `--tau-ff` injection in sim
  for RT-timing): deferred until hardware is available.
- **Per-command slew limiting** of `tau_ff` on fresh commands: not needed now; the
  caller owns command smoothness. The watchdog decay handles only the drop-to-zero.

## Acceptance criteria

- `JointTorqueMode` exists with the params, setter, watchdog, and `compute`
  semantics above; `tau_ff = 0` reproduces the current grav-comp output exactly.
- `GravityCompTorqueMode` is removed and every call site (app, tests, CMake, docs)
  is rewired with no behavior change to the gravity-validation workflow.
- `cmake --build build && ctest` passes, including the new
  `joint_torque_mode_test` and the existing `rt_safety_test`.

## Reconciled 2026-08-26 (ported to main)

- `torque_limit` is now **per-joint** (`JointVec`, default `(39,39,39,39,9,9,9)`),
  not the scalar `39.0` this spec describes. A scalar sized for the proximal
  joints overruns the wrist by 4x — the same defect `JointImpedanceParams`
  documents.
- The "Consolidation" section's removal of `GravityCompTorqueMode` was carried
  out. `benchmark_grav_comp` was **kept and retargeted** to `JointTorqueMode`,
  not removed: it backs the on-robot procedure in
  `docs/integration/grav_comp_static_check.md` and is now the joint-torque-path
  benchmark.
