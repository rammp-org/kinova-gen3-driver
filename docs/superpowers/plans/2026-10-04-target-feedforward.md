# Target Feedforward + Joint Impedance Benchmark (v1.3.0 Plan 2) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Joint-space targets carry `{q, qd, qdd}` end to end, and `JointImpedanceMode` damps the velocity *error* and feeds the planned acceleration forward — removing the standing tracking lag (`2·zeta·qd·√(M/Kq)`) that makes default gains feel mushy on trajectories. A `benchmark_joint_impedance` app lands FIRST so the before/after per-cycle cost is measured, not assumed.

**Architecture:** Port of unmerged `feat/target-feedforward` (`89f0777`, Aug 14) onto the v1.2.0+gains lineage. `JointTargetSink` gains a `JointTarget` struct and a `set_joint_target` virtual (position-only `set_target` stays as a non-virtual convenience, so every existing caller compiles unchanged). The executor's `sample_target` returns the derivatives of the very polynomial `sample()` evaluates, **scaled by the applied speed scale** (`qd × s`, `qdd × s²`) — the scale landed after the old branch and full-speed derivatives under a slowed goal would be wrong. The mode uses the ACHIEVED reference velocity (post-rate-limit `(q_d − q_prev)/dt`), so the limiter bounds the feedforward for free.

**Deliberately NOT ported from `89f0777`:** the position-mode velocity-feedforward half (`JointCommand::velocity_active`, the KORTEX kPosition velocity write, `JointPositionParams::velocity_feedforward`). It shipped off-by-default, its firmware semantics are explicitly unverifiable in sim, and it adds Jetson-only compile surface. It stays a future change; `JointPositionMode` here only adapts to the new sink signature (uses `q`, ignores derivatives).

**Tech Stack:** C++17, gtest, Eigen; sim benchmark on the configured `./build`.

**Spec:** `docs/superpowers/specs/2026-10-04-compliant-execution-everywhere-design.md` §4

## Global Constraints

- No allocation/locks/blocking added to `compute()` or the executor tick; `rt_safety_test` gates.
- Benchmark before/after on the same machine, same flags; record p50/p99/p99.9/max + faults in the commit message.
- The teleop/pose/entry-hold paths keep their behavior EXACTLY: feedforward engages only when the active target carries a profile (`has_velocity`), and never while the staleness freeze is latched.

## Review Focus

1. **Frozen reference + stale profile target** — the freeze must also zero the feedforward, or the damper fights the hold. Test in Task 2.
2. **Speed-scaled goal** — `qd`/`qdd` must scale with the dilated clock (`s`, `s²`), or a 0.2× goal feeds forward 5× the achieved motion. Test in Task 3.
3. **Rate-limiter clamp** — achieved-reference velocity (not planner `qd`) must be what's fed forward when `max_ref_speed` clamps. Test in Task 2.
4. **Positions-only trajectory** — no `has_velocities` ⇒ no feedforward anywhere; linear-interp behavior byte-identical. Test in Task 3.
5. **Hold segments** (before first / after last waypoint of a profiled trajectory) — `has_velocity` true with zero `qd`, so the damper damps toward zero while holding, not toward a phantom. Test in Task 3.

---

### Task 1: `benchmark_joint_impedance` (baseline first)

**Files:**
- Create: `apps/benchmark_joint_impedance.cpp` (adapted from `apps/benchmark_cartesian_impedance.cpp`)
- Modify: `CMakeLists.txt` (register the executable, same block shape as the cartesian one at `:180-198`)

**Interfaces:**
- Consumes: `JointImpedanceMode`, `RtExecutor`, `SimTransport`, telemetry — all existing.
- Produces: `./build/benchmark_joint_impedance --sim --urdf models/gen3_7dof_2f85.urdf --rate 1000 --duration 5 [--track] [--kq N] [--zeta Z] [--csv path]`. `--track` streams a 0.3 rad, 0.2 Hz sinusoidal joint target with analytic qd/qdd at 100 Hz from a non-RT thread (exercises the kJoint + feedforward path); default holds entry.

- [ ] **Step 1:** Write the app: clone the cartesian benchmark's structure (arg parse, transport, drain thread, report block verbatim), swap the mode for `JointImpedanceMode mode(dyn, p)` with `--kq/--zeta/--torque-limit/--leash` flags mapping onto `JointImpedanceParams`, drop the dry-run/orientation printing, and add the `--track` target thread (sin wave on joints 0–6, `set_target(JointVec)` for now — Task 2 upgrades it to a profiled `JointTarget`).
- [ ] **Step 2:** Build and run the BASELINE: `cmake --build build -j && ./build/benchmark_joint_impedance --sim --urdf models/gen3_7dof_2f85.urdf --rate 1000 --duration 5 --track`. Record `compute_ns` percentiles.
- [ ] **Step 3:** Commit (`feat(bench): joint impedance benchmark`) with the baseline numbers in the message.

### Task 2: `JointTarget` through the sink; impedance feedforward

**Files:**
- Modify: `include/kinova_lowlevel/joint_target_sink.h` (add `JointTarget`, `set_joint_target` pure virtual, non-virtual `set_target` convenience)
- Modify: `include/kinova_lowlevel/joint_impedance_mode.h` + `src/joint_impedance_mode.cpp` (target buffer becomes `JointTarget[2]`; `set_joint_target`; `qd_ref_` + `last_ref_velocity()`; compute: damp `(fb.qd − qd_ref_)`, add `M·qdd` when provided)
- Modify: `include/kinova_lowlevel/joint_position_mode.h` + `src/joint_position_mode.cpp` (rename override to `set_joint_target(const JointTarget& t)`, store `t.q`, behavior unchanged)
- Modify: `apps/benchmark_joint_impedance.cpp` (`--track` sends a profiled `JointTarget`)
- Test: `tests/joint_impedance_mode_test.cpp` additions

**Interfaces:**
- Produces: `kinova::JointTarget { JointVec q, qd, qdd; bool has_velocity; bool has_acceleration; }`; `JointTargetSink::set_joint_target(const JointTarget&)` (virtual); `set_target(const JointVec&)` = `set_joint_target(JointTarget{q})` (non-virtual — every existing caller unchanged); `JointImpedanceMode::last_ref_velocity()`.

- [ ] **Step 1:** Write failing tests: (a) profiled target at constant qd ⇒ steady-state spring error ≈ 0 (vs. the documented lag without ff); (b) rate-limiter clamp ⇒ `last_ref_velocity()` equals the CLAMPED step/dt, not the planner's qd; (c) staleness freeze with a profiled target ⇒ `last_ref_velocity()` is zero and torque equals the hold torque; (d) position-only `set_target` ⇒ `last_ref_velocity()` zero (teleop path unchanged).
- [ ] **Step 2:** Implement per `git show 89f0777` adapted to the current file state (watchdog `wd_.bump()` stays in `set_joint_target`; the `frozen_` branch sets no ff flags; `qd_ref_` computed post-rate-limit, pre-wrap, only when `ff_velocity && dt_s > 0`).
- [ ] **Step 3:** Full suite green; commit.

### Task 3: Executor `sample_target` with speed-scale-correct derivatives

**Files:**
- Modify: `src/interface/trajectory_executor.cpp` (add `sample_target`; `tick` builds the ref, scales `qd *= applied_`, `qdd *= applied_ * applied_`, calls `sink_.set_joint_target(ref)`)
- Modify: `include/kinova_lowlevel/interface/trajectory_executor.h` (declare `sample_target` next to `sample` for tests)
- Test: `tests/interface/trajectory_executor_test.cpp` additions

**Interfaces:**
- Produces: `kinova::JointTarget sample_target(const Trajectory&, double t_s)` (free function, same namespace as `sample`).

- [ ] **Step 1:** Failing tests: (a) `sample_target` qd/qdd match finite differences of `sample` on a profiled trajectory (1e-6 step, 1e-3 tolerance), interior point; (b) positions-only trajectory ⇒ `has_velocity == false`; (c) before-start/after-end of a profiled trajectory ⇒ `has_velocity == true` with zero qd; (d) executor at `speed_scale 0.5` ⇒ sink receives qd ≈ 0.5× the unscaled analytic qd (capture via a recording fake sink).
- [ ] **Step 2:** Implement (port `sample_target` from `89f0777` verbatim — the coefficient math is unchanged — then the tick wiring with scaling).
- [ ] **Step 3:** Full suite green; re-run the Task 1 benchmark with `--track` (now profiled); compare percentiles against baseline in the commit message. Commit.
