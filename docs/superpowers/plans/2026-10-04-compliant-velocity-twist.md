# Compliant Velocity/Twist (v1.3.0 Plan 3) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `joint velocity × kImpedance` and `EE twist × kImpedance` become valid streaming
pairs: the velocity/twist command integrates into a leashed joint position reference —
the same proven mechanism `JointVelocityMode` has used since v1.1.1 (#34) — and drives
`JointImpedanceMode` through its `JointTargetSink`, with the commanded `qd` fed forward.

**Architecture:** The reusable math (integrate-and-leash step; twist→qd damped-least-squares
resolution with null-space posture) is factored out of `JointVelocityMode` into a small
shared unit (`velocity_reference.h|cpp`); `JointVelocityMode` keeps byte-identical behavior.
For impedance sessions the integration runs in the Supervisor's **sampler loop at 1 kHz**
(client rates are irregular; the backend thread only stores the latest command). The twist
path needs a Jacobian, so `pump_dyn_` calls gain a mutex shared by the pump and sampler
threads (both non-RT). The RT path — every mode's `compute()` and `RtExecutor` — is untouched.

**Tech Stack:** C++17, gtest, Eigen, Pinocchio (via `Dynamics`). Sim-only x86 build
(`./build` is configured; `cmake --build build -j`).

**Spec:** `docs/superpowers/specs/2026-10-04-compliant-execution-everywhere-design.md` §3

## Global Constraints

- Nothing in the RT path may allocate, lock, or block; this plan must not touch any mode's
  `compute()` or `RtExecutor`. All new work lives on the sampler/backend threads.
- `JointVelocityMode` stays stiff by contract and behaviorally untouched: its existing tests
  pass unchanged.
- Guard rails unchanged: `velocity × kVelocity` and `twist × kVelocity` keep their existing
  path through `vel_`.
- The session deadline (sampler-side `close_stream`) is the staleness authority for the new
  pairs: the sampler's 1 kHz writes keep `imp_`'s own watchdog fresh by design.
- Lock order: `stream_mtx_` before `dyn_mtx_`; `dyn_mtx_` is never taken on the RT path.
- Full suite green at every task end (known flake: `Supervisor.StreamingAPoseIntoPositionModeIsAccepted`
  under full-suite load — rerun in isolation to confirm).
- Every commit message ends with the session attribution lines.

## Review Focus

1. **Byte-identical `JointVelocityMode`** after the refactor — its 25 existing tests are the
   proof; no parameter, limit, or ordering change rides along.
2. **Two writers on one double buffer** — the backend must never call `imp_.set_joint_target`
   for the new pairs (only the sampler writes it), and the sampler must re-check
   `stream_open_` under `stream_mtx_` so it cannot write after `close_stream`'s hold latch.
3. **Leash against measured q** — a blocked/lagging arm must cap the reference lead at
   0.1 rad, exactly like `JointVelocityMode` (windup guard; the bound on spring force under
   contact). Test with the static sim echo.
4. **Seeding at open** — `q_ref` seeds from measured q and the stored command zeroes, so a
   session that opens and says nothing holds (no stale command from a previous session).
5. **Deadlock-free Jacobian sharing** — pump takes only `dyn_mtx_`; sampler takes
   `stream_mtx_` then `dyn_mtx_`; `close_stream` takes only `stream_mtx_`. No inversion.

---

### Task 1: Factor the shared velocity-reference math out of JointVelocityMode

**Files:**
- Create: `include/kinova_lowlevel/velocity_reference.h`, `src/velocity_reference.cpp`
- Create: `tests/velocity_reference_test.cpp`
- Modify: `include/kinova_lowlevel/joint_velocity_mode.h`, `src/joint_velocity_mode.cpp`
  (delegate to the shared unit; public surface unchanged)
- Modify: `CMakeLists.txt` (lib + test source lists)

**Produces:**
- `kinova::kVelocityRefMaxLead` (= 0.1 rad, the leash)
- `void integrate_leashed_reference(JointVec& q_ref, const JointVec& qd, double dt_s,
  const JointVec& q_meas, double max_lead, const std::array<bool,kNumJoints>& continuous,
  const JointVec& q_lower, const JointVec& q_upper) noexcept` — one integrate+leash+wrap+clamp step
- `struct TwistDlsParams { dls_damping, w_threshold, dls_damping_max, posture_gain, q_rest }`
  (defaults identical to `JointVelocityParams`)
- `class TwistDlsSolver` — preallocated scratch; `solve(J, q, V, params, continuous, qd_out)`
  (caller supplies the Jacobian — `JointVelocityMode` computes it on the RT thread, the
  Supervisor under `dyn_mtx_`); `last_manipulability()`, `reset()`

- [ ] Write failing tests (`tests/velocity_reference_test.cpp`): integration advances `q_ref`
  by `qd*dt`; leash caps the lead at `max_lead` when `q_meas` lags; continuous joints wrap
  across ±π (short way) and bounded joints clamp to limits; `TwistDlsSolver` reproduces a
  commanded twist away from singularity (`J*qd ≈ V`); null-space posture pulls toward
  `q_rest` under a zero twist without disturbing the task (`J*qd ≈ 0`).
- [ ] Add the new unit + wire CMake; run `--gtest_filter='VelocityReference*:TwistDls*'` → pass.
- [ ] Refactor `JointVelocityMode` to delegate: `compute()`'s integrate/leash tail calls
  `integrate_leashed_reference`; `solve_twist` computes `J_` then calls the solver member;
  `last_manipulability()` reads the solver; `on_enter` resets it.
- [ ] Run `--gtest_filter='JointVelocityMode*'` → all pass UNCHANGED, then the full suite.
- [ ] Commit: `refactor(modes): factor the velocity->leashed-reference integrator and twist DLS out of JointVelocityMode (#63)`

### Task 2: Pair table admits velocity/twist × impedance; gains-at-open covers them

**Files:**
- Modify: `src/interface/streaming_session.cpp` (pair table), `include/kinova_lowlevel/interface/streaming_session.h` (comment)
- Modify: `tests/interface/streaming_session_test.cpp` (pair-table tests; the stale
  "twist×impedance is nonsense" expectations flip)
- Modify: `tests/interface/supervisor_test.cpp` (`StreamOpenRefusesABadRequestBeforeSwitchingModes`'s
  bad pair becomes `kJointTorque × kImpedance`; new accept + gains-at-open tests)

**Note:** `on_stream_open` already switches on `r.control_mode`, so an impedance open
requests `imp_`, arms `imp_`'s watchdog, and applies gains-at-open with no supervisor change —
Task 2 proves that with tests; Task 3 makes the session actually drive the mode.

- [ ] Write failing tests: `pair_supported(kJointVelocity|kEeTwist, kImpedance)` true (both
  still false for kPosition/kTorque); supervisor accepts both opens and `on_query_stream()`
  reports `control_mode == kImpedance`; a `kStiff` profile at a velocity×impedance open
  lands on `imp_.params()`.
- [ ] Flip the two table lines; update the stale tests; run the suite.
- [ ] Commit: `feat(interface): joint velocity and EE twist streams may open in impedance (#63)`

### Task 3: Sampler-side integration drives the impedance mode

**Files:**
- Modify: `include/kinova_lowlevel/interface/supervisor.h` (members: `dyn_mtx_`,
  `stream_q_ref_`, `stream_qd_cmd_`, `stream_twist_cmd_`, cached URDF limits,
  `TwistDlsSolver` + `Jacobian6` sampler scratch)
- Modify: `src/interface/supervisor.cpp`:
  - `pump_loop`: `fk`/`jacobian` under `dyn_mtx_`
  - `on_stream_open`: for the two new pairs, seed `stream_q_ref_` from measured q
    (fallback `imp_.reference()`) and zero the stored commands, under `stream_mtx_`
  - `on_setpoint_joint_velocity` / `on_setpoint_twist`: when `session_.control_mode() == kImpedance`,
    store the command (under `stream_mtx_`) instead of calling `vel_`
  - `sampler_loop`: each tick while such a session is open — resolve twist→qd (Jacobian under
    `dyn_mtx_`), `integrate_leashed_reference` against measured q with measured dt, then
    `imp_.set_joint_target({q=q_ref, qd=cmd, has_velocity=true})`; re-check `stream_open_`
    under `stream_mtx_`
- Modify: `tests/interface/supervisor_test.cpp`

No zero-velocity latch analogous to `vel_` is needed in `close_stream`: integration stops
with the session, and the existing `sink_for(kImpedance)` hold latches measured q.

- [ ] Write failing tests:
  - velocity×impedance stream at 0.05 rad/s for ~0.5 s moves `imp_.reference()` off the
    entry pose by roughly rate×time (sim q static ⇒ stays under the leash)
  - leash: 1.0 rad/s for ~0.4 s caps `imp_.reference() - q_meas` at ≈0.1 rad
  - twist×impedance (fixture seeded off-singularity): nonzero twist ⇒ `imp_.reference()` moves
  - stale session: no setpoints past `timeout_s` ⇒ closed, `stream_close_cause() == kDeadlineExpired`,
    hold at measured q
- [ ] Implement; run the new tests, then the full suite (RT untouched — `rt_safety_test` proves it).
- [ ] Commit: `feat(interface): velocity and twist streams integrate into a leashed impedance reference (#63)`

### Task 4: Documentation

**Files:**
- Modify: `docs/guide/streaming.md` (pair table rows + the two new controllers' semantics:
  compliant, leashed, qd fed forward; staleness authority = session deadline, motion
  continues leash-bounded for up to `timeout_s` after the last setpoint)
- Do NOT touch `docs/guide/control-modes.md`, `docs/reference/api.md`, `docs/interface.md`,
  `CHANGELOG.md` (orchestrator-owned).

- [ ] Write the docs; `mkdocs build` if available.
- [ ] Commit: `docs(streaming): compliant velocity/twist pairs -- semantics and pair table (#63)`
