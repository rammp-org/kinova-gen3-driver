# JointTorqueMode Port Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Land `JointTorqueMode` on `main`, retire `GravityCompTorqueMode`, and leave the joint-torque path benchmarked and documented.

**Architecture:** `JointTorqueMode` computes `tau = scale*gravity(q) - damping*qd + tau_ff`, clamped per joint, where `tau_ff` is published by one non-RT thread through a double-buffer and read once per RT cycle. A staleness watchdog decays `tau_ff` to zero when commands stop, reverting to gravity-compensation hold. With `tau_ff` never set it is exactly `GravityCompTorqueMode`, which is why that mode is removed rather than kept alongside.

**Tech Stack:** C++17, Eigen, Pinocchio (via `Dynamics`), GoogleTest, CMake.

**Spec:** `docs/superpowers/specs/2026-06-17-joint-torque-mode-design.md` (ported in Task 1), and Plan 1 of `docs/superpowers/specs/2026-08-26-streaming-setpoints-design.md`

## Global Constraints

- **Builds on Linux/aarch64 only (the Jetson, `abra`).** x86_64 dev boxes cannot build — no Pinocchio. Every "green" claim needs a real build+ctest on the Jetson.
- Build: `cmake -S . -B build -DCMAKE_PREFIX_PATH=/usr/local/lib/python3.10/dist-packages/cmeel.prefix && cmake --build build -j && ctest --test-dir build --output-on-failure`
- Subset: `./build/unit_tests --gtest_filter='JointTorque*'` — all tests are one gtest binary registered as a single ctest test.
- **Nothing in the RT path (`compute`, executor cycle) may allocate, lock, or block.** No clock calls in `compute()` either — staleness is tracked by summing `dt_s`.
- **SI / radians internally.** `kNumJoints = 7`.
- **`Dynamics::gravity(const JointVec& q, JointVec& tau_out)`** is the two-argument, alloc-free form on `main`. The ported code already uses it.
- There is **no CI on this repo**. The Jetson is the only gate.
- Work on a branch off `main`: `git checkout -b feat/joint-torque-mode-port origin/main`. Do **not** branch off `docs/streaming-setpoints-spec` or `feat/arm-arbitration` — this plan is independent of both and must be reviewable on its own.

## Approach: port, do not rebase

`origin/feat/joint-torque-mode` is **8 commits ahead and 97 behind** `main`. Replaying those commits would conflict in `CMakeLists.txt`, `README.md` and `tests/rt_safety_test.cpp` — all of which have moved substantially — for no benefit, because the branch's actual *content* is three new files plus integration edits that are faster to redo than to merge.

So: take the new files verbatim from the branch, and redo every integration edit by hand against current `main`. The branch is a source of content, not a merge base. Do **not** `git rebase` or `git merge` it.

## File Structure

| File | Responsibility |
|---|---|
| `include/kinova_lowlevel/joint_torque_mode.h` (create, from branch) | Params, the double-buffered `set_torque` seam, watchdog state |
| `src/joint_torque_mode.cpp` (create, from branch) | `compute()`: freshness, decay, compose, clamp |
| `tests/joint_torque_mode_test.cpp` (create, from branch) | Mode unit tests incl. the watchdog |
| `docs/superpowers/specs/2026-06-17-joint-torque-mode-design.md` (create, from branch) | The mode's design record |
| `include/kinova_lowlevel/gravity_comp_mode.h`, `src/gravity_comp_mode.cpp`, `tests/gravity_comp_mode_test.cpp` (delete) | Superseded |
| `tests/rt_safety_test.cpp` (modify, lines 32 and 242) | Retarget the two baseline RT tests |
| `apps/benchmark_grav_comp.cpp` (modify) | Retarget to `JointTorqueMode`; add `--ee-frame` (issue #18) |
| `CMakeLists.txt` (modify) | Swap the source and test entries |
| `README.md`, `CLAUDE.md`, `docs/reference/api.md`, `docs/guide/control-modes.md`, `docs/rt-tuning.md`, `docs/getting-started.md`, `docs/integration-runbook.md`, `docs/integration/grav_comp_static_check.md` (modify) | Reframe gravity comp as `JointTorqueMode` at zero feedforward |

June-dated files under `docs/superpowers/specs|plans/` that mention `benchmark_grav_comp` are **historical records — do not edit them.**

---

### Task 1: Port the mode alongside the old one

`GravityCompTorqueMode` stays for now. Both coexist so this task's deliverable is independently verifiable: the new mode builds and passes its own tests before anything is deleted.

**Files:**
- Create: `include/kinova_lowlevel/joint_torque_mode.h`, `src/joint_torque_mode.cpp`, `docs/superpowers/specs/2026-06-17-joint-torque-mode-design.md`
- Test: `tests/joint_torque_mode_test.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Produces: `kinova::JointTorqueMode(Dynamics&, JointTorqueParams p = {})`, `void set_torque(const JointVec& tau_ff) noexcept`, and `struct JointTorqueParams { double scale; double damping; double torque_limit; double cmd_timeout_s; double cmd_decay_s; }`. Tasks 2–5 all consume these. **Note `torque_limit` is a `double` here and becomes a `JointVec` in Task 2.**

- [ ] **Step 1: Copy the four files from the branch**

```bash
git show origin/feat/joint-torque-mode:include/kinova_lowlevel/joint_torque_mode.h > include/kinova_lowlevel/joint_torque_mode.h
git show origin/feat/joint-torque-mode:src/joint_torque_mode.cpp                  > src/joint_torque_mode.cpp
git show origin/feat/joint-torque-mode:tests/joint_torque_mode_test.cpp           > tests/joint_torque_mode_test.cpp
git show origin/feat/joint-torque-mode:docs/superpowers/specs/2026-06-17-joint-torque-mode-design.md \
  > docs/superpowers/specs/2026-06-17-joint-torque-mode-design.md
```

- [ ] **Step 2: Register in CMake**

In `CMakeLists.txt`, add to `KINOVA_LIB_SOURCES` next to `src/gravity_comp_mode.cpp`:

```cmake
    src/joint_torque_mode.cpp
```

and to the `unit_tests` source list next to `tests/gravity_comp_mode_test.cpp`:

```cmake
    tests/joint_torque_mode_test.cpp
```

- [ ] **Step 3: Run the tests to verify they pass**

Run: `cmake -S . -B build -DCMAKE_PREFIX_PATH=/usr/local/lib/python3.10/dist-packages/cmeel.prefix && cmake --build build -j && ./build/unit_tests --gtest_filter='JointTorque*'`
Expected: PASS. If it fails to compile, the likely cause is drift in `Dynamics` or `ControlMode` since June — reconcile against the current headers rather than changing the mode's logic.

- [ ] **Step 4: Run the whole suite**

Run: `ctest --test-dir build --output-on-failure`
Expected: PASS — `GravityCompTorqueMode` is untouched, so nothing else moves.

- [ ] **Step 5: Commit**

```bash
git add include/kinova_lowlevel/joint_torque_mode.h src/joint_torque_mode.cpp \
        tests/joint_torque_mode_test.cpp docs/superpowers/specs/2026-06-17-joint-torque-mode-design.md \
        CMakeLists.txt
git commit -m "feat(modes): port JointTorqueMode (feedforward torque + gravity comp)"
```

---

### Task 2: Per-joint torque limit

`JointTorqueParams::torque_limit` is a scalar `39.0`. `JointImpedanceParams` already documents why that is wrong: *"The URDF gives joints 5-7 an effort limit of 9 N·m; the single scalar CartesianImpedanceParams uses would overrun the wrist by 4x."* This mode has the bug the impedance mode was written to avoid.

**Files:**
- Modify: `include/kinova_lowlevel/joint_torque_mode.h`, `src/joint_torque_mode.cpp`
- Test: `tests/joint_torque_mode_test.cpp`

**Interfaces:**
- Consumes: `JointTorqueMode` from Task 1.
- Produces: `JointTorqueParams::torque_limit` is now `JointVec`, defaulting to `(39, 39, 39, 39, 9, 9, 9)`. Tasks 3–5 construct the mode with this type.

- [ ] **Step 1: Write the failing test**

Append to `tests/joint_torque_mode_test.cpp`:

```cpp
TEST(JointTorqueMode, WristClampsAtItsOwnLowerLimit) {
  Dynamics dyn(URDF_PATH);
  // Default limits: 39 N*m for joints 1-4, 9 N*m for the wrist (5-7).
  JointTorqueMode m(dyn, {1.0, 0.0, (JointVec() << 39,39,39,39,9,9,9).finished(), 0.0, 0.0});
  m.set_torque(JointVec::Constant(1000.0));         // demand far beyond every limit
  JointFeedback fb; fb.q.setZero(); fb.qd.setZero();
  JointCommand out;
  m.compute(fb, 0.001, out);
  EXPECT_NEAR(out.torque[0], 39.0, 1e-9);           // proximal joint at its limit
  EXPECT_NEAR(out.torque[5], 9.0, 1e-9);            // wrist must NOT be allowed 39
  EXPECT_NEAR(out.torque[6], 9.0, 1e-9);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='JointTorqueMode.WristClamps*'`
Expected: compile error — `JointTorqueParams` takes a `double` for `torque_limit`, not a `JointVec`.

- [ ] **Step 3: Change the parameter type**

In `include/kinova_lowlevel/joint_torque_mode.h`, replace the scalar field:

```cpp
  // Per-joint ceiling on the TOTAL output. The URDF gives joints 5-7 an effort
  // limit of 9 N*m; a single scalar sized for the proximal joints would overrun
  // the wrist by 4x. Mirrors JointImpedanceParams::torque_limit.
  JointVec torque_limit = (JointVec() << 39, 39, 39, 39, 9, 9, 9).finished();
```

In `src/joint_torque_mode.cpp`, change the clamp loop:

```cpp
  for (int i = 0; i < kNumJoints; ++i) {
    tau_[i] = std::clamp(tau_[i], -p_.torque_limit[i], p_.torque_limit[i]);
  }
```

- [ ] **Step 4: Fix the ported tests that construct a scalar limit**

The tests copied in Task 1 build params with a scalar third field, e.g. `{1.0, 0.0, 39.0, ...}`. Change each to `{1.0, 0.0, JointVec::Constant(39.0), ...}` — using a uniform `39.0` preserves each test's original intent, so only the type changes, not what is being asserted.

Find them with: `grep -n "JointTorqueMode m" tests/joint_torque_mode_test.cpp`

- [ ] **Step 5: Run tests to verify they pass**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='JointTorque*'`
Expected: PASS, including the new wrist test.

- [ ] **Step 6: Commit**

```bash
git add include/kinova_lowlevel/joint_torque_mode.h src/joint_torque_mode.cpp tests/joint_torque_mode_test.cpp
git commit -m "fix(modes): per-joint torque limit — a scalar overruns the wrist 4x"
```

---

### Task 3: Retire `GravityCompTorqueMode`

**Files:**
- Delete: `include/kinova_lowlevel/gravity_comp_mode.h`, `src/gravity_comp_mode.cpp`, `tests/gravity_comp_mode_test.cpp`
- Modify: `tests/rt_safety_test.cpp` (lines 32 and 242), `CMakeLists.txt`

**Interfaces:**
- Consumes: `JointTorqueMode` with the `JointVec` limit from Task 2.

- [ ] **Step 1: Retarget the two RT-safety tests**

In `tests/rt_safety_test.cpp`, replace the include:

```cpp
#include "kinova_lowlevel/joint_torque_mode.h"
```

and both construction sites (in `RtSafety.NoMajorFaultsSteadyState` around line 32 and `RtSafety.NanosleepPacingProducesSamples` around line 242):

```cpp
  JointTorqueMode mode(dyn);      // defaults: tau_ff never set == gravity-comp hold
```

Find them with: `grep -n "GravityCompTorqueMode" tests/rt_safety_test.cpp`

- [ ] **Step 2: Run the RT-safety tests before deleting anything**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='RtSafety*'`
Expected: PASS — all entries, zero major page faults, zero dropped samples. This proves the substitution is behaviourally equivalent *before* the old mode is removed, so a failure here is unambiguous.

- [ ] **Step 3: Retarget the benchmark first, so the build never breaks**

`apps/benchmark_grav_comp.cpp` includes the header about to be deleted. Retarget it *before* deleting, or this task cannot verify itself. In that file, replace the include `kinova_lowlevel/gravity_comp_mode.h` with `kinova_lowlevel/joint_torque_mode.h`, and the construction (around line 191):

```cpp
  JointTorqueMode mode(dyn, {scale, damping, JointVec::Constant(torque_limit), 0.0, 0.0});
```

`cmd_timeout_s = 0.0` disables the watchdog, which is correct here: the benchmark never calls `set_torque`, so it runs as gravity-comp hold and there is no command to go stale.

- [ ] **Step 4: Delete the old mode**

```bash
git rm include/kinova_lowlevel/gravity_comp_mode.h src/gravity_comp_mode.cpp tests/gravity_comp_mode_test.cpp
```

In `CMakeLists.txt`, remove `src/gravity_comp_mode.cpp` from `KINOVA_LIB_SOURCES` and `tests/gravity_comp_mode_test.cpp` from the `unit_tests` list.

- [ ] **Step 5: Verify nothing else references it**

Run: `grep -rn "GravityCompTorqueMode\|gravity_comp_mode" --include='*.cpp' --include='*.h' --include='*.txt' .`
Expected: **no hits at all.** Any hit must be fixed now.

- [ ] **Step 6: Run the whole suite**

Run: `cmake -S . -B build -DCMAKE_PREFIX_PATH=/usr/local/lib/python3.10/dist-packages/cmeel.prefix && cmake --build build -j && ctest --test-dir build --output-on-failure`
Expected: PASS, with the `GravityCompTorqueMode` cases gone and everything else unchanged.

- [ ] **Step 7: Commit**

```bash
git add -A
git commit -m "refactor(modes): retire GravityCompTorqueMode — it is JointTorqueMode at tau_ff=0"
```

---

### Task 4: Fix issue #18 — the documented benchmark invocation throws

The binary stays: it backs the on-robot procedure in `docs/integration/grav_comp_static_check.md` and becomes the joint-torque-path benchmark alongside `benchmark_cartesian_impedance`. Task 3 already retargeted it to `JointTorqueMode`; this task makes the **documented command actually run**.

**Issue #18 root cause:** `Dynamics`'s default EE frame is `gen3_end_effector_link`, but `models/gen3_7dof.urdf` (the bare arm, and the benchmark's default URDF) contains `end_effector_link`. So the documented invocation throws `Dynamics: EE frame 'gen3_end_effector_link' not in URDF`.

**Files:**
- Modify: `apps/benchmark_grav_comp.cpp`

**Interfaces:**
- Consumes: `JointTorqueMode` with the `JointVec` limit from Task 2.

- [ ] **Step 1: Reproduce issue #18**

Run: `./build/benchmark_grav_comp --sim --urdf models/gen3_7dof.urdf --rate 1000 --duration 5`
Expected: throws `Dynamics: EE frame 'gen3_end_effector_link' not in URDF`. Confirm the failure before fixing it.

- [ ] **Step 2: Add `--ee-frame`**

Next to the existing `--urdf` parsing (around line 74):

```cpp
    else if (a == "--ee-frame") ee_frame = next("--ee-frame");
```

with the default declared beside `urdf` (around line 49):

```cpp
  // Matches the DEFAULT --urdf below. models/gen3_7dof.urdf (bare arm) has
  // "end_effector_link"; Dynamics defaults to "gen3_end_effector_link", which
  // only exists in the gripper model. See issue #18.
  std::string ee_frame = "end_effector_link";
```

and pass it through (around line 102):

```cpp
  Dynamics dyn(urdf, ee_frame);
```

- [ ] **Step 3: Verify both URDFs work**

```bash
cmake --build build -j
./build/benchmark_grav_comp --sim --urdf models/gen3_7dof.urdf --rate 1000 --duration 5
./build/benchmark_grav_comp --sim --urdf models/gen3_7dof_2f85.urdf --ee-frame gen3_end_effector_link --rate 1000 --duration 5
```
Expected: both complete and print percentiles. If the second frame name is absent from the gripper model, find the right one with `grep -o 'link name="[^"]*"' models/gen3_7dof_2f85.urdf` and document that instead.

- [ ] **Step 4: Record the numbers**

Save the p50 / p99 / p99.9 / max, overruns, faults and dropped counts from the bare-arm run. This is the joint-torque path's baseline, and Plan 2 will compare against it.

- [ ] **Step 5: Commit**

```bash
git add apps/benchmark_grav_comp.cpp
git commit -m "fix(bench): --ee-frame so the documented invocation runs (#18)"
```

---

### Task 5: Documentation sweep

**Files:**
- Modify: `README.md`, `CLAUDE.md`, `docs/reference/api.md`, `docs/guide/control-modes.md`, `docs/rt-tuning.md`, `docs/getting-started.md`, `docs/integration-runbook.md`, `docs/integration/grav_comp_static_check.md`

- [ ] **Step 1: Find every live reference**

Run: `grep -rln "GravityCompTorqueMode\|gravity comp mode" --include='*.md' . | grep -v superpowers`
Expected: the files above. **Do not edit anything under `docs/superpowers/specs|plans/` dated 2026-06-*** — those are historical records of what was true when written.

- [ ] **Step 2: Reframe the mode in the API reference**

In `docs/reference/api.md`, replace the `### GravityCompTorqueMode — gravity_comp_mode.h` section with a `JointTorqueMode — joint_torque_mode.h` section documenting `JointTorqueParams` (including the per-joint `torque_limit` and the watchdog fields), `set_torque`, and the law `tau = scale*gravity(q) - damping*qd + tau_ff`, clamped. State explicitly that with `tau_ff` never set the mode is gravity-compensation hold.

- [ ] **Step 3: Update the architecture and mode listings**

In `README.md` (the `ControlMode` row of the architecture table, and the line reading `GravityCompTorqueMode.compute` = gravity + position passthrough) and `CLAUDE.md` (the `ControlMode` bullet listing the concretes), replace `GravityCompTorqueMode` with `JointTorqueMode` and note gravity comp is the zero-feedforward case rather than a mode of its own.

- [ ] **Step 4: Fix the documented benchmark invocation**

In `CLAUDE.md`, `docs/getting-started.md`, `docs/rt-tuning.md` and `docs/integration-runbook.md`, the `benchmark_grav_comp` command lines still work unchanged after Task 4's default — verify each by running it. Where a command passes the gripper URDF, add `--ee-frame gen3_end_effector_link`.

- [ ] **Step 5: Update the static-check procedure**

In `docs/integration/grav_comp_static_check.md`, keep the procedure and reframe what it exercises: `JointTorqueMode` with no feedforward, i.e. gravity-compensation hold. The commands themselves are unchanged.

- [ ] **Step 6: Reconcile the ported design record**

Task 1 copied `docs/superpowers/specs/2026-06-17-joint-torque-mode-design.md` verbatim as a historical record, and Tasks 2-4 have since diverged from it in two ways. Append a short dated note at the end of that file — do not rewrite the body, which is June's design as it stood:

```markdown
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
```

This is the one exception to "do not edit June-dated superpowers docs" in Step 1: that rule protects records of what was true when written, and `CLAUDE.md` separately requires specs be kept in step with the code rather than left to drift. Appending a dated reconciliation note satisfies both; rewriting the body would not.

- [ ] **Step 7: Verify the docs build**

Run: `mkdocs build` (or `uv run --with mkdocs --with mkdocs-material mkdocs build -d /tmp/site`)
Expected: no new warnings. Pre-existing warnings about `../README.md` and missing anchors are unrelated.

- [ ] **Step 8: Commit**

```bash
git add -A
git commit -m "docs(modes): reframe gravity comp as JointTorqueMode at zero feedforward"
```

---

### Task 6: Close out

- [ ] **Step 1: Full verification on the Jetson**

Run: `cmake -S . -B build -DCMAKE_PREFIX_PATH=/usr/local/lib/python3.10/dist-packages/cmeel.prefix && cmake --build build -j && ctest --test-dir build --output-on-failure`
Expected: PASS. Record the test count — it should be the pre-port count, minus the deleted `gravity_comp_mode_test` cases, plus the `JointTorque*` cases.

- [ ] **Step 2: Confirm RT safety explicitly**

Run: `./build/unit_tests --gtest_filter='RtSafety*'`
Expected: every entry PASS — zero major page faults, zero dropped samples. Read the output; do not infer it from the suite passing.

- [ ] **Step 3: Open the PR and close the stale one**

Open a PR from this branch. In the body, state that it supersedes **PR #3** (ported rather than rebased, because that branch is 97 commits behind), note the per-joint `torque_limit` fix, the `GravityCompTorqueMode` removal, and that it closes **#18**. Then close PR #3 with a comment pointing at the new one.

## Not in this plan

- **Attended hardware.** `JointTorqueMode` has not run on the arm since June. Running it there — and the `grav_comp_static_check` procedure — gates Plan 2, not this merge.
- **Streaming.** No `StreamSink`, no session, no setpoint path. `set_torque` is wired to a streaming path in Plan 3.
- **`JointVelocityMode`** and the position-mode pose path — Plan 2.
