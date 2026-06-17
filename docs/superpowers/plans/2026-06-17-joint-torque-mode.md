# JointTorqueMode Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `JointTorqueMode` — an RT control mode that commands joint torques as a feedforward term on top of gravity compensation — and consolidate the existing `GravityCompTorqueMode` into it (gravity comp = zero feedforward).

**Architecture:** `JointTorqueMode` implements the `ControlMode` interface. Each cycle it outputs `tau = scale*gravity(q) - damping*qd + tau_ff_applied`, clamped per-joint. `tau_ff` is published by one non-RT thread via `set_torque()` (two-buffer + atomic-index, same pattern as `CartesianImpedanceMode::set_target`) and read once per cycle by `compute()`. A staleness watchdog (accumulating the `dt_s` the executor passes in — no clock calls) decays `tau_ff` to zero after `cmd_timeout_s`, reverting to gravity-comp hold. Built incrementally with TDD; the old class is removed only after the new one is proven and all call sites are rewired.

**Tech Stack:** C++17, Eigen (fixed-size `JointVec = Matrix<double,7,1>`), Pinocchio (via `Dynamics`), GoogleTest, CMake.

## Global Constraints

- **Units:** SI throughout — rad, rad/s, N·m. `JointVec` is `Eigen::Matrix<double,7,1>`, `kNumJoints == 7`.
- **RT-safety:** `compute()` must not allocate and must not call any clock/`steady_clock`; staleness is tracked by summing the `dt_s` argument. All buffers/scratch are members allocated in the constructor. (Enforced by `rt_safety_test`'s zero-major-fault assertion.)
- **Concurrency:** `set_torque()` has exactly ONE non-RT writer; `compute()` is the single RT reader. Publication uses a two-element buffer + `std::atomic<int>` active index + `std::atomic<uint64_t>` write counter (release on write, acquire on read).
- **Torque clamp** is applied to the **TOTAL** output, per joint, at `±torque_limit` (default `39.0` N·m).
- **Behavior preservation:** with `tau_ff` never set, `JointTorqueMode` output must equal the current `GravityCompTorqueMode` output exactly.
- **Build/test:** `cmake --build build -j` then `ctest --test-dir build --output-on-failure`. Single test group: `./build/unit_tests --gtest_filter='JointTorque.*'`. (Assumes `build/` already configured per the `CMakeLists.txt` header comment; if not, configure with the `cmake -S . -B build -DCMAKE_PREFIX_PATH=...` line from that comment first.)

---

### Task 1: `JointTorqueMode` core — gravity-comp-equivalent compute (no feedforward applied yet)

**Files:**
- Create: `include/kinova_lowlevel/joint_torque_mode.h`
- Create: `src/joint_torque_mode.cpp`
- Create: `tests/joint_torque_mode_test.cpp`
- Modify: `CMakeLists.txt` (add the new source + test; leave `gravity_comp_mode.*` in place for now)

**Interfaces:**
- Consumes: `ControlMode` (`include/kinova_lowlevel/control_mode.h`), `Dynamics::gravity(const JointVec&, JointVec&)` (`include/kinova_lowlevel/dynamics.h`), `JointFeedback`/`JointCommand`/`JointVec`/`ActuatorMode`/`ActuatorModes`/`kNumJoints` (`joint_types.h`, `transport.h`, pulled in via `control_mode.h`).
- Produces (relied on by Tasks 2–5):
  - `struct kinova::JointTorqueParams { double scale=1.0, damping=0.0, torque_limit=39.0, cmd_timeout_s=0.1, cmd_decay_s=0.05; };`
  - `class kinova::JointTorqueMode : public ControlMode` with ctor `JointTorqueMode(Dynamics& dyn, JointTorqueParams p = {})`, overrides `required_modes`/`on_enter`/`compute`/`on_exit`, and `void set_torque(const JointVec& tau_ff) noexcept;`

- [ ] **Step 1: Write the failing tests (grav-comp equivalence)**

Create `tests/joint_torque_mode_test.cpp`:

```cpp
#include <gtest/gtest.h>
#include <cmath>
#include "kinova_lowlevel/joint_torque_mode.h"
using namespace kinova;

// With no feedforward ever set, JointTorqueMode must reproduce gravity-comp:
// tau = gravity(q), clamped, with position passthrough and all-torque modes.
TEST(JointTorque, ZeroFeedforwardEqualsGravityCompClampedPassthrough) {
  Dynamics dyn(URDF_PATH);
  JointTorqueMode m(dyn, {1.0, 0.0, 39.0});
  JointFeedback fb; fb.q.setZero(); fb.q[1] = M_PI / 2; fb.qd.setZero();
  for (auto x : m.required_modes()) EXPECT_EQ(x, ActuatorMode::kTorque);
  JointCommand c; m.on_enter(fb); m.compute(fb, 0.001, c);
  JointVec g; dyn.gravity(fb.q, g);
  for (int i = 0; i < kNumJoints; ++i) {
    EXPECT_LE(std::abs(c.torque[i]), 39.0 + 1e-9);
    if (std::abs(g[i]) < 39.0) EXPECT_NEAR(c.torque[i], g[i], 1e-6);
  }
  EXPECT_NEAR(c.position[1], fb.q[1], 1e-9);
  EXPECT_EQ(c.mode, ActuatorMode::kTorque);
}

TEST(JointTorque, DampingSubtractsVelocityTerm) {
  Dynamics dyn(URDF_PATH);
  JointTorqueMode m(dyn, {1.0, 2.0, 1e9});
  JointFeedback fb; fb.q.setZero(); fb.qd.setConstant(1.0);
  JointCommand c; m.on_enter(fb); m.compute(fb, 0.001, c);
  JointVec g; dyn.gravity(fb.q, g);
  for (int i = 0; i < kNumJoints; ++i)
    EXPECT_NEAR(c.torque[i], g[i] - 2.0 * 1.0, 1e-6);
}
```

- [ ] **Step 2: Add the new source + test to CMake, leaving the old mode in place**

In `CMakeLists.txt`, in `KINOVA_LIB_SOURCES` (the `add_library`/source list ending at `src/rt_executor.cpp)`), add the new source right after `src/gravity_comp_mode.cpp`:

```cmake
    src/gravity_comp_mode.cpp
    src/joint_torque_mode.cpp
    src/rt_executor.cpp)
```

In the `add_executable(unit_tests ...)` list, add the new test right after `tests/gravity_comp_mode_test.cpp`:

```cmake
    tests/gravity_comp_mode_test.cpp
    tests/joint_torque_mode_test.cpp
    tests/rt_safety_test.cpp)
```

- [ ] **Step 3: Run the tests to verify they fail**

Run: `cmake --build build -j`
Expected: FAIL to compile — `joint_torque_mode.h` / `JointTorqueMode` not found.

- [ ] **Step 4: Create the header**

Create `include/kinova_lowlevel/joint_torque_mode.h`:

```cpp
#pragma once
#include <atomic>
#include <cstdint>
#include "kinova_lowlevel/control_mode.h"
#include "kinova_lowlevel/dynamics.h"
namespace kinova {

struct JointTorqueParams {
  double scale         = 1.0;   // gravity scale
  double damping       = 0.0;   // joint velocity damping (N·m·s/rad)
  double torque_limit  = 39.0;  // per-joint clamp on TOTAL output (N·m)
  double cmd_timeout_s = 0.1;   // staleness watchdog window; <=0 disables
  double cmd_decay_s   = 0.05;  // ramp tau_ff -> 0 over this window on staleness
                                // (<=0 => hard zero)
};

// Joint-torque control: tau = scale*gravity(q) - damping*qd + tau_ff, clamped.
// tau_ff is published by ONE non-RT thread via set_torque() (two-buffer +
// atomic index) and read once per cycle by compute() (RT). A staleness watchdog
// decays tau_ff to zero if no fresh command arrives within cmd_timeout_s,
// reverting to gravity-compensation hold. With tau_ff never set, this mode is
// identical to gravity compensation.
class JointTorqueMode : public ControlMode {
 public:
  JointTorqueMode(Dynamics& dyn, JointTorqueParams p = {});
  ActuatorModes required_modes() const override;
  void on_enter(const JointFeedback& fb) override;
  void compute(const JointFeedback& fb, double dt_s, JointCommand& out) override;
  void on_exit() override {}

  // Non-RT: call from a single supervisor thread.
  void set_torque(const JointVec& tau_ff) noexcept;

 private:
  Dynamics& dyn_;
  JointTorqueParams p_;

  // tau_ff publication: single writer (set_torque) -> single reader (compute).
  JointVec tau_ff_buf_[2];
  std::atomic<int> tau_ff_active_{0};
  std::atomic<uint64_t> write_count_{0};

  // RT-thread-only watchdog state.
  uint64_t last_seen_write_ = 0;
  JointVec tau_ff_target_  = JointVec::Zero();   // latest adopted command
  JointVec tau_ff_applied_ = JointVec::Zero();   // post-decay value summed in
  double   stale_s_ = 0.0;

  // Preallocated RT scratch.
  JointVec g_;
  JointVec tau_;
};
}  // namespace kinova
```

- [ ] **Step 5: Create the source (full compute, incl. the feedforward + watchdog path)**

Create `src/joint_torque_mode.cpp`. The full `compute()` is written now; Tasks 2–3 add the tests that exercise `set_torque` and the watchdog against this implementation.

```cpp
#include "kinova_lowlevel/joint_torque_mode.h"

#include <algorithm>
#include <cmath>

namespace kinova {

JointTorqueMode::JointTorqueMode(Dynamics& dyn, JointTorqueParams p)
    : dyn_(dyn), p_(p) {
  tau_ff_buf_[0].setZero();
  tau_ff_buf_[1].setZero();
  g_.setZero();
  tau_.setZero();
}

ActuatorModes JointTorqueMode::required_modes() const {
  ActuatorModes modes;
  modes.fill(ActuatorMode::kTorque);
  return modes;
}

void JointTorqueMode::set_torque(const JointVec& tau_ff) noexcept {
  // Single non-RT writer: fill the inactive buffer, publish the index, then bump
  // the write counter. compute() detects freshness via the counter and reads the
  // active buffer; the release/acquire on the index makes the buffer write
  // visible to the RT reader.
  const int inactive = 1 - tau_ff_active_.load(std::memory_order_relaxed);
  tau_ff_buf_[inactive] = tau_ff;
  tau_ff_active_.store(inactive, std::memory_order_release);
  write_count_.fetch_add(1, std::memory_order_release);
}

void JointTorqueMode::on_enter(const JointFeedback&) {
  // Enter as gravity-comp hold: discard any prior command and reset the
  // watchdog. Snapping last_seen_write_ to the current count means a command
  // sent before entry is NOT treated as fresh.
  tau_ff_target_.setZero();
  tau_ff_applied_.setZero();
  stale_s_ = 0.0;
  last_seen_write_ = write_count_.load(std::memory_order_acquire);
}

void JointTorqueMode::compute(const JointFeedback& fb, double dt_s,
                              JointCommand& out) {
  // --- read the published feedforward and advance the staleness watchdog ----
  const uint64_t wc = write_count_.load(std::memory_order_acquire);
  if (wc != last_seen_write_) {
    const int active = tau_ff_active_.load(std::memory_order_acquire);
    tau_ff_target_ = tau_ff_buf_[active];  // fresh command this cycle
    last_seen_write_ = wc;
    stale_s_ = 0.0;
  } else {
    stale_s_ += dt_s;
  }

  // Resolve the applied feedforward: hold target until stale, then ramp to 0.
  if (p_.cmd_timeout_s > 0.0 && stale_s_ >= p_.cmd_timeout_s) {
    if (p_.cmd_decay_s > 0.0) {
      // Per-cycle decrement proportional to the held target magnitude; reaches
      // exactly zero after cmd_decay_s of staleness. Sign preserved.
      const double frac = dt_s / p_.cmd_decay_s;
      for (int i = 0; i < kNumJoints; ++i) {
        const double dec = frac * std::abs(tau_ff_target_[i]);
        double a = tau_ff_applied_[i];
        if (a > dec) a -= dec;
        else if (a < -dec) a += dec;
        else a = 0.0;
        tau_ff_applied_[i] = a;
      }
    } else {
      tau_ff_applied_.setZero();
    }
  } else {
    tau_ff_applied_ = tau_ff_target_;
  }

  // --- compose and clamp ----------------------------------------------------
  dyn_.gravity(fb.q, g_);  // RT-safe: no alloc
  tau_ = p_.scale * g_ - p_.damping * fb.qd + tau_ff_applied_;
  for (int i = 0; i < kNumJoints; ++i) {
    tau_[i] = std::clamp(tau_[i], -p_.torque_limit, p_.torque_limit);
  }
  out.mode = ActuatorMode::kTorque;
  out.torque = tau_;
  out.position = fb.q;
}

}  // namespace kinova
```

- [ ] **Step 6: Run the tests to verify they pass**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='JointTorque.*'`
Expected: PASS — `ZeroFeedforwardEqualsGravityCompClampedPassthrough` and `DampingSubtractsVelocityTerm` green. (The full suite still builds because `gravity_comp_mode.*` remain.)

- [ ] **Step 7: Commit**

```bash
git add include/kinova_lowlevel/joint_torque_mode.h src/joint_torque_mode.cpp \
        tests/joint_torque_mode_test.cpp CMakeLists.txt
git commit -m "feat: JointTorqueMode core (gravity-comp-equivalent at tau_ff=0)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 2: Feedforward command path (`set_torque` adds in; total clamp)

**Files:**
- Modify: `tests/joint_torque_mode_test.cpp` (add tests)

**Interfaces:**
- Consumes: `JointTorqueMode::set_torque(const JointVec&)`, `JointTorqueParams` (from Task 1). No production code changes — Task 1's `compute()` already sums and clamps the feedforward; this task proves it.

- [ ] **Step 1: Write the failing tests**

Append to `tests/joint_torque_mode_test.cpp`:

```cpp
TEST(JointTorque, FeedforwardAddsToGravity) {
  Dynamics dyn(URDF_PATH);
  JointTorqueMode m(dyn, {1.0, 0.0, 1e9});  // huge limit: no clamp interference
  JointFeedback fb; fb.q.setZero(); fb.q[1] = M_PI / 2; fb.qd.setZero();
  JointVec ff; ff.setConstant(3.0);
  JointCommand c; m.on_enter(fb); m.set_torque(ff); m.compute(fb, 0.001, c);
  JointVec g; dyn.gravity(fb.q, g);
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(c.torque[i], g[i] + 3.0, 1e-6);
}

TEST(JointTorque, TotalOutputClampedWithFeedforward) {
  Dynamics dyn(URDF_PATH);
  JointTorqueMode m(dyn, {1.0, 0.0, 39.0});
  JointFeedback fb; fb.q.setZero(); fb.qd.setZero();
  JointVec ff; ff.setConstant(1000.0);  // gravity + 1000 >> 39 on every joint
  JointCommand c; m.on_enter(fb); m.set_torque(ff); m.compute(fb, 0.001, c);
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(c.torque[i], 39.0, 1e-9);
}
```

- [ ] **Step 2: Run the tests to verify they pass**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='JointTorque.*'`
Expected: PASS — the two new tests plus the Task 1 tests are green (Task 1's `compute()` already implements the feedforward sum and the total clamp).

> Note: these tests pass against existing code because Task 1 implemented the full `compute()`. That is intentional — they lock the feedforward contract before the watchdog logic is exercised in Task 3.

- [ ] **Step 3: Commit**

```bash
git add tests/joint_torque_mode_test.cpp
git commit -m "test: JointTorqueMode feedforward adds to gravity and total clamp

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 3: Staleness watchdog behavior

**Files:**
- Modify: `tests/joint_torque_mode_test.cpp` (add tests)

**Interfaces:**
- Consumes: `JointTorqueMode::set_torque`, `JointTorqueParams.cmd_timeout_s`, `JointTorqueParams.cmd_decay_s`, `on_enter` (from Task 1). Uses `cmd_decay_s = 0.0` for crisp hard-zero assertions.

- [ ] **Step 1: Write the failing tests**

Append to `tests/joint_torque_mode_test.cpp`:

```cpp
// After cmd_timeout_s of cycles with no fresh command, the feedforward is
// dropped (hard zero with cmd_decay_s=0) and output reverts to gravity comp.
TEST(JointTorque, WatchdogZerosStaleFeedforward) {
  Dynamics dyn(URDF_PATH);
  JointTorqueMode m(dyn, {1.0, 0.0, 1e9, 0.05, 0.0});  // timeout 50ms, hard zero
  JointFeedback fb; fb.q.setZero(); fb.q[1] = M_PI / 2; fb.qd.setZero();
  JointVec g; dyn.gravity(fb.q, g);
  JointVec ff; ff.setConstant(5.0);
  JointCommand c; m.on_enter(fb); m.set_torque(ff);
  m.compute(fb, 0.001, c);                       // first cycle adopts the command
  EXPECT_NEAR(c.torque[0], g[0] + 5.0, 1e-6);
  for (int k = 0; k < 100; ++k) m.compute(fb, 0.001, c);  // 0.1s > 0.05s timeout
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(c.torque[i], g[i], 1e-6);
}

// Re-issuing the command every cycle keeps it applied; stopping lets it lapse.
TEST(JointTorque, FreshCommandResetsWatchdog) {
  Dynamics dyn(URDF_PATH);
  JointTorqueMode m(dyn, {1.0, 0.0, 1e9, 0.05, 0.0});
  JointFeedback fb; fb.q.setZero(); fb.qd.setZero();
  JointVec g; dyn.gravity(fb.q, g);
  JointVec ff; ff.setConstant(5.0);
  JointCommand c; m.on_enter(fb);
  for (int k = 0; k < 100; ++k) { m.set_torque(ff); m.compute(fb, 0.001, c); }
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(c.torque[i], g[i] + 5.0, 1e-6);
  for (int k = 0; k < 100; ++k) m.compute(fb, 0.001, c);  // stop issuing -> lapse
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(c.torque[i], g[i], 1e-6);
}

// A command issued BEFORE on_enter must be discarded on entry.
TEST(JointTorque, OnEnterDiscardsPriorCommand) {
  Dynamics dyn(URDF_PATH);
  JointTorqueMode m(dyn, {1.0, 0.0, 1e9, 0.05, 0.0});
  JointFeedback fb; fb.q.setZero(); fb.qd.setZero();
  JointVec g; dyn.gravity(fb.q, g);
  JointVec ff; ff.setConstant(7.0);
  JointCommand c;
  m.set_torque(ff);   // issued before entry
  m.on_enter(fb);     // entering discards it
  m.compute(fb, 0.001, c);
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(c.torque[i], g[i], 1e-6);
}
```

- [ ] **Step 2: Run the tests to verify they pass**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='JointTorque.*'`
Expected: PASS — all `JointTorque.*` tests green (the watchdog + `on_enter` reset implemented in Task 1 satisfy these).

> If `WatchdogZerosStaleFeedforward` or `OnEnterDiscardsPriorCommand` fail, the bug is in Task 1's `compute()`/`on_enter` watchdog logic — fix it there, then re-run.

- [ ] **Step 3: Commit**

```bash
git add tests/joint_torque_mode_test.cpp
git commit -m "test: JointTorqueMode staleness watchdog + on_enter reset

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 4: Consolidate — rewire call sites to `JointTorqueMode`, remove `GravityCompTorqueMode`

**Files:**
- Modify: `apps/benchmark_grav_comp.cpp:29,191`
- Modify: `tests/rt_safety_test.cpp:7,24,82`
- Modify: `CMakeLists.txt` (drop `src/gravity_comp_mode.cpp` and `tests/gravity_comp_mode_test.cpp`)
- Delete: `include/kinova_lowlevel/gravity_comp_mode.h`, `src/gravity_comp_mode.cpp`, `tests/gravity_comp_mode_test.cpp`

**Interfaces:**
- Consumes: `JointTorqueMode`, `JointTorqueParams` (from Task 1). The benchmark app constructs `JointTorqueMode{scale, damping, torque_limit}` (leaving `cmd_timeout_s`/`cmd_decay_s` at defaults) and never calls `set_torque`, so the watchdog holds `tau_ff = 0` — behavior identical to the old grav-comp mode, including the read-only `--dry-run` gravity validation (which doesn't command torque at all).

- [ ] **Step 1: Rewire the benchmark app**

In `apps/benchmark_grav_comp.cpp`, change the include (line 29):

```cpp
#include "kinova_lowlevel/joint_torque_mode.h"
```

and the mode construction (line 191):

```cpp
  JointTorqueMode mode(dyn, {scale, damping, torque_limit});
```

(The app's comments still describe gravity-comp behavior, which remains accurate — `tau_ff` is never set.)

- [ ] **Step 2: Rewire the RT-safety test**

In `tests/rt_safety_test.cpp`, change the include (line 7):

```cpp
#include "kinova_lowlevel/joint_torque_mode.h"
```

and both mode constructions (lines 24 and 82) from `GravityCompTorqueMode mode(dyn);` to:

```cpp
  JointTorqueMode mode(dyn);
```

- [ ] **Step 3: Drop the old mode from CMake**

In `CMakeLists.txt`, remove the `src/gravity_comp_mode.cpp` line from `KINOVA_LIB_SOURCES` (leaving `src/joint_torque_mode.cpp`), and remove the `tests/gravity_comp_mode_test.cpp` line from the `unit_tests` target (leaving `tests/joint_torque_mode_test.cpp`). Resulting fragments:

```cmake
    src/sim_transport.cpp
    src/joint_torque_mode.cpp
    src/rt_executor.cpp)
```

```cmake
    tests/sim_transport_test.cpp
    tests/joint_torque_mode_test.cpp
    tests/rt_safety_test.cpp)
```

- [ ] **Step 4: Delete the old mode files**

```bash
git rm include/kinova_lowlevel/gravity_comp_mode.h \
       src/gravity_comp_mode.cpp \
       tests/gravity_comp_mode_test.cpp
```

- [ ] **Step 5: Build and run the full suite**

Run: `cmake --build build -j && ctest --test-dir build --output-on-failure`
Expected: PASS — no references to `GravityCompTorqueMode` remain; `unit_tests` (incl. `JointTorque.*` and `RtSafety.*`) and the benchmark target all build and pass. Confirm nothing dangling:

Run: `grep -rn "GravityComp\|gravity_comp_mode" src include apps tests CMakeLists.txt`
Expected: no matches.

- [ ] **Step 6: Commit**

```bash
git add -A
git commit -m "refactor: replace GravityCompTorqueMode with JointTorqueMode at all call sites

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 5: Documentation reframing

**Files:**
- Modify: `README.md:12,44,244`
- Modify: `docs/integration-runbook.md:174`

**Interfaces:** none (prose only). Conceptual "gravity-comp" mentions in `docs/integration/grav_comp_static_check.md` and `docs/rt-tuning.md` describe the *validation workflow* (still real) and don't name the class — leave them unchanged.

- [ ] **Step 1: Update README class references**

In `README.md`, line 44, change the `ControlMode` row's concrete-class mention:

```
| `ControlMode` (interface) | The compute boundary — `required_modes()`, `on_enter`, RT-safe `compute(fb, dt, out)`, `on_exit`. Concrete: `JointTorqueMode` (gravity comp = zero feedforward). |
```

Line 244, change the coverage bullet:

```
  `JointTorqueMode.compute` = gravity + feedforward + position passthrough.
```

Line 12, broaden the "supported control" sentence:

```
Supported control today: **torque with gravity compensation**, plus **feedforward joint-torque commands** (`JointTorqueMode`; gravity comp is the zero-feedforward case). Designed so
```

(Keep the remainder of the sentence on line 13 intact.)

- [ ] **Step 2: Update the integration runbook claim**

In `docs/integration-runbook.md`, line 174, change:

```
- **Velocity / current modes ported by analogy.** Only torque (gravity-comp + feedforward joint torque via `JointTorqueMode`) is
```

(Keep the continuation of the sentence intact.)

- [ ] **Step 3: Verify no stale class references remain in active docs**

Run: `grep -rn "GravityCompTorqueMode" README.md docs/integration-runbook.md docs/rt-tuning.md docs/integration/grav_comp_static_check.md`
Expected: no matches. (Historical files under `docs/superpowers/specs` and `docs/superpowers/plans` are intentionally left as written.)

- [ ] **Step 4: Commit**

```bash
git add README.md docs/integration-runbook.md
git commit -m "docs: reframe gravity-comp mode as JointTorqueMode (zero feedforward)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Self-Review

**Spec coverage:**
- `JointTorqueMode` class + params + setter + watchdog + compute semantics → Task 1 (impl), Tasks 2–3 (proven). ✓
- `tau_ff = 0` reproduces grav comp exactly → Task 1 equivalence tests. ✓
- Total-output clamp → Task 2. ✓
- Staleness watchdog decays to zero / resets / `on_enter` reset → Task 3. ✓
- Remove `GravityCompTorqueMode`, rewire app + rt_safety + CMake, keep `--dry-run` validation → Task 4. ✓
- Docs reframing → Task 5. ✓
- Out-of-scope items (EE wrench, drift root-cause, benchmark app) → not planned, per spec. ✓

**Placeholder scan:** none — every code/command step shows full content.

**Type consistency:** `JointTorqueParams` field order (`scale, damping, torque_limit, cmd_timeout_s, cmd_decay_s`) is consistent across the header (Task 1), the `{...}` brace-inits in tests (Tasks 1–3) and the app (Task 4). `set_torque(const JointVec&)`, `tau_ff_target_`/`tau_ff_applied_`/`stale_s_`/`write_count_`/`tau_ff_active_`/`tau_ff_buf_` names match between header and source. The benchmark/rt_safety rewires use the exact ctor signatures from Task 1.
