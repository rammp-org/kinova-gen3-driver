# Arm Interface — Trajectory Execution Core Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the non-RT trajectory execution core — the logic that samples a joint trajectory over time, drives a joint-target sink, and enforces completion / divergence / preemption semantics — as a standalone, fully unit-tested unit.

**Architecture:** A single non-RT class, `TrajectoryExecutor`, that owns the active (and one queued) trajectory and, on each `tick(now, q_meas)`, interpolates the trajectory to a joint target and calls an injected `JointTargetSink`. It reports status via a plain struct. It has **no** dependency on ROS2, Pinocchio, the RT executor, the real `ControlMode`s, or the robot — those integrate in later plans. This is the first of a plan sequence (execution core → ports+supervisor → Ros2Backend → robot bring-up).

**Tech Stack:** C++17, Eigen (already a driver dependency), GoogleTest. No new third-party deps.

## Global Constraints

- **SI / radians everywhere.** Joint angles in radians, time in seconds. (Spec: driver convention.)
- **Non-RT only.** This unit is never called from the RT thread; it may allocate. It must remain free of ROS2/Pinocchio includes. (Spec: RT-safety invariants — interface work stays off the RT thread.)
- **`kNumJoints == 7`.** Fixed-size joint vectors; reuse `kinova::JointVec` from `include/kinova_lowlevel/joint_types.h` (confirm the exact type name/alias there before first use — it is the fixed 7-element joint vector the driver already uses). (Spec: joint_types is the POD value layer.)
- **Builds and tests run on abra** (aarch64 Jetson) via the standard CMake+ctest flow; muk cannot build the driver. (Spec/CLAUDE.md.)
- **Decisions encoded here:** driver-side execution (D2); joint-space, mode carried per-goal (D3/D8); time-based completion + divergence guard + per-goal queue/latest-wins + gapless queue (D4); reject mode change while in flight (D10).

---

## File Structure

- `include/kinova_lowlevel/interface/trajectory_executor.h` — public types (`JointWaypoint`, `Trajectory`, `Preemption`, `ControlModeKind`, `SubmitResult`, `ExecStatus`, `JointTargetSink`) + the `TrajectoryExecutor` class. One responsibility: execution semantics of a joint trajectory.
- `src/interface/trajectory_executor.cpp` — implementation.
- `tests/interface/trajectory_executor_test.cpp` — gtest unit tests (added to the existing single `unit_tests` binary).
- `CMakeLists.txt` — add the source to `KINOVA_LIB_SOURCES` and the test to the `unit_tests` target.

New directories `include/kinova_lowlevel/interface/`, `src/interface/`, `tests/interface/` group the interface layer separately from the RT core.

---

### Task 1: Trajectory value types + linear interpolation

**Files:**
- Create: `include/kinova_lowlevel/interface/trajectory_executor.h`
- Create: `src/interface/trajectory_executor.cpp`
- Create: `tests/interface/trajectory_executor_test.cpp`
- Modify: `CMakeLists.txt` (add source + test)

**Interfaces:**
- Produces:
  - `struct JointWaypoint { kinova::JointVec q; double t_s; };` — `t_s` is time-from-start (seconds).
  - `struct Trajectory { std::vector<JointWaypoint> points; double duration_s() const; };`
  - `kinova::JointVec sample(const Trajectory&, double t_s);` — free function; clamps `t_s` to `[0, duration]`; **linear** interpolation between bracketing waypoints (dense cuRobo output makes linear sufficient; cubic is a later refinement).

- [x] **Step 1: Write the failing test**

```cpp
// tests/interface/trajectory_executor_test.cpp
#include <gtest/gtest.h>
#include "kinova_lowlevel/interface/trajectory_executor.h"
using namespace kinova::interface;

static kinova::JointVec vec7(double v) { kinova::JointVec q; q.setConstant(v); return q; }

TEST(TrajectorySample, LinearInterpBetweenWaypoints) {
  Trajectory tr;
  tr.points = { {vec7(0.0), 0.0}, {vec7(1.0), 2.0} };   // 0 -> 1 rad over 2 s
  EXPECT_NEAR(sample(tr, 0.0)[0], 0.0, 1e-9);
  EXPECT_NEAR(sample(tr, 1.0)[0], 0.5, 1e-9);           // halfway
  EXPECT_NEAR(sample(tr, 2.0)[0], 1.0, 1e-9);
  EXPECT_NEAR(sample(tr, 5.0)[0], 1.0, 1e-9);           // clamps past end
  EXPECT_NEAR(sample(tr, -1.0)[0], 0.0, 1e-9);          // clamps before start
  EXPECT_NEAR(tr.duration_s(), 2.0, 1e-9);
}
```

- [x] **Step 2: Run test to verify it fails**

Run (on abra): `cmake --build build -j && ./build/unit_tests --gtest_filter='TrajectorySample*'`
Expected: FAIL to compile (`trajectory_executor.h` not found).

- [x] **Step 3: Write the header + minimal implementation**

```cpp
// include/kinova_lowlevel/interface/trajectory_executor.h
#pragma once
#include <vector>
#include "kinova_lowlevel/joint_types.h"   // kinova::JointVec, kNumJoints
namespace kinova::interface {

struct JointWaypoint { kinova::JointVec q; double t_s; };
struct Trajectory {
  std::vector<JointWaypoint> points;
  double duration_s() const { return points.empty() ? 0.0 : points.back().t_s; }
};

kinova::JointVec sample(const Trajectory& tr, double t_s);

}  // namespace kinova::interface
```

```cpp
// src/interface/trajectory_executor.cpp
#include "kinova_lowlevel/interface/trajectory_executor.h"
#include <algorithm>
namespace kinova::interface {

kinova::JointVec sample(const Trajectory& tr, double t_s) {
  const auto& p = tr.points;
  if (p.empty()) return kinova::JointVec::Zero();
  if (t_s <= p.front().t_s) return p.front().q;
  if (t_s >= p.back().t_s)  return p.back().q;
  // find first waypoint with t_s greater than the query
  auto hi = std::upper_bound(p.begin(), p.end(), t_s,
      [](double t, const JointWaypoint& w){ return t < w.t_s; });
  const JointWaypoint& b = *hi;
  const JointWaypoint& a = *(hi - 1);
  const double span = b.t_s - a.t_s;
  const double u = span > 0.0 ? (t_s - a.t_s) / span : 0.0;
  return a.q + u * (b.q - a.q);
}

}  // namespace kinova::interface
```

Add to `CMakeLists.txt`: append `src/interface/trajectory_executor.cpp` to `KINOVA_LIB_SOURCES`, and append `tests/interface/trajectory_executor_test.cpp` to the `unit_tests` `add_executable(...)` list.

- [x] **Step 4: Run test to verify it passes**

Run: `cmake -S . -B build -DCMAKE_PREFIX_PATH=/usr/local/lib/python3.10/dist-packages/cmeel.prefix && cmake --build build -j && ./build/unit_tests --gtest_filter='TrajectorySample*'`
Expected: PASS.

- [x] **Step 5: Commit**

```bash
git add include/kinova_lowlevel/interface/trajectory_executor.h src/interface/trajectory_executor.cpp tests/interface/trajectory_executor_test.cpp CMakeLists.txt
git commit -m "feat(interface): trajectory value types + linear sampling"
```

---

### Task 2: Executor skeleton — submit / accept-reject (incl. mode-change rejection)

**Files:**
- Modify: `include/kinova_lowlevel/interface/trajectory_executor.h`
- Modify: `src/interface/trajectory_executor.cpp`
- Modify: `tests/interface/trajectory_executor_test.cpp`

**Interfaces:**
- Consumes: `Trajectory`, `sample()` (Task 1).
- Produces:
  - `enum class Preemption { kQueue, kLatestWins };`
  - `enum class ControlModeKind { kPosition, kImpedance };`
  - `enum class SubmitResult { kAccepted, kRejectedModeChangeWhileMoving, kRejectedEmpty };`
  - `class JointTargetSink { public: virtual ~JointTargetSink() = default; virtual void set_joint_target(const kinova::JointVec&) = 0; };`
  - `class TrajectoryExecutor` with:
    - `explicit TrajectoryExecutor(JointTargetSink& sink);`
    - `SubmitResult submit(const Trajectory&, ControlModeKind, Preemption);`
    - `bool is_active() const;`  // a trajectory is executing or queued
    - `ControlModeKind active_mode() const;`

- [x] **Step 1: Write the failing test**

```cpp
// append to tests/interface/trajectory_executor_test.cpp
namespace {
struct RecordingSink : kinova::interface::JointTargetSink {
  std::vector<kinova::JointVec> calls;
  void set_joint_target(const kinova::JointVec& q) override { calls.push_back(q); }
};
kinova::interface::Trajectory ramp(double dur) {  // helper: 0->1 rad over dur
  return { { {vec7(0.0), 0.0}, {vec7(1.0), dur} } };
}
}  // namespace

TEST(ExecutorSubmit, AcceptsFirstGoalAndRejectsEmpty) {
  RecordingSink sink;
  kinova::interface::TrajectoryExecutor ex(sink);
  using kinova::interface::SubmitResult; using kinova::interface::ControlModeKind;
  using kinova::interface::Preemption;
  EXPECT_EQ(ex.submit(kinova::interface::Trajectory{}, ControlModeKind::kPosition, Preemption::kLatestWins),
            SubmitResult::kRejectedEmpty);
  EXPECT_EQ(ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins),
            SubmitResult::kAccepted);
  EXPECT_TRUE(ex.is_active());
  EXPECT_EQ(ex.active_mode(), ControlModeKind::kPosition);
}

TEST(ExecutorSubmit, RejectsModeChangeWhileInFlight) {
  RecordingSink sink;
  kinova::interface::TrajectoryExecutor ex(sink);
  using kinova::interface::SubmitResult; using kinova::interface::ControlModeKind;
  using kinova::interface::Preemption;
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins);   // now in flight, position
  EXPECT_EQ(ex.submit(ramp(2.0), ControlModeKind::kImpedance, Preemption::kLatestWins),
            SubmitResult::kRejectedModeChangeWhileMoving);
  // same-mode goal is fine
  EXPECT_EQ(ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins),
            SubmitResult::kAccepted);
}
```

- [x] **Step 2: Run test to verify it fails**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='ExecutorSubmit*'`
Expected: FAIL to compile (`TrajectoryExecutor` not defined).

- [x] **Step 3: Implement the skeleton**

```cpp
// add to trajectory_executor.h (inside namespace kinova::interface)
enum class Preemption { kQueue, kLatestWins };
enum class ControlModeKind { kPosition, kImpedance };
enum class SubmitResult { kAccepted, kRejectedModeChangeWhileMoving, kRejectedEmpty };

class JointTargetSink {
 public:
  virtual ~JointTargetSink() = default;
  virtual void set_joint_target(const kinova::JointVec&) = 0;
};

class TrajectoryExecutor {
 public:
  explicit TrajectoryExecutor(JointTargetSink& sink) : sink_(sink) {}
  SubmitResult submit(const Trajectory& tr, ControlModeKind mode, Preemption p);
  bool is_active() const { return active_.has_value() || queued_.has_value(); }
  ControlModeKind active_mode() const { return mode_; }

 private:
  struct Active { Trajectory tr; double start_time = 0.0; bool started = false; };
  JointTargetSink& sink_;
  ControlModeKind mode_ = ControlModeKind::kPosition;
  std::optional<Active> active_;
  std::optional<Trajectory> queued_;
  Preemption queued_pre_ = Preemption::kLatestWins;
};
```
Add `#include <optional>` to the header.

```cpp
// add to trajectory_executor.cpp
SubmitResult TrajectoryExecutor::submit(const Trajectory& tr, ControlModeKind mode, Preemption p) {
  if (tr.points.empty()) return SubmitResult::kRejectedEmpty;
  if (is_active() && mode != mode_) return SubmitResult::kRejectedModeChangeWhileMoving;
  if (!is_active()) {                       // idle -> adopt immediately
    mode_ = mode;
    active_ = Active{tr, 0.0, false};
    queued_.reset();
    return SubmitResult::kAccepted;
  }
  if (p == SubmitResult::kAccepted) {}       // (placeholder line removed in Task 5/6)
  // Preemption handling lands in Tasks 5-6; for now, latest-wins replaces active.
  active_ = Active{tr, 0.0, false};
  return SubmitResult::kAccepted;
}
```
> Note: the `submit` preemption branch is intentionally minimal here (accept + replace) so this task compiles and its two tests pass; Tasks 5–6 replace the tail with real queue / latest-wins logic. Delete the stray `if (p == ...)` line — it is a no-op marker, not real logic.

- [x] **Step 4: Run test to verify it passes**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='ExecutorSubmit*'`
Expected: PASS (both submit tests).

- [x] **Step 5: Commit**

```bash
git add include/kinova_lowlevel/interface/trajectory_executor.h src/interface/trajectory_executor.cpp tests/interface/trajectory_executor_test.cpp
git commit -m "feat(interface): executor submit + mode-change-while-moving rejection"
```

---

### Task 3: `tick()` — sampling, fraction_complete, time-based completion

**Files:**
- Modify: `include/kinova_lowlevel/interface/trajectory_executor.h`
- Modify: `src/interface/trajectory_executor.cpp`
- Modify: `tests/interface/trajectory_executor_test.cpp`

**Interfaces:**
- Produces:
  - `struct ExecStatus { bool active; bool completed; double fraction; int error_code; };`
    with `error_code` constants `kOk = 0`, `kPathToleranceViolated = -4` (mirrors FJT).
  - `ExecStatus TrajectoryExecutor::tick(double now_s, const kinova::JointVec& q_meas);`
    First `tick` after a `submit` latches `start_time = now_s` (relative clock). Each tick calls `sink_.set_joint_target(sample(tr, now_s - start_time))`. Completion (`completed=true`, and the goal leaves active) when `now_s - start_time >= duration`. `fraction = clamp((now-start)/duration, 0, 1)`.

- [x] **Step 1: Write the failing test**

```cpp
TEST(ExecutorTick, SamplesToSinkAndCompletesOnTime) {
  RecordingSink sink;
  kinova::interface::TrajectoryExecutor ex(sink);
  using namespace kinova::interface;
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins);

  ExecStatus s0 = ex.tick(10.0, vec7(0.0));   // start clock at t=10
  EXPECT_TRUE(s0.active); EXPECT_FALSE(s0.completed);
  EXPECT_NEAR(sink.calls.back()[0], 0.0, 1e-9);

  ExecStatus s1 = ex.tick(11.0, vec7(0.0));   // 1s in -> halfway
  EXPECT_NEAR(sink.calls.back()[0], 0.5, 1e-9);
  EXPECT_NEAR(s1.fraction, 0.5, 1e-9);
  EXPECT_FALSE(s1.completed);

  ExecStatus s2 = ex.tick(12.0, vec7(1.0));   // at/after final timestamp
  EXPECT_TRUE(s2.completed);
  EXPECT_EQ(s2.error_code, ExecStatus::kOk);
  EXPECT_FALSE(ex.is_active());               // goal left active on completion
}
```

- [x] **Step 2: Run test to verify it fails**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='ExecutorTick*'`
Expected: FAIL to compile (`tick` / `ExecStatus` not defined).

- [x] **Step 3: Implement `tick`**

```cpp
// add to trajectory_executor.h
struct ExecStatus {
  static constexpr int kOk = 0;
  static constexpr int kPathToleranceViolated = -4;
  bool active; bool completed; double fraction; int error_code;
};
// in class TrajectoryExecutor public:
ExecStatus tick(double now_s, const kinova::JointVec& q_meas);
```

```cpp
// add to trajectory_executor.cpp
#include <cmath>
ExecStatus TrajectoryExecutor::tick(double now_s, const kinova::JointVec& /*q_meas*/) {
  if (!active_) return ExecStatus{false, false, 0.0, ExecStatus::kOk};
  Active& a = *active_;
  if (!a.started) { a.start_time = now_s; a.started = true; }
  const double elapsed = now_s - a.start_time;
  const double dur = a.tr.duration_s();
  sink_.set_joint_target(sample(a.tr, elapsed));
  const double frac = dur > 0.0 ? std::min(1.0, std::max(0.0, elapsed / dur)) : 1.0;
  if (elapsed >= dur) {
    active_.reset();
    return ExecStatus{false, true, 1.0, ExecStatus::kOk};   // time-based completion
  }
  return ExecStatus{true, false, frac, ExecStatus::kOk};
}
```

- [x] **Step 4: Run test to verify it passes**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='ExecutorTick*'`
Expected: PASS.

- [x] **Step 5: Commit**

```bash
git add -A && git commit -m "feat(interface): executor tick — sampling + time-based completion"
```

---

### Task 4: Divergence guard (path tolerance → abort)

**Files:**
- Modify: `include/kinova_lowlevel/interface/trajectory_executor.h`
- Modify: `src/interface/trajectory_executor.cpp`
- Modify: `tests/interface/trajectory_executor_test.cpp`

**Interfaces:**
- Produces: `submit(...)` gains a per-joint tolerance argument
  `submit(const Trajectory&, ControlModeKind, Preemption, const kinova::JointVec& path_tol)`
  (a tolerance of `<= 0` on a joint disables the check for that joint). `tick` compares
  `|q_meas - q_desired|` against `path_tol`; on violation it aborts (leaves active,
  `completed=true`, `error_code=kPathToleranceViolated`).
- Update existing Task 2/3 tests that call `submit` to pass a disabling tolerance
  `kinova::JointVec::Constant(-1.0)` (keeps them green).

- [x] **Step 1: Write the failing test**

```cpp
TEST(ExecutorDivergence, AbortsWhenErrorExceedsPathTolerance) {
  RecordingSink sink;
  kinova::interface::TrajectoryExecutor ex(sink);
  using namespace kinova::interface;
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins, vec7(0.05));
  ex.tick(0.0, vec7(0.0));                      // start; desired 0, meas 0 -> ok
  ExecStatus s = ex.tick(1.0, vec7(0.9));       // desired 0.5, meas 0.9 -> err 0.4 > 0.05
  EXPECT_TRUE(s.completed);
  EXPECT_EQ(s.error_code, ExecStatus::kPathToleranceViolated);
  EXPECT_FALSE(ex.is_active());
}
```

- [x] **Step 2: Run test to verify it fails**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='ExecutorDivergence*'`
Expected: FAIL (compile error: `submit` arity; then assertion once arity fixed).

- [x] **Step 3: Implement the guard**

Add a `kinova::JointVec path_tol_;` member; store it in `submit`; in `tick`, after computing
`q_desired = sample(...)`, before the completion check:

```cpp
const kinova::JointVec q_desired = sample(a.tr, elapsed);
sink_.set_joint_target(q_desired);
for (int i = 0; i < kinova::kNumJoints; ++i) {
  if (path_tol_[i] > 0.0 && std::abs(q_meas[i] - q_desired[i]) > path_tol_[i]) {
    active_.reset();
    return ExecStatus{false, true, frac, ExecStatus::kPathToleranceViolated};
  }
}
```
Change `submit` signatures (header + cpp + all call sites/tests) to take `const kinova::JointVec& path_tol` and store it. Update the Task 2/3 tests to pass `vec7(-1.0)`.

- [x] **Step 4: Run test to verify it passes**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='Executor*'`
Expected: PASS (all executor tests, including updated Task 2/3).

- [x] **Step 5: Commit**

```bash
git add -A && git commit -m "feat(interface): executor divergence guard (path tolerance)"
```

---

### Task 5: Preemption — latest-wins

**Files:**
- Modify: `src/interface/trajectory_executor.cpp`, `tests/interface/trajectory_executor_test.cpp`

**Interfaces:**
- Behavior: with `kLatestWins`, a `submit` while active replaces the active trajectory and
  **resets the relative clock** (the new goal's next `tick` latches its own start). Same-mode only
  (mode change already rejected in Task 2).

- [x] **Step 1: Write the failing test**

```cpp
TEST(ExecutorPreempt, LatestWinsReplacesAndRestartsClock) {
  RecordingSink sink;
  kinova::interface::TrajectoryExecutor ex(sink);
  using namespace kinova::interface;
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins, vec7(-1.0));
  ex.tick(0.0, vec7(0.0));
  ex.tick(1.0, vec7(0.5));                       // halfway through first ramp
  // preempt with a fresh 0->1 ramp
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins, vec7(-1.0));
  ExecStatus s = ex.tick(1.0, vec7(0.5));        // first tick of NEW goal -> desired ~0 (clock reset)
  EXPECT_NEAR(sink.calls.back()[0], 0.0, 1e-9);
  EXPECT_NEAR(s.fraction, 0.0, 1e-9);
}
```

- [x] **Step 2: Run test to verify it fails**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='ExecutorPreempt*'`
Expected: FAIL (clock not reset — desired ~0.5, not 0).

- [x] **Step 3: Implement latest-wins**

Replace the minimal tail of `submit` from Task 2 with:

```cpp
  // active, same mode:
  if (p == Preemption::kLatestWins) {
    active_ = Active{tr, 0.0, false};   // replace + reset clock (started=false)
    queued_.reset();
    return SubmitResult::kAccepted;
  }
  // kQueue handled in Task 6:
  queued_ = tr; queued_pre_ = p;
  return SubmitResult::kAccepted;
```

- [x] **Step 4: Run test to verify it passes**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='Executor*'`
Expected: PASS.

- [x] **Step 5: Commit**

```bash
git add -A && git commit -m "feat(interface): executor latest-wins preemption"
```

---

### Task 6: Preemption — gapless queue

**Files:**
- Modify: `src/interface/trajectory_executor.cpp`, `tests/interface/trajectory_executor_test.cpp`

**Interfaces:**
- Behavior: with `kQueue`, a `submit` while active stores the trajectory in `queued_`. When the
  active goal reaches completion in `tick`, instead of going idle the executor **immediately
  promotes** the queued trajectory and latches its start clock to the *same* `now_s` — **zero gap**
  (the arm is not reported idle between them).

- [x] **Step 1: Write the failing test**

```cpp
TEST(ExecutorPreempt, QueuePromotesGaplesslyOnCompletion) {
  RecordingSink sink;
  kinova::interface::TrajectoryExecutor ex(sink);
  using namespace kinova::interface;
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins, vec7(-1.0)); // active
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kQueue,     vec7(-1.0)); // queued
  ex.tick(0.0, vec7(0.0));
  ExecStatus at_end = ex.tick(2.0, vec7(1.0));   // first ramp ends here
  EXPECT_TRUE(at_end.active);                    // NOT idle — queued promoted
  EXPECT_FALSE(at_end.completed);                // continuous motion, no completion gap
  ExecStatus mid = ex.tick(3.0, vec7(0.5));      // 1s into promoted ramp -> desired 0.5
  EXPECT_NEAR(sink.calls.back()[0], 0.5, 1e-9);
  EXPECT_TRUE(mid.active);
}
```

- [x] **Step 2: Run test to verify it fails**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='ExecutorPreempt*'`
Expected: FAIL (`at_end.active` is false — current code goes idle on completion).

- [x] **Step 3: Implement gapless promotion**

In `tick`, replace the completion branch (`if (elapsed >= dur) { active_.reset(); return ...; }`) with:

```cpp
  if (elapsed >= dur) {
    if (queued_) {                               // gapless promotion
      active_ = Active{*queued_, now_s, true};   // latch start to NOW (started=true), zero gap
      queued_.reset();
      return ExecStatus{true, false, 0.0, ExecStatus::kOk};
    }
    active_.reset();
    return ExecStatus{false, true, 1.0, ExecStatus::kOk};
  }
```

- [x] **Step 4: Run test to verify it passes**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='Executor*'`
Expected: PASS (all executor tests).

- [x] **Step 5: Commit**

```bash
git add -A && git commit -m "feat(interface): executor gapless queue promotion"
```

---

### Task 7: Full-suite regression + docs note

**Files:**
- Modify: `docs/superpowers/plans/2026-08-10-arm-interface-execution-core.md` (check off tasks)

- [x] **Step 1: Run the entire test binary**

Run: `ctest --test-dir build --output-on-failure`
Expected: PASS — the whole `unit_tests` suite green (executor tests + all pre-existing driver tests), proving the new unit did not disturb the RT core.

- [x] **Step 2: Confirm RT-safety unaffected**

Run: `./build/unit_tests --gtest_filter='*rt_safety*:*RtSafety*'`
Expected: PASS (zero major faults / zero drops) — the execution core is non-RT and links no RT path, so this must remain green.

- [x] **Step 3: Commit the checked-off plan**

```bash
git add docs/superpowers/plans/2026-08-10-arm-interface-execution-core.md
git commit -m "docs(interface): execution-core plan complete"
```

---

## Self-Review

**Spec coverage (this plan's slice):** driver-side execution (D2) ✓ Task 3; joint-space + mode per goal (D3/D8) ✓ Tasks 2–3; time-based completion (D4) ✓ Task 3; divergence guard (D4) ✓ Task 4; latest-wins + gapless queue (D4) ✓ Tasks 5–6; reject mode-change while moving (D10) ✓ Task 2. **Deferred to later plans (correctly not here):** ports/DI, `Ros2Backend`, `ExecuteJointTrajectory` action, telemetry/`JointState`, supervisor threading + per-thread `Dynamics`, integration with the real joint modes, arbitration hooks.

**Placeholder scan:** the only intentional stub is the Task 2 `submit` tail, explicitly flagged and fully replaced in Tasks 5–6; the stray `if (p == ...)` marker is called out for deletion. No TBDs.

**Type consistency:** `sample`, `Trajectory`, `JointWaypoint`, `Preemption`, `ControlModeKind`, `SubmitResult`, `JointTargetSink`, `ExecStatus`, and the `submit`/`tick` signatures are consistent across tasks. Note: `submit` arity changes in Task 4 (adds `path_tol`); Task 4 explicitly updates earlier call sites/tests — flagged, not silent.

**One thing the implementer must confirm first:** the exact `kinova::JointVec` type/alias and its Eigen ops (`setConstant`, `setZero`, indexing, `Constant`) in `include/kinova_lowlevel/joint_types.h`, and that `kinova::kNumJoints == 7`. If the alias differs, adjust the `vec7` helper and member types accordingly — the logic is unchanged.

## Next plans in the sequence (not this plan)

- **Plan 2 — Ports + supervisor + sim integration:** Layer A ports (Action/Stream/Service, driven+driving), the supervisor threading (single-writer sink, per-thread `Dynamics`, `FeedbackTap`/`Seqlock` telemetry pump), wiring `TrajectoryExecutor` to a real `ControlMode`'s `set_joint_target`, driven end-to-end against `SimTransport` + a fake in-process backend. **Gated on** the joint modes exposing `set_joint_target`.
- **Plan 3 — `Ros2Backend`:** the `ExecuteJointTrajectory` action (wrapping FJT messages), `sensor_msgs/JointState` stream, services; behind a `KINOVA_ENABLE_ROS2` CMake option.
- **Plan 4 — On-robot bring-up (attended):** cuRobo as the real client; runbook.

---

## Completion status

**COMPLETE — 2026-08-10.** All 7 tasks implemented TDD-first and verified on abra (aarch64);
the full `unit_tests` suite is green (interface tests + all pre-existing driver tests) and the
`RtSafety` tests still pass, confirming the non-RT execution core did not disturb the RT core.

Two deviations from the literal task snippets, both correctness fixes surfaced in task review:
- **Task 5 (tolerance isolation):** `submit` no longer sets the shared `path_tol_` unconditionally.
  A `kQueue` submit stored its tolerance in a new `queued_tol_` member instead of clobbering the
  divergence guard of the still-running active trajectory. Regression test:
  `ExecutorPreempt.QueueDoesNotClobberActivePathTolerance`.
- **Task 6 (promotion adopts tolerance):** gapless promotion applies `path_tol_ = queued_tol_`, so a
  promoted trajectory is guarded by its own tolerance. Test:
  `ExecutorPreempt.PromotedGoalAdoptsItsOwnPathTolerance`.

Final executor test suite (7): `TrajectorySample.LinearInterpBetweenWaypoints`,
`ExecutorSubmit.{AcceptsFirstGoalAndRejectsEmpty,RejectsModeChangeWhileInFlight}`,
`ExecutorTick.SamplesToSinkAndCompletesOnTime`,
`ExecutorDivergence.AbortsWhenErrorExceedsPathTolerance`,
`ExecutorPreempt.{LatestWinsReplacesAndRestartsClock,QueueDoesNotClobberActivePathTolerance,QueuePromotesGaplesslyOnCompletion,PromotedGoalAdoptsItsOwnPathTolerance}`.
