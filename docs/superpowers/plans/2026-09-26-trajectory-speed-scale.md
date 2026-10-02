# Trajectory Speed Scale — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A trajectory goal can ask to be executed slower, and an operator can slow everything down at runtime, by dilating the executor's clock rather than by rewriting waypoints or throttling the follower.

**Architecture:** `TrajectoryExecutor` stops computing elapsed time as a difference of wall stamps and starts accumulating it at `dt × s`. The per-goal scale rides with the trajectory (like `path_tolerance` already does); the runtime override is an atomic on the `Supervisor`; the effective scale is their minimum, clamped to `(0, 1]`. Because `sample()` is parameterised on trajectory time, a slower clock makes the commanded velocity `s·qd` and acceleration `s²·qdd` automatically — no derivative plumbing is needed or wanted.

**Tech Stack:** C++17, GoogleTest (one `unit_tests` binary), CMake, fixed-size POD value types, `std::atomic` for cross-thread scalars.

**Spec:** GitHub issue `rammp-org/kinova-gen3-driver#69`. Read it before starting — it carries the reasoning for the clock mechanism over waypoint rewriting, and the explicit non-goals.

## Global Constraints

- **Nothing in the RT path may allocate, lock, or block.** `tests/rt_safety_test.cpp` asserts zero major page faults and zero dropped samples in steady state and is a CI gate (`--gtest_filter='RtSafety*-RtSafety.NanosleepPacingProducesSamples'`). `TrajectoryExecutor::tick()` runs on the sampler thread today, but #58 intends to move it onto the RT thread — keep it allocation-free and branch-simple.
- **SI units, radians, everywhere internally.**
- **Fail loud at startup, never silent mis-mapping** — an out-of-range scale is refused, never clamped into something plausible.
- **`JointTargetSink::set_target(const JointVec&)` is position-only** (`include/kinova_lowlevel/joint_target_sink.h:14`). Do not add a derivative-carrying sink in this plan; the scaling shows up through the clock.
- **Documentation changes land in the same commit as the behaviour change** (`CLAUDE.md`).
- The scale may only ever **slow the arm**: effective scale is clamped to `(0, 1]` and an override above 1.0 is refused.

## Build and test — this machine CAN build this tree

`CLAUDE.md` says Jetson-only; that is true of the KORTEX build, not the sim-only tree. The x86_64 toolchain is staged. **`abra` is unreachable as of 2026-09-26, so do not try to build there.**

```sh
/home/swapnil/atdev/.driver-deps/build.sh /home/swapnil/atdev/kinova-driver-speed
/home/swapnil/atdev/.driver-deps/build.sh /home/swapnil/atdev/kinova-driver-speed 'Executor*'
```

Baseline verified before this plan: the whole suite passes in ~30 s. First argument is the worktree; optional second is a gtest filter. The RT gate:

```sh
/home/swapnil/atdev/.driver-deps/build.sh /home/swapnil/atdev/kinova-driver-speed 'RtSafety*-RtSafety.NanosleepPacingProducesSamples'
```

RT *numbers* (latency percentiles, jitter) are only meaningful on the Jetson's isolated core; x86 proves correctness and allocation-freedom, not timing.

## Review Focus

Five things the spec implies that no task's happy path exercises.

1. **A scale change mid-motion must not step the commanded velocity.** The override is settable while a goal runs; applying it instantly moves the reference discontinuously. → Task 3 owns the slew limit and its test.
2. **Gapless promotion must re-latch the scaled clock.** `tick()` latches `start_time` in two places — first tick and promotion (`trajectory_executor.cpp:113`) — and a promoted goal carries its own scale. Miss the second and the follow-on trajectory starts mid-way through itself. → Task 2, Step 9.
3. **`kLatestWins` preemption resets the clock** via `started=false`; the accumulator must reset with it or the replacement trajectory inherits the old one's elapsed time. → Task 2, Step 11.
4. **A goal whose scale is out of range must be refused, not clamped** — including NaN, which compares false against every bound and would sail through a naive `if (s > 1.0)`. → Task 4, Step 7.
5. **A cancel pushes a default-constructed `TrajectoryGoal`** (`supervisor.cpp:359`), so every new field must be harmless at its default on that path. → Task 4, Step 9.

---

### Task 1: The value field and the effective-scale rule

**Files:**
- Modify: `include/kinova_lowlevel/interface/value_types.h:98-109` (the field)
- Modify: `include/kinova_lowlevel/interface/trajectory_executor.h` (the helper and the constant)
- Test: `tests/interface/trajectory_executor_test.cpp`

**Where the helper goes, and why it is not negotiable:** `trajectory_executor.h` is the *lower* header — it defines `Preemption` and `ControlModeKind` (`:32-33`) and `value_types.h` builds on it. `TrajectoryExecutor::tick()` needs `effective_scale`, so putting it in `value_types.h` or `supervisor.h` would make the executor depend on a header that depends on it. Put it in `trajectory_executor.h`; `supervisor.h` reaches it transitively.

**Interfaces:**
- Consumes: nothing.
- Produces: `TrajectoryGoal::speed_scale` (double, default `1.0`); `kinova::interface::effective_scale(double goal, double override) -> double`, a free inline function.

- [ ] **Step 1: Write the failing tests**

Append to `tests/interface/trajectory_executor_test.cpp`:

```cpp
TEST(EffectiveScale, TakesTheSlowerOfGoalAndOverride) {
  EXPECT_DOUBLE_EQ(effective_scale(1.0, 1.0), 1.0);
  EXPECT_DOUBLE_EQ(effective_scale(0.25, 1.0), 0.25);
  EXPECT_DOUBLE_EQ(effective_scale(1.0, 0.3), 0.3);
  EXPECT_DOUBLE_EQ(effective_scale(0.5, 0.2), 0.2);
}

TEST(EffectiveScale, NeverExceedsOneAndNeverReachesZero) {
  // The contract is "this can only ever slow the arm down".
  EXPECT_DOUBLE_EQ(effective_scale(2.0, 1.0), 1.0);
  EXPECT_DOUBLE_EQ(effective_scale(1.0, 5.0), 1.0);
  EXPECT_GT(effective_scale(0.0, 1.0), 0.0);
  EXPECT_GT(effective_scale(-1.0, 1.0), 0.0);
}

TEST(EffectiveScale, NonFiniteFallsBackToFullSpeed) {
  // NaN compares false against every bound, so a naive clamp would pass it
  // straight through and stop the clock forever.
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_DOUBLE_EQ(effective_scale(nan, 1.0), 1.0);
  EXPECT_DOUBLE_EQ(effective_scale(1.0, nan), 1.0);
}
```

- [ ] **Step 2: Run them to verify they fail**

Run: `/home/swapnil/atdev/.driver-deps/build.sh /home/swapnil/atdev/kinova-driver-speed 'EffectiveScale*'`
Expected: compile error — `effective_scale` not declared.

- [ ] **Step 3: Add the field and the helper**

In `include/kinova_lowlevel/interface/value_types.h`, inside `TrajectoryGoal`, after `goal_time_tolerance_s`:

```cpp
  // Execute this goal slower: (0, 1], 1.0 = as planned. Dilates the executor's
  // clock, so the path is unchanged and the commanded velocity scales with it.
  double speed_scale = 1.0;
```

In `include/kinova_lowlevel/interface/trajectory_executor.h`, just after the `Preemption` / `ControlModeKind` enums:

```cpp
// The slower of the goal's own scale and the runtime override, clamped into
// (0, 1]. Free and inline so it is unit-testable without threads. A non-finite
// input is treated as "no request" rather than propagated: NaN compares false
// against every bound, and a NaN scale would stop the trajectory clock dead.
inline double effective_scale(double goal_scale, double override_scale) {
  const double g = std::isfinite(goal_scale) ? goal_scale : 1.0;
  const double o = std::isfinite(override_scale) ? override_scale : 1.0;
  double s = g < o ? g : o;
  if (s > 1.0) s = 1.0;
  if (s < kMinSpeedScale) s = kMinSpeedScale;
  return s;
}
```

and above it:

```cpp
// Floor for the effective scale. Zero would stop the clock and hang the goal
// forever; this is slow enough to be a hold in practice and still terminates.
inline constexpr double kMinSpeedScale = 0.01;
```

Add `#include <cmath>` to `trajectory_executor.h`, and `#include <cmath>` / `#include <limits>` to the test file if not already present.

- [ ] **Step 4: Run and confirm they pass**

Run: `/home/swapnil/atdev/.driver-deps/build.sh /home/swapnil/atdev/kinova-driver-speed 'EffectiveScale*'`
Expected: 3 tests pass.

- [ ] **Step 5: Commit**

```bash
git add include/kinova_lowlevel/interface/value_types.h include/kinova_lowlevel/interface/trajectory_executor.h tests/interface/trajectory_executor_test.cpp
git commit -m "feat(interface): TrajectoryGoal::speed_scale and the effective-scale rule

The rule is a free inline function so it is testable without threads, and
it treats a non-finite scale as 'no request' rather than propagating a NaN
into the trajectory clock."
```

---

### Task 2: The scaled clock

**Files:**
- Modify: `include/kinova_lowlevel/interface/trajectory_executor.h:49-79`
- Modify: `src/interface/trajectory_executor.cpp:49-122`
- Test: `tests/interface/trajectory_executor_test.cpp`

**Interfaces:**
- Consumes: `effective_scale` (Task 1).
- Produces:
  - `SubmitResult submit(const Trajectory&, ControlModeKind, Preemption, const kinova::JointVec& path_tol, double speed_scale = 1.0)`
  - `ExecStatus tick(double now_s, const kinova::JointVec& q_meas, double override_scale = 1.0)`
  - `double applied_scale() const` — the scale actually in force this tick, for tests and telemetry.

Note both new parameters are defaulted, so every existing call site and test compiles unchanged.

- [ ] **Step 1: Write the failing test for the scaled clock**

Append to `tests/interface/trajectory_executor_test.cpp` (the file already has `vec7`, `ramp`, and a `RecordingSink`-style fixture — reuse them):

```cpp
TEST(ExecutorSpeedScale, HalfScaleTakesTwiceAsLongInWallTime) {
  RecordingSink sink;
  TrajectoryExecutor ex(sink);
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins,
            kinova::JointVec::Constant(-1.0), 0.5);

  ex.tick(0.0, vec7(0.0));               // latch the clock
  ExecStatus mid = ex.tick(2.0, vec7(0.0));   // 2 s wall = 1 s trajectory time
  EXPECT_TRUE(mid.active);
  EXPECT_NEAR(mid.fraction, 0.5, 1e-9) << "fraction must track the SCALED clock";
  EXPECT_NEAR(sink.last[0], 0.5, 1e-9) << "halfway along a 0->1 ramp";

  ExecStatus end = ex.tick(4.0, vec7(1.0));   // 4 s wall = 2 s trajectory time
  EXPECT_TRUE(end.completed);
  EXPECT_NEAR(end.fraction, 1.0, 1e-9);
}

TEST(ExecutorSpeedScale, FullScaleIsIdenticalToBeforeTheFeature) {
  RecordingSink sink;
  TrajectoryExecutor ex(sink);
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins,
            kinova::JointVec::Constant(-1.0));  // default scale
  ex.tick(10.0, vec7(0.0));
  ExecStatus mid = ex.tick(11.0, vec7(0.0));
  EXPECT_NEAR(mid.fraction, 0.5, 1e-9);
  EXPECT_NEAR(sink.last[0], 0.5, 1e-9);
}
```

If the test file has no recording sink, add a minimal one in its anonymous namespace:

```cpp
struct RecordingSink : kinova::JointTargetSink {
  kinova::JointVec last = kinova::JointVec::Zero();
  void set_target(const kinova::JointVec& q) override { last = q; }
};
```

- [ ] **Step 2: Run it to verify it fails**

Run: `/home/swapnil/atdev/.driver-deps/build.sh /home/swapnil/atdev/kinova-driver-speed 'ExecutorSpeedScale*'`
Expected: compile error — `submit` takes four arguments.

- [ ] **Step 3: Extend the executor's state**

In `include/kinova_lowlevel/interface/trajectory_executor.h`, change `Active` and add the scale members:

```cpp
  struct Active {
    Trajectory tr;
    // Trajectory time, ACCUMULATED at dt * scale — not a difference of wall
    // stamps. That is what lets the scale change while a goal is running.
    double traj_t = 0.0;
    double last_now_s = 0.0;
    bool started = false;
  };
```

and beside `path_tol_` / `queued_tol_`:

```cpp
  double scale_ = 1.0;         // the ACTIVE goal's own requested scale
  double queued_scale_ = 1.0;  // adopted when queued_ is promoted
  double applied_ = 1.0;       // the scale actually in force (slew-limited, Task 3)
```

Change the two signatures as given in the Interfaces block above, and add:

```cpp
  double applied_scale() const { return applied_; }
```

- [ ] **Step 4: Rewrite the clock in `tick()`**

In `src/interface/trajectory_executor.cpp`, replace the start-latch and elapsed computation:

```cpp
  Active& a = *active_;
  if (!a.started) {
    a.last_now_s = now_s;
    a.traj_t = 0.0;
    a.started = true;
  }
  const double dt_wall = now_s - a.last_now_s;
  a.last_now_s = now_s;
  applied_ = effective_scale(scale_, override_scale);
  a.traj_t += dt_wall * applied_;
  const double elapsed = a.traj_t;
```

Everything downstream — `sample(a.tr, elapsed)`, the fraction, the completion test — stays as it is and is now on the scaled clock by construction. No new include is needed: `effective_scale` lives in this unit's own header (Task 1).

- [ ] **Step 5: Store the scale in `submit`**

In each of the three `submit` branches (`cpp:53-72`): the idle-adopt and `kLatestWins` branches set `scale_ = speed_scale`; the `kQueue` branch sets `queued_scale_ = speed_scale` and leaves `scale_` alone, exactly mirroring how `path_tol_` and `queued_tol_` are handled.

- [ ] **Step 6: Run the tests**

Run: `/home/swapnil/atdev/.driver-deps/build.sh /home/swapnil/atdev/kinova-driver-speed 'Executor*'`
Expected: the two new tests pass and every pre-existing `Executor*` test still passes — they use the defaulted parameters, so a regression there means the default path changed.

- [ ] **Step 7: Commit**

```bash
git add include/kinova_lowlevel/interface/trajectory_executor.h src/interface/trajectory_executor.cpp tests/interface/trajectory_executor_test.cpp
git commit -m "feat(interface): dilate the executor's clock by a per-goal speed scale

Elapsed trajectory time becomes an accumulator advanced at dt * scale,
rather than a difference of wall stamps, so the scale can change while a
goal is running."
```

- [ ] **Step 8: Write the promotion test (Review Focus 2)**

```cpp
TEST(ExecutorSpeedScale, PromotedGoalStartsItsOwnClockAtItsOwnScale) {
  RecordingSink sink;
  TrajectoryExecutor ex(sink);
  ex.submit(ramp(1.0), ControlModeKind::kPosition, Preemption::kLatestWins,
            kinova::JointVec::Constant(-1.0), 1.0);
  ex.submit(ramp(1.0), ControlModeKind::kPosition, Preemption::kQueue,
            kinova::JointVec::Constant(-1.0), 0.5);

  ex.tick(0.0, vec7(0.0));
  ExecStatus p = ex.tick(1.0, vec7(1.0));       // first finishes, second promoted
  ASSERT_TRUE(p.promoted);
  EXPECT_NEAR(p.fraction, 0.0, 1e-9) << "the promoted goal starts at zero";

  ExecStatus mid = ex.tick(2.0, vec7(0.0));     // 1 s wall at scale 0.5
  EXPECT_NEAR(mid.fraction, 0.5, 1e-9)
      << "the promoted goal must run at ITS scale, not the finished goal's";
}
```

- [ ] **Step 9: Make it pass**

At the promotion site (`cpp:112-117`), replace `active_ = Active{*queued_, now_s, true}` with a form that latches the new clock and adopts the queued scale:

```cpp
      active_ = Active{*queued_, 0.0, now_s, true};  // traj_t=0, last_now_s=now
      path_tol_ = queued_tol_;
      scale_ = queued_scale_;
      queued_scale_ = 1.0;
      queued_.reset();
```

Check the aggregate initialiser matches the new member order in `Active`.

- [ ] **Step 10: Write the preemption test (Review Focus 3)**

```cpp
TEST(ExecutorSpeedScale, LatestWinsResetsTheScaledClock) {
  RecordingSink sink;
  TrajectoryExecutor ex(sink);
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins,
            kinova::JointVec::Constant(-1.0), 1.0);
  ex.tick(0.0, vec7(0.0));
  ex.tick(1.5, vec7(0.0));  // 1.5 s into the first trajectory

  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins,
            kinova::JointVec::Constant(-1.0), 1.0);
  ex.tick(2.0, vec7(0.0));
  ExecStatus s = ex.tick(2.5, vec7(0.0));
  EXPECT_NEAR(s.fraction, 0.25, 1e-9)
      << "the replacement starts from zero, not from the old elapsed time";
}
```

- [ ] **Step 11: Make it pass and commit**

The `kLatestWins` branch already sets `started = false`; ensure it also zeroes `traj_t` (setting `started=false` is enough if the first-tick branch zeroes `traj_t`, which Step 4 does — verify, do not assume).

Run: `/home/swapnil/atdev/.driver-deps/build.sh /home/swapnil/atdev/kinova-driver-speed 'Executor*'`

```bash
git add src/interface/trajectory_executor.cpp tests/interface/trajectory_executor_test.cpp
git commit -m "test(interface): promotion and preemption re-latch the scaled clock"
```

---

### Task 3: Ramp the scale instead of stepping it

**Files:**
- Modify: `include/kinova_lowlevel/interface/trajectory_executor.h`, `src/interface/trajectory_executor.cpp`
- Test: `tests/interface/trajectory_executor_test.cpp`

**Interfaces:**
- Consumes: Task 2's `applied_`.
- Produces: `kinova::interface::kScaleSlewPerSec` (a constant) and slew-limited `applied_scale()`.

- [ ] **Step 1: Write the failing test**

```cpp
TEST(ExecutorSpeedScale, AnOverrideChangeRampsRatherThanSteps) {
  RecordingSink sink;
  TrajectoryExecutor ex(sink);
  ex.submit(ramp(10.0), ControlModeKind::kPosition, Preemption::kLatestWins,
            kinova::JointVec::Constant(-1.0), 1.0);
  ex.tick(0.0, vec7(0.0), 1.0);
  EXPECT_NEAR(ex.applied_scale(), 1.0, 1e-9);

  // Slam the override to 0.1 and step 10 ms. A step would move the reference
  // discontinuously; the slew limit must keep the change bounded.
  ex.tick(0.01, vec7(0.0), 0.1);
  EXPECT_GT(ex.applied_scale(), 0.1 + 1e-6) << "must not arrive instantly";
  EXPECT_LT(ex.applied_scale(), 1.0);

  for (double t = 0.02; t < 3.0; t += 0.01) ex.tick(t, vec7(0.0), 0.1);
  EXPECT_NEAR(ex.applied_scale(), 0.1, 1e-6) << "and must get there";
}
```

- [ ] **Step 2: Run it and watch it fail**

Run: `/home/swapnil/atdev/.driver-deps/build.sh /home/swapnil/atdev/kinova-driver-speed 'ExecutorSpeedScale.AnOverride*'`
Expected: FAIL — `applied_scale()` jumps straight to 0.1.

- [ ] **Step 3: Implement the slew limit**

In the header, beside the other constants:

```cpp
// How fast the applied scale may change, per second of wall time. A step in
// the scale is a step in the commanded velocity; this bounds it. 2.0 means a
// full 1.0 -> 0.0 change takes half a second.
inline constexpr double kScaleSlewPerSec = 2.0;
```

In `tick()`, replace `applied_ = effective_scale(scale_, override_scale);` with:

```cpp
  const double want = effective_scale(scale_, override_scale);
  const double max_step = kScaleSlewPerSec * (dt_wall > 0.0 ? dt_wall : 0.0);
  if (want > applied_ + max_step)      applied_ += max_step;
  else if (want < applied_ - max_step) applied_ -= max_step;
  else                                 applied_ = want;
```

Initialise `applied_` to the effective scale on the first tick of a goal, so a goal submitted at 0.25 starts at 0.25 rather than ramping down from 1.0: in the `!a.started` branch, set `applied_ = effective_scale(scale_, override_scale);`.

- [ ] **Step 4: Run it, and the whole suite**

Run: `/home/swapnil/atdev/.driver-deps/build.sh /home/swapnil/atdev/kinova-driver-speed 'Executor*'`
Then the full suite: `/home/swapnil/atdev/.driver-deps/build.sh /home/swapnil/atdev/kinova-driver-speed`
Expected: all pass. The full run matters here — the slew changes timing for every trajectory test that passes a non-default override, and there should be none.

- [ ] **Step 5: Commit**

```bash
git add include/kinova_lowlevel/interface/trajectory_executor.h src/interface/trajectory_executor.cpp tests/interface/trajectory_executor_test.cpp
git commit -m "feat(interface): slew-limit the applied speed scale

A step in the scale is a step in the commanded velocity. A goal that asks
for a slow scale still starts at it; only CHANGES are ramped."
```

---

### Task 4: The Supervisor — plumb the goal scale and own the override

**Files:**
- Modify: `include/kinova_lowlevel/interface/supervisor.h`, `src/interface/supervisor.cpp`
- Test: `tests/interface/supervisor_test.cpp`

**Interfaces:**
- Consumes: Tasks 1-3.
- Produces:
  - `SpeedResult Supervisor::set_speed_override(double s)` where `struct SpeedResult { bool accepted = false; std::string message; };` in `value_types.h`, mirroring `GainsResult`.
  - `double Supervisor::speed_override() const` — for tests and telemetry.

- [ ] **Step 1: Write the failing tests**

```cpp
TEST(SupervisorSpeed, OverrideIsAcceptedInRangeAndRefusedOutside) {
  SupFix f;
  EXPECT_TRUE(f.sup.set_speed_override(0.5).accepted);
  EXPECT_DOUBLE_EQ(f.sup.speed_override(), 0.5);

  const SpeedResult too_fast = f.sup.set_speed_override(1.5);
  EXPECT_FALSE(too_fast.accepted);
  EXPECT_FALSE(too_fast.message.empty()) << "a refusal must say why";
  EXPECT_DOUBLE_EQ(f.sup.speed_override(), 0.5) << "a refused set changes nothing";

  EXPECT_FALSE(f.sup.set_speed_override(0.0).accepted);
  EXPECT_FALSE(f.sup.set_speed_override(-0.2).accepted);
  EXPECT_FALSE(f.sup.set_speed_override(std::numeric_limits<double>::quiet_NaN()).accepted);
  EXPECT_DOUBLE_EQ(f.sup.speed_override(), 0.5);
}
```

- [ ] **Step 2: Run it and watch it fail**

Run: `/home/swapnil/atdev/.driver-deps/build.sh /home/swapnil/atdev/kinova-driver-speed 'SupervisorSpeed*'`
Expected: compile error — no such member.

- [ ] **Step 3: Add the storage and the setter**

In `value_types.h`, beside `GainsResult`:

```cpp
struct SpeedResult {
  bool accepted = false;
  std::string message;
};
```

In `supervisor.h`, beside `active_mode_kind_` (the established cross-thread scalar pattern):

```cpp
  // Written by the backend thread, read by the sampler. Only ever slows the
  // arm: values outside (0, 1] are refused, not clamped.
  std::atomic<double> speed_override_{1.0};
```

and publicly:

```cpp
  SpeedResult set_speed_override(double s);
  double speed_override() const { return speed_override_.load(); }
```

In `supervisor.cpp`:

```cpp
SpeedResult Supervisor::set_speed_override(double s) {
  if (!std::isfinite(s)) return {false, "speed override must be finite"};
  if (s <= 0.0 || s > 1.0)
    return {false, "speed override must be in (0, 1]; got " + std::to_string(s)};
  speed_override_.store(s);
  return {true, ""};
}
```

Add a `static_assert` beside the member that `std::atomic<double>` is lock-free, matching how `CommandWatchdog` guards its atomics:

```cpp
  static_assert(std::atomic<double>::is_always_lock_free,
                "speed_override_ is read from the sampler thread; must be lock-free");
```

- [ ] **Step 4: Plumb both ends**

At the submit site (`supervisor.cpp:258-259`), pass the goal's scale as the fifth argument:

```cpp
    const SubmitResult sr = traj_->submit(in.goal.trajectory, in.goal.control_mode,
                                          in.goal.preemption, in.goal.path_tolerance,
                                          in.goal.speed_scale);
```

At the tick site (`supervisor.cpp:303`), pass the override:

```cpp
      const ExecStatus st = traj_->tick(secs_since(t0), q_meas, speed_override_.load());
```

- [ ] **Step 5: Run and confirm**

Run: `/home/swapnil/atdev/.driver-deps/build.sh /home/swapnil/atdev/kinova-driver-speed 'Supervisor*'`
Expected: the new test passes, every existing Supervisor test still passes.

- [ ] **Step 6: Commit**

```bash
git add include/kinova_lowlevel/interface/value_types.h include/kinova_lowlevel/interface/supervisor.h src/interface/supervisor.cpp tests/interface/supervisor_test.cpp
git commit -m "feat(interface): a validated runtime speed override on the Supervisor

Atomic, backend-writes/sampler-reads like active_mode_kind_, and refused
rather than clamped outside (0, 1]."
```

- [ ] **Step 7: Write the goal-validation tests (Review Focus 4)**

```cpp
TEST(SupervisorSpeed, AGoalWithAnOutOfRangeScaleIsRefused) {
  SupFix f;
  f.sup.start();
  f.run_rt();
  for (double bad : {1.5, 0.0, -0.5, std::numeric_limits<double>::quiet_NaN()}) {
    TrajectoryGoal g;
    g.trajectory = ramp7(0.0, 0.2, 1.0);
    g.speed_scale = bad;
    EXPECT_NE(f.sup.on_trajectory_goal(g), GoalResponse::kAccept)
        << "scale " << bad << " must be refused at accept time";
  }
  f.sup.stop();
  f.teardown();
}
```

- [ ] **Step 8: Make it pass**

In `Supervisor::on_trajectory_goal` (`supervisor.cpp:340-352`), beside the existing empty-points check:

```cpp
  if (!std::isfinite(g.speed_scale) || g.speed_scale <= 0.0 || g.speed_scale > 1.0)
    return GoalResponse::kReject;
```

Note for the reviewer: `GoalResponse` carries no message, matching how an empty trajectory is already refused. The *reason* reaches a caller at the ROS boundary, which validates the same range and can say why (`kinova-gen3-ros2#37`).

- [ ] **Step 9: Write the default-goal test (Review Focus 5)**

```cpp
TEST(SupervisorSpeed, ADefaultConstructedGoalIsFullSpeed) {
  // on_trajectory_cancel pushes a default-constructed TrajectoryGoal onto the
  // inbox; every field must be harmless at its default on that path.
  const TrajectoryGoal g;
  EXPECT_DOUBLE_EQ(g.speed_scale, 1.0);
  EXPECT_DOUBLE_EQ(effective_scale(g.speed_scale, 1.0), 1.0);
}
```

- [ ] **Step 10: Run the full suite and commit**

Run: `/home/swapnil/atdev/.driver-deps/build.sh /home/swapnil/atdev/kinova-driver-speed`

```bash
git add src/interface/supervisor.cpp tests/interface/supervisor_test.cpp
git commit -m "feat(interface): refuse an out-of-range goal speed scale at accept time"
```

---

### Task 5: Reach the override through the port

**Files:**
- Modify: `include/kinova_lowlevel/interface/ports.h`
- Modify: `include/kinova_lowlevel/interface/arbiter.h`, `src/interface/arbiter.cpp`
- Test: `tests/interface/arbiter_test.cpp`

**Interfaces:**
- Consumes: `Supervisor::set_speed_override` (Task 4).
- Produces: `CommandSink::on_set_speed_override(double) -> SpeedResult`, **non-pure with a default implementation**, and `Arbiter`'s forwarding of it.

- [ ] **Step 1: Write the failing test**

```cpp
TEST(ArbiterSpeed, ForwardsTheOverrideWithoutRequiringTheToken) {
  // Slowing the arm down is always allowed: it cannot make the arm do
  // anything it was not already doing, and an operator reaching for the dial
  // should never be refused because someone else holds the token.
  FakeSink sink;
  Arbiter arb(sink, /* enforced */ true);
  const SpeedResult r = arb.on_set_speed_override(0.3);
  EXPECT_TRUE(r.accepted);
  EXPECT_DOUBLE_EQ(sink.last_speed_override, 0.3);
}
```

Add `double last_speed_override = 1.0;` and the override method to the test's `FakeSink`.

- [ ] **Step 2: Run it and watch it fail**

Run: `/home/swapnil/atdev/.driver-deps/build.sh /home/swapnil/atdev/kinova-driver-speed 'Arbiter*'`
Expected: compile error — no such method.

- [ ] **Step 3: Add the port method with a default**

In `ports.h`, on `CommandSink`, beside `on_set_gains`:

```cpp
  // Slow everything down at runtime. NOT token-gated: it can only reduce
  // speed, so it can never make the arm do something it was not already
  // doing. Non-pure so existing implementers keep compiling; the default
  // refuses, which is honest for a sink that cannot honour it.
  virtual SpeedResult on_set_speed_override(double) {
    return {false, "speed override not supported by this sink"};
  }
```

In `Supervisor`, override it to call `set_speed_override`. In `Arbiter`, forward it to `down_` **without** a token check, with a comment saying why.

- [ ] **Step 4: Run and commit**

Run: `/home/swapnil/atdev/.driver-deps/build.sh /home/swapnil/atdev/kinova-driver-speed 'Arbiter*:Supervisor*'`

```bash
git add include/kinova_lowlevel/interface/ports.h include/kinova_lowlevel/interface/arbiter.h src/interface/arbiter.cpp include/kinova_lowlevel/interface/supervisor.h src/interface/supervisor.cpp tests/interface/arbiter_test.cpp
git commit -m "feat(interface): on_set_speed_override on CommandSink, ungated in the Arbiter

Default implementation refuses, so existing sinks keep compiling. The
Arbiter forwards without a token: the override can only slow the arm."
```

---

### Task 6: Prove the RT contract, then document

**Files:**
- Modify: `docs/deep-dive/trajectory-interpolation.md`, `docs/reference/api.md`, `CHANGELOG.md`
- Verify: `tests/rt_safety_test.cpp` (unchanged)

- [ ] **Step 1: Run the RT gate**

Run: `/home/swapnil/atdev/.driver-deps/build.sh /home/swapnil/atdev/kinova-driver-speed 'RtSafety*-RtSafety.NanosleepPacingProducesSamples'`
Expected: all pass — zero major page faults, zero dropped samples. Paste the output into your report. If this fails, stop and report: something in the clock change allocates or blocks, and no documentation should be written over it.

- [ ] **Step 2: Add a scaled-clock case to the RT suite**

In `tests/rt_safety_test.cpp`, in `SupervisorInLoopNoMajorFaultsSteadyState`, set a non-default override before the measured window (`f.sup.set_speed_override(0.25)` or the local equivalent) so the gate exercises the slew path rather than only the `applied_ == want` fast path. Keep both assertions as they are.

- [ ] **Step 3: Re-run the gate and the full suite**

Run: `/home/swapnil/atdev/.driver-deps/build.sh /home/swapnil/atdev/kinova-driver-speed`
Expected: everything green.

- [ ] **Step 4: Document the behaviour**

In `docs/deep-dive/trajectory-interpolation.md`, after the sampler-rate section, add a short "Speed scale" section stating: the scale dilates the executor's clock, so the path is identical and the commanded velocity scales with `s` and acceleration with `s²`; the effective scale is the minimum of the goal's and the override's, clamped to `(0, 1]`; changes are slew-limited at `kScaleSlewPerSec`; and that this is **not** `max_ref_speed`, which is a rate limit on the reference and would manufacture the divergence that aborts a goal.

In `docs/reference/api.md`, add `speed_scale` to the `TrajectoryGoal` field list and `set_speed_override`/`on_set_speed_override` to the interface entries.

In `CHANGELOG.md` under `## [Unreleased]` → `### Added`:

```markdown
- `TrajectoryGoal::speed_scale` and a runtime speed override on the `Supervisor`
  (`on_set_speed_override`): execute a planned trajectory slower by dilating the
  executor's clock. The path is unchanged; velocity scales with `s`, acceleration
  with `s²`. Effective scale is the slower of goal and override, clamped to
  `(0, 1]`, and changes are slew-limited so the commanded velocity never steps
  (#69).
```

- [ ] **Step 5: Commit**

```bash
git add docs/ CHANGELOG.md tests/rt_safety_test.cpp
git commit -m "docs: the trajectory speed scale, and exercise it in the RT gate"
```

---

## Notes for the reviewer

- **The derivative scaling is implicit, deliberately.** `sample()` returns positions only and `JointTargetSink` is position-only, so running the clock at rate `s` gives `s·qd` and `s²·qdd` in wall time for free. Do not expect — or ask for — a separate derivative-scaling path; there is no consumer for one.
- **`applied_scale()` exists for tests and telemetry**, not for control flow. If any production code branches on it, that is a finding.
- **The override is ungated on purpose.** It can only slow the arm, so requiring the capability token would mean an operator reaching for the speed dial gets refused because another client holds ownership.
