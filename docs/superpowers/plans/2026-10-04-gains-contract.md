# Gains Contract (v1.3.0 Plan 1) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** One validated `GainsSpec` contract (named profiles + raw escape) on trajectory goals, stream opens, and `set_gains`, with no gain leakage, no partial resets, and no mode-switch gating.

**Architecture:** A new pure unit (`interface/gains.h|cpp`) owns the profile table, validation bounds, and spec→params resolution. The Supervisor keeps a mutex-guarded session default and applies resolved gains on every impedance goal execution and impedance stream open; `on_set_gains` un-stubs into "replace the session default." The RT path is untouched — all changes live on the backend/sampler threads.

**Tech Stack:** C++17, gtest, Eigen, Pinocchio (via `Dynamics`, test-only here). Sim-only x86 build (`./build` is configured; `cmake --build build -j`).

**Spec:** `docs/superpowers/specs/2026-10-04-compliant-execution-everywhere-design.md`

## Global Constraints

- Nothing in the RT path (`compute`, executor cycle) may allocate, lock, or block; this plan must not touch `compute()` or `RtExecutor`.
- Fail loud: invalid gains are **rejected with a reason**, never clamped.
- Breaking change is accepted and ships as v1.3.0 (spec §Compatibility); do not add back-compat shims for `has_gains`.
- Tests are one gtest binary `unit_tests`; run subsets with `--gtest_filter`.
- `URDF_PATH` is compile-time injected and points at the 2F-85 (gripper) model — the worst case for gravity, which is what the floor derivation needs.
- Every commit message ends with the session attribution lines.

## Review Focus

1. **zeta = 0 or negative** (undamped/anti-damped spring; oscillates or diverges) → rejected. Test in Task 1.
2. **torque_limit below the gravity floor** (#64: clamp eats gravity, arm falls) → rejected, including the all-zeros message default. Tests in Tasks 1, 2, 3.
3. **NaN smuggled through custom gains** (NaN survives `std::clamp` in `compute`) → rejected at accept. Test in Task 1.
4. **Gains supplied where they can't act** (position goal, velocity/torque/position stream) → rejected loudly, not silently ignored. Tests in Tasks 2 and 3.
5. **`set_gains` with `kSessionDefault`** (circular: "set the default to the default") → rejected with a message. Test in Task 3.

---

### Task 1: Gains vocabulary, profile table, validation

**Files:**
- Modify: `include/kinova_lowlevel/interface/value_types.h:92-96` (add `GainsProfile`, `GainsSpec` next to `JointImpedanceGains`)
- Create: `include/kinova_lowlevel/interface/gains.h`
- Create: `src/interface/gains.cpp`
- Create: `tests/interface/gains_test.cpp`
- Modify: `CMakeLists.txt:106-109` (lib sources), `:356-360` (test sources)

**Interfaces:**
- Consumes: `JointImpedanceParams` (`joint_impedance_mode.h:13`), `JointImpedanceGains` (`value_types.h:92`), `Dynamics` (test only).
- Produces (later tasks and the ROS repo rely on these exact names):
  - `enum class GainsProfile { kSessionDefault, kSoft, kMedium, kStiff, kCustom }`
  - `struct GainsSpec { GainsProfile profile = GainsProfile::kSessionDefault; JointImpedanceGains custom{}; }` (both in `value_types.h`)
  - `kinova::JointImpedanceParams profile_params(GainsProfile)` — named entries only; throws `std::invalid_argument` for `kCustom`/`kSessionDefault`
  - `struct GainsCheck { bool ok = false; std::string message; }`
  - `GainsCheck validate_custom(const JointImpedanceGains&)`
  - `JointImpedanceParams resolve_gains(const GainsSpec&, const JointImpedanceParams& session_default)`
  - bounds constants: `kKqMin`, `kKqMax`, `kZetaMin`, `kZetaMax`, `kTorqueLimitFloor`, `kTorqueLimitCeil` (all in `kinova::interface`, declared in `gains.h`)

- [ ] **Step 1: Add the spec types to `value_types.h`**

Directly below `JointImpedanceGains` (line 96):

```cpp
// How a command names its compliance. kSessionDefault = "whatever the session
// default points at" (initially the kMedium profile); named profiles are
// complete, core-owned parameter sets; kCustom overrides kq/zeta/torque_limit
// on top of the session default and MUST pass validate_custom at accept time.
enum class GainsProfile { kSessionDefault, kSoft, kMedium, kStiff, kCustom };
struct GainsSpec {
  GainsProfile profile = GainsProfile::kSessionDefault;
  JointImpedanceGains custom{};  // read iff profile == kCustom
};
```

- [ ] **Step 2: Write the failing tests** (`tests/interface/gains_test.cpp`)

```cpp
#include <gtest/gtest.h>

#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/interface/gains.h"
using namespace kinova;
using namespace kinova::interface;

namespace {
JointImpedanceGains good() {
  JointImpedanceGains g;
  g.kq = (JointVec() << 80, 80, 80, 80, 30, 30, 30).finished();
  g.zeta = 0.5;
  g.torque_limit = (JointVec() << 39, 39, 39, 39, 9, 9, 9).finished();
  return g;
}
}  // namespace

TEST(GainsValidation, AcceptsTheModeDefaults) { EXPECT_TRUE(validate_custom(good()).ok); }

TEST(GainsValidation, RejectsZeroTorqueLimit) {
  // THE #64 shape: a zero-filled message default. The clamp would eat gravity
  // and the arm falls. Must be refused with a reason, never clamped.
  JointImpedanceGains g = good();
  g.torque_limit = JointVec::Zero();
  const GainsCheck c = validate_custom(g);
  EXPECT_FALSE(c.ok);
  EXPECT_FALSE(c.message.empty());
}

TEST(GainsValidation, RejectsTorqueLimitBelowGravityFloor) {
  JointImpedanceGains g = good();
  g.torque_limit = kTorqueLimitFloor * 0.5;
  EXPECT_FALSE(validate_custom(g).ok);
}

TEST(GainsValidation, RejectsNonFiniteAndNegativeFields) {
  JointImpedanceGains g = good();
  g.kq[2] = std::numeric_limits<double>::quiet_NaN();  // NaN survives std::clamp in compute()
  EXPECT_FALSE(validate_custom(g).ok);
  g = good();
  g.zeta = 0.0;  // undamped spring
  EXPECT_FALSE(validate_custom(g).ok);
  g = good();
  g.zeta = -0.3;  // anti-damped
  EXPECT_FALSE(validate_custom(g).ok);
  g = good();
  g.kq[0] = kKqMax * 2.0;
  EXPECT_FALSE(validate_custom(g).ok);
  g = good();
  g.kq[0] = 0.0;  // no spring at all is not impedance
  EXPECT_FALSE(validate_custom(g).ok);
}

TEST(GainsProfiles, EveryNamedEntryPassesItsOwnValidation) {
  for (GainsProfile p : {GainsProfile::kSoft, GainsProfile::kMedium, GainsProfile::kStiff}) {
    const JointImpedanceParams jp = profile_params(p);
    JointImpedanceGains g;
    g.kq = jp.Kq;
    g.zeta = jp.zeta;
    g.torque_limit = jp.torque_limit;
    EXPECT_TRUE(validate_custom(g).ok) << static_cast<int>(p);
  }
}

TEST(GainsProfiles, MediumMatchesTheModeDefaults) {
  // "Switch to impedance, change nothing else" must behave exactly like the
  // tuned constructor defaults everyone has been running.
  const JointImpedanceParams m = profile_params(GainsProfile::kMedium);
  const JointImpedanceParams d{};
  EXPECT_TRUE(m.Kq.isApprox(d.Kq));
  EXPECT_DOUBLE_EQ(m.zeta, d.zeta);
  EXPECT_TRUE(m.torque_limit.isApprox(d.torque_limit));
  EXPECT_DOUBLE_EQ(m.max_tracking_error, d.max_tracking_error);
}

TEST(GainsProfiles, NamelessKindsThrow) {
  EXPECT_THROW(profile_params(GainsProfile::kCustom), std::invalid_argument);
  EXPECT_THROW(profile_params(GainsProfile::kSessionDefault), std::invalid_argument);
}

TEST(GainsResolve, SessionDefaultReturnsTheSessionDefault) {
  JointImpedanceParams def = profile_params(GainsProfile::kSoft);
  const JointImpedanceParams r = resolve_gains(GainsSpec{}, def);
  EXPECT_TRUE(r.Kq.isApprox(def.Kq));
  EXPECT_DOUBLE_EQ(r.max_tracking_error, def.max_tracking_error);
}

TEST(GainsResolve, CustomOverridesOnlyItsThreeFieldsOnTheSessionDefault) {
  // No-partial-reset guarantee: leash/ramp/ref-speed come from the session
  // default, NOT from a default-constructed params (the old mode-switch bug).
  JointImpedanceParams def = profile_params(GainsProfile::kSoft);  // leash 0.45
  GainsSpec s;
  s.profile = GainsProfile::kCustom;
  s.custom = good();
  s.custom.zeta = 0.8;
  const JointImpedanceParams r = resolve_gains(s, def);
  EXPECT_DOUBLE_EQ(r.zeta, 0.8);
  EXPECT_TRUE(r.kq_unused_sentinel_never_exists);  // DELETE THIS LINE WHEN WRITING: see note
  EXPECT_DOUBLE_EQ(r.max_tracking_error, def.max_tracracking_error);
}

TEST(GainsFloor, FloorsCoverWorstCaseGravityWithMargin) {
  // The floor constants are hard-coded; THIS test keeps them honest against the
  // URDF. Sample the joint space, record max |g_i|, require floor >= 1.2x that
  // and floor <= ceil. On failure it prints the measured max so the constant
  // can be re-pinned after a model change.
  Dynamics dyn(URDF_PATH);  // 2F-85 model: worst-case payload
  JointVec lo, hi;
  dyn.joint_limits(lo, hi);
  JointVec max_g = JointVec::Zero(), g = JointVec::Zero();
  std::srand(42);
  for (int n = 0; n < 20000; ++n) {
    JointVec q;
    for (int i = 0; i < kNumJoints; ++i) {
      const double a = std::isfinite(lo[i]) ? lo[i] : -M_PI;
      const double b = std::isfinite(hi[i]) ? hi[i] : M_PI;
      q[i] = a + (b - a) * (std::rand() / double(RAND_MAX));
    }
    dyn.gravity(q, g);
    max_g = max_g.cwiseMax(g.cwiseAbs());
  }
  for (int i = 0; i < kNumJoints; ++i) {
    EXPECT_GE(kTorqueLimitFloor[i], 1.2 * max_g[i])
        << "joint " << i << ": measured worst-case |gravity| = " << max_g[i];
    EXPECT_LE(kTorqueLimitFloor[i], kTorqueLimitCeil[i]);
  }
}
```

Note on `CustomOverridesOnly...`: the two marked lines are intentionally broken
placeholders in THIS PLAN ONLY — when writing the file, assert instead:
`EXPECT_TRUE(r.Kq.isApprox(s.custom.kq));`,
`EXPECT_TRUE(r.torque_limit.isApprox(s.custom.torque_limit));`,
`EXPECT_DOUBLE_EQ(r.max_tracking_error, def.max_tracking_error);`,
`EXPECT_DOUBLE_EQ(r.gain_ramp_s, def.gain_ramp_s);`.

- [ ] **Step 3: Write `gains.h`**

```cpp
#pragma once
#include <string>

#include "kinova_lowlevel/interface/value_types.h"
#include "kinova_lowlevel/joint_impedance_mode.h"
namespace kinova::interface {

// Validation bounds. Rejection posture throughout: a value outside these is
// REFUSED with a reason, never clamped -- clamping is how #64 dropped the arm.
constexpr double kKqMin = 1.0;     // N*m/rad; below this it is not a spring
constexpr double kKqMax = 400.0;   // 5x the default shoulder stiffness
constexpr double kZetaMin = 0.05;  // an (almost) undamped spring oscillates
constexpr double kZetaMax = 2.0;
// Gravity headroom per joint: the impedance clamp applies to the TOTAL torque,
// gravity included, so a limit below worst-case |g_i(q)| lets the clamp eat
// gravity and the arm falls (#64). Pinned constants; GainsFloor test re-derives
// the worst case from the URDF and fails loudly if these go stale.
extern const JointVec kTorqueLimitFloor;
extern const JointVec kTorqueLimitCeil;  // URDF effort limits

struct GainsCheck {
  bool ok = false;
  std::string message;
};
// Bounds-check raw (kCustom) gains. Pure; callable from any thread.
GainsCheck validate_custom(const JointImpedanceGains& g);
// The complete parameter set a NAMED profile stands for. kCustom and
// kSessionDefault are not names -- std::invalid_argument, fail loud.
kinova::JointImpedanceParams profile_params(GainsProfile p);
// What a command's spec means, given the current session default:
//   kSessionDefault -> session_default verbatim
//   named profile   -> profile_params(p)
//   kCustom         -> session_default with kq/zeta/torque_limit overridden
//                      (validate_custom MUST have accepted it upstream)
kinova::JointImpedanceParams resolve_gains(const GainsSpec& s,
                                           const kinova::JointImpedanceParams& session_default);
}  // namespace kinova::interface
```

- [ ] **Step 4: Write `gains.cpp`**

```cpp
#include "kinova_lowlevel/interface/gains.h"

#include <cmath>
#include <stdexcept>
namespace kinova::interface {

// PROVISIONAL floors -- Step 6 pins them from the GainsFloor test's measured
// worst-case gravity (1.2x margin, rounded up to one decimal).
const JointVec kTorqueLimitFloor = (JointVec() << 5, 25, 5, 15, 2, 3, 0.5).finished();
const JointVec kTorqueLimitCeil = (JointVec() << 39, 39, 39, 39, 9, 9, 9).finished();

GainsCheck validate_custom(const JointImpedanceGains& g) {
  auto fail = [](const std::string& m) { return GainsCheck{false, m}; };
  if (!std::isfinite(g.zeta) || g.zeta < kZetaMin || g.zeta > kZetaMax)
    return fail("zeta must be finite and in [" + std::to_string(kZetaMin) + ", " +
                std::to_string(kZetaMax) + "]");
  for (int i = 0; i < kNumJoints; ++i) {
    if (!std::isfinite(g.kq[i]) || g.kq[i] < kKqMin || g.kq[i] > kKqMax)
      return fail("kq[" + std::to_string(i) + "] must be finite and in [" +
                  std::to_string(kKqMin) + ", " + std::to_string(kKqMax) + "]");
    if (!std::isfinite(g.torque_limit[i]) || g.torque_limit[i] < kTorqueLimitFloor[i] ||
        g.torque_limit[i] > kTorqueLimitCeil[i])
      return fail("torque_limit[" + std::to_string(i) + "] must be finite and in [" +
                  std::to_string(kTorqueLimitFloor[i]) + ", " +
                  std::to_string(kTorqueLimitCeil[i]) +
                  "]: below the floor the clamp eats gravity and the arm falls");
  }
  return {true, ""};
}

kinova::JointImpedanceParams profile_params(GainsProfile p) {
  kinova::JointImpedanceParams jp;  // defaults ARE the medium tuning
  switch (p) {
    case GainsProfile::kMedium:
      return jp;
    case GainsProfile::kSoft:
      jp.Kq = (JointVec() << 40, 40, 40, 40, 15, 15, 15).finished();
      jp.zeta = 0.4;
      jp.max_tracking_error = 0.45;  // softer spring, longer leash
      return jp;
    case GainsProfile::kStiff:
      jp.Kq = (JointVec() << 160, 160, 160, 160, 60, 60, 60).finished();
      jp.zeta = 0.7;
      jp.max_tracking_error = 0.25;  // stiffer spring, tighter leash
      return jp;
    case GainsProfile::kCustom:
    case GainsProfile::kSessionDefault:
      break;
  }
  throw std::invalid_argument("profile_params: not a named profile");
}

kinova::JointImpedanceParams resolve_gains(const GainsSpec& s,
                                           const kinova::JointImpedanceParams& session_default) {
  switch (s.profile) {
    case GainsProfile::kSessionDefault:
      return session_default;
    case GainsProfile::kCustom: {
      kinova::JointImpedanceParams jp = session_default;
      jp.Kq = s.custom.kq;
      jp.zeta = s.custom.zeta;
      jp.torque_limit = s.custom.torque_limit;
      return jp;
    }
    default:
      return profile_params(s.profile);
  }
}
}  // namespace kinova::interface
```

Initial profile values above are the shipped starting point; the on-arm tuning
pass (spec §5) is the authority and may retune them.

- [ ] **Step 5: Wire CMake and build**

Add `src/interface/gains.cpp` to the library source list (after
`src/interface/streaming_session.cpp`, line 109) and
`tests/interface/gains_test.cpp` to the test list (line 360). Then:

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='Gains*'`
Expected: `GainsFloor.FloorsCoverWorstCaseGravityWithMargin` may FAIL, printing
the measured per-joint worst-case gravity; everything else PASSES.

- [ ] **Step 6: Pin the floors from the measured numbers**

Take each printed `measured worst-case |gravity|`, multiply by 1.2, round UP to
one decimal, and replace the provisional `kTorqueLimitFloor` values. If a floor
would exceed its ceil (it should not on this arm), stop and flag it — that
would mean the URDF cannot be held compliantly at that joint.

Run: `./build/unit_tests --gtest_filter='Gains*'`
Expected: ALL PASS.

- [ ] **Step 7: Commit**

```bash
git add include/kinova_lowlevel/interface/value_types.h include/kinova_lowlevel/interface/gains.h \
        src/interface/gains.cpp tests/interface/gains_test.cpp CMakeLists.txt
git commit -m "feat(interface): GainsSpec vocabulary, profile table, accept-time validation (#63, #64)"
```

---

### Task 2: Trajectory goals carry GainsSpec; gains apply on every impedance goal

**Files:**
- Modify: `include/kinova_lowlevel/interface/value_types.h:98-112` (`TrajectoryGoal`)
- Modify: `include/kinova_lowlevel/interface/supervisor.h` (members + helper; `#include gains.h`)
- Modify: `src/interface/supervisor.cpp:238-248` (drain), `:344-363` (`on_trajectory_goal`)
- Modify: `include/kinova_lowlevel/joint_impedance_mode.h:94` (make `params()` public)
- Modify: `tests/interface/supervisor_test.cpp:33,326,413` (old-field sites) + new tests

**Interfaces:**
- Consumes: `GainsSpec`, `validate_custom`, `resolve_gains`, `profile_params` (Task 1).
- Produces:
  - `TrajectoryGoal.gains` is now `GainsSpec` (no `has_gains`) — the ROS mapping (Plan 4) builds against this.
  - `Supervisor::apply_impedance_gains(const GainsSpec&)` (private) — Task 3 reuses it at stream open.
  - `Supervisor` member `session_default_params_` + `gains_mtx_` — Task 3's `on_set_gains` writes it.
  - `JointImpedanceMode::params()` public — tests and diagnostics read the live tuning.

- [ ] **Step 1: Swap the goal fields**

In `TrajectoryGoal` replace

```cpp
  JointImpedanceGains gains{};
  bool has_gains = false;
```

with

```cpp
  // Which compliance this goal runs under when control_mode == kImpedance.
  // Defaults to the session default (initially the kMedium profile). A
  // position goal carrying a non-default spec is REJECTED: gains that cannot
  // act are a caller bug, surfaced loudly, not ignored.
  GainsSpec gains{};
```

- [ ] **Step 2: Make `JointImpedanceMode::params()` public**

Move the declaration from the private section (`joint_impedance_mode.h:94`) to
the public section (below `last_damping()`), with the comment:

```cpp
  // Snapshot of the live parameter set (RT-safe copy). For tests, diagnostics
  // and gain read-back -- the one way to see what tuning is actually in force.
  JointImpedanceParams params() const noexcept;
```

- [ ] **Step 3: Write the failing tests** (append to `tests/interface/supervisor_test.cpp`; also fix the three old-field sites: line 33 becomes `EXPECT_EQ(g.gains.profile, interface::GainsProfile::kSessionDefault);`, lines 326/413 become `gi.gains.profile = interface::GainsProfile::kCustom; gi.gains.custom = <the gains the test was setting>;`)

```cpp
namespace {
interface::TrajectoryGoal imp_goal(double to, interface::GainsSpec spec = {}) {
  interface::TrajectoryGoal g;
  g.trajectory = ramp7(0.0, to, 0.3);
  g.control_mode = interface::ControlModeKind::kImpedance;
  g.path_tolerance = JointVec::Constant(-1.0);
  g.gains = spec;
  return g;
}
void run_goal(SupFix& f, const interface::TrajectoryGoal& g, uint8_t id0) {
  interface::GoalId id{};
  id[0] = id0;
  ASSERT_EQ(f.sup.on_trajectory_goal(g), interface::GoalResponse::kAccept);
  f.sup.on_trajectory_accepted(id, g);
  std::this_thread::sleep_for(std::chrono::milliseconds(900));  // run + settle
}
}  // namespace

TEST(SupervisorGains, ImpedanceGoalWithoutSpecRunsTheSessionDefault) {
  SupFix f;
  f.sup.start();
  f.run_rt();
  run_goal(f, imp_goal(0.04), 1);
  f.sup.stop();
  f.teardown();
  const JointImpedanceParams want = interface::profile_params(interface::GainsProfile::kMedium);
  EXPECT_TRUE(f.imp.params().Kq.isApprox(want.Kq));
  EXPECT_DOUBLE_EQ(f.imp.params().max_tracking_error, want.max_tracking_error);
}

TEST(SupervisorGains, CustomGainsDoNotLeakIntoTheNextGoal) {
  SupFix f;
  f.sup.start();
  f.run_rt();
  interface::GainsSpec s;
  s.profile = interface::GainsProfile::kCustom;
  s.custom.kq = JointVec::Constant(50.0);
  s.custom.zeta = 0.9;
  s.custom.torque_limit = (JointVec() << 39, 39, 39, 39, 9, 9, 9).finished();
  run_goal(f, imp_goal(0.03, s), 1);
  EXPECT_NEAR(f.imp.params().Kq[0], 50.0, 1e-12);  // custom took effect...
  run_goal(f, imp_goal(0.06), 2);                  // ...and a bare goal resets to default
  f.sup.stop();
  f.teardown();
  const JointImpedanceParams want = interface::profile_params(interface::GainsProfile::kMedium);
  EXPECT_TRUE(f.imp.params().Kq.isApprox(want.Kq));
}

TEST(SupervisorGains, GainsApplyEvenWithoutAModeSwitch) {
  // Old bug: gains sat inside the mode-switch branch, so the second impedance
  // goal's gains were silently ignored.
  SupFix f;
  f.sup.start();
  f.run_rt();
  run_goal(f, imp_goal(0.03), 1);  // enter impedance with defaults
  interface::GainsSpec s;
  s.profile = interface::GainsProfile::kStiff;
  run_goal(f, imp_goal(0.06, s), 2);  // same mode, new gains
  f.sup.stop();
  f.teardown();
  const JointImpedanceParams want = interface::profile_params(interface::GainsProfile::kStiff);
  EXPECT_TRUE(f.imp.params().Kq.isApprox(want.Kq));
  EXPECT_DOUBLE_EQ(f.imp.params().max_tracking_error, want.max_tracking_error);
}

TEST(SupervisorGains, RejectsInvalidCustomGainsAtAccept) {
  SupFix f;  // no threads needed: on_trajectory_goal is a pure pre-check
  interface::GainsSpec s;
  s.profile = interface::GainsProfile::kCustom;  // all-zero custom = #64 shape
  EXPECT_EQ(f.sup.on_trajectory_goal(imp_goal(0.05, s)), interface::GoalResponse::kReject);
}

TEST(SupervisorGains, RejectsGainsOnAPositionGoal) {
  SupFix f;
  interface::TrajectoryGoal g;
  g.trajectory = ramp7(0.0, 0.05, 0.4);
  g.control_mode = interface::ControlModeKind::kPosition;
  g.gains.profile = interface::GainsProfile::kStiff;  // cannot act in position mode
  EXPECT_EQ(f.sup.on_trajectory_goal(g), interface::GoalResponse::kReject);
}
```

- [ ] **Step 4: Run tests to verify they fail**

Run: `cmake --build build -j 2>&1 | tail -5`
Expected: compile errors in `supervisor.cpp` (`has_gains` gone) — that is the
point; proceed to Step 5.

- [ ] **Step 5: Rewire the Supervisor**

`supervisor.h`: add `#include "kinova_lowlevel/interface/gains.h"`; in the
private section add:

```cpp
  // Resolve a command's GainsSpec against the session default and push it into
  // imp_. gains_mtx_ makes the two writer sites (sampler drain, backend stream
  // open) mutually exclusive on imp_.set_gains' single-writer double-buffer --
  // they are already mutually exclusive by the goal/stream gating, but that
  // argument is three files wide; the mutex makes it local. Never on the RT path.
  void apply_impedance_gains(const GainsSpec& s);
  std::mutex gains_mtx_;  // guards session_default_params_ + serialises set_gains
  JointImpedanceParams session_default_params_ = profile_params(GainsProfile::kMedium);
```

`supervisor.cpp`:

```cpp
void Supervisor::apply_impedance_gains(const GainsSpec& s) {
  std::lock_guard<std::mutex> l(gains_mtx_);
  imp_.set_gains(resolve_gains(s, session_default_params_));
}
```

In `on_trajectory_goal`, after the kVelocity/kTorque rejection (line 356):

```cpp
  // Gains that cannot act are a caller bug -- reject loudly, don't ignore.
  if (g.control_mode == ControlModeKind::kPosition &&
      g.gains.profile != GainsProfile::kSessionDefault)
    return GoalResponse::kReject;
  // Custom gains are bounds-checked at ACCEPT, so a bad request dies with the
  // goal response instead of reaching the arm (#64).
  if (g.control_mode == ControlModeKind::kImpedance &&
      g.gains.profile == GainsProfile::kCustom && !validate_custom(g.gains.custom).ok)
    return GoalResponse::kReject;
```

In the drain, replace the gains block inside the mode-switch branch (lines
238-248): delete the `if (in.goal.has_gains) { ... imp_.set_gains(p); }` block
entirely, and immediately BEFORE the `if (in.goal.control_mode != traj_bound_kind_ || ...)`
rebind check (line 228) insert:

```cpp
      // EVERY impedance goal applies its resolved gains -- mode switch or not.
      // Resolution happens here (execution), not at accept: the session default
      // is whatever it is when the goal RUNS. No goal's gains outlive it: the
      // next bare goal resolves kSessionDefault and overwrites them.
      if (in.goal.control_mode == ControlModeKind::kImpedance)
        apply_impedance_gains(in.goal.gains);
```

- [ ] **Step 6: Run the tests**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='SupervisorGains*:Gains*'`
Expected: ALL PASS.

Run: `ctest --test-dir build --output-on-failure`
Expected: PASS (the full binary, including `rt_safety_test`'s steady-state
assertions — the RT path is untouched, prove it rather than assume it).

- [ ] **Step 7: Commit**

```bash
git add -A && git commit -m "feat(interface): trajectory goals carry GainsSpec; gains apply per goal, validated at accept (#63)"
```

---

### Task 3: Streams open with gains; on_set_gains sets the session default

**Files:**
- Modify: `include/kinova_lowlevel/interface/value_types.h:29-34` (`StreamOpenRequest`), `:140-143` (`GainsRequest`)
- Modify: `src/interface/supervisor.cpp:417` (`on_set_gains`), `:509-575` (`on_stream_open`)
- Modify: `tests/interface/supervisor_test.cpp` (new tests), `tests/interface/arbiter_test.cpp` (any `GainsRequest{...}` literal gains a `.spec` instead of `.gains`)

**Interfaces:**
- Consumes: `apply_impedance_gains`, `session_default_params_`, `gains_mtx_` (Task 2); `validate_custom`, `resolve_gains` (Task 1).
- Produces:
  - `StreamOpenRequest.gains` (`GainsSpec`) — Plan 4's `OpenStream.srv` maps onto it.
  - `GainsRequest.spec` (`GainsSpec`, replaces the `gains` member) — Plan 4's `SetGains.srv` maps onto it.
  - `on_set_gains` semantics: validates, replaces the session default, touches no live mode; next bare impedance command picks it up.

- [ ] **Step 1: Extend the request types**

`StreamOpenRequest` gains a field after `timeout_s`:

```cpp
  // Compliance for an impedance session, resolved and applied AT OPEN. A
  // non-default spec on a non-impedance open is REJECTED. Mid-session gain
  // changes are deliberately out of scope for v1.3.0: close and re-open.
  GainsSpec gains{};
```

`GainsRequest` becomes:

```cpp
struct GainsRequest {
  GainsSpec spec{};
  Token token{};
};
```

- [ ] **Step 2: Write the failing tests** (append to `supervisor_test.cpp`)

```cpp
TEST(SupervisorGains, ImpedanceStreamOpensWithRequestedProfile) {
  SupFix f;
  f.sup.start();
  f.run_rt();
  interface::StreamOpenRequest r;
  r.kind = interface::SetpointKind::kJointPosition;
  r.control_mode = interface::ControlModeKind::kImpedance;
  r.timeout_s = 0.2;
  r.gains.profile = interface::GainsProfile::kStiff;
  ASSERT_TRUE(f.sup.on_stream_open(r).accepted);
  f.sup.on_stream_close({});
  f.sup.stop();
  f.teardown();
  const JointImpedanceParams want = interface::profile_params(interface::GainsProfile::kStiff);
  EXPECT_TRUE(f.imp.params().Kq.isApprox(want.Kq));
}

TEST(SupervisorGains, StreamGainsDoNotLeakAcrossSessions) {
  SupFix f;
  f.sup.start();
  f.run_rt();
  interface::StreamOpenRequest r;
  r.kind = interface::SetpointKind::kJointPosition;
  r.control_mode = interface::ControlModeKind::kImpedance;
  r.timeout_s = 0.2;
  r.gains.profile = interface::GainsProfile::kStiff;
  ASSERT_TRUE(f.sup.on_stream_open(r).accepted);
  f.sup.on_stream_close({});
  r.gains = {};  // bare re-open: session default, not the last session's stiff
  ASSERT_TRUE(f.sup.on_stream_open(r).accepted);
  f.sup.on_stream_close({});
  f.sup.stop();
  f.teardown();
  const JointImpedanceParams want = interface::profile_params(interface::GainsProfile::kMedium);
  EXPECT_TRUE(f.imp.params().Kq.isApprox(want.Kq));
}

TEST(SupervisorGains, RejectsGainsOnANonImpedanceStream) {
  SupFix f;
  f.sup.start();
  f.run_rt();
  interface::StreamOpenRequest r;
  r.kind = interface::SetpointKind::kJointVelocity;
  r.control_mode = interface::ControlModeKind::kVelocity;
  r.timeout_s = 0.2;
  r.gains.profile = interface::GainsProfile::kSoft;  // cannot act here
  EXPECT_FALSE(f.sup.on_stream_open(r).accepted);
  f.sup.stop();
  f.teardown();
}

TEST(SupervisorGains, SetGainsReplacesTheSessionDefault) {
  SupFix f;
  f.sup.start();
  f.run_rt();
  interface::GainsRequest gr;
  gr.spec.profile = interface::GainsProfile::kSoft;
  EXPECT_TRUE(f.sup.on_set_gains(gr).accepted);
  run_goal(f, imp_goal(0.04), 1);  // bare goal now resolves to soft
  f.sup.stop();
  f.teardown();
  const JointImpedanceParams want = interface::profile_params(interface::GainsProfile::kSoft);
  EXPECT_TRUE(f.imp.params().Kq.isApprox(want.Kq));
  EXPECT_DOUBLE_EQ(f.imp.params().max_tracking_error, want.max_tracking_error);
}

TEST(SupervisorGains, SetGainsRejectsCircularAndInvalidSpecs) {
  SupFix f;
  interface::GainsRequest gr;  // kSessionDefault: "set the default to the default"
  const interface::GainsResult r1 = f.sup.on_set_gains(gr);
  EXPECT_FALSE(r1.accepted);
  EXPECT_FALSE(r1.message.empty());
  gr.spec.profile = interface::GainsProfile::kCustom;  // all-zero custom: #64 shape
  const interface::GainsResult r2 = f.sup.on_set_gains(gr);
  EXPECT_FALSE(r2.accepted);
  EXPECT_FALSE(r2.message.empty());
}
```

- [ ] **Step 3: Run to verify failure**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='SupervisorGains.ImpedanceStreamOpens*:SupervisorGains.StreamGains*:SupervisorGains.Rejects*:SupervisorGains.SetGains*'`
Expected: the stream tests FAIL (gains ignored → medium where stiff expected;
non-impedance open accepted); the set-gains tests FAIL (`on_set_gains` returns
`{}` → `accepted == false` with an EMPTY message — the reject tests fail on the
non-empty-message assertion).

- [ ] **Step 4: Implement**

`on_stream_open`, immediately after the `pair_supported` rejection (line 523):

```cpp
  // Gains that cannot act are a caller bug -- reject loudly, don't ignore.
  if (r.control_mode != ControlModeKind::kImpedance &&
      r.gains.profile != GainsProfile::kSessionDefault)
    return {false, result_code::kStreamRejected, "gains supplied for a non-impedance stream"};
  if (r.control_mode == ControlModeKind::kImpedance) {
    if (r.gains.profile == GainsProfile::kCustom) {
      const GainsCheck c = validate_custom(r.gains.custom);
      if (!c.ok) return {false, result_code::kStreamRejected, c.message};
    }
    // Applied BEFORE the mode switch so the first impedance cycle already runs
    // this session's tuning. If session_.open later refuses, the gains stay on
    // imp_ harmlessly: every next impedance command applies its own resolution,
    // so nothing can run under them.
    apply_impedance_gains(r.gains);
  }
```

Replace the `on_set_gains` stub (line 428):

```cpp
// Sets the SESSION DEFAULT -- what kSessionDefault resolves to from now on. It
// deliberately touches no live mode: a running impedance session keeps the
// tuning it opened with (open-time-only semantics, spec open item resolved);
// the next bare command picks the new default up.
GainsResult Supervisor::on_set_gains(const GainsRequest& r) {
  if (r.spec.profile == GainsProfile::kSessionDefault)
    return {false, "set_gains needs a named profile or custom gains"};
  if (r.spec.profile == GainsProfile::kCustom) {
    const GainsCheck c = validate_custom(r.spec.custom);
    if (!c.ok) return {false, c.message};
  }
  std::lock_guard<std::mutex> l(gains_mtx_);
  session_default_params_ = resolve_gains(r.spec, session_default_params_);
  return {true, ""};
}
```

Fix any `arbiter_test.cpp` compile breaks: `GainsRequest` literals change from
`.gains` to `.spec` (the Arbiter itself only reads `.token` — no logic change).

- [ ] **Step 5: Run the full suite**

Run: `cmake --build build -j && ctest --test-dir build --output-on-failure`
Expected: ALL PASS.

- [ ] **Step 6: Commit**

```bash
git add -A && git commit -m "feat(interface): streams open with gains; on_set_gains sets the session default (#63)"
```

---

### Task 4: Documentation and changelog

**Files:**
- Modify: `docs/interface.md` (gains contract section), `docs/guide/control-modes.md` (profiles table), `docs/reference/api.md` (GainsSpec, validation bounds, on_set_gains semantics), `CHANGELOG.md` (unreleased v1.3.0 section, breaking-change note)

**Interfaces:** none (prose).

- [ ] **Step 1: Write the docs**

Cover, in the matching layer: the three-way `GainsSpec` (profile / custom /
absent-means-session-default), the soft/medium/stiff table with its initial
values and the note that the arm pass may retune them, the validation bounds
(incl. the gravity-floor rationale), the no-leak and apply-per-command
guarantees, open-time-only stream gains, `set_gains` = set-the-default, and the
breaking field changes (`has_gains` gone; `GainsRequest.spec`). CHANGELOG gets
an Unreleased entry flagging the breaking API change per the spec's
compatibility section.

- [ ] **Step 2: Verify the docs build**

Run: `mkdocs build 2>&1 | tail -3` (skip if mkdocs is not installed locally —
no new pages are added, so no `nav:` change is needed).
Expected: no errors.

- [ ] **Step 3: Commit**

```bash
git add docs CHANGELOG.md && git commit -m "docs: gains contract -- profiles, validation bounds, set_gains semantics"
```
