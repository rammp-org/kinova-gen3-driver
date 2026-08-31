# JointVelocityMode + JointPositionMode Pose Path — Implementation Plan (Plan 2)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Land the two remaining mode capabilities the streaming tier refuses today — a stiff `JointVelocityMode` (joint-velocity native, EE twist via damped least squares) and an EE-pose path into `JointPositionMode` — then flip the three `pair_supported` rows that gate them.

**Architecture:** `JointVelocityMode` is a new `ControlMode` commanding `ActuatorMode::kVelocity`. A twist is mapped to joint velocity by a single damped-least-squares 6×7 solve whose damping rises as manipulability falls, plus a null-space posture term so the redundant DOF is decided rather than left to drift. `JointPositionMode` gains a `DiffIkSolver` and a three-way target source, so a live pose target resolves to a joint target that then feeds the mode's **existing** rate-limit → leash → wrap → clamp pipeline unchanged.

**Tech Stack:** C++17, Eigen (fixed-size, no heap on the RT path), Pinocchio (behind `Dynamics` only), GoogleTest (one `unit_tests` binary), CMake.

**Spec:** `docs/superpowers/specs/2026-08-26-streaming-setpoints-design.md` — Components 3 and 4, decision 5 ("Velocity mode is stiff, and says so"), decision 6 (staleness), and the Open Decision on benchmarking, which Task 6 closes.

## Global Constraints

- **SI / radians everywhere internally.** Unit conversion happens only at the `Transport` boundary.
- **Nothing in `compute()` may allocate, lock, or block.** All Eigen scratch is fixed-size and preallocated as members. `rt_safety_test` (zero major page faults, zero dropped samples) is the gate.
- **`kNumJoints = 7`.** All joint types are fixed-size POD.
- **`Dynamics` is the only unit that includes Pinocchio**; modes call `jacobian()`/`fk()`/`gravity()` only.
- **Fail loud at startup, never silent mis-mapping.**
- **Non-RT setters publish via single-writer double-buffer + release-store**; `CommandWatchdog::bump()` is always called **last** in a setter, because its release publishes everything stored above it.
- **Continuous joints** (both URDF limits infinite) require `wrap_to_pi` on any difference of angles. Gen3 has continuous joints at indices 0, 2, 4, 6.
- Build: `cmake -S . -B build -DCMAKE_PREFIX_PATH=/usr/local/lib/python3.10/dist-packages/cmeel.prefix && cmake --build build -j && ctest --test-dir build --output-on-failure`
- Tests run without a robot (`SimTransport`); URDF is injected at compile time via `-DURDF_PATH=…`.

## Plan-level decisions that deviate from the spec

Three, each flagged so a reviewer can reject them without reading the code:

1. **No `TwistTargetSink` abstract port.** The spec (Component 4) calls for one. But `kinova_arm_ros2` reaches the driver **only** through `StreamSink` — it never touches a mode — so the twist sink would have exactly one implementer and one caller (`Supervisor`). That is the abstraction-for-single-use the repo's coding discipline rejects, and it diverges from the precedent Plan 1 set, where `Supervisor` holds a concrete `JointTorqueMode&`. `JointVelocityMode` therefore exposes concrete `set_velocity_target()` / `set_twist_target()`. Extract an interface if a second velocity-consuming mode ever appears.
2. **Velocity limiting is uniform-scale-then-clamp, not per-joint clamp alone.** The spec says "clamp per joint to `max_qd`". A bare per-joint clamp silently **rotates** the commanded EE twist when one joint saturates — precisely the silent semantic difference decision 5 exists to forbid. Scaling the whole vector down so the fastest joint just reaches its cap preserves twist direction; the per-joint clamp is retained underneath as a hard backstop that holds even if `max_qd` contains a zero.
3. **`TargetSource` is spelled `kEntry`, not `kEntryPose`.** `JointPositionMode` captures a joint *configuration* at entry (`entry_q_`), not a pose; `kEntryPose` would be a false name.

## File structure

| File | Responsibility |
|---|---|
| `include/kinova_lowlevel/joint_velocity_mode.h` (**create**) | `JointVelocityParams` + `JointVelocityMode` declaration |
| `src/joint_velocity_mode.cpp` (**create**) | DLS twist map, null-space posture, limiting, staleness |
| `tests/joint_velocity_mode_test.cpp` (**create**) | Mode unit tests |
| `include/kinova_lowlevel/joint_position_mode.h` (**modify**) | Add `DiffIkSolver`, `TargetSource`, `ik_faulted_` |
| `src/joint_position_mode.cpp` (**modify**) | Pose→IK→target branch in `compute()` |
| `tests/joint_position_mode_test.cpp` (**modify**) | Pose path tests |
| `src/interface/streaming_session.cpp` (**modify**) | Flip three `pair_supported` rows |
| `include/kinova_lowlevel/interface/supervisor.h` + `src/interface/supervisor.cpp` (**modify**) | Hold `JointVelocityMode&`; implement the two velocity setpoint methods |
| `tests/interface/supervisor_test.cpp` (**modify**) | Streaming rows for velocity/twist and pose→position |
| `apps/benchmark_joint_velocity.cpp` (**create**) | Closes the spec's open benchmarking decision |
| `CMakeLists.txt` (**modify**) | Register the new source, test and benchmark |
| `docs/guide/streaming.md`, `docs/reference/api.md`, `mkdocs.yml` (**modify**) | Document the mode and the newly-supported pairs |

---

### Task 1: `JointVelocityMode` — skeleton, native joint-velocity target, limiting, staleness

Builds the mode with **no Jacobian and no twist**. Task 2 adds the twist path. This split exists so the limiting and staleness contracts are locked by tests before any kinematics can obscure them.

**Files:**
- Create: `include/kinova_lowlevel/joint_velocity_mode.h`
- Create: `src/joint_velocity_mode.cpp`
- Create: `tests/joint_velocity_mode_test.cpp`
- Modify: `CMakeLists.txt` (add source to `kinova_lowlevel`, test to `unit_tests`)

**Interfaces:**
- Consumes: `ControlMode`, `Dynamics::velocity_limits`, `CommandWatchdog`, `JointVec`, `ActuatorMode::kVelocity`.
- Produces: `struct JointVelocityParams`; `class JointVelocityMode` with `set_velocity_target(const JointVec&) noexcept`, `set_params(const JointVelocityParams&) noexcept`, `set_command_timeout(double) noexcept`, `JointVelocityParams params() const noexcept`, `JointVec commanded() const noexcept`. Tasks 2, 5 and 6 depend on these exact names.

- [ ] **Step 1: Write the failing tests**

Create `tests/joint_velocity_mode_test.cpp`:

```cpp
#include <gtest/gtest.h>
#include "kinova_lowlevel/joint_velocity_mode.h"

using namespace kinova;

namespace {
JointFeedback fb_at(const JointVec& q) { JointFeedback fb; fb.q = q; return fb; }
}  // namespace

TEST(JointVelocityMode, RequiresVelocityOnEveryActuator) {
  Dynamics dyn(URDF_PATH);
  JointVelocityMode m(dyn);
  for (auto mode : m.required_modes()) EXPECT_EQ(mode, ActuatorMode::kVelocity);
}

TEST(JointVelocityMode, CommandsZeroBeforeAnyTarget) {
  Dynamics dyn(URDF_PATH);
  JointVelocityMode m(dyn);
  m.on_enter(fb_at(JointVec::Zero()));
  JointCommand out;
  m.compute(fb_at(JointVec::Zero()), 0.001, out);
  EXPECT_EQ(out.mode, ActuatorMode::kVelocity);
  EXPECT_TRUE(out.velocity.isZero());
}

TEST(JointVelocityMode, TracksJointVelocityTargetExactly) {
  Dynamics dyn(URDF_PATH);
  JointVelocityMode m(dyn);
  m.on_enter(fb_at(JointVec::Zero()));
  JointVec qd = JointVec::Constant(0.2);
  m.set_velocity_target(qd);
  JointCommand out;
  m.compute(fb_at(JointVec::Zero()), 0.001, out);
  // Stiff mode: what was asked for is what is commanded, no shaping.
  EXPECT_TRUE(out.velocity.isApprox(qd, 1e-12));
}

TEST(JointVelocityMode, SeedsMaxQdFromUrdfWhenLeftNonFinite) {
  Dynamics dyn(URDF_PATH);
  JointVec v_max; dyn.velocity_limits(v_max);
  JointVelocityMode m(dyn);              // default params: max_qd all +inf
  const JointVelocityParams p = m.params();
  for (int i = 0; i < kNumJoints; ++i) EXPECT_DOUBLE_EQ(p.max_qd[i], v_max[i]);
}

TEST(JointVelocityMode, ClampsARequestAboveTheUrdfLimitDownToIt) {
  Dynamics dyn(URDF_PATH);
  JointVec v_max; dyn.velocity_limits(v_max);
  JointVelocityParams p;
  p.max_qd = JointVec::Constant(1e3);    // absurd request
  JointVelocityMode m(dyn, p);
  for (int i = 0; i < kNumJoints; ++i) EXPECT_DOUBLE_EQ(m.params().max_qd[i], v_max[i]);
}

TEST(JointVelocityMode, ScalesUniformlySoDirectionSurvivesSaturation) {
  Dynamics dyn(URDF_PATH);
  JointVelocityParams p;
  p.max_qd = JointVec::Constant(0.1);
  JointVelocityMode m(dyn, p);
  m.on_enter(fb_at(JointVec::Zero()));
  JointVec qd = JointVec::Zero();
  qd[0] = 1.0; qd[1] = 0.5;              // 2:1 ratio, both over the cap
  m.set_velocity_target(qd);
  JointCommand out;
  m.compute(fb_at(JointVec::Zero()), 0.001, out);
  EXPECT_NEAR(out.velocity[0], 0.1, 1e-12);
  EXPECT_NEAR(out.velocity[1], 0.05, 1e-12);   // ratio preserved, not clamped to 0.1
  EXPECT_LE(out.velocity.cwiseAbs().maxCoeff(), 0.1 + 1e-12);
}

TEST(JointVelocityMode, CommandsZeroWhenTheStreamGoesStale) {
  Dynamics dyn(URDF_PATH);
  JointVelocityParams p; p.cmd_timeout_s = 0.1;
  JointVelocityMode m(dyn, p);
  m.on_enter(fb_at(JointVec::Zero()));
  m.set_velocity_target(JointVec::Constant(0.2));
  JointCommand out;
  m.compute(fb_at(JointVec::Zero()), 0.001, out);
  ASSERT_FALSE(out.velocity.isZero());
  for (int i = 0; i < 150; ++i) m.compute(fb_at(JointVec::Zero()), 0.001, out);
  EXPECT_TRUE(out.velocity.isZero());     // stale -> zero, per decision 6
}

TEST(JointVelocityMode, DisarmingTheWatchdogDoesNotResurrectAStaleTarget) {
  Dynamics dyn(URDF_PATH);
  JointVelocityParams p; p.cmd_timeout_s = 0.1;
  JointVelocityMode m(dyn, p);
  m.on_enter(fb_at(JointVec::Zero()));
  m.set_velocity_target(JointVec::Constant(0.2));
  JointCommand out;
  for (int i = 0; i < 150; ++i) m.compute(fb_at(JointVec::Zero()), 0.001, out);
  ASSERT_TRUE(out.velocity.isZero());
  m.set_command_timeout(0.0);             // disarm
  m.compute(fb_at(JointVec::Zero()), 0.001, out);
  EXPECT_TRUE(out.velocity.isZero());     // the freeze is LATCHED
}

TEST(JointVelocityMode, AFreshTargetReleasesTheFreeze) {
  Dynamics dyn(URDF_PATH);
  JointVelocityParams p; p.cmd_timeout_s = 0.1;
  JointVelocityMode m(dyn, p);
  m.on_enter(fb_at(JointVec::Zero()));
  m.set_velocity_target(JointVec::Constant(0.2));
  JointCommand out;
  for (int i = 0; i < 150; ++i) m.compute(fb_at(JointVec::Zero()), 0.001, out);
  ASSERT_TRUE(out.velocity.isZero());
  m.set_velocity_target(JointVec::Constant(0.15));
  m.compute(fb_at(JointVec::Zero()), 0.001, out);
  EXPECT_NEAR(out.velocity[0], 0.15, 1e-12);
}

TEST(JointVelocityMode, OnEnterDropsATargetSentBeforeEntry) {
  Dynamics dyn(URDF_PATH);
  JointVelocityMode m(dyn);
  m.set_velocity_target(JointVec::Constant(0.3));   // sent BEFORE entry
  m.on_enter(fb_at(JointVec::Zero()));
  JointCommand out;
  m.compute(fb_at(JointVec::Zero()), 0.001, out);
  EXPECT_TRUE(out.velocity.isZero());
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `cmake --build build -j 2>&1 | tail -20`
Expected: FAIL — `joint_velocity_mode.h: No such file or directory`.

- [ ] **Step 3: Write the header**

Create `include/kinova_lowlevel/joint_velocity_mode.h`:

```cpp
#pragma once
#include <atomic>
#include <limits>
#include <Eigen/Cholesky>
#include "kinova_lowlevel/cartesian_types.h"
#include "kinova_lowlevel/command_watchdog.h"
#include "kinova_lowlevel/control_mode.h"
#include "kinova_lowlevel/dynamics.h"
namespace kinova {

struct JointVelocityParams {
  // Per-joint cap on commanded velocity [rad/s]. Non-finite entries are seeded
  // from the URDF; finite entries are clamped DOWN to it. No configuration can
  // ask for more than the hardware is rated for.
  JointVec max_qd = JointVec::Constant(std::numeric_limits<double>::infinity());

  // Baseline Levenberg-Marquardt damping for the 6x7 twist solve.
  double dls_damping = 1e-3;
  // Manipulability w = sqrt(det(J J^T)) below which damping is raised toward
  // dls_damping_max. This is the ONLY thing keeping commanded velocity bounded
  // near a singularity: in a torque law a bad solve produces a bounded torque,
  // but here whatever is computed goes straight to the actuators.
  double w_threshold     = 0.02;
  double dls_damping_max = 0.10;

  // Null-space posture bias [1/s]. Without it the redundant DOF drifts and the
  // elbow wanders while the tool tracks the twist perfectly.
  double posture_gain = 0.5;
  JointVec q_rest =   // elbow-up home; matches DiffIkParams::q_rest
      (JointVec() << 0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57).finished();

  // Staleness watchdog. 0 DISABLES it, matching every other mode's default.
  double cmd_timeout_s = 0.0;
};

// Joint-space velocity control. Commands every actuator in kVelocity and lets the
// actuator's own servo close the loop.
//
// STIFF BY CONTRACT. This mode does not yield to contact and makes no attempt to.
// A compliant velocity law is a DIFFERENT promise and belongs in a different mode
// -- delivering compliance from something named "velocity" is exactly the silent
// semantic difference this driver refuses to ship.
//
// Two target shapes:
//   set_velocity_target(qd)  - native; commanded through unchanged (then limited)
//   set_twist_target(V)      - EE twist [linear; angular] in the base frame,
//                              mapped by damped least squares + null-space posture
//
// Staleness commands ZERO velocity and LATCHES until a fresh target arrives.
//
// Live setters publish via a single-writer (non-RT) double-buffer; compute()
// (RT thread) reads one snapshot per cycle.
class JointVelocityMode : public ControlMode {
 public:
  JointVelocityMode(Dynamics& dyn, JointVelocityParams p = {});
  ActuatorModes required_modes() const override;
  void on_enter(const JointFeedback& fb) override;
  void compute(const JointFeedback& fb, double dt_s, JointCommand& out) override;
  void on_exit() override {}

  // Non-RT setters (call from one supervisor thread). Latest setter wins: a twist
  // target supersedes a joint-velocity target and vice-versa.
  void set_velocity_target(const JointVec& qd_d) noexcept;
  void set_twist_target(const Vector6& V) noexcept;
  void set_params(const JointVelocityParams& p) noexcept;

  // s >= 0 arms with s; s < 0 restores params().cmd_timeout_s.
  void set_command_timeout(double s) noexcept;

  // ACTIVE parameters, i.e. after URDF seeding and clamping -- not necessarily
  // what was passed in. Read this back when a speed request seems ignored.
  JointVelocityParams params() const noexcept;

  // RT-thread-owned state, for tests and post-stop inspection. NOT synchronized.
  JointVec commanded() const noexcept { return qd_cmd_; }
  // Manipulability at the last twist solve, sqrt(det(J J^T)). 0 until a twist
  // has been solved. Worth watching: it is what drives the damping.
  double last_manipulability() const noexcept { return w_last_; }

 private:
  void seed_limits(JointVelocityParams& p) const noexcept;
  // RT: J(q) -> damped least squares -> null-space posture. Writes qd_out.
  void solve_twist(const JointVec& q, const Vector6& V,
                   const JointVelocityParams& p, JointVec& qd_out) noexcept;
  // RT: uniform scale so the fastest joint just reaches its cap, then a hard
  // per-joint clamp as a backstop.
  static void limit(const JointVelocityParams& p, JointVec& qd) noexcept;

  Dynamics& dyn_;
  JointVec v_max_urdf_ = JointVec::Zero();   // cached in ctor: set_params must not
                                             // touch Dynamics off the RT thread
  std::array<bool, kNumJoints> continuous_{};

  JointVelocityParams params_[2];
  std::atomic<int> params_active_{0};

  enum class Source : int { kNone, kJoint, kTwist };
  std::atomic<Source> source_{Source::kNone};
  JointVec ext_qd_[2];
  std::atomic<int> qd_active_{0};
  Vector6 ext_twist_[2];
  std::atomic<int> tw_active_{0};

  CommandWatchdog wd_;
  // RT-owned. Latched rather than recomputed per cycle so DISARMING the watchdog
  // cannot un-freeze and resurrect a target nobody is maintaining.
  bool frozen_ = false;

  // RT-owned adopted targets and preallocated scratch.
  JointVec qd_target_ = JointVec::Zero();
  Vector6  twist_target_ = Vector6::Zero();
  JointVec qd_cmd_ = JointVec::Zero();
  Jacobian6 J_ = Jacobian6::Zero();
  Eigen::Matrix<double, 6, 6> A_ = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt_;
  Vector6  y_ = Vector6::Zero();
  JointVec bias_ = JointVec::Zero();
  double   w_last_ = 0.0;
};

}  // namespace kinova
```

- [ ] **Step 4: Write the implementation (no twist path yet)**

Create `src/joint_velocity_mode.cpp`:

```cpp
#include "kinova_lowlevel/joint_velocity_mode.h"
#include <algorithm>
#include <cmath>
#include "kinova_lowlevel/units.h"
namespace kinova {

JointVelocityMode::JointVelocityMode(Dynamics& dyn, JointVelocityParams p)
    : dyn_(dyn) {
  // Cache the URDF limits once. set_params runs on a non-RT thread and must never
  // touch Dynamics -- it is not thread-safe against the RT loop.
  JointVec lo, hi;
  dyn.joint_limits(lo, hi);
  dyn.velocity_limits(v_max_urdf_);
  for (int i = 0; i < kNumJoints; ++i)
    continuous_[i] = !std::isfinite(lo[i]) && !std::isfinite(hi[i]);
  seed_limits(p);
  params_[0] = p;
  params_[1] = p;
  ext_qd_[0].setZero(); ext_qd_[1].setZero();
  ext_twist_[0].setZero(); ext_twist_[1].setZero();
  wd_.arm(p.cmd_timeout_s);
}

void JointVelocityMode::seed_limits(JointVelocityParams& p) const noexcept {
  for (int i = 0; i < kNumJoints; ++i) {
    const double v = std::isfinite(p.max_qd[i]) ? p.max_qd[i] : v_max_urdf_[i];
    // A caller may ask for less than the hardware can do, never for more. A
    // negative request stops the joint rather than reversing it, which would also
    // violate std::clamp's lo <= hi precondition below.
    p.max_qd[i] = std::clamp(v, 0.0, v_max_urdf_[i]);
  }
}

ActuatorModes JointVelocityMode::required_modes() const {
  ActuatorModes modes; modes.fill(ActuatorMode::kVelocity); return modes;
}

JointVelocityParams JointVelocityMode::params() const noexcept {
  return params_[params_active_.load(std::memory_order_acquire)];
}

void JointVelocityMode::set_params(const JointVelocityParams& p) noexcept {
  const int next = 1 - params_active_.load(std::memory_order_relaxed);
  params_[next] = p;
  seed_limits(params_[next]);
  params_active_.store(next, std::memory_order_release);
}

void JointVelocityMode::set_velocity_target(const JointVec& qd_d) noexcept {
  const int next = 1 - qd_active_.load(std::memory_order_relaxed);
  ext_qd_[next] = qd_d;
  qd_active_.store(next, std::memory_order_release);
  source_.store(Source::kJoint, std::memory_order_release);
  wd_.bump();   // must be LAST: its release publishes everything above it
}

void JointVelocityMode::set_twist_target(const Vector6& V) noexcept {
  const int next = 1 - tw_active_.load(std::memory_order_relaxed);
  ext_twist_[next] = V;
  tw_active_.store(next, std::memory_order_release);
  source_.store(Source::kTwist, std::memory_order_release);
  wd_.bump();   // must be LAST
}

void JointVelocityMode::set_command_timeout(double s) noexcept {
  wd_.arm(s >= 0.0 ? s : params().cmd_timeout_s);
}

void JointVelocityMode::on_enter(const JointFeedback&) {
  // Drop any target from a previous session: re-entering must not resume a motion
  // someone asked for minutes ago.
  source_.store(Source::kNone, std::memory_order_release);
  qd_target_.setZero();
  twist_target_.setZero();
  qd_cmd_.setZero();
  frozen_ = false;
  w_last_ = 0.0;
  wd_.reset();
}

void JointVelocityMode::limit(const JointVelocityParams& p, JointVec& qd) noexcept {
  // Scale UNIFORMLY so the fastest joint just reaches its cap. A bare per-joint
  // clamp would silently ROTATE the commanded EE twist when one joint saturates,
  // which is the one thing a mode named "velocity" must not do.
  double s = 1.0;
  for (int i = 0; i < kNumJoints; ++i) {
    const double a = std::abs(qd[i]);
    if (a > p.max_qd[i] && a > 0.0) s = std::min(s, p.max_qd[i] / a);
  }
  qd *= s;
  // Hard backstop: scaling covers the normal case, this holds even when max_qd
  // contains a zero (scale would be 0/0) or the scale underflows.
  for (int i = 0; i < kNumJoints; ++i)
    qd[i] = std::clamp(qd[i], -p.max_qd[i], p.max_qd[i]);
}

void JointVelocityMode::compute(const JointFeedback& fb, double dt_s,
                                JointCommand& out) {
  const JointVelocityParams p = params();   // own a snapshot for the whole cycle
  out.mode = ActuatorMode::kVelocity;

  // Staleness: the stream stopped, so stop moving. Zero is the only safe command
  // for a stiff velocity mode -- holding the last velocity would keep the arm
  // travelling toward nothing. LATCHED, so disarming cannot un-freeze it.
  const bool stale = wd_.tick(dt_s);
  if (stale) frozen_ = true;
  else if (wd_.fresh()) frozen_ = false;

  const Source src = source_.load(std::memory_order_acquire);
  if (frozen_ || src == Source::kNone) {
    qd_cmd_.setZero();
    out.velocity = qd_cmd_;
    return;
  }

  // Adopt the payload exactly when the counter moves, never merely because the
  // stream is not yet stale -- otherwise a target published before on_enter would
  // be picked up after it.
  if (src == Source::kJoint) {
    if (wd_.fresh()) qd_target_ = ext_qd_[qd_active_.load(std::memory_order_acquire)];
    qd_cmd_ = qd_target_;
  } else {
    if (wd_.fresh()) twist_target_ = ext_twist_[tw_active_.load(std::memory_order_acquire)];
    solve_twist(fb.q, twist_target_, p, qd_cmd_);
  }

  limit(p, qd_cmd_);
  out.velocity = qd_cmd_;
}

// Task 2 fills this in.
void JointVelocityMode::solve_twist(const JointVec&, const Vector6&,
                                    const JointVelocityParams&,
                                    JointVec& qd_out) noexcept {
  qd_out.setZero();
}

}  // namespace kinova
```

- [ ] **Step 5: Register in CMake**

In `CMakeLists.txt`, add `src/joint_velocity_mode.cpp` to the `kinova_lowlevel` source list (next to `src/joint_position_mode.cpp`), and `tests/joint_velocity_mode_test.cpp` to the `add_executable(unit_tests ...)` list (next to `tests/joint_position_mode_test.cpp`).

- [ ] **Step 6: Run tests to verify they pass**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='JointVelocityMode*'`
Expected: all 10 PASS.

- [ ] **Step 7: Commit**

```bash
git add include/kinova_lowlevel/joint_velocity_mode.h src/joint_velocity_mode.cpp \
        tests/joint_velocity_mode_test.cpp CMakeLists.txt
git commit -m "feat(modes): JointVelocityMode — native joint-velocity target, URDF-seeded limiting"
```

---

### Task 2: The damped-least-squares twist map

**Files:**
- Modify: `src/joint_velocity_mode.cpp` (fill in `solve_twist`)
- Modify: `tests/joint_velocity_mode_test.cpp` (append)

**Interfaces:**
- Consumes: `Dynamics::jacobian(q, J_out)` → `Jacobian6` (6×7, `LOCAL_WORLD_ALIGNED`), `JointVelocityParams::{dls_damping, w_threshold, dls_damping_max}`, `JointVelocityMode::last_manipulability()`.
- Produces: nothing new; fills the existing private `solve_twist`.

- [ ] **Step 1: Write the failing tests**

Append to `tests/joint_velocity_mode_test.cpp`:

```cpp
namespace {
// A comfortably non-singular, elbow-up configuration.
JointVec nominal_q() {
  return (JointVec() << 0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57).finished();
}
// Straight-arm: the classic wrist/elbow singularity for this arm.
JointVec straight_q() { return JointVec::Zero(); }
}  // namespace

TEST(JointVelocityModeTwist, ReproducesTheCommandedTwistAwayFromSingularities) {
  Dynamics dyn(URDF_PATH);
  JointVelocityParams p;
  p.posture_gain = 0.0;               // isolate the task term
  p.max_qd = JointVec::Constant(10.0);   // clamped to URDF, but well above need
  JointVelocityMode m(dyn, p);
  const JointVec q = nominal_q();
  m.on_enter(fb_at(q));

  Vector6 V = Vector6::Zero();
  V[0] = 0.05;                        // 5 cm/s along base x
  m.set_twist_target(V);
  JointCommand out;
  m.compute(fb_at(q), 0.001, out);

  Jacobian6 J; dyn.jacobian(q, J);
  const Vector6 achieved = J * out.velocity;
  // Light damping means near-exact reproduction, not exact.
  EXPECT_NEAR((achieved - V).norm(), 0.0, 1e-3);
}

TEST(JointVelocityModeTwist, StaysBoundedAtAStraightArmSingularity) {
  Dynamics dyn(URDF_PATH);
  JointVelocityParams p;
  p.max_qd = JointVec::Constant(1e3);    // seeded/clamped to URDF anyway
  JointVelocityMode m(dyn, p);
  const JointVec q = straight_q();
  m.on_enter(fb_at(q));

  Vector6 V = Vector6::Zero();
  V[2] = 0.10;                        // push along the degenerate direction
  m.set_twist_target(V);
  JointCommand out;
  m.compute(fb_at(q), 0.001, out);

  EXPECT_TRUE(out.velocity.allFinite());
  JointVec v_max; dyn.velocity_limits(v_max);
  for (int i = 0; i < kNumJoints; ++i)
    EXPECT_LE(std::abs(out.velocity[i]), v_max[i] + 1e-12);
}

TEST(JointVelocityModeTwist, ManipulabilityIsLowerAtTheSingularity) {
  Dynamics dyn(URDF_PATH);
  JointVelocityMode m(dyn);
  Vector6 V = Vector6::Zero(); V[2] = 0.05;
  JointCommand out;

  m.on_enter(fb_at(nominal_q()));
  m.set_twist_target(V);
  m.compute(fb_at(nominal_q()), 0.001, out);
  const double w_nominal = m.last_manipulability();

  m.on_enter(fb_at(straight_q()));
  m.set_twist_target(V);
  m.compute(fb_at(straight_q()), 0.001, out);
  const double w_singular = m.last_manipulability();

  EXPECT_GT(w_nominal, w_singular);
  // Records the real numbers so w_threshold's default is grounded in measurement
  // rather than guessed. Update the default if this ever prints far from it.
  std::printf("w_nominal=%.6f w_singular=%.6f\n", w_nominal, w_singular);
}

TEST(JointVelocityModeTwist, AZeroTwistCommandsZero) {
  Dynamics dyn(URDF_PATH);
  JointVelocityParams p; p.posture_gain = 0.0;
  JointVelocityMode m(dyn, p);
  m.on_enter(fb_at(nominal_q()));
  m.set_twist_target(Vector6::Zero());
  JointCommand out;
  m.compute(fb_at(nominal_q()), 0.001, out);
  EXPECT_NEAR(out.velocity.norm(), 0.0, 1e-9);
}
```

- [ ] **Step 2: Run to verify they fail**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='JointVelocityModeTwist*'`
Expected: FAIL — `solve_twist` returns zero, so `ReproducesTheCommandedTwist` fails on the norm and `ManipulabilityIsLowerAtTheSingularity` gets 0 for both.

- [ ] **Step 3: Implement `solve_twist`**

Replace the stub in `src/joint_velocity_mode.cpp`:

```cpp
void JointVelocityMode::solve_twist(const JointVec& q, const Vector6& V,
                                    const JointVelocityParams& p,
                                    JointVec& qd_out) noexcept {
  dyn_.jacobian(q, J_);
  A_.noalias() = J_ * J_.transpose();          // 6x6, symmetric positive semi-definite

  // Decompose UNDAMPED first, purely to measure conditioning: LDLT hands us
  // det(J J^T) as prod(D) for free, so manipulability costs no extra solve.
  ldlt_.compute(A_);
  const double w2 = ldlt_.vectorD().prod();
  w_last_ = w2 > 0.0 ? std::sqrt(w2) : 0.0;

  // Damping rises as manipulability falls. This is a REQUIRED part of the design,
  // not a refinement: in velocity mode whatever is computed goes to the actuators,
  // so there is no torque clamp standing behind a bad solve.
  double lambda = p.dls_damping;
  if (p.w_threshold > 0.0 && w_last_ < p.w_threshold) {
    const double r = 1.0 - w_last_ / p.w_threshold;   // 0 at threshold, 1 at singular
    lambda = p.dls_damping + (p.dls_damping_max - p.dls_damping) * r * r;
  }
  A_.diagonal().array() += lambda * lambda;
  ldlt_.compute(A_);

  // Task term: qd = J^T (J J^T + lambda^2 I)^-1 V
  y_.noalias() = ldlt_.solve(V);
  qd_out.noalias() = J_.transpose() * y_;

  if (p.posture_gain == 0.0) return;

  // Null-space posture bias, projected without ever forming the 7x7 projector:
  //   (I - J^T (JJ^T + lambda^2 I)^-1 J) b  ==  b - J^T ((JJ^T + lambda^2 I)^-1 (J b))
  // Continuous joints must take the SHORT way to the rest posture, or the bias
  // pushes the joint most of a turn the wrong way.
  for (int i = 0; i < kNumJoints; ++i) {
    double d = p.q_rest[i] - q[i];
    if (continuous_[i]) d = wrap_to_pi(d);
    bias_[i] = p.posture_gain * d;
  }
  y_.noalias() = ldlt_.solve(J_ * bias_);
  qd_out.noalias() += bias_ - J_.transpose() * y_;
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `./build/unit_tests --gtest_filter='JointVelocityMode*'`
Expected: all PASS. Note the printed `w_nominal`/`w_singular` values.

- [ ] **Step 5: Ground `w_threshold` in the measured numbers**

Read the printed manipulabilities. If `w_threshold`'s default of `0.02` does not sit between `w_singular` and `w_nominal`, edit the default in `joint_velocity_mode.h` so it does — roughly one tenth of `w_nominal` — and note the two measured values in the comment above the field. Re-run the tests.

- [ ] **Step 6: Commit**

```bash
git add src/joint_velocity_mode.cpp include/kinova_lowlevel/joint_velocity_mode.h \
        tests/joint_velocity_mode_test.cpp
git commit -m "feat(modes): map EE twist to joint velocity by damped least squares"
```

---

### Task 3: Null-space posture holding

Task 2 implemented the posture term; this task proves it does what it is for, which is a separate claim from "the twist is reproduced".

**Files:**
- Modify: `tests/joint_velocity_mode_test.cpp` (append)

**Interfaces:** none new.

- [ ] **Step 1: Write the failing tests**

Append to `tests/joint_velocity_mode_test.cpp`:

```cpp
TEST(JointVelocityModePosture, DrivesTheRedundantDofTowardQRestUnderAZeroTwist) {
  Dynamics dyn(URDF_PATH);
  JointVelocityParams p;
  p.posture_gain = 0.5;
  p.max_qd = JointVec::Constant(10.0);
  JointVelocityMode m(dyn, p);

  JointVec q = nominal_q();
  q[2] += 0.30;                         // perturb the redundant DOF off the rest posture
  m.on_enter(fb_at(q));
  m.set_twist_target(Vector6::Zero());  // hold the tool still

  JointCommand out;
  m.compute(fb_at(q), 0.001, out);
  // The posture term must push joint 2 back DOWN toward q_rest.
  EXPECT_LT(out.velocity[2], 0.0);
  EXPECT_GT(out.velocity.norm(), 1e-6);   // it is not simply doing nothing
}

TEST(JointVelocityModePosture, DoesNotDisturbTheCommandedTwist) {
  Dynamics dyn(URDF_PATH);
  JointVelocityParams p;
  p.posture_gain = 0.5;
  p.max_qd = JointVec::Constant(10.0);
  JointVelocityMode m(dyn, p);

  JointVec q = nominal_q();
  q[2] += 0.30;                         // a posture error the null-space term will fight
  m.on_enter(fb_at(q));
  Vector6 V = Vector6::Zero(); V[1] = 0.04;
  m.set_twist_target(V);
  JointCommand out;
  m.compute(fb_at(q), 0.001, out);

  Jacobian6 J; dyn.jacobian(q, J);
  const Vector6 achieved = J * out.velocity;
  // The whole point of a NULL-space term: posture correction lives where it
  // cannot show up in the task velocity.
  EXPECT_NEAR((achieved - V).norm(), 0.0, 1e-3);
}

TEST(JointVelocityModePosture, ConvergesTowardQRestOverRepeatedCycles) {
  Dynamics dyn(URDF_PATH);
  JointVelocityParams p;
  p.posture_gain = 0.5;
  p.max_qd = JointVec::Constant(10.0);
  JointVelocityMode m(dyn, p);

  JointVec q = nominal_q();
  q[2] += 0.30;
  const double err0 = std::abs(q[2] - p.q_rest[2]);
  m.on_enter(fb_at(q));

  JointCommand out;
  for (int i = 0; i < 500; ++i) {       // integrate the commanded velocity forward
    m.set_twist_target(Vector6::Zero());
    m.compute(fb_at(q), 0.001, out);
    q += out.velocity * 0.001;
  }
  EXPECT_LT(std::abs(q[2] - p.q_rest[2]), err0);
}
```

- [ ] **Step 2: Run tests**

Run: `./build/unit_tests --gtest_filter='JointVelocityModePosture*'`
Expected: PASS — the implementation from Task 2 already covers this. If `DrivesTheRedundantDofTowardQRest` fails on sign, the projector subtraction in `solve_twist` has its sign inverted; if `DoesNotDisturbTheCommandedTwist` fails, `bias_` is being added outside the projection.

- [ ] **Step 3: Commit**

```bash
git add tests/joint_velocity_mode_test.cpp
git commit -m "test(modes): null-space posture holds the redundant DOF without disturbing the twist"
```

---

### Task 4: `JointPositionMode` gains an EE-pose path

**Files:**
- Modify: `include/kinova_lowlevel/joint_position_mode.h`
- Modify: `src/joint_position_mode.cpp`
- Modify: `tests/joint_position_mode_test.cpp` (append)

**Interfaces:**
- Consumes: `DiffIkSolver(Dynamics&, DiffIkParams)`, `DiffIkSolver::solve(const Pose&, JointVec&) -> IkResult`, `PoseTargetSink::set_target(const Pose&)`.
- Produces: `JointPositionMode` additionally implements `PoseTargetSink`; new accessors `IkResult last_ik() const noexcept` and `bool ik_faulted() const noexcept`; new `JointPositionParams` fields `DiffIkParams ik` and `double ik_fault_s`.

- [ ] **Step 1: Write the failing tests**

Append to `tests/joint_position_mode_test.cpp`:

```cpp
TEST(JointPositionModePose, APoseTargetDrivesTheReferenceTowardIt) {
  Dynamics dyn(URDF_PATH);
  JointPositionMode m(dyn);
  const JointVec q0 = (JointVec() << 0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57).finished();
  JointFeedback fb; fb.q = q0;
  m.on_enter(fb);

  Pose target = dyn.fk(q0);
  target.p.x() += 0.05;                 // 5 cm along base x
  m.set_target(target);

  JointCommand out;
  for (int i = 0; i < 500; ++i) { m.compute(fb, 0.001, out); fb.q = out.position; }

  const Pose reached = dyn.fk(out.position);
  EXPECT_LT((reached.p - target.p).norm(), (dyn.fk(q0).p - target.p).norm());
}

TEST(JointPositionModePose, AJointTargetStillBypassesIkEntirely) {
  Dynamics dyn(URDF_PATH);
  JointPositionMode m(dyn);
  JointFeedback fb; fb.q = JointVec::Zero();
  m.on_enter(fb);
  const JointVec q_d = JointVec::Constant(0.1);
  m.set_target(q_d);                    // JointTargetSink overload
  JointCommand out;
  m.compute(fb, 0.001, out);
  // iters stays 0: a joint target must not pay for a solve it does not use.
  EXPECT_EQ(m.last_ik().iters, 0);
}

TEST(JointPositionModePose, TheLatestSetterWins) {
  Dynamics dyn(URDF_PATH);
  JointPositionMode m(dyn);
  const JointVec q0 = (JointVec() << 0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57).finished();
  JointFeedback fb; fb.q = q0;
  m.on_enter(fb);

  Pose pose = dyn.fk(q0); pose.p.x() += 0.05;
  m.set_target(pose);
  JointCommand out;
  m.compute(fb, 0.001, out);
  ASSERT_GT(m.last_ik().iters, 0);

  m.set_target(q0);                     // joint target supersedes the pose
  m.compute(fb, 0.001, out);
  EXPECT_EQ(m.last_ik().iters, 0);
}

TEST(JointPositionModePose, SustainedNonConvergenceRaisesIkFaulted) {
  Dynamics dyn(URDF_PATH);
  JointPositionParams p;
  p.ik_fault_s = 0.05;
  JointPositionMode m(dyn, p);
  const JointVec q0 = (JointVec() << 0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57).finished();
  JointFeedback fb; fb.q = q0;
  m.on_enter(fb);

  Pose unreachable = dyn.fk(q0);
  unreachable.p.x() += 5.0;             // metres away: never reachable
  m.set_target(unreachable);

  JointCommand out;
  m.compute(fb, 0.001, out);
  EXPECT_FALSE(m.ik_faulted());         // one bad solve is NOT a fault
  for (int i = 0; i < 100; ++i) m.compute(fb, 0.001, out);
  EXPECT_TRUE(m.ik_faulted());          // sustained non-convergence IS
}

TEST(JointPositionModePose, AConvergedSolveClearsTheFaultAccumulator) {
  Dynamics dyn(URDF_PATH);
  JointPositionParams p;
  p.ik_fault_s = 0.05;
  JointPositionMode m(dyn, p);
  const JointVec q0 = (JointVec() << 0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57).finished();
  JointFeedback fb; fb.q = q0;
  m.on_enter(fb);

  Pose unreachable = dyn.fk(q0); unreachable.p.x() += 5.0;
  m.set_target(unreachable);
  JointCommand out;
  for (int i = 0; i < 40; ++i) m.compute(fb, 0.001, out);   // under the threshold
  ASSERT_FALSE(m.ik_faulted());

  m.set_target(dyn.fk(q0));             // reachable: converges immediately
  for (int i = 0; i < 40; ++i) m.compute(fb, 0.001, out);
  EXPECT_FALSE(m.ik_faulted());
}

TEST(JointPositionModePose, OnEnterClearsAPreviousIkFault) {
  Dynamics dyn(URDF_PATH);
  JointPositionParams p;
  p.ik_fault_s = 0.05;
  JointPositionMode m(dyn, p);
  const JointVec q0 = (JointVec() << 0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57).finished();
  JointFeedback fb; fb.q = q0;
  m.on_enter(fb);
  Pose unreachable = dyn.fk(q0); unreachable.p.x() += 5.0;
  m.set_target(unreachable);
  JointCommand out;
  for (int i = 0; i < 100; ++i) m.compute(fb, 0.001, out);
  ASSERT_TRUE(m.ik_faulted());

  m.on_enter(fb);
  EXPECT_FALSE(m.ik_faulted());
}
```

- [ ] **Step 2: Run to verify they fail**

Run: `cmake --build build -j 2>&1 | tail -20`
Expected: FAIL — no `set_target(const Pose&)` overload, no `last_ik()`, no `ik_faulted()`, no `ik_fault_s`.

- [ ] **Step 3: Update the header**

In `include/kinova_lowlevel/joint_position_mode.h`:

Add includes `#include "kinova_lowlevel/diff_ik.h"` and `#include "kinova_lowlevel/pose_target_sink.h"`.

Append to `JointPositionParams`:

```cpp
  // Sustained IK non-convergence threshold [s]. A single non-converged solve is
  // NOT a fault -- a momentarily unreachable pose is normal while a client servos
  // toward something. Expressed in TIME, not cycles: compute() runs at whatever
  // the loop rate is, so a cycle count would silently mean something different at
  // a different rate. <= 0 disables.
  double ik_fault_s = 0.1;
  DiffIkParams ik{};
```

Change the class declaration and add members:

```cpp
class JointPositionMode : public ControlMode,
                          public JointTargetSink,
                          public PoseTargetSink {
```

Add to the public section:

```cpp
  // Cartesian target: resolved to a joint reference by in-loop IK, which then
  // feeds the SAME rate_limit -> leash -> wrap -> clamp pipeline a joint target
  // does, so the whole safety envelope comes along unchanged (PoseTargetSink).
  void set_target(const Pose& x_d) noexcept override;
  using JointTargetSink::set_target;   // keep the JointVec overload visible

  IkResult last_ik() const noexcept { return last_ik_; }
  // Set when IK has failed to converge for longer than ik_fault_s. Published for
  // the sampler thread: the mode cannot end a streaming session (modes know
  // nothing about the interface layer), so it freezes the reference immediately
  // at 1 kHz and lets the lifecycle teardown catch up.
  bool ik_faulted() const noexcept { return ik_faulted_.load(std::memory_order_acquire); }
```

Replace the `has_ext_target_` member block with:

```cpp
  // Target source, selected by the most recent setter (single-writer, non-RT).
  // Named kEntry rather than kEntryPose because what is captured at entry is a
  // joint CONFIGURATION, not a pose.
  enum class TargetSource : int { kEntry, kPose, kJoint };
  JointVec entry_q_ = JointVec::Zero();
  JointVec ext_target_[2];
  std::atomic<int> ext_active_{0};
  Pose pose_target_[2];
  std::atomic<int> pose_active_{0};
  std::atomic<TargetSource> source_{TargetSource::kEntry};
```

And add alongside the existing members:

```cpp
  Dynamics& dyn_;
  DiffIkSolver ik_;
  IkResult last_ik_{};
  double ik_bad_s_ = 0.0;                  // RT-owned: summed dt while !converged
  std::atomic<bool> ik_faulted_{false};    // RT writer, non-RT (sampler) reader
  JointVec ik_q_ = JointVec::Zero();       // preallocated RT scratch for the solve
```

Update the class docstring: the existing claim that the mode *"runs no dynamics at all — no gravity term, no mass matrix, no IK… the cheapest control path in the driver"* is now conditional. Replace that sentence with:

```
// With a JOINT target this still runs no dynamics at all -- no gravity term, no
// mass matrix, no IK -- and remains the cheapest control path in the driver. Only
// a live POSE target pulls in the in-loop IK solve.
```

- [ ] **Step 4: Update the implementation**

In `src/joint_position_mode.cpp`:

Constructor — take and store `Dynamics&`, construct the solver, and seed the IK limits:

```cpp
JointPositionMode::JointPositionMode(Dynamics& dyn, JointPositionParams p)
    : dyn_(dyn), ik_(dyn, p.ik) {
```

Extend `seed_limits` so the solver's limits are never left unbounded, mirroring `JointImpedanceMode`:

```cpp
    if (!std::isfinite(p.ik.q_lower[i])) p.ik.q_lower[i] = q_lower_urdf_[i];
    if (!std::isfinite(p.ik.q_upper[i])) p.ik.q_upper[i] = q_upper_urdf_[i];
```

and after the loop in `set_params`, push the refreshed IK params into the solver:
`ik_.set_params(params_[next].ik);`

**The constructor needs the same push.** `ik_(dyn, p.ik)` runs in the initialiser list, *before* the body calls `seed_limits(p)` — so the solver would otherwise hold the unseeded (infinite) joint limits for the life of the mode. Add `ik_.set_params(params_[0].ik);` at the end of the constructor body, after `params_[0] = p;`.

Replace `set_target(const JointVec&)`'s `has_ext_target_` store with
`source_.store(TargetSource::kJoint, std::memory_order_release);`
and add the pose overload:

```cpp
void JointPositionMode::set_target(const Pose& x_d) noexcept {
  const int next = 1 - pose_active_.load(std::memory_order_relaxed);
  pose_target_[next] = x_d;
  pose_active_.store(next, std::memory_order_release);
  source_.store(TargetSource::kPose, std::memory_order_release);
  wd_.bump();   // must be LAST: its release publishes everything above it
}
```

In `on_enter`, replace the `has_ext_target_` reset:

```cpp
  source_.store(TargetSource::kEntry, std::memory_order_release);
  ik_bad_s_ = 0.0;
  ik_faulted_.store(false, std::memory_order_release);
  last_ik_ = IkResult{};
```

In `compute`, replace the `const JointVec target = …` selection with:

```cpp
  JointVec target;
  if (stale) {
    target = fb.q;
  } else {
    switch (source_.load(std::memory_order_acquire)) {
      case TargetSource::kJoint:
        target = ext_target_[ext_active_.load(std::memory_order_acquire)];
        last_ik_ = IkResult{};   // last_ik() means THIS cycle's solve; without the
                                 // reset a stale result from a previous pose target
                                 // outlives the target itself and reads as an IK
                                 // that never ran having run.
        break;
      case TargetSource::kPose: {
        // Warm-start from the current reference: the solve refines in place, and
        // seeding from q_ref_ keeps the solution on the branch we are already on.
        ik_q_ = q_ref_;
        last_ik_ = ik_.solve(pose_target_[pose_active_.load(std::memory_order_acquire)],
                             ik_q_);
        // Sustained non-convergence is a fault; a single miss is not. Accumulated
        // clock-free from the dt the caller already has, exactly like the watchdog.
        if (last_ik_.converged) {
          ik_bad_s_ = 0.0;
        } else if (p.ik_fault_s > 0.0) {
          ik_bad_s_ += dt_s;
          if (ik_bad_s_ >= p.ik_fault_s)
            ik_faulted_.store(true, std::memory_order_release);
        }
        // Position mode is STIFF. Holding a stale reference while the client
        // believes it is tracking is the silent divergence this driver exists to
        // fail loud on -- so freeze at the measured configuration immediately and
        // let the sampler tear the session down.
        target = ik_faulted_.load(std::memory_order_relaxed) ? fb.q : ik_q_;
        break;
      }
      case TargetSource::kEntry:
      default:
        target = entry_q_;
        last_ik_ = IkResult{};
        break;
    }
  }
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='JointPositionMode*'`
Expected: all PASS, including the pre-existing `JointPositionMode` tests — the joint-target path must be untouched.

- [ ] **Step 6: Commit**

```bash
git add include/kinova_lowlevel/joint_position_mode.h src/joint_position_mode.cpp \
        tests/joint_position_mode_test.cpp
git commit -m "feat(modes): JointPositionMode accepts an EE pose via in-loop IK"
```

---

### Task 5: Wire both into the Supervisor and open the `pair_supported` rows

**Files:**
- Modify: `src/interface/streaming_session.cpp`
- Modify: `include/kinova_lowlevel/interface/supervisor.h`
- Modify: `src/interface/supervisor.cpp`
- Modify: `tests/interface/supervisor_test.cpp` (append)
- Modify: `tests/interface/streaming_session_test.cpp` (append)

**Interfaces:**
- Consumes: `JointVelocityMode::{set_velocity_target, set_twist_target, set_command_timeout}`, `JointPositionMode::set_target(const Pose&)`.
- Produces: `Supervisor` constructor widens to
  `Supervisor(JointPositionMode& pos, JointImpedanceMode& imp, JointTorqueMode& tau, JointVelocityMode& vel, RtExecutor& exec, Seqlock<JointFeedback>& snap, Dynamics& pump_dyn, StreamPort& stream, ActionServerPort& action, SupervisorConfig cfg = {})`.
  **This breaks `kinova_arm_ros2`'s `bringup_node.cpp` — flagged deliberately.**

- [ ] **Step 1: Write the failing tests**

Append to `tests/interface/streaming_session_test.cpp`:

```cpp
TEST(PairSupported, VelocityKindsAreNowSupportedInVelocityMode) {
  EXPECT_TRUE(pair_supported(SetpointKind::kJointVelocity, ControlModeKind::kVelocity));
  EXPECT_TRUE(pair_supported(SetpointKind::kEeTwist,       ControlModeKind::kVelocity));
}

TEST(PairSupported, EePoseIsNowSupportedInPositionMode) {
  EXPECT_TRUE(pair_supported(SetpointKind::kEePose, ControlModeKind::kPosition));
}

TEST(PairSupported, VelocityKindsAreStillRefusedInEveryOtherMode) {
  for (auto m : {ControlModeKind::kPosition, ControlModeKind::kImpedance,
                 ControlModeKind::kTorque}) {
    EXPECT_FALSE(pair_supported(SetpointKind::kJointVelocity, m));
    EXPECT_FALSE(pair_supported(SetpointKind::kEeTwist, m));
  }
}

TEST(PairSupported, VelocityModeRefusesEveryNonVelocityKind) {
  for (auto k : {SetpointKind::kJointPosition, SetpointKind::kEePose,
                 SetpointKind::kJointTorque}) {
    EXPECT_FALSE(pair_supported(k, ControlModeKind::kVelocity));
  }
}
```

Append to `tests/interface/supervisor_test.cpp`. **Match the file's existing idiom exactly**: plain `TEST(Supervisor, …)` with a local `SupFix f;`, a running RT thread, and assertions taken **after** `f.teardown()` — `JointVelocityMode::commanded()` is RT-owned and explicitly not synchronized, so reading it while the loop runs is a race. Do **not** use `TEST_F`; `SupFix` is a plain struct in an anonymous namespace, and a fixture-style test will not compile.

```cpp
TEST(Supervisor, StreamingJointVelocityDrivesTheVelocityMode) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::StreamOpenRequest r;
  r.kind = interface::SetpointKind::kJointVelocity;
  r.control_mode = interface::ControlModeKind::kVelocity;
  r.timeout_s = 1.0;
  ASSERT_TRUE(f.sup.on_stream_open(r).accepted);

  interface::JointSetpoint sp; sp.values = JointVec::Constant(0.05);
  for (int i = 0; i < 20; ++i) {
    f.sup.on_setpoint_joint_velocity(sp);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  interface::StreamCloseRequest c;
  f.sup.on_stream_close(c);
  f.sup.stop(); f.teardown();

  // Read only after the RT thread is joined.
  EXPECT_GT(f.vel.commanded().cwiseAbs().maxCoeff(), 0.0);
}

TEST(Supervisor, StreamingATwistDrivesTheVelocityMode) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::StreamOpenRequest r;
  r.kind = interface::SetpointKind::kEeTwist;
  r.control_mode = interface::ControlModeKind::kVelocity;
  r.timeout_s = 1.0;
  ASSERT_TRUE(f.sup.on_stream_open(r).accepted);

  interface::TwistSetpoint sp; sp.twist = Vector6::Zero(); sp.twist[0] = 0.02;
  for (int i = 0; i < 20; ++i) {
    f.sup.on_setpoint_twist(sp);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  interface::StreamCloseRequest c;
  f.sup.on_stream_close(c);
  f.sup.stop(); f.teardown();

  EXPECT_GT(f.vel.last_manipulability(), 0.0);   // the DLS solve actually ran
}

TEST(Supervisor, StreamingAPoseIntoPositionModeIsAccepted) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::StreamOpenRequest r;
  r.kind = interface::SetpointKind::kEePose;
  r.control_mode = interface::ControlModeKind::kPosition;
  r.timeout_s = 1.0;
  ASSERT_TRUE(f.sup.on_stream_open(r).accepted);

  interface::PoseSetpoint sp; sp.pose = f.pump_dyn.fk(f.init.q);
  sp.pose.p.x() += 0.02;
  for (int i = 0; i < 20; ++i) {
    f.sup.on_setpoint_pose(sp);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  interface::StreamCloseRequest c;
  f.sup.on_stream_close(c);
  f.sup.stop(); f.teardown();

  EXPECT_GT(f.pos.last_ik().iters, 0);           // IK ran in position mode
}

TEST(Supervisor, RefusesAJointPositionSetpointInVelocityMode) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::StreamOpenRequest r;
  r.kind = interface::SetpointKind::kJointPosition;
  r.control_mode = interface::ControlModeKind::kVelocity;
  r.timeout_s = 1.0;
  EXPECT_FALSE(f.sup.on_stream_open(r).accepted);
  f.sup.stop(); f.teardown();
}
```

- [ ] **Step 2: Run to verify they fail**

Run: `cmake --build build -j 2>&1 | tail -20`
Expected: FAIL — `pair_supported` still returns false for those rows; `vel` is not a member of the fixture.

- [ ] **Step 3: Open the `pair_supported` rows**

In `src/interface/streaming_session.cpp`:

```cpp
bool pair_supported(SetpointKind k, ControlModeKind m) {
  switch (k) {
    case SetpointKind::kJointPosition:
      return m == ControlModeKind::kPosition || m == ControlModeKind::kImpedance;
    case SetpointKind::kEePose:
      return m == ControlModeKind::kImpedance || m == ControlModeKind::kPosition;
    case SetpointKind::kJointTorque:
      return m == ControlModeKind::kTorque;
    case SetpointKind::kJointVelocity:
    case SetpointKind::kEeTwist:
      return m == ControlModeKind::kVelocity;
  }
  return false;
}
```

- [ ] **Step 4: Widen the Supervisor**

In `include/kinova_lowlevel/interface/supervisor.h`: add `#include "kinova_lowlevel/joint_velocity_mode.h"`, add `JointVelocityMode& vel` as the fourth constructor parameter, and add the member `JointVelocityMode& vel_;`.

In `src/interface/supervisor.cpp`: store it in the initialiser list, then make the four edits below. **Follow the file's existing idiom exactly** — every `on_setpoint_*` takes `stream_mtx_`, admits through `session_.admit(kind, secs_since(t0_))`, and resolves the destination through a mode-kind map rather than an inline conditional.

**(a) Replace the two empty stubs** (currently commented "until Plan 2 adds JointVelocityMode"):

```cpp
void Supervisor::on_setpoint_joint_velocity(const JointSetpoint& s) {
  std::lock_guard<std::mutex> l(stream_mtx_);
  if (!session_.admit(SetpointKind::kJointVelocity, secs_since(t0_))) return;
  if (session_.control_mode() == ControlModeKind::kVelocity) vel_.set_velocity_target(s.values);
}
void Supervisor::on_setpoint_twist(const TwistSetpoint& s) {
  std::lock_guard<std::mutex> l(stream_mtx_);
  if (!session_.admit(SetpointKind::kEeTwist, secs_since(t0_))) return;
  if (session_.control_mode() == ControlModeKind::kVelocity) vel_.set_twist_target(s.twist);
}
```

**(b) Add a `pose_sink_for` helper next to `sink_for`**, and use it. `on_setpoint_pose` currently hardcodes `imp_.set_target(s.pose)` under the comment *"only impedance has a PoseTargetSink"* — Task 4 makes that comment false, and an inline ternary would recreate exactly the "binary impedance-or-else" that `sink_for`'s comment records as having been wrong before:

```cpp
kinova::PoseTargetSink* Supervisor::pose_sink_for(ControlModeKind k) {
  switch (k) {
    case ControlModeKind::kImpedance: return &imp_;
    case ControlModeKind::kPosition:  return &pos_;   // Plan 2: position gained IK
    case ControlModeKind::kVelocity:
    case ControlModeKind::kTorque:    return nullptr;
  }
  return nullptr;
}

void Supervisor::on_setpoint_pose(const PoseSetpoint& s) {
  std::lock_guard<std::mutex> l(stream_mtx_);
  if (!session_.admit(SetpointKind::kEePose, secs_since(t0_))) return;
  // nullptr means the running mode has no pose sink, so the setpoint is dropped
  // rather than written into a mode the executor is not running.
  if (kinova::PoseTargetSink* sink = pose_sink_for(session_.control_mode()))
    sink->set_target(s.pose);
}
```

Declare `pose_sink_for` in `supervisor.h` beside `sink_for`.

**(c) `sink_for` gains a `kVelocity` arm returning `nullptr`.** `JointVelocityMode` is deliberately **not** a `JointTargetSink` — it has no position target. It falls in the same bucket as `kTorque`: no joint sink, so the drain loop's hold-at-measured-q has nothing to latch. If `sink_for` switches exhaustively over `ControlModeKind` without a `default`, adding `kVelocity` is required for the build to stay warning-clean.

**(d) Velocity mode's safe-stop is zero, and it must be latched on close.** In `close_stream()`, where the other modes' targets are restored, add `vel_.set_velocity_target(JointVec::Zero())` so a closed session cannot leave the arm coasting on its last commanded velocity. Arm and disarm `vel_.set_command_timeout()` alongside the other modes on stream open and close.

- [ ] **Step 5: Add `vel` to the test fixture**

In `tests/interface/supervisor_test.cpp`, add the member to `SupFix` next to the other modes and widen the `Supervisor` construction in place:

```cpp
  JointPositionMode pos{dyn};
  JointImpedanceMode imp{dyn};
  JointTorqueMode tau{dyn};
  JointVelocityMode vel{dyn};                     // NEW
  RtExecutor exec{tap, ring, {1000.0, kinova::Pacing::kSleepSpin, {}}};
  FakeBackend be;
  interface::Supervisor sup{pos, imp, tau, vel, exec, snap, pump_dyn, be, be};
```

There are **three other construction sites**, all of which must be widened or the build breaks:

- `tests/rt_safety_test.cpp:362` and `:445` — `Supervisor sup(pos, imp, tau, ex, snap, pump_dyn, be, be);`
- `apps/stream_check.cpp:260` — `Supervisor sup(pos, imp, tau, ex, snap, pump_dyn, backend, backend);`

Each needs a `JointVelocityMode vel{dyn};` declared alongside its existing modes and passed as the fourth argument. `stream_check` only needs to **compile** — extending its `--kind`/`--mode` table to the new pairs is out of scope for this plan.

Confirm nothing was missed: `grep -rn "Supervisor sup\|interface::Supervisor " tests/ apps/ src/` should show every site taking ten arguments.

- [ ] **Step 6: Run the whole suite**

Run: `cmake --build build -j && ctest --test-dir build --output-on-failure`
Expected: all PASS. The pre-existing streaming tests that asserted the three rows were *refused* will now fail — update those assertions to expect acceptance; do not delete the tests.

- [ ] **Step 7: Commit**

```bash
git add src/interface/streaming_session.cpp include/kinova_lowlevel/interface/supervisor.h \
        src/interface/supervisor.cpp tests/interface/
git commit -m "feat(interface): stream velocity, twist and pose-into-position setpoints"
```

---

### Task 6: Benchmark, RT-safety coverage, docs

Closes the spec's Open Decision: *"How to measure `JointVelocityMode`'s per-cycle cost… this must be resolved in Plan 2, not left open."* A dedicated binary is chosen over a flag on `benchmark_cartesian_impedance` because a binary named for Cartesian impedance that runs velocity mode is a misnomer, and the spec sanctions either.

**Files:**
- Create: `apps/benchmark_joint_velocity.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/rt_safety_test.cpp`
- Modify: `docs/guide/streaming.md`, `docs/reference/api.md`, `mkdocs.yml`

**Interfaces:** none new.

- [ ] **Step 1: Add RT-safety coverage for the new path**

In `tests/rt_safety_test.cpp`, add a case that runs `JointVelocityMode` with a **twist** target (the path with the Jacobian and both LDLT decompositions — the only genuinely new RT cost in this plan) through the same steady-state harness the other modes use, asserting zero major page faults and zero dropped samples. Model it exactly on the existing `CartesianImpedanceMode` case in that file.

- [ ] **Step 2: Run it**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='RtSafety*'`
Expected: PASS. **If it fails on page faults, `Eigen::LDLT::compute` is allocating** — replace the two `ldlt_.compute()` calls with an explicit fixed-size `Eigen::LLT` on the damped matrix only, and re-derive manipulability from `A_.determinant()`.

- [ ] **Step 3: Write the benchmark**

Create `apps/benchmark_joint_velocity.cpp`, mirroring the structure of `apps/benchmark_cartesian_impedance.cpp` exactly: same argument parsing (`--sim`, `--urdf`, `--rate`, `--duration`, `--csv`), same `SampleRing` drain and percentile reporting. Drive `JointVelocityMode` with a slowly rotating unit twist so the DLS solve runs every cycle rather than short-circuiting on a zero target, and add a `--kind {joint,twist}` flag so the native path and the twist path can be measured separately — the difference between them **is** the DLS + null-space cost.

- [ ] **Step 4: Register it in CMake**

```cmake
add_executable(benchmark_joint_velocity apps/benchmark_joint_velocity.cpp)
target_link_libraries(benchmark_joint_velocity PRIVATE kinova_lowlevel Eigen3::Eigen)
```

- [ ] **Step 5: Measure and record**

Run:
```sh
./build/benchmark_joint_velocity --sim --urdf models/gen3_7dof_2f85.urdf --rate 1000 --duration 5 --kind joint
./build/benchmark_joint_velocity --sim --urdf models/gen3_7dof_2f85.urdf --rate 1000 --duration 5 --kind twist
```
Record p50/p99/p99.9/max, overruns and faults for both in the commit message. "Looks fine" is not evidence for a driver this much depends on.

- [ ] **Step 6: Update the docs**

- `docs/guide/streaming.md` — the valid-pair table now has **no** unsupported rows for velocity or pose-into-position; remove the "needs Plan 2" markers. State plainly that velocity mode is stiff and does not yield to contact, that a stale stream commands zero, and that the twist map preserves direction under saturation by scaling rather than clamping per joint.
- `docs/reference/api.md` — add `JointVelocityMode` and `JointPositionMode`'s `PoseTargetSink` overload.
- Add a `docs/deep-dive/` page for the DLS twist map: the `qd = J^T (JJ^T + λ²I)^-1 V` derivation, why damping is scheduled on manipulability, and the projector-free null-space form.
- `mkdocs.yml` — add the new deep-dive page to `nav:`, or it will not appear on the site.

- [ ] **Step 7: Full suite and commit**

```bash
cmake --build build -j && ctest --test-dir build --output-on-failure
git add apps/benchmark_joint_velocity.cpp CMakeLists.txt tests/rt_safety_test.cpp \
        docs/ mkdocs.yml
git commit -m "feat(bench): measure JointVelocityMode's per-cycle cost; document the streaming pairs"
```

---

## Self-review

**Spec coverage.** Component 3 (pose path) → Task 4, including the time-based `ik_fault_s` threshold, the `ik_faulted_` atomic for the sampler, the freeze-immediately behaviour, and the docstring debt the spec calls out by name. Component 4 (`JointVelocityMode`) → Tasks 1–3: `kVelocity` (T1), DLS (T2), null-space (T3), URDF-seeded clamp (T1), staleness→zero (T1), singularity boundedness as a required test (T2). Decision 5 (stiff) → the class docstring and `TracksJointVelocityTargetExactly`. Decision 6 (staleness) → T1. Open Decision on benchmarking → T6. `pair_supported` rows → T5.

**Gaps deliberately left.** `SetpointKind::kEeWrench` and a `CartesianWrenchMode` are out of scope — `kinova_arm_ros2` already carries that as a known gap (`core_backed=false`). Adding an `ee_pose_position` row to that repo's `registry()` is a one-line follow-up there, not here.

**Type consistency.** `set_velocity_target`/`set_twist_target` are used identically in T1, T5 and T6. `params().max_qd` is the seeded value everywhere. `last_ik()` returns `IkResult` in both T4's tests and the header. The `Supervisor` constructor's fourth parameter is `JointVelocityMode&` in T5's Interfaces block, the header edit and the fixture edit.

**Known breakage this plan causes.** `kinova_arm_ros2/src/bringup_node.cpp:147` constructs `Supervisor` with nine arguments; T5 makes it ten. Intentional and called out in T5's Interfaces block.
