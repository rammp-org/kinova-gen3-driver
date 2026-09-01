# Gripper Core Plumbing — Implementation Plan (Gripper Plan 1)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Carry everything the 2F-85 accepts and reports across the transport boundary — position, speed and force out; position, velocity, effort, current and presence back — and promote the app-local `GripperInjector` into a library `GripperController`.

**Architecture:** `JointCommand::gripper` and `JointFeedback::gripper` widen from bare scalars to fixed-size POD structs. `KortexTransport` stops pinning speed and force to constants and starts reading the three feedback fields it currently ignores. `GripperController` decorates `Transport`, stamping the command on the way past — the gripper never touches `ControlMode` or `RtExecutor`, so it stays orthogonal to control modes.

**Tech Stack:** C++17, fixed-size POD value types, GoogleTest (one `unit_tests` binary), CMake, KORTEX SDK 2.8.0 (aarch64).

**Spec:** `docs/superpowers/specs/2026-09-01-gripper-tier-design.md` — decomposition stage 1. Decisions 2, 3 and 5 bind this plan; decisions 1 and 4 (arbitration, the Grasp action) are stage 2 and explicitly **not** in scope here.

## Global Constraints

- **SI / normalized units internally.** Every gripper quantity crossing the driver's interfaces is 0..1 except `GripperFeedback::current`, which is amps. KORTEX's percent (0..100) conversion happens **only** inside `KortexTransport`, the same rule the arm's degrees/radians conversion follows.
- **`force` is a ceiling, not a setpoint.** It limits motor current; the gripper closes at `speed` toward `position` and stalls at the limit. No force servo exists on this hardware by any path.
- **Nothing in the RT path may allocate, lock, or block.** `GripperController` adds one fixed-size struct copy per `exchange`. The known exception is inherited and unchanged: `KortexTransport` lazily allocates the interconnect submessage on the first commanded cycle.
- **Non-RT setters publish via a single-writer double-buffer + release-store**, and the RT reader takes one snapshot per cycle — the discipline every mode in this repo already follows.
- **Fail loud, never silent mis-mapping.** `GripperFeedback::present` exists because today "no gripper attached" and "gripper fully open" are both reported as position 0.
- **Defaults must preserve today's behaviour exactly.** `speed = 1.0` and `force = 0.5` are the current hardcoded `kGripperVelocityPct = 100.0f` and `kGripperForcePct = 50.0f`. A caller that sets only position must produce a byte-identical wire command to today's.
- **Commands are stateless.** Speed and force do not persist between commands; every command carries all three fields. This follows the streaming tier's rule that a setpoint is a command and never an increment.

## Build and test — this machine cannot build this project

No Pinocchio, wrong architecture. Builds run on `abra` (aarch64 Jetson):

```sh
.superpowers/rbuild.sh                 # build + FULL suite
.superpowers/rbuild.sh 'Gripper*'      # build + filtered gtest
```

It rsyncs the **working tree** (not commits) to abra and builds there; its exit status is trustworthy. Verified green on this branch before Task 1.

**Task 3 additionally needs a KORTEX-enabled compile**, because `src/kortex_transport.cpp` is only compiled when `KINOVA_ENABLE_KORTEX=ON` and is therefore invisible to the default build. Its step gives the exact command.

## File structure

| File | Responsibility |
|---|---|
| `include/kinova_lowlevel/gripper_types.h` (**create**) | `GripperCommand` and `GripperFeedback` POD value types |
| `include/kinova_lowlevel/joint_types.h` (**modify**) | `JointCommand`/`JointFeedback` carry the new structs |
| `src/sim_transport.cpp` + header (**modify**) | echo position, derive velocity, expose a stall hook |
| `src/kortex_transport.cpp` (**modify**) | write speed/force; read velocity, current, presence |
| `include/kinova_lowlevel/gripper_controller.h` + `src/gripper_controller.cpp` (**create**) | the `Transport` decorator |
| `apps/teleop_socket_server.cpp` (**modify**) | retire the local `GripperInjector` |
| `tests/gripper_controller_test.cpp` (**create**) | decorator behaviour |
| `tests/sim_transport_test.cpp` (**modify**) | existing gripper cases, widened |
| `CMakeLists.txt` (**modify**) | new library source, new test file |

---

### Task 1: The value types, and widening `JointCommand` / `JointFeedback`

This is the breaking change. It lands in **one** task so the tree is never left half-migrated: every call site moves in the same commit.

**Files:**
- Create: `include/kinova_lowlevel/gripper_types.h`
- Modify: `include/kinova_lowlevel/joint_types.h`
- Modify: `src/sim_transport.cpp:27,39`
- Modify: `apps/teleop_socket_server.cpp:109-110,448`
- Modify: `tests/sim_transport_test.cpp:36-60`

**Interfaces:**
- Produces: `struct GripperCommand { float position; float speed; float force; bool active; }` and `struct GripperFeedback { float position; float velocity; float effort; float current; bool present; }`. `JointCommand::gripper` is now a `GripperCommand` (replacing `float gripper` + `bool gripper_active`); `JointFeedback::gripper` is now a `GripperFeedback` (replacing `float gripper`). Tasks 2, 3 and 4 all depend on these exact names.

- [ ] **Step 1: Write the failing test**

Append to `tests/sim_transport_test.cpp`:

```cpp
TEST(GripperTypes, CommandDefaultsMatchTodaysHardcodedConstants) {
  // speed 1.0 and force 0.5 ARE the old kGripperVelocityPct=100 / kGripperForcePct=50.
  // A caller that sets only position must produce what the driver sent before this change.
  GripperCommand c;
  EXPECT_FLOAT_EQ(c.position, 0.0f);
  EXPECT_FLOAT_EQ(c.speed,    1.0f);
  EXPECT_FLOAT_EQ(c.force,    0.5f);
  EXPECT_FALSE(c.active);
}

TEST(GripperTypes, FeedbackDefaultsToAbsentRatherThanOpen) {
  // present=false is the whole point: position 0 with no gripper attached must not
  // be indistinguishable from an attached, fully-open gripper.
  GripperFeedback f;
  EXPECT_FALSE(f.present);
  EXPECT_FLOAT_EQ(f.position, 0.0f);
  EXPECT_FLOAT_EQ(f.velocity, 0.0f);
  EXPECT_FLOAT_EQ(f.effort,   0.0f);
  EXPECT_FLOAT_EQ(f.current,  0.0f);
}
```

Then update the two existing gripper cases in the same file to the new shape — replacing, not deleting, them:

```cpp
TEST(SimTransport, EchoesGripperWhenActive) {
  JointFeedback init;
  SimTransport t(init);
  t.connect();
  JointCommand c;
  c.gripper.position = 0.42f;
  c.gripper.active   = true;
  JointFeedback fb;
  t.exchange(c, fb);
  EXPECT_NEAR(fb.gripper.position, 0.42f, 1e-6f);
}

TEST(SimTransport, LeavesGripperUntouchedWhenInactive) {
  JointFeedback init;
  init.gripper.position = 0.7f;   // pre-existing measured position
  SimTransport t(init);
  t.connect();
  JointCommand c;
  c.gripper.position = 0.1f;
  c.gripper.active   = false;     // no gripper intent
  JointFeedback fb;
  t.exchange(c, fb);
  EXPECT_NEAR(fb.gripper.position, 0.7f, 1e-6f);   // unchanged
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `.superpowers/rbuild.sh 'Gripper*'`
Expected: FAIL to compile — `gripper_types.h: No such file or directory`, and `'struct kinova::JointCommand' has no member named 'gripper'` of struct type.

- [ ] **Step 3: Write the value types**

Create `include/kinova_lowlevel/gripper_types.h`:

```cpp
#pragma once
namespace kinova {

// What the 2F-85 accepts, normalized. KORTEX speaks percent (0..100); the single
// conversion happens inside KortexTransport, exactly as the arm's degrees do.
struct GripperCommand {
  float position = 0.0f;   // 0 (open) .. 1 (closed)
  float speed    = 1.0f;   // fraction of max closing speed
  // A CEILING on motor current, not a force setpoint. The gripper closes at
  // `speed` toward `position` and stalls when it reaches this limit. No force
  // servo exists on this hardware -- GripperMode has no force mode at all, and
  // the high-level API that would host one needs SINGLE_LEVEL_SERVOING, which is
  // incompatible with the low-level servoing our 1 kHz torque control requires.
  float force    = 0.5f;   // fraction of max grip force
  // When false, no gripper command is emitted at all. This keeps the gripper limp
  // at startup rather than actuating it from a seeded default.
  bool  active   = false;
};

// What it reports back. NOTE the asymmetry with the command: MotorFeedback carries
// no force field, so `effort` is DERIVED from motor current and is a fraction of
// maximum, never Newtons. Publishing a number labelled in force units that is wrong
// by an unknown factor would be worse than publishing nothing.
struct GripperFeedback {
  float position = 0.0f;   // 0 (open) .. 1 (closed)
  float velocity = 0.0f;   // normalized; sign and scale TO CONFIRM on hardware
  float effort   = 0.0f;   // 0..1, derived from current
  float current  = 0.0f;   // amps, raw, exactly as reported
  // False when no interconnect gripper is attached. Without this, a missing gripper
  // and a fully-open one are both position 0 -- a silent mis-mapping.
  bool  present  = false;
};

}  // namespace kinova
```

- [ ] **Step 4: Widen the joint types**

In `include/kinova_lowlevel/joint_types.h`, add `#include "kinova_lowlevel/gripper_types.h"`, then:

In `JointFeedback`, replace `float gripper = 0.0f;` and its comment with:
```cpp
  GripperFeedback gripper{};
```

In `JointCommand`, replace `float gripper = 0.0f;` and `bool gripper_active = false;` with:
```cpp
  GripperCommand gripper{};
```

- [ ] **Step 5: Update every call site**

`src/sim_transport.cpp` — both `exchange` (line 27) and `send` (line 39):
```cpp
  if (cmd.gripper.active) state_.gripper.position = cmd.gripper.position;
```

`apps/teleop_socket_server.cpp` — in `GripperInjector::stamp` (lines 109-110):
```cpp
    out.gripper.active   = active_.load(std::memory_order_acquire);
    out.gripper.position = gripper_.load(std::memory_order_relaxed);
```
and the feedback packet (line 448):
```cpp
        pkt.gripper_state = fb.gripper.position;  // actual measured position from snapshot
```

Confirm nothing was missed: `grep -rn "gripper_active\|\.gripper\b" src/ include/ apps/ tests/` should show no bare-scalar uses left.

- [ ] **Step 6: Run tests to verify they pass**

Run: `.superpowers/rbuild.sh` (full suite — this change touches types every unit shares)
Expected: all PASS.

- [ ] **Step 7: Commit**

```bash
git add include/kinova_lowlevel/gripper_types.h include/kinova_lowlevel/joint_types.h \
        src/sim_transport.cpp apps/teleop_socket_server.cpp tests/sim_transport_test.cpp
git commit -m "refactor(types): gripper command and feedback become structs"
```

---

### Task 2: `SimTransport` derives gripper velocity and can be made to stall

The sim's gripper is currently an instant echo, which cannot express *moving* or *stalled* — the two things stage 2's grasp lifecycle needs to test. This task gives it a first-order approach and a stall hook.

**Files:**
- Modify: `include/kinova_lowlevel/sim_transport.h`
- Modify: `src/sim_transport.cpp`
- Modify: `tests/sim_transport_test.cpp` (append)

**Interfaces:**
- Consumes: `GripperCommand`, `GripperFeedback` from Task 1.
- Produces: `SimTransport::set_gripper_lag(float per_cycle_fraction)` and `SimTransport::set_gripper_blocked_at(float position)`. Stage 2's grasp tests depend on both.

- [ ] **Step 1: Write the failing test**

Append to `tests/sim_transport_test.cpp`:

```cpp
TEST(SimTransport, GripperApproachesTheTargetRatherThanTeleporting) {
  JointFeedback init;
  SimTransport t(init);
  t.connect();
  t.set_gripper_lag(0.25f);          // closes 25% of the remaining gap per cycle
  JointCommand c;
  c.gripper.position = 1.0f;
  c.gripper.active   = true;
  JointFeedback fb;
  t.exchange(c, fb);
  EXPECT_NEAR(fb.gripper.position, 0.25f, 1e-5f);
  t.exchange(c, fb);
  EXPECT_NEAR(fb.gripper.position, 0.4375f, 1e-5f);   // 0.25 + 0.75*0.25
  EXPECT_GT(fb.gripper.velocity, 0.0f);               // moving, and closing
}

TEST(SimTransport, GripperVelocityIsZeroWhenSettled) {
  JointFeedback init;
  SimTransport t(init);
  t.connect();
  t.set_gripper_lag(1.0f);           // reach the target immediately
  JointCommand c;
  c.gripper.position = 0.6f;
  c.gripper.active   = true;
  JointFeedback fb;
  t.exchange(c, fb);                 // moves 0 -> 0.6
  t.exchange(c, fb);                 // already there
  EXPECT_NEAR(fb.gripper.position, 0.6f, 1e-6f);
  EXPECT_NEAR(fb.gripper.velocity, 0.0f, 1e-6f);
}

TEST(SimTransport, ABlockedGripperStallsShortOfTheTargetAndLoadsUp) {
  JointFeedback init;
  SimTransport t(init);
  t.connect();
  t.set_gripper_lag(0.5f);
  t.set_gripper_blocked_at(0.4f);    // an object stops the fingers here
  JointCommand c;
  c.gripper.position = 1.0f;         // ask for fully closed
  c.gripper.active   = true;
  c.gripper.force    = 0.8f;
  JointFeedback fb;
  for (int i = 0; i < 20; ++i) t.exchange(c, fb);
  EXPECT_NEAR(fb.gripper.position, 0.4f, 1e-5f);      // stopped by the object
  EXPECT_NEAR(fb.gripper.velocity, 0.0f, 1e-5f);      // not moving
  EXPECT_NEAR(fb.gripper.effort, 0.8f, 1e-5f);        // loaded to the commanded cap
}

TEST(SimTransport, ReportsTheGripperAsPresent) {
  JointFeedback init;
  SimTransport t(init);
  t.connect();
  JointCommand c;
  JointFeedback fb;
  t.exchange(c, fb);
  EXPECT_TRUE(fb.gripper.present);   // the sim always has one
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `.superpowers/rbuild.sh 'SimTransport*'`
Expected: FAIL to compile — `'class kinova::SimTransport' has no member named 'set_gripper_lag'`.

- [ ] **Step 3: Extend the header**

In `include/kinova_lowlevel/sim_transport.h`, add to the public section:

```cpp
  // Test knobs for the gripper. The sim's gripper closes a FRACTION of the remaining
  // gap each cycle rather than teleporting, because "moving" and "stalled" are the two
  // states the grasp lifecycle needs to distinguish and an instant echo has neither.
  void set_gripper_lag(float per_cycle_fraction) { gripper_lag_ = per_cycle_fraction; }
  // Simulate an object: the fingers cannot close past this position, and effort rises
  // to the commanded force cap once they are stopped by it. Negative disables.
  void set_gripper_blocked_at(float position) { gripper_block_ = position; }
```

and to the private section:

```cpp
  float gripper_lag_   = 1.0f;    // default: reach the target in one cycle (old behaviour)
  float gripper_block_ = -1.0f;   // no object
```

- [ ] **Step 4: Implement the gripper step**

In `src/sim_transport.cpp`, add a helper above `exchange`:

```cpp
namespace {
constexpr float kSettledEps = 1e-6f;
}  // namespace

// Advance the simulated gripper one cycle toward its target. Extracted because
// exchange() and send() must step it identically -- they already duplicate the
// command latch, and a divergence here would make send-driven tests lie.
void SimTransport::step_gripper(const GripperCommand& g) {
  state_.gripper.present = true;
  if (!g.active) { state_.gripper.velocity = 0.0f; return; }

  float target = g.position;
  if (gripper_block_ >= 0.0f && target > gripper_block_) target = gripper_block_;

  const float before = state_.gripper.position;
  state_.gripper.position += (target - before) * gripper_lag_;
  state_.gripper.velocity = state_.gripper.position - before;

  // Loaded only when an object is what stopped us -- i.e. we are held at the block
  // while the caller is still asking for more. Reaching a freely-commanded target is
  // not a grasp and must not report effort, or every close looks like a grasp.
  const bool blocked_by_object =
      gripper_block_ >= 0.0f && g.position > gripper_block_ &&
      std::fabs(state_.gripper.position - gripper_block_) < kSettledEps;
  state_.gripper.effort  = blocked_by_object ? g.force : 0.0f;
  state_.gripper.current = state_.gripper.effort;   // sim: 1 A of "max" for a clean ratio
}
```

Declare it in the header's private section: `void step_gripper(const GripperCommand&);`
Add `#include <cmath>` to the source.

Replace the gripper line in **both** `exchange` and `send`:
```cpp
  step_gripper(cmd.gripper);
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `.superpowers/rbuild.sh`
Expected: all PASS, including the two Task 1 cases — `set_gripper_lag` defaults to 1.0, so the old instant-echo behaviour is preserved for every existing caller.

- [ ] **Step 6: Commit**

```bash
git add include/kinova_lowlevel/sim_transport.h src/sim_transport.cpp tests/sim_transport_test.cpp
git commit -m "feat(sim): the simulated gripper moves, settles, and can stall on an object"
```

---

### Task 3: `KortexTransport` sends speed and force, and reads what comes back

The only task whose code the default build never compiles. Its verification step is a KORTEX-enabled build, not a unit test.

**Files:**
- Modify: `src/kortex_transport.cpp:28-32` (constants), `:97-103` (feedback), `:141-158` (command)

**Interfaces:**
- Consumes: `GripperCommand`, `GripperFeedback` from Task 1.
- Produces: nothing new — this fills in an existing boundary.

- [ ] **Step 1: Replace the pinned constants**

In `src/kortex_transport.cpp`, replace the two `kGripper*Pct` constants with the normalizing constant, keeping the percent conversion local to this file:

```cpp
// KORTEX speaks percent (0..100) for gripper position, velocity and force; the driver
// speaks 0..1. This is the only place that conversion happens.
constexpr float kPctPerUnit = 100.0f;

// Normalizer for GripperFeedback::effort. MotorFeedback carries NO force field -- only
// current_motor -- so effort is |current| / this, a fraction of maximum rather than a
// force in Newtons. PROVENANCE: the 2F-85's rated stall current, pending measurement on
// the arm. Everything downstream is a fraction of it, so it is the one gripper number
// worth measuring early; see the spec's Open questions.
constexpr float kGripperMaxCurrentA = 0.8f;
```

- [ ] **Step 2: Read the three ignored feedback fields**

Replace the feedback block (currently lines 97-103):

```cpp
    // Gripper feedback from the interconnect. A robot with no interconnect gripper
    // reports zero motors -- record that as ABSENT rather than leaving position at 0,
    // which is indistinguishable from an attached, fully-open gripper.
    const auto& ic = fb_.interconnect();
    if (ic.gripper_feedback().motor_size() > 0) {
      const auto& m = ic.gripper_feedback().motor(0);
      fb.gripper.present  = true;
      fb.gripper.position = float(m.position()) / kPctPerUnit;
      fb.gripper.velocity = float(m.velocity()) / kPctPerUnit;
      fb.gripper.current  = float(m.current_motor());
      // MotorFeedback has no force field; effort is derived, normalized, and NOT Newtons.
      const float e = std::fabs(fb.gripper.current) / kGripperMaxCurrentA;
      fb.gripper.effort = e > 1.0f ? 1.0f : e;
    } else {
      fb.gripper.present = false;
    }
```

`<cmath>` is already included at the top of this file, so `std::fabs` needs no new include.

- [ ] **Step 3: Send all three command fields**

In `write_command`, replace the body of the `if (cmd.gripper_active)` block — the condition itself becomes `if (cmd.gripper.active)`, and the three `set_*` calls stop using constants:

```cpp
    if (cmd.gripper.active) {
      // NOTE: the interconnect/gripper submessages are allocated lazily on the first
      // commanded cycle -- a one-time heap alloc on the RT path. Deliberate: it keeps
      // the gripper UNcommanded until the first gripper command, so the gripper stays
      // limp at startup rather than being actuated by a seeded default. Pre-seeding in
      // set_servoing_low_level() would make this alloc-free but would actuate from
      // cycle 1; still to be validated on hardware.
      auto* gripper = cmd_.mutable_interconnect()->mutable_gripper_command();
      if (gripper->motor_cmd_size() == 0) gripper->add_motor_cmd();
      auto* m = gripper->mutable_motor_cmd(0);
      m->set_position(clamp01(cmd.gripper.position) * kPctPerUnit);
      m->set_velocity(clamp01(cmd.gripper.speed)    * kPctPerUnit);
      // A CEILING on motor current, not a force setpoint -- see GripperCommand.
      m->set_force(clamp01(cmd.gripper.force)       * kPctPerUnit);
    }
```

Add the helper to the anonymous namespace that already exists at `src/kortex_transport.cpp:24`, replacing the open-coded two-line clamp the old code used on position:

```cpp
inline float clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }
```

- [ ] **Step 4: Verify it compiles against the real SDK**

The default build never compiles this file. Build with KORTEX enabled on abra:

```sh
rsync -rlpgoD --checksum --delete --exclude '.git' --exclude 'build' \
  --exclude 'build_kortex' --exclude '.superpowers' \
  /home/swapnil/atdev/kinova-gen3-driver/ abra:/home/abra/sdd-build/
ssh abra 'cd /home/abra/sdd-build && cmake -S . -B build_kortex \
  -DCMAKE_PREFIX_PATH=/usr/local/lib/python3.10/dist-packages/cmeel.prefix \
  -DKINOVA_ENABLE_KORTEX=ON -DKORTEX_HW_DIR=$HOME/kortex_api_2.8.0_aarch64 \
  > /tmp/k_cfg.log 2>&1 && cmake --build build_kortex -j"$(nproc)" > /tmp/k_build.log 2>&1; \
  rc=$?; echo "rc=$rc"; [ $rc -ne 0 ] && grep -E "error:" /tmp/k_build.log | head -30'
```

Expected: `rc=0`. A non-zero result with `no member named 'current_motor'` means the field name differs in your SDK version — check `messages/GripperCyclicMessage.pb.h` and report rather than guessing.

- [ ] **Step 5: Run the default suite for regressions**

Run: `.superpowers/rbuild.sh`
Expected: all PASS. Nothing in the sim build changed, so this is a guard, not a proof.

- [ ] **Step 6: Commit**

```bash
git add src/kortex_transport.cpp
git commit -m "feat(transport): command gripper speed and force; read velocity, current, presence"
```

---

### Task 4: `GripperController` — the injector, promoted into the library

**Files:**
- Create: `include/kinova_lowlevel/gripper_controller.h`
- Create: `src/gripper_controller.cpp`
- Create: `tests/gripper_controller_test.cpp`
- Modify: `apps/teleop_socket_server.cpp:78-113,261,342` (retire the local class)
- Modify: `CMakeLists.txt:73-90` (library source), `:277-300` (test file)

**Interfaces:**
- Consumes: `GripperCommand` (Task 1), `Transport`, `SimTransport::set_gripper_lag` (Task 2).
- Produces: `class GripperController : public Transport` with `explicit GripperController(Transport& inner)`, `void set_target(const GripperCommand&) noexcept`, `void release() noexcept`, and `GripperCommand target() const noexcept`. Stage 2's `Supervisor` writes through exactly these.

- [ ] **Step 1: Write the failing test**

Create `tests/gripper_controller_test.cpp`:

```cpp
#include <gtest/gtest.h>
#include "kinova_lowlevel/gripper_controller.h"
#include "kinova_lowlevel/sim_transport.h"

using namespace kinova;

TEST(GripperController, StampsNothingBeforeTheFirstCommand) {
  JointFeedback init;
  SimTransport sim(init);
  GripperController gc(sim);
  gc.connect();
  JointCommand c;
  JointFeedback fb;
  gc.exchange(c, fb);
  // Untouched: the gripper stays limp at startup rather than being driven to a default.
  EXPECT_FALSE(sim.last_command().gripper.active);
}

TEST(GripperController, StampsThePositionItWasGiven) {
  JointFeedback init;
  SimTransport sim(init);
  GripperController gc(sim);
  gc.connect();
  GripperCommand g;
  g.position = 0.65f;
  gc.set_target(g);
  JointCommand c;
  JointFeedback fb;
  gc.exchange(c, fb);
  EXPECT_TRUE(sim.last_command().gripper.active);
  EXPECT_NEAR(sim.last_command().gripper.position, 0.65f, 1e-6f);
}

TEST(GripperController, APositionOnlyCommandCarriesTheDefaultSpeedAndForce) {
  JointFeedback init;
  SimTransport sim(init);
  GripperController gc(sim);
  gc.connect();
  GripperCommand g;
  g.position = 0.3f;               // speed and force left at their defaults
  gc.set_target(g);
  JointCommand c;
  JointFeedback fb;
  gc.exchange(c, fb);
  EXPECT_NEAR(sim.last_command().gripper.speed, 1.0f, 1e-6f);
  EXPECT_NEAR(sim.last_command().gripper.force, 0.5f, 1e-6f);
}

TEST(GripperController, SpeedAndForceDoNotPersistBetweenCommands) {
  // The statelessness decision, pinned. A caller that sets force once and then sends a
  // position-only command gets the DEFAULT force back -- not the earlier value. This is
  // the streaming tier's rule: a setpoint is a command, never an increment.
  JointFeedback init;
  SimTransport sim(init);
  GripperController gc(sim);
  gc.connect();
  JointCommand c;
  JointFeedback fb;

  GripperCommand strong;
  strong.position = 1.0f;
  strong.force    = 0.9f;
  gc.set_target(strong);
  gc.exchange(c, fb);
  ASSERT_NEAR(sim.last_command().gripper.force, 0.9f, 1e-6f);

  GripperCommand plain;
  plain.position = 0.5f;           // force not set
  gc.set_target(plain);
  gc.exchange(c, fb);
  EXPECT_NEAR(sim.last_command().gripper.force, 0.5f, 1e-6f);
}

TEST(GripperController, ReleaseStopsStampingAndDoesNotOpenTheGripper) {
  // The halt decision: e-stop means stop moving, and opening is itself a motion. The
  // hardware self-locks, so ceasing to command IS holding.
  JointFeedback init;
  SimTransport sim(init);
  GripperController gc(sim);
  gc.connect();
  GripperCommand g;
  g.position = 0.8f;
  gc.set_target(g);
  JointCommand c;
  JointFeedback fb;
  gc.exchange(c, fb);
  ASSERT_TRUE(sim.last_command().gripper.active);

  gc.release();
  gc.exchange(c, fb);
  EXPECT_FALSE(sim.last_command().gripper.active);   // no longer commanded
  // and specifically NOT commanded open:
  EXPECT_NE(sim.last_command().gripper.position, 0.0f);
}

TEST(GripperController, PassesFeedbackThroughUntouched) {
  JointFeedback init;
  init.q[0] = 0.25;
  SimTransport sim(init);
  GripperController gc(sim);
  gc.connect();
  JointCommand c;
  JointFeedback fb;
  gc.exchange(c, fb);
  EXPECT_NEAR(fb.q[0], 0.25, 1e-9);      // the decorator is transparent to arm state
  EXPECT_TRUE(fb.gripper.present);
}

TEST(GripperController, StampsSendTheSameWayAsExchange) {
  // send() and exchange() must stamp identically, or a send-driven caller silently
  // loses its gripper command.
  JointFeedback init;
  SimTransport sim(init);
  GripperController gc(sim);
  gc.connect();
  GripperCommand g;
  g.position = 0.42f;
  gc.set_target(g);
  JointCommand c;
  gc.send(c);
  EXPECT_TRUE(sim.last_command().gripper.active);
  EXPECT_NEAR(sim.last_command().gripper.position, 0.42f, 1e-6f);
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `.superpowers/rbuild.sh 'GripperController*'`
Expected: FAIL — `gripper_controller.h: No such file or directory`.

- [ ] **Step 3: Write the header**

Create `include/kinova_lowlevel/gripper_controller.h`:

```cpp
#pragma once
#include <atomic>
#include "kinova_lowlevel/gripper_types.h"
#include "kinova_lowlevel/transport.h"
namespace kinova {

// Stamps the gripper command into every outgoing JointCommand.
//
// A DECORATOR, not a ControlMode, and deliberately so. Modes are mutually exclusive,
// so making the gripper a mode would mean giving up arm control to move it. The gripper
// is not a control law -- it has no feedback term and no RT computation; it is a field
// on the outgoing frame. That makes the gripper ORTHOGONAL to control modes: you can
// grip during a trajectory, an impedance hold, or a velocity stream, and nothing about
// the gripper touches mode switching or widens the RT-safe surface.
//
// This is the same shape FeedbackTap uses to decorate Transport and Arbiter uses to
// decorate CommandSink.
//
// Threading: set_target() and release() belong to ONE non-RT thread. The stamp happens
// on the RT thread inside exchange()/send(). Published through a double-buffer with a
// release store, so the RT reader never observes a torn command.
class GripperController : public Transport {
 public:
  explicit GripperController(Transport& inner) : inner_(inner) {}

  // Non-RT. Latest wins; every call carries all three fields, because speed and force
  // are deliberately NOT sticky -- see the statelessness decision in the spec.
  void set_target(const GripperCommand& c) noexcept;

  // Non-RT, the halt path. Stops stamping. The 2F-85 is effectively self-locking, so
  // ceasing to command it holds the grip -- which is the point: e-stop means stop
  // moving, and opening would itself be a motion.
  void release() noexcept;

  // RT-thread-owned view of what is currently being stamped. NOT synchronized: for
  // tests and post-stop inspection only.
  GripperCommand target() const noexcept { return buf_[active_.load(std::memory_order_acquire)]; }

  void connect() override { inner_.connect(); }
  void set_servoing_low_level() override { inner_.set_servoing_low_level(); }
  void set_actuator_modes(const ActuatorModes& m) override { inner_.set_actuator_modes(m); }
  void exchange(const JointCommand& c, JointFeedback& fb) override { inner_.exchange(stamp(c), fb); }
  void send(const JointCommand& c) override { inner_.send(stamp(c)); }
  void receive(JointFeedback& fb) override { inner_.receive(fb); }
  void safe_shutdown() override { inner_.safe_shutdown(); }
  void clear_faults() override { inner_.clear_faults(); }

 private:
  JointCommand stamp(const JointCommand& c) const noexcept;

  Transport& inner_;
  GripperCommand buf_[2];
  std::atomic<int> active_{0};
  // Separate from buf_[].active so release() can stop stamping without destroying the
  // target -- the last commanded position stays readable for diagnostics.
  std::atomic<bool> stamping_{false};
};

}  // namespace kinova
```

- [ ] **Step 4: Write the implementation**

Create `src/gripper_controller.cpp`:

```cpp
#include "kinova_lowlevel/gripper_controller.h"
namespace kinova {

void GripperController::set_target(const GripperCommand& c) noexcept {
  const int next = 1 - active_.load(std::memory_order_relaxed);
  buf_[next] = c;
  active_.store(next, std::memory_order_release);
  stamping_.store(true, std::memory_order_release);   // LAST: publishes the buffer above it
}

void GripperController::release() noexcept {
  stamping_.store(false, std::memory_order_release);
}

JointCommand GripperController::stamp(const JointCommand& c) const noexcept {
  JointCommand out = c;
  // acquire pairs with the release in set_target: once stamping_ reads true, the
  // buffer stored before it is guaranteed visible, so there is no first-cycle stale read.
  if (!stamping_.load(std::memory_order_acquire)) {
    out.gripper.active = false;
    return out;
  }
  out.gripper = buf_[active_.load(std::memory_order_acquire)];
  out.gripper.active = true;
  return out;
}

}  // namespace kinova
```

- [ ] **Step 5: Retire the app-local injector**

In `apps/teleop_socket_server.cpp`: delete the `GripperInjector` class (lines 78-113), add `#include "kinova_lowlevel/gripper_controller.h"`, and replace its two uses:

Line 261: `GripperController injector(*base_transport);`

Line 342, where a `POSE_TARGET` arrives:
```cpp
          GripperCommand g;
          g.position = pkt.gripper;
          injector.set_target(g);
```

Update the file's header comment (line 16) from *"via a GripperInjector"* to *"via a GripperController"*.

- [ ] **Step 6: Register in CMake**

Add `src/gripper_controller.cpp` to `KINOVA_LIB_SOURCES` (after `src/sim_transport.cpp`), and `tests/gripper_controller_test.cpp` to the `unit_tests` source list (after `tests/sim_transport_test.cpp`).

- [ ] **Step 7: Run the full suite**

Run: `.superpowers/rbuild.sh`
Expected: all PASS, including the teleop protocol tests — the app change is a swap, not a behaviour change.

- [ ] **Step 8: Commit**

```bash
git add include/kinova_lowlevel/gripper_controller.h src/gripper_controller.cpp \
        tests/gripper_controller_test.cpp apps/teleop_socket_server.cpp CMakeLists.txt
git commit -m "feat(gripper): GripperController decorates Transport; retire the app-local injector"
```

---

## Self-review

**Spec coverage (stage 1 only).** `GripperFeedback` and `GripperCommand` → Task 1, including the `present` mis-mapping fix and the defaults-preserve-behaviour constraint. `KortexTransport` reading all four fields and writing all three → Task 3, with the normalized-effort-not-Newtons decision carried into the code comments. `GripperController` promoted into the library → Task 4. `SimTransport` gripper echo → Task 2, extended past the spec's wording to include stall simulation, because stage 2's grasp lifecycle cannot be tested without it and adding it later would mean re-touching the same file.

**Deliberately out of scope, per the spec's decomposition:** `GripperSink`, `Supervisor` and `Arbiter` wiring, the `Grasp` action, and stall *detection* (as opposed to stall *simulation*) are all stage 2.

**Correction (final fix wave):** this plan originally deferred the `rt_safety_test` case the spec asks for, claiming it needed a `Supervisor` to put a mode behind. That was wrong on the facts — `RtSafety.SupervisorInLoopNoMajorFaultsSteadyState` already showed the shape (a `FeedbackTap`-decorated `SimTransport`, a real mode, `RtExecutor`) without needing `Supervisor` for the gripper's own sake. `RtSafety.GripperControllerInLoopNoMajorFaultsSteadyState` adds it in stage 1: `GripperController` in the transport chain, a real mode under `RtExecutor`, and a non-RT thread calling `set_target()` throughout the measured window. Task 4's decorator tests still cover the stamp's logical behaviour; this test covers its RT-safety.

**Type consistency.** `GripperCommand`/`GripperFeedback` field names are identical in Tasks 1-4. `set_target` is the name in both `GripperController` and the teleop call site. `set_gripper_lag`/`set_gripper_blocked_at` match between Task 2's header, its tests, and Task 4's fixture use.

**Known limitation, stated rather than hidden.** Task 3's changes are verified only by compilation. `MotorFeedback::velocity`'s units and sign, and `kGripperMaxCurrentA`, cannot be settled without the arm — both are flagged in the spec's Open questions, and the first on-robot session should capture an open-close cycle with `--csv` to settle them.
