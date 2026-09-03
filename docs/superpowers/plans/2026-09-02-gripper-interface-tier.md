# Gripper Interface Tier — Implementation Plan (Gripper Plan 2)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the gripper reachable through the interface layer — a token-carrying setpoint in, a state snapshot out, arbitration and halt applied — and replace the `Supervisor`'s ten-positional-argument constructor with a dependency struct so the next collaborator is an additive change rather than a breaking one.

**Architecture:** `Supervisor` gains a `GripperSink` implementation that writes through the `GripperController` decorator built in Plan 1. The gripper needs no new feedback plumbing: it already arrives inside the same `JointFeedback` the pump snapshots for `ArmState`. `Arbiter` decorates the new port with the same token check it already applies to `CommandSink` and `StreamSink`. Halt is a deliberate no-op in the command path — `GripperController::release()` stops stamping, and the self-locking hardware holds.

**Tech Stack:** C++17, GoogleTest (one `unit_tests` binary), CMake, fixed-size POD value types.

**Spec:** `docs/superpowers/specs/2026-09-01-gripper-tier-design.md` — decomposition stage 2. Decisions 1 (shared token), 3 (normalized effort) and 5 (hold on halt) bind this plan. Decision 4 is **cut to its first half**: topic-only, no `Grasp` action and no stall detection.

## Global Constraints

- **The gripper rides the arm's token.** No separate ownership. `Arbiter` checks the same token against the same generation; a stale token is refused and counted.
- **On halt, the gripper holds.** E-stop means stop moving, and opening is itself a motion. `release()` stops stamping; the 2F-85 is effectively self-locking. **Do not command the gripper open on halt.**
- **Commands are stateless.** Speed and force do not persist between commands — every command carries all three fields, matching the streaming tier's rule that a setpoint is a command and never an increment.
- **`effort` is normalized, never Newtons**; `current` is amps. There is **no velocity field** — it was removed after measurement showed `MotorFeedback::velocity` to be the commanded speed echoed back.
- **Nothing in the RT path may allocate, lock, or block.** This tier is entirely non-RT: the setpoint handlers run on the backend thread and legitimately take a mutex, and state is served from the existing `Seqlock` snapshot.
- **Fail loud at startup, never silent mis-mapping.** A missing required dependency throws at construction, naming the field.
- **`GripperController` is optional.** A robot may genuinely have no gripper — `GripperFeedback::present` exists for exactly that. A null gripper dependency is legal; gripper commands become no-ops and `on_query_gripper` reports `present = false`.

## Build and test — this machine cannot build this project

No Pinocchio, wrong architecture. Builds run on `abra` (aarch64 Jetson):

```sh
.superpowers/rbuild.sh                  # build + FULL suite
.superpowers/rbuild.sh 'Supervisor*'    # build + filtered gtest
```

It rsyncs the **working tree** (not commits) and its exit status is trustworthy. Green at branch head before Task 1.

## Why a deps struct, and why pointers — and how that matches the codebase

The constructor has grown from eight parameters to ten across two features, breaking
`kinova_arm_ros2`'s `bringup_node.cpp` each time. A call site now reads
`Supervisor sup(pos, imp, tau, vel, ex, snap, pump_dyn, be, be)` — and the only way to know
which of those two `be`s is the `StreamPort` and which is the `ActionServerPort` is to count.

**References alone would not fix it.** A struct of reference members cannot be additive in
C++17: references have no default, so adding one still fails to compile at every
aggregate-initialisation site — the exact breakage this replaces. Worse, reference members
force *positional* aggregate initialisation, so the readability problem survives untouched.
The property being bought — fields you can name, and a new collaborator existing callers
ignore — requires defaultable members, which means pointers.

**This is the codebase's existing rule, not a new one.** Measured across
`include/kinova_lowlevel/`: every stored dependency is a reference — twelve of them,
`Dynamics&`, `Transport&`, `SampleRing&`, `RtExecutor&` — with exactly one raw pointer in
the whole library, `std::atomic<ControlMode*> requested_{nullptr}` in `RtExecutor`, and it
is a pointer precisely because a mode may be absent or swapped. So the convention already
in force is:

> **a reference means "always there"; a pointer means "may be absent."**

This plan applies that rule rather than bending it:

- **Pointers live only at the construction boundary**, in `SupervisorDeps`, so fields can be
  named and defaulted.
- **`require()` converts pointer to reference immediately.** Every required dependency the
  `Supervisor` *stores* stays a reference, exactly as all twelve others do. The pointer-ness
  does not leak past the constructor.
- **`grip_` stays a pointer all the way through** — and that is the rule being honoured, not
  broken. A robot may genuinely have no gripper; `GripperFeedback::present` exists for that
  case. It is the same meaning `RtExecutor::requested_` already carries.

Pointers at the boundary cost a null check. That is paid once, in a constructor that throws
naming the missing field, which is this repo's stated posture: fail loud at startup rather
than degrade at 1 kHz.

## File structure

| File | Responsibility |
|---|---|
| `include/kinova_lowlevel/interface/supervisor.h` (**modify**) | `SupervisorDeps`; constructor takes it |
| `src/interface/supervisor.cpp` (**modify**) | Validate deps; implement `GripperSink` |
| `include/kinova_lowlevel/interface/value_types.h` (**modify**) | `GripperSetpoint`, `GripperState` |
| `include/kinova_lowlevel/interface/ports.h` (**modify**) | `GripperSink` |
| `include/kinova_lowlevel/interface/arbiter.h` + `src/interface/arbiter.cpp` (**modify**) | Decorate `GripperSink` |
| `tests/interface/supervisor_test.cpp` (**modify**) | Deps migration, gripper command/state/halt |
| `tests/interface/arbiter_test.cpp` (**modify**) | Token gating for the gripper |
| `tests/rt_safety_test.cpp`, `apps/stream_check.cpp` (**modify**) | Call-site migration |
| `docs/guide/gripper.md` (**create**), `docs/reference/api.md`, `mkdocs.yml` (**modify**) | The user-facing story, now complete |

---

### Task 1: `SupervisorDeps` — the constructor migration

Pure refactor. **No behaviour changes.** The gripper is deliberately *not* added here; Task 3 adds it, which is what demonstrates the struct earning its keep.

**Files:**
- Modify: `include/kinova_lowlevel/interface/supervisor.h:32-37`
- Modify: `src/interface/supervisor.cpp` (constructor)
- Modify: `tests/interface/supervisor_test.cpp:92`
- Modify: `tests/rt_safety_test.cpp:578,662`
- Modify: `apps/stream_check.cpp:262`

**Interfaces:**
- Produces: `struct SupervisorDeps` with pointer members `pos, imp, tau, vel, exec, snap, pump_dyn, stream, action` (all required) and `SupervisorConfig cfg`. `explicit Supervisor(const SupervisorDeps&)`. Tasks 3 and 4 depend on this shape.

- [ ] **Step 1: Write the failing test**

Append to `tests/interface/supervisor_test.cpp`:

```cpp
TEST(SupervisorDepsTest, ThrowsNamingTheMissingDependency) {
  // Fail loud at construction rather than dereferencing null on the sampler thread
  // three seconds later. The message must name the field, because with ten
  // dependencies "something was null" is not an actionable error.
  Dynamics dyn{URDF_PATH};
  JointPositionMode pos{dyn};
  interface::SupervisorDeps d;
  d.pos = &pos;                      // everything else deliberately left null
  try {
    interface::Supervisor sup{d};
    FAIL() << "expected a throw for the missing dependencies";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("imp"), std::string::npos)
        << "the message must name a missing field, got: " << e.what();
  }
}

TEST(SupervisorDepsTest, AcceptsAFullyPopulatedSetOfDependencies) {
  SupFix f;                          // the fixture builds a complete deps set
  f.sup.start();
  f.run_rt();
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  f.sup.stop();
  f.teardown();
  SUCCEED();                         // constructed, ran and tore down
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `.superpowers/rbuild.sh 'SupervisorDeps*'`
Expected: FAIL to compile — `'SupervisorDeps' is not a member of 'kinova::interface'`.

- [ ] **Step 3: Declare the struct and the new constructor**

In `include/kinova_lowlevel/interface/supervisor.h`, add `#include <stdexcept>` and, immediately after `SupervisorConfig`:

```cpp
// Everything the Supervisor needs, in one named place.
//
// POINTERS here, references everywhere else -- which is this library's existing rule, not
// an exception to it. Every stored dependency in include/kinova_lowlevel/ is a reference;
// the one raw pointer is RtExecutor's requested_ mode, and it is a pointer because a mode
// may be absent or swapped. A reference means "always there"; a pointer means "may be
// absent."
//
// The pointers exist ONLY at this construction boundary, so fields can be named and
// defaulted -- a struct of references would force positional initialisation and would
// still fail to compile at every site when a member is added, which is the breakage being
// replaced. require() converts each one to a reference immediately, so what the Supervisor
// STORES matches every other unit and the pointer-ness never leaks past the constructor.
//
// The cost is a null check, paid once in a constructor that throws naming the field. This
// repo fails loud at startup rather than degrading at 1 kHz.
//
// Assign by name at the call site:
//     SupervisorDeps d;
//     d.pos = &pos; d.imp = &imp; ... d.stream = &backend; d.action = &router;
//     Supervisor sup{d};
// which is legible in a way that ten positional arguments -- two of which were the same
// object passed as different port types -- was not.
struct SupervisorDeps {
  JointPositionMode*      pos      = nullptr;
  JointImpedanceMode*     imp      = nullptr;
  JointTorqueMode*        tau      = nullptr;
  JointVelocityMode*      vel      = nullptr;
  RtExecutor*             exec     = nullptr;
  Seqlock<JointFeedback>* snap     = nullptr;
  Dynamics*               pump_dyn = nullptr;
  StreamPort*             stream   = nullptr;
  ActionServerPort*       action   = nullptr;
  SupervisorConfig        cfg{};
};
```

Replace the constructor declaration with:

```cpp
  explicit Supervisor(const SupervisorDeps& deps);
```

- [ ] **Step 4: Validate and bind in the constructor**

In `src/interface/supervisor.cpp`, replace the constructor's parameter list and initialiser list. The member declarations in the header stay references — only how they are bound changes:

```cpp
namespace {
// Named so the throw says which field, not merely that something was null.
template <typename T>
T& require(T* p, const char* field) {
  if (!p) throw std::invalid_argument(std::string("SupervisorDeps::") + field +
                                      " is required and was null");
  return *p;
}
}  // namespace

Supervisor::Supervisor(const SupervisorDeps& d)
    : pos_(require(d.pos, "pos")), imp_(require(d.imp, "imp")),
      tau_(require(d.tau, "tau")), vel_(require(d.vel, "vel")),
      exec_(require(d.exec, "exec")), snap_(require(d.snap, "snap")),
      pump_dyn_(require(d.pump_dyn, "pump_dyn")), stream_(require(d.stream, "stream")),
      action_(require(d.action, "action")), cfg_(d.cfg) {
```

Keep the rest of the constructor body exactly as it is.

- [ ] **Step 5: Migrate the four call sites**

`tests/interface/supervisor_test.cpp` — in `SupFix`, replace the `interface::Supervisor sup{...}` member. Because the fixture's members are declared in order, build the deps in a helper so `sup` can still be a direct member:

```cpp
  interface::SupervisorDeps deps{&pos, &imp, &tau, &vel, &exec, &snap, &pump_dyn, &be, &be};
  interface::Supervisor sup{deps};
```

`tests/rt_safety_test.cpp:578` and `:662`, and `apps/stream_check.cpp:262` — replace each `Supervisor sup(...)` with:

```cpp
  interface::SupervisorDeps deps;
  deps.pos = &pos; deps.imp = &imp; deps.tau = &tau; deps.vel = &vel;
  deps.exec = &ex; deps.snap = &snap; deps.pump_dyn = &pump_dyn;
  deps.stream = &be; deps.action = &be;
  Supervisor sup(deps);
```

adjusting the local names to match each site (`stream_check.cpp` uses `backend` for both ports; the `rt_safety_test.cpp` sites use `ex` and `be`).

Confirm none were missed: `grep -rn "Supervisor sup\|interface::Supervisor " tests/ apps/ src/` should show no site passing more than one argument.

- [ ] **Step 6: Run the full suite**

Run: `.superpowers/rbuild.sh`
Expected: all PASS. This is a pure refactor — any behavioural test that changes result means the migration altered something it should not have. Report rather than adjusting the test.

- [ ] **Step 7: Commit**

```bash
git add include/kinova_lowlevel/interface/supervisor.h src/interface/supervisor.cpp \
        tests/interface/supervisor_test.cpp tests/rt_safety_test.cpp apps/stream_check.cpp
git commit -m "refactor(interface): Supervisor takes a SupervisorDeps struct"
```

---

### Task 2: `GripperSetpoint`, `GripperState`, and the `GripperSink` port

**Files:**
- Modify: `include/kinova_lowlevel/interface/value_types.h`
- Modify: `include/kinova_lowlevel/interface/ports.h`
- Modify: `tests/interface/supervisor_test.cpp` (append)

**Interfaces:**
- Consumes: `GripperCommand { float position; float speed; float force; bool active; }` and `GripperFeedback { float position; float effort; float current; bool present; }` from `kinova_lowlevel/gripper_types.h`; `Token` from `value_types.h`.
- Produces: `struct GripperSetpoint { kinova::GripperCommand command; Token token{}; }`, `struct GripperState { float position, effort, current; bool present; double stamp_s; }`, and `class GripperSink` with `on_gripper_setpoint(const GripperSetpoint&)` and `on_query_gripper() const`. Tasks 3 and 4 implement this port.

- [ ] **Step 1: Write the failing test**

Append to `tests/interface/supervisor_test.cpp`:

```cpp
TEST(GripperValueTypes, SetpointCarriesTheCommandAndItsOwnAuthority) {
  // Every command carries its own token, exactly as JointSetpoint does -- the
  // gripper rides the arm's token, so the Arbiter can gate it with the same check.
  interface::GripperSetpoint s;
  EXPECT_FLOAT_EQ(s.command.position, 0.0f);
  EXPECT_FLOAT_EQ(s.command.speed,    1.0f);   // GripperCommand's defaults survive
  EXPECT_FLOAT_EQ(s.command.force,    0.5f);
  EXPECT_FALSE(s.command.active);
  EXPECT_EQ(s.token, interface::Token{});
}

TEST(GripperValueTypes, StateDefaultsToAbsent) {
  interface::GripperState g;
  EXPECT_FALSE(g.present);
  EXPECT_FLOAT_EQ(g.position, 0.0f);
  EXPECT_FLOAT_EQ(g.effort,   0.0f);
  EXPECT_FLOAT_EQ(g.current,  0.0f);
  EXPECT_DOUBLE_EQ(g.stamp_s, 0.0);
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `.superpowers/rbuild.sh 'GripperValueTypes*'`
Expected: FAIL to compile — `'GripperSetpoint' is not a member of 'kinova::interface'`.

- [ ] **Step 3: Add the value types**

In `include/kinova_lowlevel/interface/value_types.h`, add `#include "kinova_lowlevel/gripper_types.h"`, then after `TwistSetpoint`:

```cpp
// The gripper's command, carrying its own authority like every other setpoint. The
// gripper rides the ARM's token (spec decision 1): one physical machine, one holder.
struct GripperSetpoint { kinova::GripperCommand command{}; Token token{}; };

// What the gripper reports. Mirrors GripperFeedback plus a stamp.
//
// There is deliberately NO velocity. MotorFeedback has one, but it was measured on the
// arm to be the commanded speed echoed back rather than a measurement, so core removed
// the field; see gripper_types.h. `effort` is a 0..1 fraction of maximum derived from
// motor current, never Newtons -- and note a SUSTAINED grasp reports a SMALL effort
// (~0.05), because the gripper spikes on contact and then settles to a low holding
// current.
struct GripperState {
  float  position = 0.0f;
  float  effort   = 0.0f;
  float  current  = 0.0f;   // amps
  bool   present  = false;
  double stamp_s  = 0.0;
};
```

- [ ] **Step 4: Add the port**

In `include/kinova_lowlevel/interface/ports.h`, after `StreamSink`:

```cpp
// Driving port for the gripper. Two methods, not four: the Grasp action and its cancel
// were cut with stall detection (spec decision 4), so this tier is topic-only. Separate
// from CommandSink and StreamSink for the same reason those are separate from each
// other -- a backend implements only the concerns it supports, and a robot with no
// gripper implements none of this.
class GripperSink { public: virtual ~GripperSink() = default;
  virtual void         on_gripper_setpoint(const GripperSetpoint&) = 0;
  virtual GripperState on_query_gripper() const = 0; };
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `.superpowers/rbuild.sh 'GripperValueTypes*'`
Expected: both PASS.

- [ ] **Step 6: Commit**

```bash
git add include/kinova_lowlevel/interface/value_types.h \
        include/kinova_lowlevel/interface/ports.h tests/interface/supervisor_test.cpp
git commit -m "feat(interface): GripperSetpoint, GripperState and the GripperSink port"
```

---

### Task 3: `Supervisor` implements `GripperSink`

This is where the deps struct pays off: the gripper arrives as a **new field existing call sites ignore**, rather than an eleventh positional argument.

**Files:**
- Modify: `include/kinova_lowlevel/interface/supervisor.h`
- Modify: `src/interface/supervisor.cpp`
- Modify: `tests/interface/supervisor_test.cpp` (append; add `grip` to `SupFix`)

**Interfaces:**
- Consumes: `GripperSink` (Task 2); `GripperController::set_target(const GripperCommand&)` and `release()` from `kinova_lowlevel/gripper_controller.h`; `SupervisorDeps` (Task 1).
- Produces: `SupervisorDeps::grip` (a `GripperController*`, **optional**, defaulting to null); `Supervisor` additionally inherits `GripperSink`. Task 4 decorates it.

- [ ] **Step 1: Write the failing test**

Append to `tests/interface/supervisor_test.cpp`. Follow the file's idiom — plain `TEST` with a local `SupFix f;`, and read RT-owned state only after `f.teardown()`:

```cpp
TEST(Supervisor, GripperSetpointReachesTheController) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::GripperSetpoint s;
  s.command.position = 0.7f;
  s.command.force    = 0.3f;
  s.command.active   = true;
  f.sup.on_gripper_setpoint(s);
  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  f.sup.stop(); f.teardown();

  EXPECT_TRUE(f.sim.last_command().gripper.active);
  EXPECT_NEAR(f.sim.last_command().gripper.position, 0.7f, 1e-6f);
  EXPECT_NEAR(f.sim.last_command().gripper.force,    0.3f, 1e-6f);
}

TEST(Supervisor, QueryGripperReportsWhatTheArmSent) {
  SupFix f; f.sup.start(); f.run_rt();
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  const interface::GripperState g = f.sup.on_query_gripper();
  f.sup.stop(); f.teardown();

  // SimTransport always reports a gripper; the state comes from the same feedback
  // snapshot the pump already reads for ArmState, so no new plumbing was needed.
  EXPECT_TRUE(g.present);
  EXPECT_GT(g.stamp_s, 0.0);
}

TEST(Supervisor, HaltStopsCommandingTheGripperWithoutOpeningIt) {
  // Spec decision 5: e-stop means stop moving, and opening is itself a motion. The
  // 2F-85 self-locks, so ceasing to command IS holding.
  SupFix f; f.sup.start(); f.run_rt();
  interface::GripperSetpoint s;
  s.command.position = 0.8f;
  s.command.active   = true;
  f.sup.on_gripper_setpoint(s);
  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  ASSERT_TRUE(f.sim.last_command().gripper.active);

  f.sup.on_halt(interface::HaltReason::kEmergencyStop);
  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  f.sup.stop(); f.teardown();

  EXPECT_FALSE(f.sim.last_command().gripper.active);        // no longer commanded
  EXPECT_NE(f.sim.last_command().gripper.position, 0.0f);   // and NOT commanded open
}

TEST(Supervisor, AbsentGripperMakesCommandsHarmlessNoOps) {
  // A robot may genuinely have no gripper. A null dependency is legal, not an error.
  SupFixNoGripper f; f.sup.start(); f.run_rt();
  interface::GripperSetpoint s;
  s.command.position = 0.9f;
  s.command.active   = true;
  f.sup.on_gripper_setpoint(s);            // must not crash
  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  const interface::GripperState g = f.sup.on_query_gripper();
  f.sup.stop(); f.teardown();
  EXPECT_FALSE(g.present);
}
```

- [ ] **Step 2: Add the fixtures**

In `tests/interface/supervisor_test.cpp`, add a `GripperController` to `SupFix` **between** the transport and the `FeedbackTap`, so the chain matches how an app wires it, and add `deps.grip`:

```cpp
  SimTransport sim;
  GripperController gc{sim};
  Seqlock<JointFeedback> snap;
  FeedbackTap tap{gc, snap};
  ...
  interface::SupervisorDeps deps{&pos, &imp, &tau, &vel, &exec, &snap, &pump_dyn, &be, &be};
```

then set `deps.grip = &gc;` in the fixture's constructor body.

Add a second fixture that is identical except it leaves `deps.grip` null. Copy `SupFix` rather than parameterising it — the fixture is a plain struct in an anonymous namespace and the duplication is more legible than a template here:

```cpp
struct SupFixNoGripper { /* identical to SupFix, but deps.grip stays nullptr and the
                            transport chain has no GripperController */ };
```

- [ ] **Step 3: Run to verify it fails**

Run: `.superpowers/rbuild.sh 'Supervisor*'`
Expected: FAIL to compile — `'class kinova::interface::Supervisor' has no member named 'on_gripper_setpoint'`.

- [ ] **Step 4: Declare it**

In `include/kinova_lowlevel/interface/supervisor.h`: add `#include "kinova_lowlevel/gripper_controller.h"`, add `GripperController* grip = nullptr;` to `SupervisorDeps` **with a comment saying it is optional and why**, add `GripperSink` to the base list, declare the two overrides, and add the member `GripperController* grip_ = nullptr;` (a pointer, not a reference — it is legitimately absent).

```cpp
  // OPTIONAL. Null means this robot has no gripper, which is a real configuration --
  // GripperFeedback::present exists for exactly that. Gripper commands become no-ops
  // and on_query_gripper reports present=false. Not validated by require().
  GripperController* grip = nullptr;
```

- [ ] **Step 5: Implement it**

In `src/interface/supervisor.cpp`, bind `grip_(d.grip)` in the initialiser list (no `require`), and add:

```cpp
void Supervisor::on_gripper_setpoint(const GripperSetpoint& s) {
  if (!grip_) return;            // no gripper on this robot: a no-op, not an error
  grip_->set_target(s.command);
}

GripperState Supervisor::on_query_gripper() const {
  GripperState g;
  if (!grip_) return g;          // present stays false
  JointFeedback fb;
  if (!snap_.load(fb)) return g; // a torn read reports absent rather than garbage
  g.position = fb.gripper.position;
  g.effort   = fb.gripper.effort;
  g.current  = fb.gripper.current;
  g.present  = fb.gripper.present;
  g.stamp_s  = secs_since(t0_);
  return g;
}
```

In the existing `on_halt` implementation, add — alongside the trajectory cancellation and the hold-at-measured-q — the gripper's half:

```cpp
  // Spec decision 5: stop stamping, do NOT command open. Opening is a motion, and
  // e-stop means stop moving; the 2F-85 self-locks, so ceasing to command holds the
  // grip. Deliberately not a "safe" open -- anything held stays held rather than
  // being dropped from wherever the arm happened to be.
  if (grip_) grip_->release();
```

- [ ] **Step 6: Run the full suite**

Run: `.superpowers/rbuild.sh`
Expected: all PASS, including every pre-existing `Supervisor` test — this task adds a concern, it does not change any existing one.

- [ ] **Step 7: Commit**

```bash
git add include/kinova_lowlevel/interface/supervisor.h src/interface/supervisor.cpp \
        tests/interface/supervisor_test.cpp
git commit -m "feat(interface): Supervisor implements GripperSink; halt holds the grip"
```

---

### Task 4: `Arbiter` decorates `GripperSink`

**Files:**
- Modify: `include/kinova_lowlevel/interface/arbiter.h`
- Modify: `src/interface/arbiter.cpp`
- Modify: `tests/interface/arbiter_test.cpp` (append)

**Interfaces:**
- Consumes: `GripperSink` (Task 2).
- Produces: `Arbiter` additionally inherits `GripperSink`; its constructor gains a `GripperSink& downstream_gripper` parameter after `downstream_stream`.

- [ ] **Step 1: Write the failing test**

Append to `tests/interface/arbiter_test.cpp`. **Match the file's idiom exactly**: it uses a single `RecordingSink` that implements *both* downstream ports and is passed twice — `Arbiter arb{sink, sink, ArbitrationMode::kEnforced, 1234}`. Extend that one struct rather than inventing separate fakes, and pass it three times.

First, extend `RecordingSink` to implement `GripperSink` too:

```cpp
struct RecordingSink : public CommandSink, public StreamSink, public GripperSink {
  // ... every existing member and override unchanged ...
  int gripper_setpoints = 0, gripper_queries = 0;
  GripperSetpoint last_gripper{};
  GripperState    gripper_state{};
  void on_gripper_setpoint(const GripperSetpoint& s) override {
    ++gripper_setpoints; last_gripper = s;
  }
  GripperState on_query_gripper() const override { return gripper_state; }
};
```

`on_query_gripper` is `const`, so it cannot bump a counter without a `mutable` member — do not add one; the tests below assert on `gripper_state` instead.

```cpp
TEST(Arbiter, GripperSetpointWithTheGrantedTokenIsDelivered) {
  RecordingSink sink; Arbiter arb{sink, sink, sink, ArbitrationMode::kEnforced, 1234};
  const auto g = arb.grant("owner");
  ASSERT_TRUE(g.accepted);
  GripperSetpoint s; s.command.position = 0.6f; s.token = g.token;
  arb.on_gripper_setpoint(s);
  EXPECT_EQ(sink.gripper_setpoints, 1);
  EXPECT_NEAR(sink.last_gripper.command.position, 0.6f, 1e-6f);
}

TEST(Arbiter, GripperSetpointWithAStaleTokenIsRefusedAndCounted) {
  // The gripper rides the ARM's token, so a revoked owner loses the gripper too --
  // one physical machine, one holder.
  RecordingSink sink; Arbiter arb{sink, sink, sink, ArbitrationMode::kEnforced, 1234};
  const auto g = arb.grant("owner");
  arb.revoke();
  GripperSetpoint s; s.token = g.token;
  const auto before = arb.status().rejected_count;
  arb.on_gripper_setpoint(s);
  EXPECT_EQ(sink.gripper_setpoints, 0);              // never reached the downstream
  EXPECT_GT(arb.status().rejected_count, before);    // and was counted
}

TEST(Arbiter, QueryGripperIsNeverGated) {
  // Reads are always open, exactly as on_query_state is -- observing the arm is not
  // commanding it, and a monitor must not need the token.
  RecordingSink sink; Arbiter arb{sink, sink, sink, ArbitrationMode::kEnforced, 1234};
  sink.gripper_state.present = true;
  const auto s = arb.on_query_gripper();             // no grant at all
  EXPECT_TRUE(s.present);
}

TEST(Arbiter, GripperSetpointIsRefusedWhileEstopped) {
  RecordingSink sink; Arbiter arb{sink, sink, sink, ArbitrationMode::kEnforced, 1234};
  const auto g = arb.grant("owner");
  arb.estop();
  GripperSetpoint s; s.token = g.token;
  arb.on_gripper_setpoint(s);
  EXPECT_EQ(sink.gripper_setpoints, 0);
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `.superpowers/rbuild.sh 'Arbiter*'`
Expected: FAIL to compile — no matching constructor taking three sinks.

- [ ] **Step 3: Declare it**

In `include/kinova_lowlevel/interface/arbiter.h`: add `GripperSink` to the base list, add the constructor parameter, declare the two overrides, and add the member `GripperSink& down_grip_;`.

```cpp
  Arbiter(CommandSink& downstream, StreamSink& downstream_stream,
          GripperSink& downstream_gripper, ArbitrationMode mode, uint64_t seed = 0);
  ...
  // GripperSink
  void         on_gripper_setpoint(const GripperSetpoint&) override;
  GripperState on_query_gripper() const override;   // never gated -- reads are always open
```

- [ ] **Step 4: Implement it**

In `src/interface/arbiter.cpp`, mirror the existing setpoint handlers exactly — the lock **is** held across delegation, so admit-and-deliver is atomic against `revoke()`/`estop()`:

```cpp
void Arbiter::on_gripper_setpoint(const GripperSetpoint& s) {
  std::lock_guard<std::mutex> l(m_);
  if (!admit(s.token)) return;
  down_grip_.on_gripper_setpoint(s);
}

// Never gated: observing the arm is not commanding it, and a monitor must not need a
// token. Same posture as on_query_state.
GripperState Arbiter::on_query_gripper() const { return down_grip_.on_query_gripper(); }
```

- [ ] **Step 5: Update every Arbiter construction site**

They are **all inside `tests/interface/arbiter_test.cpp`** — verified by
`grep -rn "Arbiter arb\|Arbiter a(" tests/ apps/ src/ | grep -v arbiter.cpp`, which finds
nothing outside that file. No app constructs an `Arbiter` today, so this task touches no
`apps/` source.

Each site reads `Arbiter arb{sink, sink, ArbitrationMode::kEnforced, 1234}`; add a third
`sink`. There are roughly a dozen — change them all, and re-run the grep to confirm none
takes two sinks afterwards.

- [ ] **Step 6: Run the full suite**

Run: `.superpowers/rbuild.sh`
Expected: all PASS, including every pre-existing arbitration test.

- [ ] **Step 7: Commit**

```bash
git add include/kinova_lowlevel/interface/arbiter.h src/interface/arbiter.cpp \
        tests/interface/arbiter_test.cpp
git commit -m "feat(interface): Arbiter gates the gripper on the arm's token"
```

---

### Task 5: Documentation

Plan 1's final review ruled that no docs were owed because the user-facing gripper story was incomplete. It is complete now — commands in, state out, arbitration and halt applied — so the docs are owed.

**Files:**
- Create: `docs/guide/gripper.md`
- Modify: `docs/reference/api.md`
- Modify: `docs/guide/control-modes.md`
- Modify: `mkdocs.yml`

**Interfaces:** none new.

- [ ] **Step 1: Write the guide page**

Create `docs/guide/gripper.md` covering, in this order:

- **What the gripper is in this driver**: not a `ControlMode`. It is a field on the outgoing frame, stamped by a `Transport` decorator, so it is orthogonal to control modes — you can grip during a trajectory, an impedance hold, or a velocity stream.
- **The command**: position, speed and force, all 0..1, all per-command. **`force` is a ceiling on motor current, not a force setpoint** — no force servo exists on this hardware by any path. Speed and force do **not** persist between commands.
- **The state**: position, effort, current, present. **No velocity**, and say why in one sentence.
- **Reading effort honestly**, with the measured numbers: a grasp **spikes** to full scale as the fingers close and then **settles to about 0.05**. A sustained grasp reports a *small* effort, not a large one. Closing on empty air reports **zero** — the gripper reaches position and stops driving.
- **Ownership**: the gripper rides the arm's token. Whoever holds the arm holds the gripper.
- **On halt the gripper holds.** State the accepted cost plainly: if the gripper is closed on something it should not be, e-stop will **not** release it — that needs a deliberate open command after the halt clears. Note that the mechanism is retransmission of the last cyclic command rather than cessation, so a gripper stalled at e-stop stays energized at the commanded current cap.
- **No grasp primitive**, and the pointer for anyone who wants one: commanded fully closed, the gripper settles at **0.8333** on an object and **0.9912** on air, so "stopped short of the commanded target" separates the cases with no effort threshold.

- [ ] **Step 2: Update the reference and the mode guide**

`docs/reference/api.md` — add `GripperCommand`, `GripperFeedback`, `GripperController`, `GripperSink`, `GripperSetpoint`, `GripperState`, and `SupervisorDeps`. The `Transport` section currently lists only `SimTransport` and `KortexTransport`; add `GripperController` and `FeedbackTap` as the two decorators.

`docs/guide/control-modes.md` — one sentence in the mode inventory noting the gripper is deliberately not a mode, linking to the new page.

- [ ] **Step 3: Wire the nav**

`mkdocs.yml` — add `guide/gripper.md` to the `nav:` under the guide section. An orphaned page does not appear on the site.

- [ ] **Step 4: Verify the links resolve**

Check that every anchor the new page references exists, and that `api.md`'s and `control-modes.md`'s links to it point at real headings.

Run: `.superpowers/rbuild.sh`
Expected: all PASS (docs do not affect the build; this is a regression guard).

- [ ] **Step 5: Commit**

```bash
git add docs/guide/gripper.md docs/reference/api.md docs/guide/control-modes.md mkdocs.yml
git commit -m "docs(gripper): a guide page for the tier, and the reference entries"
```

---

## Self-review

**Spec coverage (stage 2).** `GripperSink` → Task 2, at the two-method size the grasp cut left it. `Supervisor` implementation → Task 3, including the state path (which needed no new plumbing — the gripper already arrives in the `JointFeedback` the pump snapshots) and the halt hook. `Arbiter` decoration with the same token → Task 4. Decision 1 (shared token) → Task 4's stale-token test. Decision 5 (hold on halt) → Task 3's halt test, which asserts both that commanding stops *and* that the position is not zeroed. Docs → Task 5.

**Deliberately out of scope.** The `Grasp` action, its cancel, and stall detection are cut (decision 4) and appear nowhere. `GripperState` carries no velocity because core has no velocity to report. The ROS2 surface is downstream and gets its own spec.

**Beyond the spec, and why.** Task 1's `SupervisorDeps` is not in the spec's stage-2 list. It is here because the constructor has broken `kinova_arm_ros2` twice already and this task would break it a third time; doing the migration in the same change that adds the gripper means one downstream fix rather than two, and the gripper is the collaborator that demonstrates the struct working.

**Type consistency.** `GripperSetpoint`/`GripperState` field names match between Tasks 2, 3 and 4. `on_gripper_setpoint` and `on_query_gripper` are the names in the port, the `Supervisor`, the `Arbiter` and every test. `SupervisorDeps::grip` is a `GripperController*` in Tasks 1 and 3.

**Known breakage this plan causes.** `kinova_arm_ros2`'s `bringup_node.cpp` needs migrating to `SupervisorDeps`, and its `Arbiter` construction needs the third sink. That is one migration covering both, which is the point of doing it here.
