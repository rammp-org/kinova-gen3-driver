# Arm Arbitration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a transport-agnostic ownership layer to the driver core that decides *who may command the arm*, plus a general `on_halt` primitive that ownership revocation and `/estop` both use.

**Architecture:** A new `kinova::interface::Arbiter` **decorates** `CommandSink` — it sits between the transport backend and the `Supervisor`, compares the capability token each command carries, and delegates or rejects. It includes nothing from controls (no `ControlMode`, `RtExecutor`, `Dynamics`, `Transport`, or Pinocchio), so the safety-critical logic is unit-testable with a fake sink, no robot, no URDF and no threads. Halting is expressed as one new `CommandSink` method whose control action the `Supervisor` performs on its sampler thread, preserving settle-exactly-once by construction.

**Tech Stack:** C++17, Eigen, GoogleTest, CMake. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-08-25-arm-arbitration-design.md`

## Global Constraints

- **Builds on Linux/aarch64 only (the Jetson, `abra`).** This box (x86_64) cannot build — no Pinocchio. Every "green" claim needs a real build+ctest on the Jetson.
- Build: `cmake -S . -B build -DCMAKE_PREFIX_PATH=/usr/local/lib/python3.10/dist-packages/cmeel.prefix && cmake --build build -j && ctest --test-dir build --output-on-failure`
- Subset: `./build/unit_tests --gtest_filter='Arbiter*'` — all tests are one gtest binary registered as a single ctest test.
- **Nothing in the RT path (`compute`, executor cycle) may allocate, lock, or block.** This work adds nothing to the RT path; `rt_safety_test` (zero major page faults, zero dropped samples) is the gate and must be run, not assumed.
- **SI / radians internally.** No KORTEX or Pinocchio types leak into `interface/`.
- **The core library has no logging facility.** `std::cerr` appears only under `apps/`. The Arbiter counts rejections and returns a distinguishable code; the backend logs.
- Value types are fixed-size and allocation-free on the command path.
- There is **no CI on this repo**. The Jetson is the only gate.

## File Structure

| File | Responsibility |
|---|---|
| `include/kinova_lowlevel/interface/value_types.h` (modify) | `Token`, `CancelRequest`, `HaltReason`, `ArbitrationMode`, `GrantResult`, `ArbitrationStatus`, new result codes, token fields on command structs |
| `include/kinova_lowlevel/interface/ports.h` (modify) | `CommandSink::on_halt`, `on_trajectory_cancel(CancelRequest)`, new `ArbitrationSink` port |
| `include/kinova_lowlevel/interface/arbiter.h` (create) | The Arbiter: state machine + admission. Controls-free. |
| `src/interface/arbiter.cpp` (create) | Its implementation. |
| `include/kinova_lowlevel/interface/supervisor.h` (modify) | `on_halt` declaration, `active_sink()` helper, halt latch members |
| `src/interface/supervisor.cpp` (modify) | `on_halt` latch + the halt branch in `sampler_loop` |
| `include/kinova_lowlevel/sim_transport.h` (modify) | `last_command()` accessor so tests can observe the latched hold target |
| `tests/interface/arbiter_test.cpp` (create) | Tier-1 unit tests: the full transition table |
| `tests/interface/supervisor_test.cpp` (modify) | Halt-path tests + `CancelRequest` call-site updates |
| `tests/interface/execution_integration_test.cpp` (modify) | Tier-2 integration: revoke mid-motion through a real Arbiter |
| `CMakeLists.txt` (modify) | Register `src/interface/arbiter.cpp` and `tests/interface/arbiter_test.cpp` |
| `docs/guide/arbitration.md` (create), `mkdocs.yml` (modify), `docs/reference/api.md` (modify) | Docs, wired into nav |

---

### Task 1: Interface change — value types and ports

The breaking change lands here and the suite stays green. No new behaviour: `Supervisor::on_halt` is a stub until Task 5.

**Files:**
- Modify: `include/kinova_lowlevel/interface/value_types.h`
- Modify: `include/kinova_lowlevel/interface/ports.h`
- Modify: `include/kinova_lowlevel/interface/supervisor.h`
- Modify: `src/interface/supervisor.cpp`
- Test: `tests/interface/supervisor_test.cpp` (existing `TEST(ValueTypes, DefaultsAndResultCodes)` + cancel call sites)

**Interfaces:**
- Produces: `Token`, `CancelRequest`, `HaltReason`, `ArbitrationMode`, `GrantResult`, `ArbitrationStatus`, `ArbitrationSink`, `CommandSink::on_halt(HaltReason)`, `GoalResponse::kRejectUnauthorized`, `result_code::kNotAuthorized`, `result_code::kHalted`. Tasks 2–7 all consume these.

- [ ] **Step 1: Write the failing test**

Add a new test alongside the existing `TEST(ValueTypes, DefaultsAndResultCodes)` in `tests/interface/supervisor_test.cpp`:

```cpp
TEST(ValueTypes, ArbitrationDefaultsAndResultCodes) {
  EXPECT_EQ(interface::result_code::kNotAuthorized, -8);
  EXPECT_EQ(interface::result_code::kHalted,        -9);
  interface::TrajectoryGoal g;
  EXPECT_EQ(g.token, (interface::Token{}));          // zero-initialised, not garbage
  interface::CancelRequest c;
  EXPECT_EQ(c.token, (interface::Token{}));
  interface::GrantResult gr;
  EXPECT_FALSE(gr.accepted);
  EXPECT_EQ(gr.generation, 0u);
  interface::ArbitrationStatus st;
  EXPECT_FALSE(st.estopped);
  EXPECT_FALSE(st.owned);
  EXPECT_EQ(st.rejected_count, 0u);
  EXPECT_EQ(st.mode, interface::ArbitrationMode::kEnforced);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `./build/unit_tests --gtest_filter='ValueTypes.*'`
Expected: compile error — `Token`, `CancelRequest`, `GrantResult`, `ArbitrationStatus`, `kNotAuthorized`, `kHalted` are not declared.

- [ ] **Step 3: Add the types**

In `include/kinova_lowlevel/interface/value_types.h`, after the `GoalId` alias:

```cpp
using Token = std::array<uint8_t, 16>;   // 128-bit capability token; POD, alloc-free

enum class ArbitrationMode { kEnforced, kDisabled };
enum class HaltReason      { kOwnershipRevoked, kEmergencyStop, kOperatorRequest };
```

Add a token field to each inbound command struct (every command carries its own authority):

```cpp
struct TrajectoryGoal {
  // ...existing fields unchanged...
  std::string sender_id;
  Token token{};
};
struct GainsRequest { JointImpedanceGains gains{}; Token token{}; };
struct CancelRequest { GoalId id{}; Token token{}; };   // new: cancel had no struct to carry a token
```

Add the arbitration value types:

```cpp
struct GrantResult       { bool accepted=false; Token token{}; uint64_t generation=0; std::string message; };
struct ArbitrationStatus { ArbitrationMode mode = ArbitrationMode::kEnforced; bool estopped=false;
                           bool owned=false; std::string owner_id; uint64_t generation=0;
                           uint64_t rejected_count=0; };
```

Extend the response enum and result codes:

```cpp
enum class GoalResponse { kAccept, kReject, kRejectUnauthorized };

namespace result_code {
  constexpr int kSuccessful = 0, kInvalidGoal = -1, kPathToleranceViolated = -4,
                kGoalToleranceViolated = -5, kPreempted = -6, kPlanningFailed = -7,
                kNotAuthorized = -8, kHalted = -9;
}
```

- [ ] **Step 4: Change the ports**

In `include/kinova_lowlevel/interface/ports.h`, change `CommandSink` and add `ArbitrationSink`:

```cpp
// Driving port — the supervisor IMPLEMENTS this; the backend calls it on inbound messages.
class CommandSink { public: virtual ~CommandSink() = default;
  virtual GoalResponse   on_trajectory_goal(const TrajectoryGoal&) = 0;
  virtual void           on_trajectory_accepted(const GoalId&, const TrajectoryGoal&) = 0;
  virtual CancelResponse on_trajectory_cancel(const CancelRequest&) = 0;   // was const GoalId&
  virtual GainsResult    on_set_gains(const GainsRequest&) = 0;
  virtual ArmState       on_query_state() = 0;
  virtual void           on_halt(HaltReason) = 0; };                        // new

// Driving port for ownership. Separate from CommandSink: "who may command" is not
// "command the arm", and a harness that ignores ownership implements only CommandSink.
class ArbitrationSink { public: virtual ~ArbitrationSink() = default;
  virtual GrantResult       grant(const std::string& owner_id) = 0;
  virtual void              revoke() = 0;
  virtual void              estop() = 0;
  virtual void              estop_clear() = 0;
  virtual ArbitrationStatus status() const = 0; };
```

`ports.h` needs `#include <string>`.

- [ ] **Step 5: Keep Supervisor compiling (stub halt)**

In `include/kinova_lowlevel/interface/supervisor.h`, change the cancel signature and declare halt:

```cpp
  CancelResponse on_trajectory_cancel(const CancelRequest&) override;
  void           on_halt(HaltReason) override;
```

In `src/interface/supervisor.cpp`:

```cpp
CancelResponse Supervisor::on_trajectory_cancel(const CancelRequest& c){
  std::lock_guard<std::mutex> l(q_mtx_); inbox_.push_back({c.id, {}, true}); return CancelResponse::kAccept;
}
void Supervisor::on_halt(HaltReason){}   // real implementation in Task 5
```

- [ ] **Step 6: Update existing cancel call sites in the tests**

Any `sup.on_trajectory_cancel(id)` becomes:

```cpp
  interface::CancelRequest cr; cr.id = id;
  f.sup.on_trajectory_cancel(cr);
```

Find them with: `grep -rn "on_trajectory_cancel" tests/`

- [ ] **Step 7: Build and run the whole suite**

Run: `cmake --build build -j && ctest --test-dir build --output-on-failure`
Expected: PASS, same test count as before plus the new `ValueTypes.ArbitrationDefaultsAndResultCodes`.

- [ ] **Step 8: Commit**

```bash
git add include/kinova_lowlevel/interface/value_types.h include/kinova_lowlevel/interface/ports.h \
        include/kinova_lowlevel/interface/supervisor.h src/interface/supervisor.cpp \
        tests/interface/supervisor_test.cpp
git commit -m "feat(interface): capability-token value types, ArbitrationSink port, on_halt"
```

---

### Task 2: Arbiter — grant, token minting, admission

**Files:**
- Create: `include/kinova_lowlevel/interface/arbiter.h`
- Create: `src/interface/arbiter.cpp`
- Create: `tests/interface/arbiter_test.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: everything from Task 1.
- Produces: `Arbiter(CommandSink& downstream, ArbitrationMode mode, uint64_t seed = 0)` implementing both `CommandSink` and `ArbitrationSink`. Tasks 3, 4 and 6 extend and use it.

- [ ] **Step 1: Write the failing test**

Create `tests/interface/arbiter_test.cpp`:

```cpp
#include <gtest/gtest.h>
#include "kinova_lowlevel/interface/arbiter.h"

using namespace kinova;
using namespace kinova::interface;

namespace {
// Records what actually reached the downstream sink. Asserting on the ABSENCE of a
// delegate is the whole point: a rejected command must not merely return an error,
// it must never touch the Supervisor.
struct RecordingSink : public CommandSink {
  int goals=0, accepted=0, cancels=0, gains=0, queries=0;
  std::vector<HaltReason> halts;
  GoalResponse   on_trajectory_goal(const TrajectoryGoal&) override { ++goals; return GoalResponse::kAccept; }
  void           on_trajectory_accepted(const GoalId&, const TrajectoryGoal&) override { ++accepted; }
  CancelResponse on_trajectory_cancel(const CancelRequest&) override { ++cancels; return CancelResponse::kAccept; }
  GainsResult    on_set_gains(const GainsRequest&) override { ++gains; return {true, ""}; }
  ArmState       on_query_state() override { ++queries; ArmState s; s.stamp_s = 42.0; return s; }
  void           on_halt(HaltReason r) override { halts.push_back(r); }
};
TrajectoryGoal goal_with(const Token& t){ TrajectoryGoal g; g.token = t; return g; }
}  // namespace

TEST(Arbiter, EnforcedRejectsCommandWithoutGrant) {
  RecordingSink sink; Arbiter arb{sink, ArbitrationMode::kEnforced, 1234};
  EXPECT_EQ(arb.on_trajectory_goal(goal_with(Token{})), GoalResponse::kRejectUnauthorized);
  EXPECT_EQ(sink.goals, 0);                       // never reached the Supervisor
  EXPECT_EQ(arb.status().rejected_count, 1u);
}

TEST(Arbiter, GrantMintsUniqueTokensAndIncrementsGeneration) {
  RecordingSink sink; Arbiter arb{sink, ArbitrationMode::kEnforced, 1234};
  const GrantResult a = arb.grant("planner");
  ASSERT_TRUE(a.accepted);
  EXPECT_EQ(a.generation, 1u);
  EXPECT_NE(a.token, (Token{}));                  // not left zeroed
  const GrantResult b = arb.grant("teleop");
  ASSERT_TRUE(b.accepted);
  EXPECT_EQ(b.generation, 2u);
  EXPECT_NE(b.token, a.token);                    // fresh per grant
  EXPECT_EQ(arb.status().owner_id, "teleop");
}

TEST(Arbiter, CorrectTokenDelegatesExactlyOnce) {
  RecordingSink sink; Arbiter arb{sink, ArbitrationMode::kEnforced, 1234};
  const Token t = arb.grant("planner").token;
  EXPECT_EQ(arb.on_trajectory_goal(goal_with(t)), GoalResponse::kAccept);
  EXPECT_EQ(sink.goals, 1);
  EXPECT_EQ(arb.status().rejected_count, 0u);
}

TEST(Arbiter, WrongTokenRejectedAndNotDelegated) {
  RecordingSink sink; Arbiter arb{sink, ArbitrationMode::kEnforced, 1234};
  arb.grant("planner");
  Token wrong{}; wrong[0] = 0xAB;
  EXPECT_EQ(arb.on_trajectory_goal(goal_with(wrong)), GoalResponse::kRejectUnauthorized);
  EXPECT_EQ(sink.goals, 0);
}

TEST(Arbiter, StaleTokenRejectedAfterRegrant) {   // the zombie-node case
  RecordingSink sink; Arbiter arb{sink, ArbitrationMode::kEnforced, 1234};
  const Token old = arb.grant("planner").token;
  arb.grant("teleop");                            // ownership moved on
  EXPECT_EQ(arb.on_trajectory_goal(goal_with(old)), GoalResponse::kRejectUnauthorized);
  EXPECT_EQ(sink.goals, 0);
}

TEST(Arbiter, QueryStateIsNeverGated) {
  RecordingSink sink; Arbiter arb{sink, ArbitrationMode::kEnforced, 1234};
  EXPECT_NEAR(arb.on_query_state().stamp_s, 42.0, 1e-9);   // no grant at all
  EXPECT_EQ(sink.queries, 1);
}

TEST(Arbiter, DisabledModeAdmitsAnyToken) {
  RecordingSink sink; Arbiter arb{sink, ArbitrationMode::kDisabled, 1234};
  EXPECT_EQ(arb.on_trajectory_goal(goal_with(Token{})), GoalResponse::kAccept);
  EXPECT_EQ(sink.goals, 1);
  EXPECT_EQ(arb.status().mode, ArbitrationMode::kDisabled);   // visible, not just logged
}

TEST(Arbiter, SetGainsIsGated) {
  RecordingSink sink; Arbiter arb{sink, ArbitrationMode::kEnforced, 1234};
  GainsRequest r;                                  // zero token
  EXPECT_FALSE(arb.on_set_gains(r).accepted);
  EXPECT_EQ(sink.gains, 0);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `./build/unit_tests --gtest_filter='Arbiter*'`
Expected: compile error — `kinova_lowlevel/interface/arbiter.h` does not exist.

- [ ] **Step 3: Write the header**

Create `include/kinova_lowlevel/interface/arbiter.h`:

```cpp
#pragma once
#include <mutex>
#include <random>
#include <string>
#include "kinova_lowlevel/interface/ports.h"
namespace kinova::interface {

// Decides WHO may command the arm, and nothing else.
//
// Decorates a CommandSink: every inbound command carries a Token; the Arbiter
// compares it against the live grant and either delegates downstream or rejects.
// It includes nothing from controls -- no ControlMode, no RtExecutor, no Dynamics,
// no Transport -- so the safety-critical admission logic is unit-testable against a
// fake sink with no robot, no URDF and no threads. Same idiom as FeedbackTap
// decorating Transport.
//
// Lock discipline (spec: "Thread safety"): the mutex IS held across command
// delegation, so admit-and-deliver is atomic against revoke()/estop() -- otherwise a
// command admitted a moment before a revoke could reach the Supervisor AFTER the
// halt and restart a stopped arm. It is NEVER held across on_halt().
class Arbiter : public CommandSink, public ArbitrationSink {
 public:
  // seed == 0 -> seed the token RNG from std::random_device.
  Arbiter(CommandSink& downstream, ArbitrationMode mode, uint64_t seed = 0);

  // ArbitrationSink
  GrantResult       grant(const std::string& owner_id) override;
  void              revoke() override;
  void              estop() override;
  void              estop_clear() override;
  ArbitrationStatus status() const override;

  // CommandSink
  GoalResponse   on_trajectory_goal(const TrajectoryGoal&) override;
  void           on_trajectory_accepted(const GoalId&, const TrajectoryGoal&) override;
  CancelResponse on_trajectory_cancel(const CancelRequest&) override;
  GainsResult    on_set_gains(const GainsRequest&) override;
  ArmState       on_query_state() override;       // never gated -- reads are always open
  void           on_halt(HaltReason) override;    // pass-through

 private:
  bool  admit(const Token&) const;   // caller holds m_
  Token mint();                      // caller holds m_

  CommandSink&    down_;
  ArbitrationMode mode_;
  mutable std::mutex m_;
  std::mt19937_64 rng_;
  bool        owned_ = false;
  bool        estopped_ = false;
  Token       token_{};
  std::string owner_id_;
  uint64_t    generation_ = 0;
  uint64_t    rejected_ = 0;
};
}  // namespace kinova::interface
```

- [ ] **Step 4: Write the implementation**

Create `src/interface/arbiter.cpp`:

```cpp
#include "kinova_lowlevel/interface/arbiter.h"
#include <cstring>
namespace kinova::interface {

Arbiter::Arbiter(CommandSink& downstream, ArbitrationMode mode, uint64_t seed)
  : down_(downstream), mode_(mode), rng_(seed ? seed : std::random_device{}()) {}

Token Arbiter::mint() {
  Token t{}; const uint64_t a = rng_(), b = rng_();
  std::memcpy(t.data(), &a, sizeof(a)); std::memcpy(t.data() + 8, &b, sizeof(b));
  return t;
}

// Admitted iff the bypass is on, or the command carries the live token.
// E-stop latches over BOTH -- it is the one thing kDisabled does not bypass.
bool Arbiter::admit(const Token& t) const {
  if (estopped_) return false;
  if (mode_ == ArbitrationMode::kDisabled) return true;
  return owned_ && t == token_;
}

GrantResult Arbiter::grant(const std::string& owner_id) {
  bool need_halt = false;
  { std::lock_guard<std::mutex> l(m_);
    if (estopped_) return {false, Token{}, generation_, "e-stopped"};
    // A re-grant is revoke-then-grant, never a silent swap under a moving arm.
    if (owned_) { owned_ = false; token_ = Token{}; need_halt = true; } }
  if (need_halt) down_.on_halt(HaltReason::kOwnershipRevoked);   // lock NOT held
  std::lock_guard<std::mutex> l(m_);
  if (estopped_) return {false, Token{}, generation_, "e-stopped"};  // raced an estop in the halt window
  token_ = mint(); owned_ = true; owner_id_ = owner_id; ++generation_;
  return {true, token_, generation_, ""};
}

void Arbiter::revoke() {
  bool need_halt = false;
  { std::lock_guard<std::mutex> l(m_);
    if (owned_) { owned_=false; token_=Token{}; owner_id_.clear(); need_halt=true; } }
  if (need_halt) down_.on_halt(HaltReason::kOwnershipRevoked);
}

void Arbiter::estop() {
  { std::lock_guard<std::mutex> l(m_);
    estopped_ = true; owned_ = false; token_ = Token{}; owner_id_.clear(); }
  down_.on_halt(HaltReason::kEmergencyStop);   // unconditional: e-stop always halts
}

void Arbiter::estop_clear() {
  std::lock_guard<std::mutex> l(m_);
  estopped_ = false;              // exits to no-owner, never straight back to owned
}

ArbitrationStatus Arbiter::status() const {
  std::lock_guard<std::mutex> l(m_);
  return {mode_, estopped_, owned_, owner_id_, generation_, rejected_};
}

GoalResponse Arbiter::on_trajectory_goal(const TrajectoryGoal& g) {
  std::lock_guard<std::mutex> l(m_);
  if (!admit(g.token)) { ++rejected_; return GoalResponse::kRejectUnauthorized; }
  return down_.on_trajectory_goal(g);
}
void Arbiter::on_trajectory_accepted(const GoalId& id, const TrajectoryGoal& g) {
  std::lock_guard<std::mutex> l(m_);
  // Re-checked: never trust that a matching on_trajectory_goal preceded this call.
  if (!admit(g.token)) { ++rejected_; return; }
  down_.on_trajectory_accepted(id, g);
}
CancelResponse Arbiter::on_trajectory_cancel(const CancelRequest& c) {
  std::lock_guard<std::mutex> l(m_);
  // Cancel is gated: a stranger must not be able to stop your motion. The
  // emergency path is estop(), not cancel.
  if (!admit(c.token)) { ++rejected_; return CancelResponse::kReject; }
  return down_.on_trajectory_cancel(c);
}
GainsResult Arbiter::on_set_gains(const GainsRequest& r) {
  std::lock_guard<std::mutex> l(m_);
  if (!admit(r.token)) { ++rejected_; return {false, "not authorized"}; }
  return down_.on_set_gains(r);
}
ArmState Arbiter::on_query_state() { return down_.on_query_state(); }
void     Arbiter::on_halt(HaltReason r) { down_.on_halt(r); }
}  // namespace kinova::interface
```

- [ ] **Step 5: Register in CMake**

In `CMakeLists.txt`, add to `KINOVA_LIB_SOURCES` next to `src/interface/supervisor.cpp`:

```cmake
    src/interface/arbiter.cpp
```

and to the `unit_tests` source list next to `tests/interface/supervisor_test.cpp`:

```cmake
    tests/interface/arbiter_test.cpp
```

- [ ] **Step 6: Run tests to verify they pass**

Run: `cmake -S . -B build -DCMAKE_PREFIX_PATH=/usr/local/lib/python3.10/dist-packages/cmeel.prefix && cmake --build build -j && ./build/unit_tests --gtest_filter='Arbiter*'`
Expected: 8 tests PASS.

- [ ] **Step 7: Commit**

```bash
git add include/kinova_lowlevel/interface/arbiter.h src/interface/arbiter.cpp \
        tests/interface/arbiter_test.cpp CMakeLists.txt
git commit -m "feat(interface): Arbiter — token minting and command admission"
```

---

### Task 3: Arbiter — revoke and the halt handshake

**Files:**
- Modify: `tests/interface/arbiter_test.cpp`

**Interfaces:**
- Consumes: `Arbiter` from Task 2. No production change is expected — `revoke()` and the re-grant halt were written in Task 2; this task proves them. If a test fails, fix `src/interface/arbiter.cpp`.

- [ ] **Step 1: Write the failing tests**

Append to `tests/interface/arbiter_test.cpp`:

```cpp
TEST(Arbiter, RevokeHaltsAndRejectsTheOldToken) {
  RecordingSink sink; Arbiter arb{sink, ArbitrationMode::kEnforced, 1234};
  const Token t = arb.grant("planner").token;
  arb.revoke();
  ASSERT_EQ(sink.halts.size(), 1u);
  EXPECT_EQ(sink.halts[0], HaltReason::kOwnershipRevoked);
  EXPECT_EQ(arb.on_trajectory_goal(goal_with(t)), GoalResponse::kRejectUnauthorized);
  EXPECT_EQ(sink.goals, 0);
  EXPECT_FALSE(arb.status().owned);
}

TEST(Arbiter, RevokeWithNoOwnerDoesNotHalt) {
  RecordingSink sink; Arbiter arb{sink, ArbitrationMode::kEnforced, 1234};
  arb.revoke();
  EXPECT_TRUE(sink.halts.empty());     // nothing to stop; don't jolt the arm for nothing
}

TEST(Arbiter, RegrantHaltsBeforeTheNewGrantIsLive) {
  RecordingSink sink; Arbiter arb{sink, ArbitrationMode::kEnforced, 1234};
  arb.grant("planner");
  const GrantResult second = arb.grant("teleop");
  ASSERT_TRUE(second.accepted);
  ASSERT_EQ(sink.halts.size(), 1u);                 // the swap stopped the arm first
  EXPECT_EQ(sink.halts[0], HaltReason::kOwnershipRevoked);
  EXPECT_EQ(arb.on_trajectory_goal(goal_with(second.token)), GoalResponse::kAccept);
}

TEST(Arbiter, CancelRequiresTheOwnerToken) {
  RecordingSink sink; Arbiter arb{sink, ArbitrationMode::kEnforced, 1234};
  const Token t = arb.grant("planner").token;
  CancelRequest stranger;                                   // zero token
  EXPECT_EQ(arb.on_trajectory_cancel(stranger), CancelResponse::kReject);
  EXPECT_EQ(sink.cancels, 0);
  CancelRequest owner; owner.token = t;
  EXPECT_EQ(arb.on_trajectory_cancel(owner), CancelResponse::kAccept);
  EXPECT_EQ(sink.cancels, 1);
}

TEST(Arbiter, AcceptedRechecksTheTokenIndependently) {
  RecordingSink sink; Arbiter arb{sink, ArbitrationMode::kEnforced, 1234};
  const Token t = arb.grant("planner").token;
  const TrajectoryGoal g = goal_with(t);
  GoalId id{}; id[0] = 7;
  ASSERT_EQ(arb.on_trajectory_goal(g), GoalResponse::kAccept);
  arb.revoke();                                    // ownership lost between accept and accepted
  arb.on_trajectory_accepted(id, g);
  EXPECT_EQ(sink.accepted, 0);                     // must not slip through on a prior accept
}
```

- [ ] **Step 2: Run tests to verify they fail (or reveal a real gap)**

Run: `./build/unit_tests --gtest_filter='Arbiter*'`
Expected: the five new tests fail to compile until built, then PASS if Task 2's implementation is correct. Any FAIL here is a genuine defect in `arbiter.cpp` — fix it there, do not weaken the test.

- [ ] **Step 3: Build and run**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='Arbiter*'`
Expected: 13 tests PASS.

- [ ] **Step 4: Commit**

```bash
git add tests/interface/arbiter_test.cpp
git commit -m "test(interface): Arbiter revoke, re-grant halt, and gated cancel"
```

---

### Task 4: Arbiter — e-stop latch and status

**Files:**
- Modify: `tests/interface/arbiter_test.cpp`

**Interfaces:**
- Consumes: `Arbiter` from Task 2.

- [ ] **Step 1: Write the failing tests**

Append to `tests/interface/arbiter_test.cpp`:

```cpp
TEST(Arbiter, EstopHaltsDropsTheGrantAndRejectsEverything) {
  RecordingSink sink; Arbiter arb{sink, ArbitrationMode::kEnforced, 1234};
  const Token t = arb.grant("planner").token;
  arb.estop();
  ASSERT_EQ(sink.halts.size(), 1u);
  EXPECT_EQ(sink.halts[0], HaltReason::kEmergencyStop);
  EXPECT_EQ(arb.on_trajectory_goal(goal_with(t)), GoalResponse::kRejectUnauthorized);
  EXPECT_EQ(sink.goals, 0);
  EXPECT_TRUE(arb.status().estopped);
  EXPECT_FALSE(arb.status().owned);
}

TEST(Arbiter, EstopLatchesAndRefusesAFreshGrant) {
  RecordingSink sink; Arbiter arb{sink, ArbitrationMode::kEnforced, 1234};
  arb.estop();
  const GrantResult g = arb.grant("planner");
  EXPECT_FALSE(g.accepted);                      // cannot grant your way out of an e-stop
  EXPECT_EQ(g.message, "e-stopped");
}

TEST(Arbiter, EstopRejectsEvenInDisabledMode) {
  RecordingSink sink; Arbiter arb{sink, ArbitrationMode::kDisabled, 1234};
  EXPECT_EQ(arb.on_trajectory_goal(goal_with(Token{})), GoalResponse::kAccept);   // bypass works
  arb.estop();
  EXPECT_EQ(arb.on_trajectory_goal(goal_with(Token{})), GoalResponse::kRejectUnauthorized);
  EXPECT_EQ(sink.goals, 1);                      // the one before the e-stop, and no more
}

TEST(Arbiter, EstopClearExitsToNoOwnerNotToOwned) {
  RecordingSink sink; Arbiter arb{sink, ArbitrationMode::kEnforced, 1234};
  const Token t = arb.grant("planner").token;
  arb.estop();
  arb.estop_clear();
  EXPECT_FALSE(arb.status().estopped);
  EXPECT_FALSE(arb.status().owned);
  EXPECT_EQ(arb.on_trajectory_goal(goal_with(t)), GoalResponse::kRejectUnauthorized);  // old token dead
  const GrantResult g = arb.grant("planner");
  EXPECT_TRUE(g.accepted);                       // but a fresh grant now works
}

TEST(Arbiter, EstopClearWorksInDisabledMode) {
  RecordingSink sink; Arbiter arb{sink, ArbitrationMode::kDisabled, 1234};
  arb.estop();
  arb.estop_clear();                             // must not be able to strand yourself
  EXPECT_EQ(arb.on_trajectory_goal(goal_with(Token{})), GoalResponse::kAccept);
}

TEST(Arbiter, StatusReflectsOwnerAndRejectionCount) {
  RecordingSink sink; Arbiter arb{sink, ArbitrationMode::kEnforced, 1234};
  arb.grant("planner");
  arb.on_trajectory_goal(goal_with(Token{}));    // reject 1
  CancelRequest c; arb.on_trajectory_cancel(c);  // reject 2
  const ArbitrationStatus st = arb.status();
  EXPECT_TRUE(st.owned);
  EXPECT_EQ(st.owner_id, "planner");
  EXPECT_EQ(st.generation, 1u);
  EXPECT_EQ(st.rejected_count, 2u);
}
```

- [ ] **Step 2: Build and run**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='Arbiter*'`
Expected: 19 tests PASS. A FAIL is a real defect in `arbiter.cpp`.

- [ ] **Step 3: Commit**

```bash
git add tests/interface/arbiter_test.cpp
git commit -m "test(interface): Arbiter e-stop latch, clear, and status"
```

---

### Task 5: Supervisor — the halt path

**Files:**
- Modify: `include/kinova_lowlevel/interface/supervisor.h`
- Modify: `src/interface/supervisor.cpp`
- Modify: `include/kinova_lowlevel/sim_transport.h`
- Test: `tests/interface/supervisor_test.cpp`

**Interfaces:**
- Consumes: `HaltReason`, `result_code::kHalted` from Task 1.
- Produces: a working `Supervisor::on_halt(HaltReason)` that settles every dropped goal with `kHalted` and latches hold-at-measured-q. Task 6 depends on it. Also `SimTransport::last_command()`.

- [ ] **Step 1: Write the failing tests**

Append to `tests/interface/supervisor_test.cpp` (uses the existing `SupFix` fixture and `ramp7` helper):

```cpp
TEST(Supervisor, HaltSettlesTheActiveGoalAsHalted) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::TrajectoryGoal g; g.trajectory = ramp7(0.0, 0.1, 2.0);
  g.control_mode = interface::ControlModeKind::kPosition;
  g.preemption   = interface::Preemption::kLatestWins;
  g.path_tolerance = JointVec::Constant(-1.0);
  interface::GoalId id{}; id[0] = 3;
  ASSERT_EQ(f.sup.on_trajectory_goal(g), interface::GoalResponse::kAccept);
  f.sup.on_trajectory_accepted(id, g);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));   // mid-motion
  f.sup.on_halt(interface::HaltReason::kOwnershipRevoked);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  f.sup.stop(); f.teardown();
  ASSERT_EQ(f.be.result_count(), 1u);
  EXPECT_EQ(f.be.last_result().error_code, interface::result_code::kHalted);
  EXPECT_EQ(f.be.last_result_id()[0], 3);
}

TEST(Supervisor, HaltSettlesTheQueuedGoalToo) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::TrajectoryGoal g; g.trajectory = ramp7(0.0, 0.1, 2.0);
  g.control_mode = interface::ControlModeKind::kPosition;
  g.path_tolerance = JointVec::Constant(-1.0);
  g.preemption = interface::Preemption::kLatestWins;
  interface::GoalId a{}; a[0] = 1;
  f.sup.on_trajectory_goal(g); f.sup.on_trajectory_accepted(a, g);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  interface::TrajectoryGoal q = g; q.preemption = interface::Preemption::kQueue;
  interface::GoalId b{}; b[0] = 2;
  f.sup.on_trajectory_goal(q); f.sup.on_trajectory_accepted(b, q);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  f.sup.on_halt(interface::HaltReason::kEmergencyStop);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  f.sup.stop(); f.teardown();
  // Both were ACCEPTed, so both must settle -- dropping the queued one silently
  // orphans a client that waits forever (the a835bf5 / d0791df bug class).
  ASSERT_EQ(f.be.result_count(), 2u);
  for (const auto& [gid, res] : f.be.all_results())
    EXPECT_EQ(res.error_code, interface::result_code::kHalted);
}

TEST(Supervisor, HaltLatchesTheTargetAtMeasuredQ) {
  SupFix f(0.0); f.sup.start(); f.run_rt();
  interface::TrajectoryGoal g; g.trajectory = ramp7(0.0, 0.5, 4.0);   // slow ramp away from 0
  g.control_mode = interface::ControlModeKind::kPosition;
  g.preemption   = interface::Preemption::kLatestWins;
  g.path_tolerance = JointVec::Constant(-1.0);
  interface::GoalId id{}; id[0] = 5;
  f.sup.on_trajectory_goal(g); f.sup.on_trajectory_accepted(id, g);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  const double moving = f.sim.last_command().q[0];
  EXPECT_GT(moving, 1e-3);                       // the reference had actually left 0
  f.sup.on_halt(interface::HaltReason::kEmergencyStop);
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  f.sup.stop(); f.teardown();
  // SimTransport is a static echo: measured q never leaves the seed, so a hold at
  // MEASURED q must snap the command back to it -- not park at the last reference.
  EXPECT_NEAR(f.sim.last_command().q[0], 0.0, 1e-6);
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='Supervisor.Halt*'`
Expected: compile error on `f.sim.last_command()`, then FAIL — `on_halt` is the Task 1 stub, so no goal ever settles (`result_count() == 0`).

- [ ] **Step 3: Add the SimTransport accessor**

In `include/kinova_lowlevel/sim_transport.h`, in the public section:

```cpp
  // Test/bench observability: the last command the loop wrote. Read it only after
  // the RT thread is joined -- it is not synchronised.
  const JointCommand& last_command() const { return last_cmd_; }
```

- [ ] **Step 4: Declare the halt state on the Supervisor**

In `include/kinova_lowlevel/interface/supervisor.h`, add to the private section:

```cpp
  kinova::JointTargetSink& active_sink();          // pos_ or imp_, per active_mode_kind_
  bool       halt_pending_ = false;                // guarded by q_mtx_
  HaltReason halt_reason_  = HaltReason::kOwnershipRevoked;   // guarded by q_mtx_
```

and add `#include "kinova_lowlevel/joint_target_sink.h"` if it is not already reachable.

- [ ] **Step 5: Implement the latch and the sampler branch**

In `src/interface/supervisor.cpp`, add near the top of the namespace:

```cpp
static const char* halt_reason_string(HaltReason r) {
  switch (r) {
    case HaltReason::kOwnershipRevoked: return "halted: ownership revoked";
    case HaltReason::kEmergencyStop:    return "halted: emergency stop";
    case HaltReason::kOperatorRequest:  return "halted: operator request";
  }
  return "halted";
}
```

Replace the Task 1 stub:

```cpp
// on_halt (backend thread): latch + flush the queue, nothing else. The sampler owns
// traj_ and settle(), so the control action happens there -- which keeps
// settle-exactly-once true by construction rather than by careful reasoning.
void Supervisor::on_halt(HaltReason r) {
  std::lock_guard<std::mutex> l(q_mtx_);
  inbox_.clear();                 // a halt must never sit behind queued trajectories
  halt_reason_ = r;
  halt_pending_ = true;
}

kinova::JointTargetSink& Supervisor::active_sink() {
  return active_mode_kind_ == ControlModeKind::kImpedance
         ? static_cast<kinova::JointTargetSink&>(imp_)
         : static_cast<kinova::JointTargetSink&>(pos_);
}
```

In `sampler_loop`, insert this **immediately before** the `// 1) drain inbox` block, inside the `while (running_...)` loop:

```cpp
    // 0) a halt jumps the queue: settle everything ACCEPTed, then hold where the arm IS.
    bool halt = false; HaltReason hr = HaltReason::kOwnershipRevoked;
    { std::lock_guard<std::mutex> l(q_mtx_);
      if (halt_pending_) { halt = true; hr = halt_reason_; halt_pending_ = false; } }
    if (halt) {
      TrajectoryResult r; r.error_code = result_code::kHalted; r.error_string = halt_reason_string(hr);
      if (have_active) action_.settle(active_id, r);
      if (have_queued) action_.settle(queued_id, r);   // ACCEPTed already; dropping it orphans the client
      traj_.emplace(active_sink());
      have_active = false; have_queued = false; in_flight_.store(false);
      JointFeedback fb; const bool ok = snap_.load(fb);
      q_meas = sampled_q(ok, fb.q, q_meas);            // never inject a phantom zero here of all places
      active_sink().set_target(q_meas);                // hold at MEASURED q, not the last reference
    }
```

- [ ] **Step 6: Run tests to verify they pass**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='Supervisor*'`
Expected: all `Supervisor.*` tests PASS, including the three new `Halt` tests.

- [ ] **Step 7: Run the whole suite**

Run: `ctest --test-dir build --output-on-failure`
Expected: PASS.

- [ ] **Step 8: Commit**

```bash
git add include/kinova_lowlevel/interface/supervisor.h src/interface/supervisor.cpp \
        include/kinova_lowlevel/sim_transport.h tests/interface/supervisor_test.cpp
git commit -m "feat(interface): Supervisor halt path — settle all dropped goals, hold at measured q"
```

---

### Task 6: Integration — a real Arbiter in front of a real Supervisor

**Files:**
- Modify: `tests/interface/execution_integration_test.cpp`

**Interfaces:**
- Consumes: `Arbiter` (Task 2) and `Supervisor::on_halt` (Task 5).

- [ ] **Step 1: Extract the shared fixture**

`SupFix` and `ramp7` currently live in an **anonymous namespace** in `tests/interface/supervisor_test.cpp`, so they are not reachable from another translation unit. Move both into a new `tests/interface/sup_fixture.h` (drop the anonymous namespace; make `ramp7` `inline`), and replace their definitions in `supervisor_test.cpp` with `#include "sup_fixture.h"`.

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='Supervisor*'`
Expected: PASS, unchanged behaviour — this step is a pure move.

- [ ] **Step 2: Write the failing tests**

Append to `tests/interface/execution_integration_test.cpp`, including `sup_fixture.h` and `kinova_lowlevel/interface/arbiter.h`:

```cpp
TEST(ArbitrationIntegration, RevokeMidMotionHaltsAndSettlesExactlyOnce) {
  SupFix f;
  interface::Arbiter arb{f.sup, interface::ArbitrationMode::kEnforced, 99};
  f.sup.start(); f.run_rt();
  const interface::Token t = arb.grant("planner").token;

  interface::TrajectoryGoal g; g.trajectory = ramp7(0.0, 0.4, 3.0);
  g.control_mode = interface::ControlModeKind::kPosition;
  g.preemption   = interface::Preemption::kLatestWins;
  g.path_tolerance = JointVec::Constant(-1.0);
  g.token = t;
  interface::GoalId id{}; id[0] = 11;
  ASSERT_EQ(arb.on_trajectory_goal(g), interface::GoalResponse::kAccept);
  arb.on_trajectory_accepted(id, g);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  arb.revoke();                                  // ownership pulled mid-motion
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // A command from the ex-owner after the revoke must not restart the arm.
  EXPECT_EQ(arb.on_trajectory_goal(g), interface::GoalResponse::kRejectUnauthorized);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  f.sup.stop(); f.teardown();

  ASSERT_EQ(f.be.result_count(), 1u);            // exactly once, not zero and not twice
  EXPECT_EQ(f.be.last_result().error_code, interface::result_code::kHalted);
  EXPECT_NEAR(f.sim.last_command().q[0], 0.0, 1e-6);   // held at measured q
}

TEST(ArbitrationIntegration, ANewOwnerCanSwitchControlModeAfterAHalt) {
  SupFix f;
  interface::Arbiter arb{f.sup, interface::ArbitrationMode::kEnforced, 99};
  f.sup.start(); f.run_rt();

  const interface::Token a = arb.grant("planner").token;
  interface::TrajectoryGoal gp; gp.trajectory = ramp7(0.0, 0.4, 3.0);
  gp.control_mode = interface::ControlModeKind::kPosition;
  gp.preemption = interface::Preemption::kLatestWins;
  gp.path_tolerance = JointVec::Constant(-1.0); gp.token = a;
  interface::GoalId ida{}; ida[0] = 21;
  arb.on_trajectory_goal(gp); arb.on_trajectory_accepted(ida, gp);
  std::this_thread::sleep_for(std::chrono::milliseconds(250));

  const interface::Token b = arb.grant("teleop").token;   // re-grant halts, then grants
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // After a halt the arm is at rest, so mode-switch-at-rest is satisfied and an
  // impedance goal from the NEW owner is accepted.
  interface::TrajectoryGoal gi; gi.trajectory = ramp7(0.0, 0.05, 1.0);
  gi.control_mode = interface::ControlModeKind::kImpedance;
  gi.preemption = interface::Preemption::kLatestWins;
  gi.path_tolerance = JointVec::Constant(-1.0); gi.token = b;
  interface::GoalId idb{}; idb[0] = 22;
  ASSERT_EQ(arb.on_trajectory_goal(gi), interface::GoalResponse::kAccept);
  arb.on_trajectory_accepted(idb, gi);
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  f.sup.stop(); f.teardown();

  const auto results = f.be.all_results();
  ASSERT_EQ(results.size(), 2u);
  EXPECT_EQ(results[0].second.error_code, interface::result_code::kHalted);      // the halted position goal
  EXPECT_EQ(results[1].second.error_code, interface::result_code::kSuccessful);  // the new owner's goal
}
```

Add `#include "kinova_lowlevel/interface/arbiter.h"` and `#include "sup_fixture.h"` at the top of the file.

- [ ] **Step 3: Run tests to verify they fail**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='ArbitrationIntegration*'`
Expected: FAIL (or compile error) until the fixture is wired.

- [ ] **Step 4: Make them pass**

No new production code should be needed. If `ANewOwnerCanSwitchControlModeAfterAHalt` fails on the mode switch, check that the halt branch cleared `in_flight_` — `on_trajectory_goal`'s cross-mode pre-check reads it.

- [ ] **Step 5: Run the whole suite**

Run: `ctest --test-dir build --output-on-failure`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add tests/interface/execution_integration_test.cpp tests/interface/sup_fixture.h tests/interface/supervisor_test.cpp
git commit -m "test(interface): arbitration integration — revoke mid-motion, re-arm in a new mode"
```

---

### Task 7: RT safety, benchmark evidence, and docs

**Files:**
- Create: `docs/guide/arbitration.md`
- Modify: `mkdocs.yml`
- Modify: `docs/reference/api.md`

**Interfaces:**
- Consumes: everything above.

- [ ] **Step 1: Run the RT-safety gate and read the result**

Run: `./build/unit_tests --gtest_filter='RtSafety*'`
Expected: all PASS — zero major page faults, zero dropped samples. Record the actual output; do not infer it from the fact that `compute()` was untouched.

- [ ] **Step 2: Capture benchmark numbers**

Run: `./build/benchmark_cartesian_impedance --sim --urdf models/gen3_7dof_2f85.urdf --rate 1000 --duration 5`

Record p50 / p99 / p99.9 / max, overruns and faults, and compare against a run from `origin/main`. (Do **not** use the documented `benchmark_grav_comp` invocation — it currently throws, core issue #18: the default EE frame is absent from the bare-arm URDF.)

- [ ] **Step 3: Write the guide page**

Create `docs/guide/arbitration.md` covering: what a grant is and who issues it; the `enforced` / `disabled` modes and why `disabled` is visible in status rather than only in a log; the token lifecycle (driver-minted, fresh per grant, void on restart); hard revoke and what the arm does; e-stop and its latch, including that it is **not** a hardware safety chain; and the accepted gaps from the spec (orchestrator death freezes ownership; mistake boundary not security boundary).

- [ ] **Step 4: Wire it into the nav**

In `mkdocs.yml`, add `arbitration.md` to the `guide/` section of `nav:` — an orphaned page will not appear on the site.

- [ ] **Step 5: Add the interfaces to the API reference**

In `docs/reference/api.md`, document `ArbitrationSink`, `CommandSink::on_halt`, `HaltReason`, `Token`, and `CancelRequest` alongside the existing port documentation.

- [ ] **Step 6: Verify the docs build**

Run: `mkdocs build`
Expected: no warnings about the new page being absent from the nav.

- [ ] **Step 7: Commit**

```bash
git add docs/guide/arbitration.md mkdocs.yml docs/reference/api.md
git commit -m "docs(arbitration): guide page, API reference, nav entry"
```

---

## Note for whoever rebases core PR #15 (feedforward)

The spec's halt path has five steps; **step 4 (zero the reference velocity) is not implementable on current `main`** — `JointTargetSink::set_target(const JointVec&)` carries position only, and there is no `qd_ref` yet. This plan therefore implements steps 1, 2, 3 and 5.

When PR #15 rebases onto this work, the halt branch in `sampler_loop` **must** also zero the reference velocity. With a stale `qd_ref` left over from the aborted trajectory, the feedforward term keeps commanding motion at exactly the moment we are trying to stop. With `qd_ref = 0` and target = measured q, the damping term becomes `-Dq*qd`, which actively brakes.

## Spec sections deliberately not implemented here

- **Component 5 (DI wiring).** Nothing in *this* repo constructs a `Supervisor` — `apps/trajectory_run.cpp` drives a `TrajectoryExecutor` directly, and the `Supervisor` is wired in the ROS2 repo's bring-up node. The three-line wiring in the spec therefore lands with the ROS2 work, not here.
- **`result_code::kNotAuthorized` has no core consumer.** In the core, an unauthorized command is refused via `GoalResponse::kRejectUnauthorized`, which produces no result. `kNotAuthorized` is defined and asserted here as a **contract constant** for the backend, which is the side that produces a result message.
- **Tier 5 (attended hardware).** Grant → move → revoke mid-motion on the real arm, and `/estop` during motion, per `docs/integration-runbook.md`. Cannot run unattended and is not part of this plan; it gates the ROS2 follow-up, not this merge.

## Out of scope for this plan

- The `kinova_arm_ros2` side (services, `/estop` topic, status publication, the `RCLCPP_WARN` reject logging, and the `on_halt`/`CancelRequest` updates to its fakes). Core lands first; that repo cannot build until it does — the same sequencing as `kPlanningFailed` / `has_velocities`.
- The streaming-setpoint tier, which gets its own brainstorm and spec.
