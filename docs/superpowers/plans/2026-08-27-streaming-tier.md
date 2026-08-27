# Streaming Tier Implementation Plan (Plan 3)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let an owner open a streaming session and push setpoints at its own rate into a running control mode, with an explicit and bounded safe-stop when the stream stops.

**Architecture:** A new `StreamSink` driving port carries session open/close plus one typed method per setpoint shape. A `StreamingSession` unit holds the session state as pure logic over injected time. Because sessions and trajectory goals are mutually exclusive, the sampler writes no targets while a session is open — so setpoints are written to the mode's target sink **directly from the backend thread**, with a mark-then-act handoff at open, close and timeout. Staleness is enforced twice from one deadline: each mode makes its own output safe at 1 kHz, and the session handles lifecycle at sampler rate.

**Tech Stack:** C++17, Eigen, GoogleTest, CMake. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-08-26-streaming-setpoints-design.md` (Plan 3 of its decomposition)

## Global Constraints

- **Builds on Linux/aarch64 only (the Jetson, `abra`).** x86_64 dev boxes cannot build — no Pinocchio. Every "green" claim needs a real build+ctest on the Jetson.
- Build: `cmake -S . -B build -DCMAKE_PREFIX_PATH=/usr/local/lib/python3.10/dist-packages/cmeel.prefix && cmake --build build -j && ctest --test-dir build --output-on-failure`
- Subset: `./build/unit_tests --gtest_filter='Streaming*'` — all tests are one gtest binary, one ctest test.
- **Nothing in the RT path (`compute`, executor cycle) may allocate, lock, or block, and `compute()` must make NO clock calls.** Staleness is tracked by summing the `dt_s` argument against a monotonic write counter — the mechanism `JointTorqueMode` already uses.
- **Target publication is single-writer → single-reader** via a double-buffer plus an atomic index. The RT reader must never observe a torn value.
- **`timeout_s <= 0` is REJECTED at open.** An unbounded stream has no safe-stop; the deadline is not optional.
- SI / radians internally. `kNumJoints = 7`. Fail loud rather than degrade silently at 1 kHz.
- There is **no CI**. The Jetson is the only gate.
- Work on a branch off `main`: `git checkout -b feat/streaming-tier origin/main` (main is `13e0c89`, with both #25 and #26 merged).

## Scope: three of five setpoint kinds

`JointVelocityMode` (Plan 2) does not exist, and `JointPositionMode` has no pose path (also Plan 2). So this plan lands:

| setpoint kind | permitted `control_mode` | status in this plan |
| --- | --- | --- |
| joint position | `kPosition`, `kImpedance` | **supported** — `JointTargetSink` on both |
| EE pose | `kImpedance` | **supported** — `PoseTargetSink` on impedance |
| EE pose | `kPosition` | **rejected at open** — needs Plan 2 |
| joint torque | `kTorque` | **supported** — `JointTorqueMode::set_torque` |
| joint velocity, EE twist | `kVelocity` | **rejected at open** — needs Plan 2 |

Rejection is the valid-pair table doing its job, not a stub. Adding the rest later is filling in table rows.

## File Structure

| File | Responsibility |
|---|---|
| `include/kinova_lowlevel/interface/value_types.h` (modify) | `SetpointKind`, setpoint structs, stream request/result types; extend `ControlModeKind` |
| `include/kinova_lowlevel/interface/ports.h` (modify) | the `StreamSink` driving port |
| `include/kinova_lowlevel/interface/streaming_session.h` (create) | session state machine — pure logic, injected time |
| `src/interface/streaming_session.cpp` (create) | its implementation |
| `include/kinova_lowlevel/interface/arbiter.h`, `src/interface/arbiter.cpp` (modify) | `Arbiter` also decorates `StreamSink` |
| `include/kinova_lowlevel/interface/supervisor.h`, `src/interface/supervisor.cpp` (modify) | implement `StreamSink`; hold a `JointTorqueMode&`; goal/stream mutual exclusion; teardown |
| `include/kinova_lowlevel/joint_position_mode.h`, `src/joint_position_mode.cpp` (modify) | staleness watchdog → freeze reference at measured q |
| `include/kinova_lowlevel/joint_impedance_mode.h`, `src/joint_impedance_mode.cpp` (modify) | same watchdog |
| `tests/interface/streaming_session_test.cpp` (create) | Tier-1 session unit tests |
| `tests/interface/supervisor_test.cpp` (modify) | integration: streaming through a real Supervisor, **and** the Tier-2 write-handoff race (the `SupFix` fixture lives here) |
| `tests/rt_safety_test.cpp` (modify) | supervisor-in-loop with a session open |
| `docs/guide/streaming.md` (create), `mkdocs.yml`, `docs/reference/api.md` (modify) | docs |

---

### Task 1: Value types and the `StreamSink` port

The breaking interface change lands here; behaviour comes later. `Supervisor` gets stubs so the suite stays green.

**Files:**
- Modify: `include/kinova_lowlevel/interface/value_types.h`, `include/kinova_lowlevel/interface/ports.h`, `include/kinova_lowlevel/interface/supervisor.h`, `src/interface/supervisor.cpp`
- Test: `tests/interface/supervisor_test.cpp`

**Interfaces:**
- Produces: `SetpointKind`, `StreamOpenRequest`, `StreamOpenResult`, `StreamCloseRequest`, `JointSetpoint`, `PoseSetpoint`, `TwistSetpoint`, `StreamSink`, `ControlModeKind::kVelocity`, `ControlModeKind::kTorque`, `result_code::kStreamRejected = -10`. Tasks 2–7 consume these.

- [ ] **Step 1: Write the failing test**

Append to `tests/interface/supervisor_test.cpp`:

```cpp
TEST(ValueTypes, StreamingDefaultsAndResultCodes) {
  EXPECT_EQ(interface::result_code::kStreamRejected, -10);
  interface::StreamOpenRequest r;
  EXPECT_EQ(r.kind, interface::SetpointKind::kJointPosition);
  EXPECT_EQ(r.control_mode, interface::ControlModeKind::kPosition);
  EXPECT_NEAR(r.timeout_s, 0.1, 1e-12);          // a deadline is mandatory, so it has a default
  EXPECT_EQ(r.token, (interface::Token{}));
  interface::StreamOpenResult res;
  EXPECT_FALSE(res.accepted);
  interface::JointSetpoint js;
  EXPECT_TRUE(js.values.isZero());
  interface::TwistSetpoint ts;
  EXPECT_TRUE(ts.twist.isZero());                // Vector6, [linear; angular]
  EXPECT_EQ(ts.token, (interface::Token{}));
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `./build/unit_tests --gtest_filter='ValueTypes.Streaming*'`
Expected: compile error — none of those types are declared.

- [ ] **Step 3: Add the value types**

In `include/kinova_lowlevel/interface/value_types.h`, after the `HaltReason` enum:

```cpp
// What a streaming client sends. The METHOD on StreamSink disambiguates which
// struct applies -- there is deliberately no tag field on the setpoint itself,
// so "kind says pose, pose field is garbage" is not representable.
enum class SetpointKind { kJointPosition, kEePose, kJointVelocity, kEeTwist, kJointTorque };

struct StreamOpenRequest {
  SetpointKind    kind         = SetpointKind::kJointPosition;
  ControlModeKind control_mode = ControlModeKind::kPosition;
  double          timeout_s    = 0.1;   // <= 0 is REJECTED at open: no deadline, no safe-stop
  Token           token{};
};
struct StreamOpenResult   { bool accepted=false; int error_code=0; std::string message; };
struct StreamCloseRequest { Token token{}; };

// One struct, three meanings -- units are per-method: rad (position), rad/s
// (velocity), N*m (feedforward torque).
struct JointSetpoint { JointVec values = JointVec::Zero(); Token token{}; };
struct PoseSetpoint  { Pose     pose{};                    Token token{}; };
struct TwistSetpoint { Vector6  twist = Vector6::Zero();   Token token{}; };  // [linear; angular], base frame
```

Add `kStreamRejected = -10` to the `result_code` list.

- [ ] **Step 4: Extend `ControlModeKind` and make the trajectory tier reject the new values**

`ControlModeKind` lives in `include/kinova_lowlevel/interface/trajectory_executor.h`. Add the two new enumerators:

```cpp
enum class ControlModeKind { kPosition, kImpedance, kVelocity, kTorque };
```

Then, in `Supervisor::on_trajectory_goal` (`src/interface/supervisor.cpp`), reject them explicitly — a planned trajectory into a velocity or torque mode is not a supported combination and must fail loud rather than fall through:

```cpp
  if (g.control_mode == ControlModeKind::kVelocity ||
      g.control_mode == ControlModeKind::kTorque) {
    return GoalResponse::kReject;    // trajectory execution is position/impedance only
  }
```

Find the existing pre-check with: `grep -n "on_trajectory_goal" src/interface/supervisor.cpp`

- [ ] **Step 5: Add the `StreamSink` port**

In `include/kinova_lowlevel/interface/ports.h`, after `ArbitrationSink`:

```cpp
// Driving port for the streaming tier. Separate from CommandSink, which is
// already six methods: a backend implements only the concerns it supports.
class StreamSink { public: virtual ~StreamSink() = default;
  virtual StreamOpenResult on_stream_open(const StreamOpenRequest&) = 0;
  virtual void             on_stream_close(const StreamCloseRequest&) = 0;
  virtual void             on_setpoint_joint_position(const JointSetpoint&) = 0;
  virtual void             on_setpoint_joint_velocity(const JointSetpoint&) = 0;
  virtual void             on_setpoint_joint_torque(const JointSetpoint&) = 0;
  virtual void             on_setpoint_pose(const PoseSetpoint&) = 0;
  virtual void             on_setpoint_twist(const TwistSetpoint&) = 0; };
```

- [ ] **Step 6: Stub `StreamSink` on the Supervisor**

In `include/kinova_lowlevel/interface/supervisor.h`, change the class declaration to `class Supervisor : public CommandSink, public StreamSink` and declare the seven methods. In `src/interface/supervisor.cpp`, stub them — real behaviour is Task 6:

```cpp
StreamOpenResult Supervisor::on_stream_open(const StreamOpenRequest&){
  return {false, result_code::kStreamRejected, "streaming not yet wired"};   // Task 6
}
void Supervisor::on_stream_close(const StreamCloseRequest&){}
void Supervisor::on_setpoint_joint_position(const JointSetpoint&){}
void Supervisor::on_setpoint_joint_velocity(const JointSetpoint&){}
void Supervisor::on_setpoint_joint_torque(const JointSetpoint&){}
void Supervisor::on_setpoint_pose(const PoseSetpoint&){}
void Supervisor::on_setpoint_twist(const TwistSetpoint&){}
```

- [ ] **Step 7: Run the whole suite**

Run: `cmake --build build -j && ctest --test-dir build --output-on-failure`
Expected: PASS, previous count plus the new `ValueTypes.StreamingDefaultsAndResultCodes`.

- [ ] **Step 8: Commit**

```bash
git add include/kinova_lowlevel/interface/value_types.h include/kinova_lowlevel/interface/ports.h \
        include/kinova_lowlevel/interface/trajectory_executor.h \
        include/kinova_lowlevel/interface/supervisor.h src/interface/supervisor.cpp \
        tests/interface/supervisor_test.cpp
git commit -m "feat(interface): StreamSink port and streaming value types"
```

---

### Task 2: `StreamingSession`

Pure state machine over injected time. No `Dynamics`, no `Transport`, no threads — so its tests need no robot, no URDF and no clock.

**Files:**
- Create: `include/kinova_lowlevel/interface/streaming_session.h`, `src/interface/streaming_session.cpp`, `tests/interface/streaming_session_test.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: the value types from Task 1.
- Produces:
  ```cpp
  class StreamingSession {
   public:
    StreamOpenResult open(const StreamOpenRequest&, double now_s);
    void             close();
    bool             admit(SetpointKind, double now_s);   // kind must match the declared one
    bool             expired(double now_s) const;
    bool             is_open() const;
    SetpointKind     kind() const;
    ControlModeKind  control_mode() const;
    uint64_t         rejected_count() const;
  };
  bool kinova::interface::pair_supported(SetpointKind, ControlModeKind);
  ```
  Tasks 3, 6 and 7 consume these.

- [ ] **Step 1: Write the failing tests**

Create `tests/interface/streaming_session_test.cpp`:

```cpp
#include <gtest/gtest.h>
#include "kinova_lowlevel/interface/streaming_session.h"

using namespace kinova::interface;

namespace {
StreamOpenRequest req(SetpointKind k, ControlModeKind m, double timeout = 0.1) {
  StreamOpenRequest r; r.kind = k; r.control_mode = m; r.timeout_s = timeout; return r;
}
}  // namespace

TEST(StreamingSession, SupportedPairsOpenAndUnsupportedAreRefused) {
  StreamingSession s;
  EXPECT_TRUE (s.open(req(SetpointKind::kJointPosition, ControlModeKind::kPosition), 0.0).accepted);
  s.close();
  EXPECT_TRUE (s.open(req(SetpointKind::kJointPosition, ControlModeKind::kImpedance), 0.0).accepted);
  s.close();
  EXPECT_TRUE (s.open(req(SetpointKind::kEePose, ControlModeKind::kImpedance), 0.0).accepted);
  s.close();
  EXPECT_TRUE (s.open(req(SetpointKind::kJointTorque, ControlModeKind::kTorque), 0.0).accepted);
  s.close();
  // Plan 2 territory -- refused, not silently degraded.
  EXPECT_FALSE(s.open(req(SetpointKind::kEePose, ControlModeKind::kPosition), 0.0).accepted);
  EXPECT_FALSE(s.open(req(SetpointKind::kEeTwist, ControlModeKind::kVelocity), 0.0).accepted);
  EXPECT_FALSE(s.open(req(SetpointKind::kJointVelocity, ControlModeKind::kVelocity), 0.0).accepted);
  // Nonsense pairing -- a client that thinks it streams twist into impedance is told.
  EXPECT_FALSE(s.open(req(SetpointKind::kEeTwist, ControlModeKind::kImpedance), 0.0).accepted);
}

TEST(StreamingSession, ZeroOrNegativeTimeoutIsRefused) {
  StreamingSession s;
  EXPECT_FALSE(s.open(req(SetpointKind::kJointPosition, ControlModeKind::kPosition, 0.0), 0.0).accepted);
  EXPECT_FALSE(s.open(req(SetpointKind::kJointPosition, ControlModeKind::kPosition, -1.0), 0.0).accepted);
  EXPECT_FALSE(s.is_open());
}

TEST(StreamingSession, SecondOpenIsRefusedCloseFirst) {
  StreamingSession s;
  ASSERT_TRUE(s.open(req(SetpointKind::kJointPosition, ControlModeKind::kPosition), 0.0).accepted);
  EXPECT_FALSE(s.open(req(SetpointKind::kJointPosition, ControlModeKind::kImpedance), 0.0).accepted);
  EXPECT_EQ(s.control_mode(), ControlModeKind::kPosition);   // the first session still owns it
}

TEST(StreamingSession, AdmitRequiresOpenAndMatchingKind) {
  StreamingSession s;
  EXPECT_FALSE(s.admit(SetpointKind::kJointPosition, 0.0));       // closed
  ASSERT_TRUE(s.open(req(SetpointKind::kJointPosition, ControlModeKind::kPosition), 0.0).accepted);
  EXPECT_TRUE (s.admit(SetpointKind::kJointPosition, 0.01));
  EXPECT_FALSE(s.admit(SetpointKind::kEePose, 0.02));             // wrong method for this session
  EXPECT_EQ(s.rejected_count(), 2u);
}

TEST(StreamingSession, ExpiresOnlyAfterTheDeadlineAndFreshSetpointsPushItOut) {
  StreamingSession s;
  ASSERT_TRUE(s.open(req(SetpointKind::kJointPosition, ControlModeKind::kPosition, 0.1), 0.0).accepted);
  EXPECT_FALSE(s.expired(0.09));
  ASSERT_TRUE (s.admit(SetpointKind::kJointPosition, 0.09));      // fresh command resets the clock
  EXPECT_FALSE(s.expired(0.18));
  EXPECT_TRUE (s.expired(0.20));
}

TEST(StreamingSession, RejectedSetpointDoesNotRefreshTheDeadline) {
  StreamingSession s;
  ASSERT_TRUE(s.open(req(SetpointKind::kJointPosition, ControlModeKind::kPosition, 0.1), 0.0).accepted);
  EXPECT_FALSE(s.admit(SetpointKind::kEeTwist, 0.09));            // wrong kind -> rejected
  EXPECT_TRUE (s.expired(0.11));   // still measured from open, not from the rejected setpoint
}

TEST(StreamingSession, CloseReturnsToClosedAndRefusesFurtherSetpoints) {
  StreamingSession s;
  ASSERT_TRUE(s.open(req(SetpointKind::kJointPosition, ControlModeKind::kPosition), 0.0).accepted);
  s.close();
  EXPECT_FALSE(s.is_open());
  EXPECT_FALSE(s.admit(SetpointKind::kJointPosition, 0.01));
  EXPECT_FALSE(s.expired(100.0));  // a closed session never expires; there is nothing to tear down
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `./build/unit_tests --gtest_filter='StreamingSession*'`
Expected: compile error — `streaming_session.h` does not exist.

- [ ] **Step 3: Write the header**

Create `include/kinova_lowlevel/interface/streaming_session.h`:

```cpp
#pragma once
#include <atomic>
#include <cstdint>
#include "kinova_lowlevel/interface/value_types.h"
namespace kinova::interface {

// Which (setpoint shape, control mode) pairs this driver can actually execute.
// Anything off the table is refused at open rather than silently degraded.
bool pair_supported(SetpointKind, ControlModeKind);

// The streaming tier's lifecycle, as pure logic over injected time.
//
// Deliberately owns NO control state: it does not know what a mode is, never
// touches Dynamics or Transport, and writes no targets. The Supervisor asks it
// whether a setpoint may proceed and where it should go; the Supervisor does the
// writing. That keeps this unit testable with no robot, no URDF and no threads.
//
// is_open() is atomic because it is read by the sampler thread (for the
// expiry check) and by Supervisor::on_trajectory_goal's fast pre-check, while
// the backend thread opens and closes.
class StreamingSession {
 public:
  StreamOpenResult open(const StreamOpenRequest&, double now_s);
  void             close();
  // Returns true if this setpoint may proceed. A matching setpoint also refreshes
  // the deadline; a rejected one does NOT (a client sending the wrong shape is not
  // evidence the stream is healthy).
  bool             admit(SetpointKind, double now_s);
  bool             expired(double now_s) const;

  bool            is_open()        const { return open_.load(std::memory_order_acquire); }
  SetpointKind    kind()           const { return kind_; }
  ControlModeKind control_mode()   const { return mode_; }
  double          timeout_s()      const { return timeout_s_; }
  uint64_t        rejected_count() const { return rejected_.load(std::memory_order_relaxed); }

 private:
  std::atomic<bool> open_{false};
  SetpointKind      kind_ = SetpointKind::kJointPosition;
  ControlModeKind   mode_ = ControlModeKind::kPosition;
  double            timeout_s_ = 0.1;
  std::atomic<double> last_s_{0.0};        // last accepted setpoint, or the open time
  std::atomic<uint64_t> rejected_{0};
};
}  // namespace kinova::interface
```

- [ ] **Step 4: Write the implementation**

Create `src/interface/streaming_session.cpp`:

```cpp
#include "kinova_lowlevel/interface/streaming_session.h"
namespace kinova::interface {

// The valid-pair table. Plan 2 adds the velocity rows and EE pose -> position;
// until then those are refused, which is the table working, not a stub.
bool pair_supported(SetpointKind k, ControlModeKind m) {
  switch (k) {
    case SetpointKind::kJointPosition:
      return m == ControlModeKind::kPosition || m == ControlModeKind::kImpedance;
    case SetpointKind::kEePose:
      return m == ControlModeKind::kImpedance;      // position mode has no IK path yet (Plan 2)
    case SetpointKind::kJointTorque:
      return m == ControlModeKind::kTorque;
    case SetpointKind::kJointVelocity:
    case SetpointKind::kEeTwist:
      return false;                                  // JointVelocityMode does not exist yet (Plan 2)
  }
  return false;
}

StreamOpenResult StreamingSession::open(const StreamOpenRequest& r, double now_s) {
  if (is_open())
    return {false, result_code::kStreamRejected, "a session is already open; close it first"};
  if (r.timeout_s <= 0.0)
    return {false, result_code::kStreamRejected,
            "timeout_s must be > 0: an unbounded stream has no safe-stop"};
  if (!pair_supported(r.kind, r.control_mode))
    return {false, result_code::kStreamRejected, "unsupported (setpoint kind, control mode) pair"};

  kind_ = r.kind; mode_ = r.control_mode; timeout_s_ = r.timeout_s;
  last_s_.store(now_s, std::memory_order_relaxed);
  open_.store(true, std::memory_order_release);      // marked LAST -- see the handoff rule
  return {true, 0, ""};
}

void StreamingSession::close() {
  open_.store(false, std::memory_order_release);     // marked FIRST -- see the handoff rule
}

bool StreamingSession::admit(SetpointKind k, double now_s) {
  if (!is_open() || k != kind_) {
    rejected_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  last_s_.store(now_s, std::memory_order_relaxed);
  return true;
}

bool StreamingSession::expired(double now_s) const {
  if (!is_open()) return false;                      // nothing open, nothing to tear down
  return (now_s - last_s_.load(std::memory_order_relaxed)) > timeout_s_;
}
}  // namespace kinova::interface
```

- [ ] **Step 5: Register in CMake**

In `CMakeLists.txt`, add `src/interface/streaming_session.cpp` to `KINOVA_LIB_SOURCES` next to `src/interface/arbiter.cpp`, and `tests/interface/streaming_session_test.cpp` to the `unit_tests` list next to `tests/interface/arbiter_test.cpp`.

- [ ] **Step 6: Run tests to verify they pass**

Run: `cmake -S . -B build -DCMAKE_PREFIX_PATH=/usr/local/lib/python3.10/dist-packages/cmeel.prefix && cmake --build build -j && ./build/unit_tests --gtest_filter='StreamingSession*'`
Expected: 7 tests PASS.

- [ ] **Step 7: Commit**

```bash
git add include/kinova_lowlevel/interface/streaming_session.h src/interface/streaming_session.cpp \
        tests/interface/streaming_session_test.cpp CMakeLists.txt
git commit -m "feat(interface): StreamingSession — valid-pair table, deadline, lifecycle"
```

---

### Task 3: The Arbiter gates streaming too

**Files:**
- Modify: `include/kinova_lowlevel/interface/arbiter.h`, `src/interface/arbiter.cpp`
- Test: `tests/interface/arbiter_test.cpp`

**Interfaces:**
- Consumes: `StreamSink` (Task 1), the existing `Arbiter`.
- Produces: `Arbiter` now also implements `StreamSink` and takes a `StreamSink&` downstream. Its constructor becomes `Arbiter(CommandSink& cmd, StreamSink& stream, ArbitrationMode, uint64_t seed = 0)`. Task 6 and Task 7 construct it this way.

- [ ] **Step 1: Write the failing tests**

Append to `tests/interface/arbiter_test.cpp`. Extend the existing `RecordingSink` to also implement `StreamSink` (add the seven methods and counters `stream_opens`, `stream_closes`, `setpoints`), then:

```cpp
TEST(Arbiter, StreamOpenRequiresTheOwnerToken) {
  RecordingSink sink; Arbiter arb{sink, sink, ArbitrationMode::kEnforced, 1234};
  StreamOpenRequest r;                                    // zero token
  EXPECT_FALSE(arb.on_stream_open(r).accepted);
  EXPECT_EQ(sink.stream_opens, 0);                        // never reached the Supervisor
  const Token t = arb.grant("servo").token;
  r.token = t;
  EXPECT_TRUE(arb.on_stream_open(r).accepted);
  EXPECT_EQ(sink.stream_opens, 1);
}

TEST(Arbiter, SetpointsAreGatedByToken) {
  RecordingSink sink; Arbiter arb{sink, sink, ArbitrationMode::kEnforced, 1234};
  const Token t = arb.grant("servo").token;
  JointSetpoint good; good.token = t;
  JointSetpoint bad;                                      // zero token
  arb.on_setpoint_joint_position(good);
  arb.on_setpoint_joint_position(bad);
  EXPECT_EQ(sink.setpoints, 1);                           // only the authorised one got through
}

TEST(Arbiter, NoSetpointLandsAfterRevoke) {
  RecordingSink sink; Arbiter arb{sink, sink, ArbitrationMode::kEnforced, 1234};
  const Token t = arb.grant("servo").token;
  JointSetpoint sp; sp.token = t;
  arb.on_setpoint_joint_position(sp);
  EXPECT_EQ(sink.setpoints, 1);
  arb.revoke();
  arb.on_setpoint_joint_position(sp);
  EXPECT_EQ(sink.setpoints, 1);                           // the ex-owner cannot keep driving
}

TEST(Arbiter, EstopBlocksStreamingEvenInDisabledMode) {
  RecordingSink sink; Arbiter arb{sink, sink, ArbitrationMode::kDisabled, 1234};
  JointSetpoint sp;                                       // no token needed in kDisabled
  arb.on_setpoint_joint_position(sp);
  EXPECT_EQ(sink.setpoints, 1);
  arb.estop();
  arb.on_setpoint_joint_position(sp);
  EXPECT_EQ(sink.setpoints, 1);                           // e-stop is what kDisabled does NOT bypass
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='Arbiter*'`
Expected: compile error — `Arbiter`'s constructor takes one sink, and it has no `on_stream_open`.

- [ ] **Step 3: Extend the Arbiter**

In `include/kinova_lowlevel/interface/arbiter.h`: derive from `StreamSink` as well, add a `StreamSink& down_stream_;` member, change the constructor to take both sinks, and declare the seven `StreamSink` methods.

In `src/interface/arbiter.cpp`, each follows the existing pattern exactly — lock held across delegation so admit-and-deliver is atomic against `revoke()`:

```cpp
StreamOpenResult Arbiter::on_stream_open(const StreamOpenRequest& r) {
  std::lock_guard<std::mutex> l(m_);
  if (!admit(r.token)) { ++rejected_; return {false, result_code::kNotAuthorized, "not authorized"}; }
  return down_stream_.on_stream_open(r);
}
void Arbiter::on_stream_close(const StreamCloseRequest& r) {
  std::lock_guard<std::mutex> l(m_);
  if (!admit(r.token)) { ++rejected_; return; }
  down_stream_.on_stream_close(r);
}
void Arbiter::on_setpoint_joint_position(const JointSetpoint& s) {
  std::lock_guard<std::mutex> l(m_);
  if (!admit(s.token)) { ++rejected_; return; }
  down_stream_.on_setpoint_joint_position(s);
}
```

Repeat that shape for `on_setpoint_joint_velocity`, `on_setpoint_joint_torque`, `on_setpoint_pose` and `on_setpoint_twist`.

- [ ] **Step 4: Run tests to verify they pass**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='Arbiter*'`
Expected: all PASS — the 19 existing plus the 4 new.

- [ ] **Step 5: Commit**

```bash
git add include/kinova_lowlevel/interface/arbiter.h src/interface/arbiter.cpp tests/interface/arbiter_test.cpp
git commit -m "feat(interface): Arbiter gates the streaming port too"
```

---

### Task 4: Per-mode staleness watchdogs

`JointTorqueMode` already has this. Position and impedance modes need it, and their safe behaviour is **freeze the reference at measured q** rather than a decay ramp — there is nothing to ramp, and freezing is instantaneous and safe.

This touches `compute()` in two modes, so it is an **RT-path change**: `RtSafety` and the benchmarks are obligations, not optional.

**Files:**
- Modify: `include/kinova_lowlevel/joint_position_mode.h`, `src/joint_position_mode.cpp`, `include/kinova_lowlevel/joint_impedance_mode.h`, `src/joint_impedance_mode.cpp`
- Test: `tests/joint_position_mode_test.cpp`, `tests/joint_impedance_mode_test.cpp` (find the actual filenames with `ls tests/`)

**Interfaces:**
- Produces: on **all three** streaming-capable modes, `void set_command_timeout(double) noexcept;` so the Supervisor can push a session's deadline in at open. Position and impedance additionally gain `double cmd_timeout_s = 0.0;` in their params struct (**0 disables**, preserving today's behaviour for every existing caller). Task 6 calls the setter.

> **`JointTorqueMode` needs the setter too.** It already has the watchdog, but its `JointTorqueParams p_` is a plain member fixed at construction — there is no thread-safe way to change the timeout at runtime. Without a setter, a torque session declaring `timeout_s = 0.5` would silently run against the mode's own 0.1, which is exactly the kind of quiet disagreement the one-deadline rule exists to prevent. Add `std::atomic<double> cmd_timeout_override_{-1.0}` (negative = use the params value) and have `compute()` prefer the override when it is non-negative.

- [ ] **Step 1: Write the failing test (position mode)**

Append to the position-mode test file:

```cpp
TEST(JointPositionMode, StaleTargetFreezesTheReferenceAtMeasuredQ) {
  Dynamics dyn(URDF_PATH);
  JointPositionParams p;
  p.max_ref_speed.setConstant(1.0);
  p.cmd_timeout_s = 0.05;                       // 50 ms of staleness is enough
  JointPositionMode m(dyn, p);
  JointFeedback fb; fb.q.setZero(); fb.qd.setZero();
  m.on_enter(fb);
  m.set_target(JointVec::Constant(0.5));        // a target far from where the arm is
  JointCommand out;
  for (int i = 0; i < 10; ++i) m.compute(fb, 0.001, out);   // 10 ms: fresh, reference advances
  const double advanced = m.reference()[0];
  EXPECT_GT(advanced, 0.0);
  for (int i = 0; i < 100; ++i) m.compute(fb, 0.001, out);  // 100 ms with no new command
  // Frozen at MEASURED q, not parked at the advanced reference: a stale target must
  // not keep slewing the arm toward a destination nobody is asking for any more.
  EXPECT_NEAR(m.reference()[0], fb.q[0], 1e-9);
}

TEST(JointPositionMode, ZeroTimeoutDisablesTheWatchdog) {
  Dynamics dyn(URDF_PATH);
  JointPositionParams p;
  p.max_ref_speed.setConstant(1.0);
  EXPECT_EQ(p.cmd_timeout_s, 0.0);              // default preserves existing behaviour
  JointPositionMode m(dyn, p);
  JointFeedback fb; fb.q.setZero(); fb.qd.setZero();
  m.on_enter(fb);
  m.set_target(JointVec::Constant(0.5));
  JointCommand out;
  for (int i = 0; i < 500; ++i) m.compute(fb, 0.001, out);  // half a second, no refresh
  EXPECT_GT(m.reference()[0], 0.1);             // still tracking; nothing froze
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='JointPositionMode.*Watchdog*:JointPositionMode.Stale*:JointPositionMode.ZeroTimeout*'`
Expected: compile error — `JointPositionParams` has no `cmd_timeout_s`.

- [ ] **Step 3: Implement in position mode**

Mirror `JointTorqueMode`'s mechanism exactly — a monotonic write counter bumped by the setter, staleness accumulated by summing `dt_s`, **no clock call in `compute()`**.

In `JointPositionParams`:

```cpp
  // Staleness watchdog for streamed targets. 0 DISABLES it, which is the default
  // and preserves the behaviour every existing caller relies on. The Supervisor
  // pushes a session's timeout in here when a streaming session opens.
  double cmd_timeout_s = 0.0;
```

In the class: `std::atomic<uint64_t> write_count_{0};`, RT-owned `uint64_t last_seen_write_ = 0;` and `double stale_s_ = 0.0;`, and `void set_command_timeout(double) noexcept;`. Bump `write_count_` (release) at the end of `set_target`; reset `last_seen_write_` and `stale_s_` in `on_enter`.

At the top of `compute()`, before the reference update:

```cpp
  const uint64_t wc = write_count_.load(std::memory_order_acquire);
  if (wc != last_seen_write_) { last_seen_write_ = wc; stale_s_ = 0.0; }
  else                        { stale_s_ += dt_s; }
  const double timeout = p.cmd_timeout_s;         // from the params snapshot
  if (timeout > 0.0 && stale_s_ >= timeout) {
    // Freeze where the arm actually IS. Parking at the last reference would keep
    // the rate limiter slewing toward a destination nobody is asking for.
    q_ref_ = fb.q;
  }
```

- [ ] **Step 4: Write the failing test (impedance mode)**

Append to the impedance-mode test file, mirroring the position-mode pair:

```cpp
TEST(JointImpedanceMode, StaleTargetFreezesTheReferenceAtMeasuredQ) {
  Dynamics dyn(URDF_PATH);
  JointImpedanceParams p;
  p.max_ref_speed.setConstant(1.0);
  p.cmd_timeout_s = 0.05;
  JointImpedanceMode m(dyn, p);
  JointFeedback fb; fb.q.setZero(); fb.qd.setZero();
  m.on_enter(fb);
  m.set_target(JointVec::Constant(0.5));
  JointCommand out;
  for (int i = 0; i < 10; ++i) m.compute(fb, 0.001, out);
  EXPECT_GT(m.reference()[0], 0.0);
  for (int i = 0; i < 100; ++i) m.compute(fb, 0.001, out);
  EXPECT_NEAR(m.reference()[0], fb.q[0], 1e-9);
}

TEST(JointImpedanceMode, ZeroTimeoutDisablesTheWatchdog) {
  Dynamics dyn(URDF_PATH);
  JointImpedanceParams p;
  p.max_ref_speed.setConstant(1.0);
  EXPECT_EQ(p.cmd_timeout_s, 0.0);
  JointImpedanceMode m(dyn, p);
  JointFeedback fb; fb.q.setZero(); fb.qd.setZero();
  m.on_enter(fb);
  m.set_target(JointVec::Constant(0.5));
  JointCommand out;
  for (int i = 0; i < 500; ++i) m.compute(fb, 0.001, out);
  EXPECT_GT(m.reference()[0], 0.1);
}
```

- [ ] **Step 5: Implement in impedance mode**

Same mechanism. Note this mode has **two** setters (`set_target(Pose)` and `set_target(JointVec)`) — **both** must bump `write_count_`, or streaming a pose would look stale to the watchdog. Freezing sets `q_d_ = fb.q` and, so the frozen state is coherent, also sets `source_` to the joint path.

- [ ] **Step 6: Add the setter to `JointTorqueMode`**

It already has the watchdog; it needs a runtime setter so the Supervisor can push a session's deadline in. Its params are a plain member, so introduce an atomic override rather than making the whole struct double-buffered:

```cpp
  // Session deadline pushed in by the Supervisor at stream open. Negative means
  // "use cmd_timeout_s from the params". Atomic because the Supervisor writes it
  // from a non-RT thread while compute() reads it at 1 kHz.
  std::atomic<double> cmd_timeout_override_{-1.0};
```

```cpp
void JointTorqueMode::set_command_timeout(double s) noexcept {
  cmd_timeout_override_.store(s, std::memory_order_release);
}
```

and in `compute()`, replace the two uses of `p_.cmd_timeout_s` with:

```cpp
  const double ov = cmd_timeout_override_.load(std::memory_order_acquire);
  const double timeout = (ov >= 0.0) ? ov : p_.cmd_timeout_s;
```

Add a test asserting the override wins: set a 0.5 s override, drive 200 ms of staleness, and confirm the feedforward has **not** decayed (it would have under the 0.1 s default).

- [ ] **Step 7: Run the mode tests**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='JointPositionMode*:JointImpedanceMode*'`
Expected: all PASS, including the four new cases.

- [ ] **Step 8: Run the RT gate — this is an RT-path change**

Run: `./build/unit_tests --gtest_filter='RtSafety*'`
Expected: every entry PASS, zero major page faults, zero dropped samples. Read the output.

- [ ] **Step 9: Benchmark before/after**

Run: `./build/benchmark_cartesian_impedance --sim --urdf models/gen3_7dof_2f85.urdf --rate 1000 --duration 5`
Compare p50 / p99 / p99.9 / max and the overruns/faults/dropped counts against a run from `origin/main`. The added work is one acquire-load, a branch and an addition, so expect no change — but the project's bar is that RT changes are measured, not assumed. Record both runs in your report.

- [ ] **Step 10: Commit**

```bash
git add include/kinova_lowlevel/joint_position_mode.h src/joint_position_mode.cpp \
        include/kinova_lowlevel/joint_impedance_mode.h src/joint_impedance_mode.cpp \
        include/kinova_lowlevel/joint_torque_mode.h src/joint_torque_mode.cpp tests/
git commit -m "feat(modes): clock-free staleness watchdog freezes the reference at measured q"
```

---

### Task 5: Supervisor holds a `JointTorqueMode`

Torque streaming needs one, and the Supervisor has only position and impedance today. Separated from Task 6 because it is a **constructor signature change** — every caller and test fixture moves, and that is worth its own reviewable step.

**Files:**
- Modify: `include/kinova_lowlevel/interface/supervisor.h`, `src/interface/supervisor.cpp`
- Test: `tests/interface/supervisor_test.cpp`, `tests/rt_safety_test.cpp`

**Interfaces:**
- Produces: `Supervisor(JointPositionMode& pos, JointImpedanceMode& imp, JointTorqueMode& tau, RtExecutor& exec, Seqlock<JointFeedback>& snap, Dynamics& pump_dyn, StreamPort& stream, ActionServerPort& action, SupervisorConfig cfg = {})` — `tau` inserted third. Task 6 and Task 7 rely on this ordering.

- [ ] **Step 1: Add the parameter and member**

In `supervisor.h`, include `kinova_lowlevel/joint_torque_mode.h`, add `JointTorqueMode& tau_;` beside `pos_` and `imp_`, and insert the parameter third in the constructor. Update the initialiser list in `supervisor.cpp`.

- [ ] **Step 2: Update every construction site**

Find them: `grep -rn "Supervisor sup\|Supervisor{" tests/ apps/`

In `tests/interface/supervisor_test.cpp`'s `SupFix`, add a `JointTorqueMode tau{dyn};` member declared **after** `dyn` and before `sup`, and pass it third. Do the same in the `RtSafety.SupervisorInLoopNoMajorFaultsSteadyState` fixture in `tests/rt_safety_test.cpp`.

- [ ] **Step 3: Run the whole suite**

Run: `cmake --build build -j && ctest --test-dir build --output-on-failure`
Expected: PASS with the same test count as before — this task adds no behaviour.

- [ ] **Step 4: Commit**

```bash
git add include/kinova_lowlevel/interface/supervisor.h src/interface/supervisor.cpp \
        tests/interface/supervisor_test.cpp tests/rt_safety_test.cpp
git commit -m "refactor(interface): Supervisor holds a JointTorqueMode (torque streaming needs it)"
```

---

### Task 6: Wire streaming through the Supervisor

**Files:**
- Modify: `include/kinova_lowlevel/interface/supervisor.h`, `src/interface/supervisor.cpp`
- Test: `tests/interface/supervisor_test.cpp`

**Interfaces:**
- Consumes: `StreamingSession` (Task 2), the `JointTorqueMode&` (Task 5), `set_command_timeout` (Task 4).

- [ ] **Step 1: Write the failing tests**

Append to `tests/interface/supervisor_test.cpp`:

```cpp
TEST(Supervisor, StreamingJointPositionDrivesTheMode) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::StreamOpenRequest r;
  r.kind = interface::SetpointKind::kJointPosition;
  r.control_mode = interface::ControlModeKind::kPosition;
  r.timeout_s = 1.0;
  ASSERT_TRUE(f.sup.on_stream_open(r).accepted);
  interface::JointSetpoint sp; sp.values = JointVec::Constant(0.05);
  for (int i = 0; i < 20; ++i) {
    f.sup.on_setpoint_joint_position(sp);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  interface::StreamCloseRequest c;
  f.sup.on_stream_close(c);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  f.sup.stop(); f.teardown();
  EXPECT_GT(f.sim.last_command().position[0], 1e-3);   // the setpoint actually reached the arm
}

TEST(Supervisor, AGoalIsRefusedWhileAStreamIsOpen) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::StreamOpenRequest r; r.timeout_s = 5.0;
  ASSERT_TRUE(f.sup.on_stream_open(r).accepted);
  interface::TrajectoryGoal g;
  g.trajectory = ramp7(0.0, 0.05, 0.4);
  g.control_mode = interface::ControlModeKind::kPosition;
  g.path_tolerance = JointVec::Constant(-1.0);
  EXPECT_EQ(f.sup.on_trajectory_goal(g), interface::GoalResponse::kReject);
  f.sup.stop(); f.teardown();
}

TEST(Supervisor, AStreamIsRefusedWhileAGoalIsInFlight) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::TrajectoryGoal g;
  g.trajectory = ramp7(0.0, 0.1, 2.0);
  g.control_mode = interface::ControlModeKind::kPosition;
  g.preemption = interface::Preemption::kLatestWins;
  g.path_tolerance = JointVec::Constant(-1.0);
  interface::GoalId id{}; id[0] = 1;
  f.sup.on_trajectory_goal(g); f.sup.on_trajectory_accepted(id, g);
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  interface::StreamOpenRequest r; r.timeout_s = 1.0;
  EXPECT_FALSE(f.sup.on_stream_open(r).accepted);
  f.sup.stop(); f.teardown();
}

TEST(Supervisor, StreamTimeoutClosesTheSession) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::StreamOpenRequest r; r.timeout_s = 0.1;
  ASSERT_TRUE(f.sup.on_stream_open(r).accepted);
  interface::JointSetpoint sp; sp.values = JointVec::Constant(0.05);
  f.sup.on_setpoint_joint_position(sp);
  std::this_thread::sleep_for(std::chrono::milliseconds(400));   // let the deadline lapse
  // The session is gone, so a goal is admissible again -- that is the observable
  // consequence of the lifecycle teardown.
  interface::TrajectoryGoal g;
  g.trajectory = ramp7(0.0, 0.05, 0.4);
  g.control_mode = interface::ControlModeKind::kPosition;
  g.path_tolerance = JointVec::Constant(-1.0);
  EXPECT_EQ(f.sup.on_trajectory_goal(g), interface::GoalResponse::kAccept);
  f.sup.stop(); f.teardown();
}

TEST(Supervisor, HaltClosesAnOpenStream) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::StreamOpenRequest r; r.timeout_s = 5.0;
  ASSERT_TRUE(f.sup.on_stream_open(r).accepted);
  f.sup.on_halt(interface::HaltReason::kEmergencyStop);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  interface::JointSetpoint sp; sp.values = JointVec::Constant(0.2);
  f.sup.on_setpoint_joint_position(sp);            // must not restart a halted arm
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  f.sup.stop(); f.teardown();
  EXPECT_NEAR(f.sim.last_command().position[0], 0.0, 1e-6);
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='Supervisor.Stream*:Supervisor.AGoal*:Supervisor.AStream*:Supervisor.Halt*'`
Expected: FAIL — `on_stream_open` is the Task 1 stub that always refuses.

- [ ] **Step 3: Add the session and the mode-selection helper**

In `supervisor.h` add `StreamingSession session_;` and a private helper:

```cpp
  // The sink a streaming session writes to, chosen by its declared control mode.
  kinova::JointTargetSink& stream_joint_sink();     // pos_ or imp_
```

- [ ] **Step 4: Implement open and close**

In `src/interface/supervisor.cpp`. Note the ordering: **mark open LAST, mark closed FIRST** — that is what makes the write handoff exclusive.

```cpp
StreamOpenResult Supervisor::on_stream_open(const StreamOpenRequest& r) {
  if (in_flight_.load())
    return {false, result_code::kStreamRejected, "a trajectory goal is in flight"};
  if (!pair_supported(r.kind, r.control_mode))
    return {false, result_code::kStreamRejected, "unsupported (setpoint kind, control mode) pair"};

  // Switch modes BEFORE the session is marked open, so no setpoint can land mid-switch.
  const ControlModeKind want = r.control_mode;
  if (want != active_mode_kind_) {
    if      (want == ControlModeKind::kImpedance) exec_.request_mode(&imp_);
    else if (want == ControlModeKind::kTorque)    exec_.request_mode(&tau_);
    else                                          exec_.request_mode(&pos_);
    active_mode_kind_ = want;
    atomic_mode_.store(want == ControlModeKind::kImpedance ? 1 : 0);
    std::this_thread::sleep_for(std::chrono::duration_cast<clock::duration>(
        std::chrono::duration<double>(cfg_.mode_settle_s)));
  }
  // One deadline, pushed into the mode so it can make the OUTPUT safe at 1 kHz
  // while the session handles lifecycle at sampler rate.
  if      (want == ControlModeKind::kPosition)  pos_.set_command_timeout(r.timeout_s);
  else if (want == ControlModeKind::kImpedance) imp_.set_command_timeout(r.timeout_s);
  else if (want == ControlModeKind::kTorque)    tau_.set_command_timeout(r.timeout_s);

  const StreamOpenResult res = session_.open(r, secs_since(t0_));
  if (res.accepted) stream_open_.store(true);      // marked LAST
  return res;
}

void Supervisor::on_stream_close(const StreamCloseRequest&) { close_stream(); }

// One teardown, three callers: graceful close, watchdog expiry, and on_halt.
void Supervisor::close_stream() {
  stream_open_.store(false);                       // marked FIRST: setpoints are refused from here
  session_.close();
  if (active_mode_kind_ == ControlModeKind::kPosition)  pos_.set_command_timeout(0.0);
  if (active_mode_kind_ == ControlModeKind::kImpedance) imp_.set_command_timeout(0.0);
  if (active_mode_kind_ == ControlModeKind::kTorque)    tau_.set_command_timeout(-1.0);  // back to its own default
}
```

Declare `close_stream()`, `std::atomic<bool> stream_open_{false};` and a `clock::time_point t0_` (set in `start()`) in the header.

- [ ] **Step 5: Implement the setpoint methods**

Each admits through the session, then writes the sink **directly** — sound because the sampler writes no targets while a session is open:

```cpp
void Supervisor::on_setpoint_joint_position(const JointSetpoint& s) {
  if (!session_.admit(SetpointKind::kJointPosition, secs_since(t0_))) return;
  stream_joint_sink().set_target(s.values);
}
void Supervisor::on_setpoint_pose(const PoseSetpoint& s) {
  if (!session_.admit(SetpointKind::kEePose, secs_since(t0_))) return;
  imp_.set_target(s.pose);                         // only impedance has a PoseTargetSink
}
void Supervisor::on_setpoint_joint_torque(const JointSetpoint& s) {
  if (!session_.admit(SetpointKind::kJointTorque, secs_since(t0_))) return;
  tau_.set_torque(s.values);
}
void Supervisor::on_setpoint_joint_velocity(const JointSetpoint&) {}   // no JointVelocityMode (Plan 2)
void Supervisor::on_setpoint_twist(const TwistSetpoint&) {}            // no JointVelocityMode (Plan 2)

kinova::JointTargetSink& Supervisor::stream_joint_sink() {
  return session_.control_mode() == ControlModeKind::kImpedance
         ? static_cast<kinova::JointTargetSink&>(imp_)
         : static_cast<kinova::JointTargetSink&>(pos_);
}
```

The two empty methods are unreachable in practice — `pair_supported` refuses those kinds at open — but they must exist to satisfy the port.

- [ ] **Step 6: Wire the goal/stream interlock and the expiry check**

In `on_trajectory_goal`'s existing fast pre-check, add:

```cpp
  if (stream_open_.load()) return GoalResponse::kReject;   // a stream owns the arm
```

In `sampler_loop`, immediately after the halt branch. **Use the member `t0_`, not the sampler's own local `t0`** — `session_.open()` and `admit()` stamp against `t0_`, so comparing expiry against a different origin would make the deadline meaningless:

```cpp
    // Lifecycle half of the deadline. The mode has already made the output safe
    // at 1 kHz; this closes the session and lets goals back in.
    if (stream_open_.load() && session_.expired(secs_since(t0_)))
      close_stream();
```

In `on_halt`, before anything else, add `close_stream();` so streaming stops being admitted before the hold is latched.

- [ ] **Step 7: Run the tests**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='Supervisor*'`
Expected: all PASS, including the five new cases.

- [ ] **Step 8: Run the whole suite and the RT gate**

Run: `ctest --test-dir build --output-on-failure && ./build/unit_tests --gtest_filter='RtSafety*'`
Expected: PASS; RtSafety all green.

- [ ] **Step 9: Commit**

```bash
git add include/kinova_lowlevel/interface/supervisor.h src/interface/supervisor.cpp \
        tests/interface/supervisor_test.cpp
git commit -m "feat(interface): stream setpoints through the Supervisor, exclusive with goals"
```

---

### Task 7: The write-handoff concurrency test

The handoff invariant — exactly one thread writes targets at any instant — cannot be proven behaviourally. It has to be raced.

**Files:**
- Modify: `tests/interface/supervisor_test.cpp`, `include/kinova_lowlevel/interface/supervisor.h`

> **Why this test lives in `supervisor_test.cpp` and not its own file:** the `SupFix` fixture is defined in that file's **anonymous namespace**, so it is not reachable from another translation unit. Rebuilding the fixture in a new file would duplicate threading setup that must stay identical for the test to mean anything. Put the test where the fixture already is.

**Interfaces:**
- Consumes: `Supervisor` (Tasks 5–6), `StreamingSession` (Task 2).
- Produces: `bool Supervisor::stream_is_open() const` (public accessor, so the test asserts state without reaching into internals).

- [ ] **Step 1: Add the accessor**

In `include/kinova_lowlevel/interface/supervisor.h`, public section:

```cpp
  // Test/diagnostic: is a streaming session currently admitting setpoints?
  bool stream_is_open() const { return stream_open_.load(); }
```

- [ ] **Step 2: Write the test**

Append to `tests/interface/supervisor_test.cpp`.

The design point that makes this actually verify something: **during the session the writer streams a setpoint equal to where the arm already is (zero), and after the close it switches to a clearly different value.** So if any post-close setpoint reaches the mode, the commanded position moves away from zero and the assertion catches it. A test that streamed one constant throughout could not distinguish "the handoff held" from "the handoff leaked."

```cpp
TEST(StreamingHandoff, NoSetpointReachesTheModeAfterClose) {
  for (int round = 0; round < 50; ++round) {
    SupFix f; f.sup.start(); f.run_rt();
    interface::StreamOpenRequest r;
    r.kind = interface::SetpointKind::kJointPosition;
    r.control_mode = interface::ControlModeKind::kPosition;
    r.timeout_s = 5.0;                       // long: this test is about close, not expiry
    ASSERT_TRUE(f.sup.on_stream_open(r).accepted);

    std::atomic<bool> stop_writer{false};
    std::atomic<bool> closed{false};

    // Before the close: stream where the arm already is, so nothing moves.
    // After the close: stream somewhere obviously different. Any leak shows up
    // as commanded motion.
    std::thread writer([&] {
      while (!stop_writer.load(std::memory_order_acquire)) {
        interface::JointSetpoint sp;
        sp.values = closed.load(std::memory_order_acquire) ? JointVec::Constant(0.4)
                                                           : JointVec::Zero();
        f.sup.on_setpoint_joint_position(sp);
        std::this_thread::yield();
      }
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    interface::StreamCloseRequest c;
    f.sup.on_stream_close(c);
    closed.store(true, std::memory_order_release);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));   // let the writer hammer
    stop_writer.store(true, std::memory_order_release);
    writer.join();

    EXPECT_FALSE(f.sup.stream_is_open());
    f.sup.stop(); f.teardown();
    // The post-close setpoints asked for 0.4 rad. If the handoff held, none of
    // them reached the mode and the command never left the entry configuration.
    EXPECT_NEAR(f.sim.last_command().position[0], 0.0, 1e-6)
        << "a setpoint landed after the session was closed (round " << round << ")";
  }
}

TEST(StreamingHandoff, NoSetpointReachesTheModeAfterHalt) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::StreamOpenRequest r;
  r.kind = interface::SetpointKind::kJointPosition;
  r.control_mode = interface::ControlModeKind::kPosition;
  r.timeout_s = 5.0;
  ASSERT_TRUE(f.sup.on_stream_open(r).accepted);

  std::atomic<bool> stop_writer{false}, halted{false};
  std::thread writer([&] {
    while (!stop_writer.load(std::memory_order_acquire)) {
      interface::JointSetpoint sp;
      sp.values = halted.load(std::memory_order_acquire) ? JointVec::Constant(0.4)
                                                         : JointVec::Zero();
      f.sup.on_setpoint_joint_position(sp);
      std::this_thread::yield();
    }
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  f.sup.on_halt(interface::HaltReason::kEmergencyStop);
  halted.store(true, std::memory_order_release);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  stop_writer.store(true, std::memory_order_release);
  writer.join();
  f.sup.stop(); f.teardown();
  // An e-stopped arm must not be restartable by a client that has not noticed.
  EXPECT_NEAR(f.sim.last_command().position[0], 0.0, 1e-6);
}
```

- [ ] **Step 3: Run the tests**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='StreamingHandoff*'`
Expected: both PASS, the first across all 50 rounds.

If either fails, the mark-then-act ordering is wrong somewhere — **fix the ordering, never the test.** The most likely cause is marking the session closed *after* the teardown action rather than before it.

- [ ] **Step 4: Run it under ThreadSanitizer**

Iteration count alone is weak evidence for a concurrency property. Build once with TSan and run the streaming tests:

```bash
cmake -S . -B build-tsan -DCMAKE_PREFIX_PATH=/usr/local/lib/python3.10/dist-packages/cmeel.prefix \
      -DCMAKE_CXX_FLAGS="-fsanitize=thread -g -O1"
cmake --build build-tsan -j --target unit_tests
./build-tsan/unit_tests --gtest_filter='StreamingHandoff*:StreamingSession*:Supervisor.Stream*'
```

Expected: PASS with no TSan reports. **A TSan report here is a real finding** — record it verbatim and fix the code; do not weaken the test to silence it.

Note `build-tsan/` is throwaway. Confirm it is gitignored (`grep -n "build" .gitignore`) and add it if not — do not commit it.

- [ ] **Step 5: Commit**

```bash
git add tests/interface/supervisor_test.cpp include/kinova_lowlevel/interface/supervisor.h
git commit -m "test(interface): race the streaming write handoff, verified under TSan"
```

---

### Task 8: Documentation and final verification

**Files:**
- Create: `docs/guide/streaming.md`
- Modify: `mkdocs.yml`, `docs/reference/api.md`

- [ ] **Step 1: Write the guide**

Create `docs/guide/streaming.md` covering: what a session is and why it must be opened explicitly; the valid-pair table **as it stands today**, with the three unsupported rows marked as needing Plan 2 rather than being described as broken; that a setpoint is a command and never an increment, so re-sending is idempotent and dropping intermediate ones is correct; the one-deadline/two-enforcement-levels model and what each mode does when the stream goes stale; and that streaming and trajectory goals are mutually exclusive with loud refusals both ways.

- [ ] **Step 2: Wire it into the nav**

In `mkdocs.yml`, add `- Streaming: guide/streaming.md` to the `Guide` section, beside the existing `Arbitration` entry.

- [ ] **Step 3: Document the port**

In `docs/reference/api.md`, add a `## Streaming — interface/streaming_session.h, interface/ports.h` section documenting `StreamSink`, `SetpointKind`, the three setpoint structs, `StreamOpenRequest`/`StreamOpenResult`, and `pair_supported`. State plainly that `timeout_s <= 0` is refused and why.

- [ ] **Step 4: Verify the docs build**

Run: `uv run --with mkdocs --with mkdocs-material mkdocs build -d /tmp/mkdocs-check`
Expected: no new warnings. Three pre-existing unrelated warnings are expected.

- [ ] **Step 5: Full verification on the Jetson**

Run: `cmake -S . -B build -DCMAKE_PREFIX_PATH=/usr/local/lib/python3.10/dist-packages/cmeel.prefix && cmake --build build -j && ctest --test-dir build --output-on-failure`
Expected: PASS. Record the test count.

- [ ] **Step 6: Read the RT gate and the benchmark**

Run: `./build/unit_tests --gtest_filter='RtSafety*'` and `./build/benchmark_cartesian_impedance --sim --urdf models/gen3_7dof_2f85.urdf --rate 1000 --duration 5`
Expected: RtSafety all green, zero major faults, zero dropped samples. Compare the benchmark percentiles against the Task 4 baseline.

- [ ] **Step 7: Commit**

```bash
git add docs/guide/streaming.md mkdocs.yml docs/reference/api.md
git commit -m "docs(streaming): guide, API reference, nav entry"
```

## Not in this plan

- **`JointVelocityMode` and the position-mode pose path** — Plan 2. Until then `pair_supported` refuses the velocity kinds and EE-pose-into-position.
- **The ROS2 frontend** — services, topics, QoS, node wiring. That is the next pass through `kinova_arm_ros2`, together with the build fixes #25 and this plan both require.
- **Attended hardware.** Nothing here has run on the arm.

## Downstream breakage this plan causes

`Supervisor`'s constructor gains a `JointTorqueMode&` (Task 5). Together with #25's `CommandSink` changes, `kinova_arm_ros2` needs both fixed in the same pass.
