# Arm Interface — Ports + Supervisor Implementation Plan (Plan 2)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the transport-agnostic Layer A ports + Layer C `Supervisor` that bind the RT execution core to a semantic command/telemetry API, driven end-to-end against `SimTransport` + a fake in-process backend — entirely ROS-free — and make the core library installable so a separate ROS2 repo (Plan 3) can `find_package` it.

**Architecture:** Hexagonal. Plain-C++ value types + pure-virtual ports (Layer A) carry commands in / telemetry out with no transport types. The `Supervisor` (Layer C) implements the inbound `CommandSink`, owns two non-RT threads — a **control/sampler** thread that owns a `TrajectoryExecutor` and drives the active `ControlMode` through `JointTargetSink`, and a **state-pump** thread that publishes `ArmState` — and coordinates mode switching via `RtExecutor::request_mode`. The RT loop keeps the main thread, byte-for-byte unchanged. `q_meas` reaches the sampler and pump through the existing `FeedbackTap`/`Seqlock` seam (Phase 1, merged).

**Tech Stack:** C++17, Eigen, gtest, plain CMake. No ROS in this plan.

## Global Constraints

- **SI / radians everywhere internally.** `kNumJoints = 7`, `JointVec = Eigen::Matrix<double,7,1>`.
- **Nothing in this plan runs on the RT thread.** Sampler, pump, backend callbacks are all non-RT. The RT cycle (`RtExecutor`, `ControlMode::compute`) is unchanged. `rt_safety_test` (zero major page faults, zero dropped samples in steady state) must still pass with the Supervisor running against `SimTransport`.
- **Single-writer command seam.** Exactly ONE thread (the control/sampler thread) ever calls a mode's `set_target` or touches the `TrajectoryExecutor`. Backend callbacks hand work to it via a mutex-guarded queue; they never touch the executor directly.
- **The driver core links no transport and no ROS.** Ports and Supervisor depend only on Layer A + the existing core.
- **Build stays plain CMake.** The default sim build/test commands are unchanged. b1 build-integration (Task 10) adds `install(EXPORT)` + a generated `kinova_lowlevelConfig.cmake` + a root `package.xml` (`<build_type>cmake</build_type>`, inert to raw cmake).
- **Result codes mirror FollowJointTrajectory:** `SUCCESSFUL=0`, `INVALID_GOAL=-1`, `PATH_TOLERANCE_VIOLATED=-4`, `GOAL_TOLERANCE_VIOLATED=-5`, plus `PREEMPTED=-6` (our extra).
- **Build/test loop is aarch64-only (abra).** Use `scratchpad/abra_test.sh '<gtest_filter>'` (rsync muk→abra:/tmp/kinova-build, cmake+build+ctest). muk cannot build.

---

## Reference: exact existing signatures (verbatim, do not guess)

```cpp
// rt_executor.h
enum class Pacing { kSleepSpin, kClockNanosleep };
struct ExecutorConfig { double rate_hz = 1000.0; Pacing pacing = Pacing::kSleepSpin; RtConfig rt{}; };
class RtExecutor {
 public:
  RtExecutor(Transport& t, SampleRing& ring, ExecutorConfig cfg);
  void request_mode(ControlMode* m) noexcept;   // stores raw ptr; adopted at cycle boundary; mode must outlive
  void run(std::atomic<bool>& stop);             // BLOCKS the calling thread
};
// control_mode.h
class ControlMode {                                    // required_modes/on_enter/compute/on_exit
  virtual ActuatorModes required_modes() const = 0;
  virtual void on_enter(const JointFeedback&) = 0;
  virtual void compute(const JointFeedback&, double dt_s, JointCommand&) = 0;   // RT-safe
  virtual void on_exit() = 0;
};
// joint_target_sink.h
class JointTargetSink { virtual void set_target(const JointVec& q_d) noexcept = 0; };
// joint_position_mode.h
struct JointPositionParams { JointVec max_ref_speed=JointVec::Constant(0.5); double max_following_error=0.35;
                             JointVec q_lower, q_upper; };
class JointPositionMode : public ControlMode, public JointTargetSink {
  JointPositionMode(Dynamics& dyn, JointPositionParams p = {});
  void set_target(const JointVec& q_d) noexcept override;
  JointVec reference() const noexcept;             // RT-owned, unsynchronized
};
// joint_impedance_mode.h
struct JointImpedanceParams { JointVec Kq; double zeta=0.5; JointVec torque_limit; double max_tracking_error=0.35;
                              JointVec max_ref_speed; double gain_ramp_s=0.5; DiffIkParams ik{}; };
class JointImpedanceMode : public ControlMode, public PoseTargetSink, public JointTargetSink {
  JointImpedanceMode(Dynamics& dyn, JointImpedanceParams p = {});
  void set_gains(const JointImpedanceParams& p) noexcept;
  void set_target(const JointVec& q_d) noexcept override;   // JointTargetSink; bypasses IK
  JointVec reference() const noexcept;
};
// dynamics.h
class Dynamics { explicit Dynamics(const std::string& urdf_path, const std::string& ee_frame="gen3_end_effector_link");
                 Pose fk(const JointVec& q); /* returns by value */ };
// cartesian_types.h
struct Pose { Eigen::Vector3d p; Eigen::Quaterniond R; };
// telemetry.h
struct CycleSample { uint64_t cycle_index; uint32_t wake_jitter_ns, comm_ns, compute_ns, cycle_ns; uint16_t flags; };
class SampleRing { explicit SampleRing(size_t capacity_pow2); bool push(const CycleSample&); bool pop(CycleSample&);
                   uint64_t dropped() const; };
// transport.h
using ActuatorModes = std::array<ActuatorMode, kNumJoints>;
class Transport { connect(); set_servoing_low_level(); set_actuator_modes(const ActuatorModes&);
                  exchange(const JointCommand&, JointFeedback&); send(const JointCommand&); receive(JointFeedback&);
                  safe_shutdown(); virtual void clear_faults() {} };
// joint_types.h
struct JointFeedback { JointVec q, qd, tau, current; uint64_t frame_id; bool fault; float gripper; };
struct JointCommand  { ActuatorMode mode; JointVec position, velocity, torque; float gripper; bool gripper_active; };
// feedback_tap.h
template<class T> class Seqlock { void store(const T&); bool load(T& out) const; };
class FeedbackTap : public Transport { FeedbackTap(Transport& inner, Seqlock<JointFeedback>& snap); };
// interface/trajectory_executor.h
namespace interface {
  struct JointWaypoint { JointVec q; double t_s; };
  struct Trajectory { std::vector<JointWaypoint> points; double duration_s() const; };
  enum class Preemption { kQueue, kLatestWins };
  enum class ControlModeKind { kPosition, kImpedance };
  enum class SubmitResult { kAccepted, kRejectedModeChangeWhileMoving, kRejectedEmpty };
  struct ExecStatus { static constexpr int kOk=0; static constexpr int kPathToleranceViolated=-4;
                      bool active; bool completed; double fraction; int error_code; };
  class TrajectoryExecutor {
    explicit TrajectoryExecutor(JointTargetSink& sink);
    SubmitResult submit(const Trajectory&, ControlModeKind, Preemption, const JointVec& path_tol);
    bool is_active() const; ControlModeKind active_mode() const;
    ExecStatus tick(double now_s, const JointVec& q_meas);
  };
}
```

---

## File structure

- `include/kinova_lowlevel/interface/value_types.h` — plain command/telemetry structs + result codes (NEW)
- `include/kinova_lowlevel/interface/ports.h` — `StreamPort`, `ActionServerPort`, `CommandSink` (NEW)
- `include/kinova_lowlevel/interface/supervisor.h` — `Supervisor` class (NEW)
- `src/interface/supervisor.cpp` — Supervisor implementation (NEW)
- `include/kinova_lowlevel/interface/trajectory_executor.h` + `src/interface/trajectory_executor.cpp` — add `bool promoted` to `ExecStatus` (MODIFY)
- `tests/interface/fake_backend.h` — in-process test double for the ports (NEW, test-only)
- `tests/interface/supervisor_test.cpp` — Supervisor unit/integration tests (NEW)
- `tests/interface/trajectory_executor_test.cpp` — add promotion-signal test (MODIFY)
- `tests/rt_safety_test.cpp` — add a supervisor-in-the-loop variant (MODIFY)
- `CMakeLists.txt` — register new sources/tests; b1 install/export (MODIFY)
- `cmake/kinova_lowlevelConfig.cmake.in` — package config template (NEW)
- `package.xml` — root manifest, `<build_type>cmake</build_type>` (NEW)

---

### Task 1: Layer A value types

**Files:**
- Create: `include/kinova_lowlevel/interface/value_types.h`
- Test: `tests/interface/supervisor_test.cpp` (new file; first test lives here)

**Interfaces:**
- Produces: `interface::GoalId` (`std::array<uint8_t,16>`), `interface::JointImpedanceGains {JointVec kq; double zeta; JointVec torque_limit;}`, `interface::TrajectoryGoal`, `interface::TrajectoryFeedback`, `interface::TrajectoryResult`, `interface::ArmState`, `interface::GainsRequest/GainsResult`, enums `GoalResponse{kAccept,kReject}` / `CancelResponse{kAccept,kReject}`, and `namespace result_code { constexpr int kSuccessful=0,kInvalidGoal=-1,kPathToleranceViolated=-4,kGoalToleranceViolated=-5,kPreempted=-6; }`.

- [ ] **Step 1: Write the failing test**

```cpp
// tests/interface/supervisor_test.cpp
#include <gtest/gtest.h>
#include "kinova_lowlevel/interface/value_types.h"
using namespace kinova;
using namespace kinova::interface;

TEST(ValueTypes, DefaultsAndResultCodes) {
  TrajectoryGoal g;                          // default-constructs
  g.control_mode = ControlModeKind::kPosition;
  g.preemption   = Preemption::kLatestWins;
  g.path_tolerance = JointVec::Constant(0.2);
  EXPECT_EQ(g.trajectory.points.size(), 0u);
  EXPECT_FALSE(g.has_gains);
  EXPECT_EQ(result_code::kSuccessful, 0);
  EXPECT_EQ(result_code::kPathToleranceViolated, -4);
  EXPECT_EQ(result_code::kPreempted, -6);
  ArmState s; s.q = JointVec::Constant(0.1);
  EXPECT_NEAR(s.q[0], 0.1, 1e-12);
  GoalId id{}; EXPECT_EQ(id.size(), 16u);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `scratchpad/abra_test.sh 'ValueTypes*'`
Expected: FAIL — `value_types.h: No such file or directory`.

- [ ] **Step 3: Write minimal implementation**

```cpp
// include/kinova_lowlevel/interface/value_types.h
#pragma once
#include <array>
#include <cstdint>
#include <string>
#include "kinova_lowlevel/joint_types.h"
#include "kinova_lowlevel/cartesian_types.h"
#include "kinova_lowlevel/interface/trajectory_executor.h"  // Trajectory, Preemption, ControlModeKind
namespace kinova::interface {

using GoalId = std::array<uint8_t, 16>;          // mirrors a ROS2 action UUID

struct JointImpedanceGains { JointVec kq = JointVec::Zero(); double zeta = 0.5;
                             JointVec torque_limit = JointVec::Zero(); };

struct TrajectoryGoal {
  Trajectory trajectory;
  JointVec path_tolerance = JointVec::Constant(-1.0);   // <0 disables (matches executor)
  JointVec goal_tolerance = JointVec::Constant(-1.0);
  double   goal_time_tolerance_s = 0.0;
  ControlModeKind control_mode = ControlModeKind::kPosition;
  Preemption      preemption   = Preemption::kLatestWins;
  JointImpedanceGains gains{};
  bool has_gains = false;
  std::string sender_id;
};
struct TrajectoryFeedback { JointVec desired=JointVec::Zero(), actual=JointVec::Zero(), error=JointVec::Zero();
                            double fraction_complete = 0.0; };
struct TrajectoryResult   { int error_code = 0; std::string error_string; JointVec final_error = JointVec::Zero(); };
struct ArmState { JointVec q=JointVec::Zero(), qd=JointVec::Zero(), tau=JointVec::Zero();
                  Pose ee_pose; bool fault=false; double stamp_s=0.0; };
struct GainsRequest { JointImpedanceGains gains{}; };
struct GainsResult  { bool accepted=false; std::string message; };

enum class GoalResponse   { kAccept, kReject };
enum class CancelResponse { kAccept, kReject };

namespace result_code {
  constexpr int kSuccessful = 0, kInvalidGoal = -1, kPathToleranceViolated = -4,
                kGoalToleranceViolated = -5, kPreempted = -6;
}
}  // namespace kinova::interface
```

- [ ] **Step 4: Register the test file + run to verify it passes**

Add `tests/interface/supervisor_test.cpp` to the `unit_tests` sources in `CMakeLists.txt` (alongside `tests/interface/trajectory_executor_test.cpp`). Then:
Run: `scratchpad/abra_test.sh 'ValueTypes*'`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add include/kinova_lowlevel/interface/value_types.h tests/interface/supervisor_test.cpp CMakeLists.txt
git commit -m "feat(interface): Layer A value types (goal/feedback/result/state + result codes)"
```

---

### Task 2: Layer A ports + a fake in-process backend

**Files:**
- Create: `include/kinova_lowlevel/interface/ports.h`
- Create: `tests/interface/fake_backend.h`
- Test: `tests/interface/supervisor_test.cpp`

**Interfaces:**
- Produces: `StreamPort::publish_state(const ArmState&)`; `ActionServerPort::publish_feedback(const GoalId&, const TrajectoryFeedback&)` + `::settle(const GoalId&, const TrajectoryResult&)`; `CommandSink` with `on_trajectory_goal(const TrajectoryGoal&)->GoalResponse`, `on_trajectory_accepted(const GoalId&, const TrajectoryGoal&)`, `on_trajectory_cancel(const GoalId&)->CancelResponse`, `on_set_gains(const GainsRequest&)->GainsResult`, `on_query_state()->ArmState`.
- Produces (test-only): `FakeBackend` implementing `StreamPort` + `ActionServerPort`, recording `states`, `feedback`, `results` under a mutex, with helper accessors.

- [ ] **Step 1: Write the failing test**

```cpp
// tests/interface/supervisor_test.cpp  (append)
#include "kinova_lowlevel/interface/ports.h"
#include "tests/interface/fake_backend.h"

TEST(Ports, FakeBackendRecordsDrivenCalls) {
  FakeBackend be;
  StreamPort& sp = be; ActionServerPort& ap = be;
  ArmState s; s.q = JointVec::Constant(0.3);
  sp.publish_state(s);
  GoalId id{}; id[0] = 7;
  TrajectoryResult r; r.error_code = interface::result_code::kSuccessful;
  ap.settle(id, r);
  EXPECT_EQ(be.state_count(), 1u);
  EXPECT_NEAR(be.last_state().q[0], 0.3, 1e-12);
  EXPECT_EQ(be.result_count(), 1u);
  EXPECT_EQ(be.last_result().error_code, 0);
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `scratchpad/abra_test.sh 'Ports*'`
Expected: FAIL — `ports.h`/`fake_backend.h` not found.

- [ ] **Step 3: Write the ports**

```cpp
// include/kinova_lowlevel/interface/ports.h
#pragma once
#include "kinova_lowlevel/interface/value_types.h"
namespace kinova::interface {
// Driven ports — the supervisor CALLS these to push data out.
class StreamPort { public: virtual ~StreamPort() = default;
  virtual void publish_state(const ArmState&) = 0; };
class ActionServerPort { public: virtual ~ActionServerPort() = default;
  virtual void publish_feedback(const GoalId&, const TrajectoryFeedback&) = 0;
  virtual void settle(const GoalId&, const TrajectoryResult&) = 0; };
// Driving port — the supervisor IMPLEMENTS this; the backend calls it on inbound messages.
class CommandSink { public: virtual ~CommandSink() = default;
  virtual GoalResponse   on_trajectory_goal(const TrajectoryGoal&) = 0;
  virtual void           on_trajectory_accepted(const GoalId&, const TrajectoryGoal&) = 0;
  virtual CancelResponse on_trajectory_cancel(const GoalId&) = 0;
  virtual GainsResult    on_set_gains(const GainsRequest&) = 0;
  virtual ArmState       on_query_state() = 0; };
}  // namespace kinova::interface
```

- [ ] **Step 4: Write the fake backend (test-only)**

```cpp
// tests/interface/fake_backend.h
#pragma once
#include <mutex>
#include <vector>
#include "kinova_lowlevel/interface/ports.h"
namespace kinova::interface {
class FakeBackend : public StreamPort, public ActionServerPort {
 public:
  void publish_state(const ArmState& s) override { std::lock_guard<std::mutex> l(m_); states_.push_back(s); }
  void publish_feedback(const GoalId&, const TrajectoryFeedback& f) override {
    std::lock_guard<std::mutex> l(m_); feedback_.push_back(f); }
  void settle(const GoalId& id, const TrajectoryResult& r) override {
    std::lock_guard<std::mutex> l(m_); results_.push_back({id, r}); }
  size_t state_count()   const { std::lock_guard<std::mutex> l(m_); return states_.size(); }
  size_t result_count()  const { std::lock_guard<std::mutex> l(m_); return results_.size(); }
  ArmState last_state()  const { std::lock_guard<std::mutex> l(m_); return states_.back(); }
  TrajectoryResult last_result() const { std::lock_guard<std::mutex> l(m_); return results_.back().second; }
  GoalId last_result_id() const { std::lock_guard<std::mutex> l(m_); return results_.back().first; }
 private:
  mutable std::mutex m_;
  std::vector<ArmState> states_;
  std::vector<TrajectoryFeedback> feedback_;
  std::vector<std::pair<GoalId, TrajectoryResult>> results_;
};
}  // namespace kinova::interface
```

- [ ] **Step 5: Run to verify pass, then commit**

Run: `scratchpad/abra_test.sh 'Ports*'` → PASS.
```bash
git add include/kinova_lowlevel/interface/ports.h tests/interface/fake_backend.h tests/interface/supervisor_test.cpp
git commit -m "feat(interface): Layer A ports (Stream/ActionServer/CommandSink) + fake backend test double"
```

---

### Task 3: `ExecStatus.promoted` signal (executor extension for correct queue results)

**Files:**
- Modify: `include/kinova_lowlevel/interface/trajectory_executor.h` (add `bool promoted` to `ExecStatus`)
- Modify: `src/interface/trajectory_executor.cpp` (set it true on the tick a queued goal is promoted)
- Test: `tests/interface/trajectory_executor_test.cpp`

**Interfaces:**
- Produces: `ExecStatus.promoted` — `true` only on the tick where a queued goal was promoted to active (gapless), else `false`.

- [ ] **Step 1: Write the failing test**

```cpp
// tests/interface/trajectory_executor_test.cpp  (append)
TEST(ExecutorPreempt, PromotionRaisesPromotedFlagExactlyOnce) {
  RecordingSink sink;
  kinova::interface::TrajectoryExecutor ex(sink);
  using namespace kinova::interface;
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kLatestWins, vec7(-1.0)); // active
  ex.submit(ramp(2.0), ControlModeKind::kPosition, Preemption::kQueue,     vec7(-1.0)); // queued
  ex.tick(0.0, vec7(0.0));
  ExecStatus mid = ex.tick(1.0, vec7(0.5));  EXPECT_FALSE(mid.promoted);
  ExecStatus at_end = ex.tick(2.0, vec7(1.0));  // first ramp ends -> promote queued
  EXPECT_TRUE(at_end.promoted);
  EXPECT_TRUE(at_end.active);
  ExecStatus after = ex.tick(3.0, vec7(0.5));  EXPECT_FALSE(after.promoted);
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `scratchpad/abra_test.sh 'ExecutorPreempt.PromotionRaisesPromotedFlagExactlyOnce'`
Expected: FAIL — `promoted` is not a member of `ExecStatus`.

- [ ] **Step 3: Implement**

In `trajectory_executor.h`, extend the struct:
```cpp
struct ExecStatus { static constexpr int kOk=0; static constexpr int kPathToleranceViolated=-4;
                    bool active; bool completed; double fraction; int error_code; bool promoted=false; };
```
In `trajectory_executor.cpp`, in `tick()`: locate the completion branch that promotes `queued_` into `active_` (gapless). Set the returned status's `promoted=true` on that path only; leave it `false` on every other return. (All existing `ExecStatus` returns default `promoted=false` via the member initializer — verify each explicit brace-init still compiles; add `, false` where a return uses positional aggregate init.)

- [ ] **Step 4: Run to verify pass (and no regressions)**

Run: `scratchpad/abra_test.sh 'Executor*:Trajectory*'`
Expected: PASS — the new test plus all pre-existing executor tests.

- [ ] **Step 5: Commit**

```bash
git add include/kinova_lowlevel/interface/trajectory_executor.h src/interface/trajectory_executor.cpp tests/interface/trajectory_executor_test.cpp
git commit -m "feat(interface): executor reports queue-promotion via ExecStatus.promoted"
```

---

### Task 4: Supervisor skeleton — construction, lifecycle, initial mode

**Files:**
- Create: `include/kinova_lowlevel/interface/supervisor.h`
- Create: `src/interface/supervisor.cpp`
- Modify: `CMakeLists.txt` (add `src/interface/supervisor.cpp` to `KINOVA_LIB_SOURCES`)
- Test: `tests/interface/supervisor_test.cpp`

**Interfaces:**
- Produces: `Supervisor` constructor
  `Supervisor(JointPositionMode& pos, JointImpedanceMode& imp, RtExecutor& exec, Seqlock<JointFeedback>& snap, Dynamics& pump_dyn, StreamPort& stream, ActionServerPort& action, SupervisorConfig cfg = {})`,
  implementing `CommandSink`. Methods: `void start()` (requests initial position mode, spawns sampler + pump threads), `void stop()` (joins threads). `SupervisorConfig { double sampler_hz=250.0; double pump_hz=100.0; double mode_settle_s=0.25; };`

- [ ] **Step 1: Write the failing test**

```cpp
// tests/interface/supervisor_test.cpp  (append)
#include "kinova_lowlevel/interface/supervisor.h"
#include "kinova_lowlevel/joint_position_mode.h"
#include "kinova_lowlevel/joint_impedance_mode.h"
#include "kinova_lowlevel/rt_executor.h"
#include "kinova_lowlevel/sim_transport.h"
#include "kinova_lowlevel/feedback_tap.h"
#include "kinova_lowlevel/dynamics.h"
#include <atomic><thread>

namespace {
// Fixture wires: SimTransport -> FeedbackTap -> RtExecutor(main-ish thread) + Supervisor + FakeBackend.
struct SupFix {
  Dynamics dyn{URDF_PATH}, pump_dyn{URDF_PATH};
  JointFeedback init;                       // q = 0
  SimTransport sim{init};
  Seqlock<JointFeedback> snap;
  FeedbackTap tap{sim, snap};
  SampleRing ring{1u << 12};
  JointPositionMode pos{dyn};
  JointImpedanceMode imp{dyn};
  RtExecutor exec{tap, ring, {1000.0, kinova::Pacing::kSleepSpin, {}}};
  FakeBackend be;
  interface::Supervisor sup{pos, imp, exec, snap, pump_dyn, be, be};
  std::atomic<bool> stop{false};
  std::thread rt;
  void run_rt() { rt = std::thread([&]{ exec.run(stop); }); }
  void teardown() { stop = true; if (rt.joinable()) rt.join(); }
};
}  // namespace

TEST(Supervisor, StartStopClean) {
  SupFix f;
  f.sup.start();
  f.run_rt();
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  f.sup.stop();
  f.teardown();
  SUCCEED();          // no crash, no hang, threads joined
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `scratchpad/abra_test.sh 'Supervisor.StartStopClean'`
Expected: FAIL — `supervisor.h` not found.

- [ ] **Step 3: Write the header**

```cpp
// include/kinova_lowlevel/interface/supervisor.h
#pragma once
#include <atomic><deque><mutex><optional><thread>
#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/feedback_tap.h"
#include "kinova_lowlevel/joint_impedance_mode.h"
#include "kinova_lowlevel/joint_position_mode.h"
#include "kinova_lowlevel/rt_executor.h"
#include "kinova_lowlevel/interface/ports.h"
#include "kinova_lowlevel/interface/trajectory_executor.h"
namespace kinova::interface {

struct SupervisorConfig { double sampler_hz = 250.0; double pump_hz = 100.0; double mode_settle_s = 0.25; };

class Supervisor : public CommandSink {
 public:
  Supervisor(JointPositionMode& pos, JointImpedanceMode& imp, RtExecutor& exec,
             Seqlock<JointFeedback>& snap, Dynamics& pump_dyn,
             StreamPort& stream, ActionServerPort& action, SupervisorConfig cfg = {});
  ~Supervisor();
  void start();   // request initial (position) mode; spawn sampler + pump threads
  void stop();    // signal + join both threads

  // CommandSink (called on the backend thread):
  GoalResponse   on_trajectory_goal(const TrajectoryGoal&) override;
  void           on_trajectory_accepted(const GoalId&, const TrajectoryGoal&) override;
  CancelResponse on_trajectory_cancel(const GoalId&) override;
  GainsResult    on_set_gains(const GainsRequest&) override;
  ArmState       on_query_state() override;

 private:
  struct Inbound { GoalId id; TrajectoryGoal goal; bool cancel=false; };
  void sampler_loop();
  void pump_loop();

  JointPositionMode& pos_;  JointImpedanceMode& imp_;  RtExecutor& exec_;
  Seqlock<JointFeedback>& snap_;  Dynamics& pump_dyn_;
  StreamPort& stream_;  ActionServerPort& action_;  SupervisorConfig cfg_;

  std::optional<TrajectoryExecutor> traj_;            // rebuilt on mode switch (Task 8)
  ControlModeKind active_mode_kind_ = ControlModeKind::kPosition;
  std::atomic<bool> in_flight_{false};                // read by on_trajectory_goal
  std::atomic<uint8_t> atomic_mode_{0};               // 0=pos 1=imp; read by on_trajectory_goal

  std::mutex q_mtx_;  std::deque<Inbound> inbox_;     // backend -> sampler handoff
  Seqlock<ArmState> state_snap_;                      // pump -> query_state

  std::atomic<bool> running_{false};
  std::thread sampler_, pump_;
};
}  // namespace kinova::interface
```

- [ ] **Step 4: Write the skeleton implementation**

```cpp
// src/interface/supervisor.cpp
#include "kinova_lowlevel/interface/supervisor.h"
#include <chrono>
namespace kinova::interface {
using clock = std::chrono::steady_clock;
static double secs_since(clock::time_point t0){ return std::chrono::duration<double>(clock::now()-t0).count(); }

Supervisor::Supervisor(JointPositionMode& pos, JointImpedanceMode& imp, RtExecutor& exec,
                       Seqlock<JointFeedback>& snap, Dynamics& pump_dyn,
                       StreamPort& stream, ActionServerPort& action, SupervisorConfig cfg)
  : pos_(pos), imp_(imp), exec_(exec), snap_(snap), pump_dyn_(pump_dyn),
    stream_(stream), action_(action), cfg_(cfg) {}
Supervisor::~Supervisor(){ stop(); }

void Supervisor::start() {
  exec_.request_mode(&pos_);                       // initial mode = position
  traj_.emplace(pos_);                             // executor bound to the active mode's sink
  active_mode_kind_ = ControlModeKind::kPosition;  atomic_mode_.store(0);
  running_.store(true);
  sampler_ = std::thread([this]{ sampler_loop(); });
  pump_    = std::thread([this]{ pump_loop(); });
}
void Supervisor::stop() {
  if (!running_.exchange(false)) return;
  if (sampler_.joinable()) sampler_.join();
  if (pump_.joinable())    pump_.join();
}

void Supervisor::pump_loop() {
  const auto period = std::chrono::duration<double>(1.0/cfg_.pump_hz);
  const auto t0 = clock::now();
  while (running_.load(std::memory_order_acquire)) {
    JointFeedback fb;
    if (snap_.load(fb)) {
      ArmState s; s.q=fb.q; s.qd=fb.qd; s.tau=fb.tau; s.fault=fb.fault; s.stamp_s=secs_since(t0);
      s.ee_pose = pump_dyn_.fk(fb.q);
      state_snap_.store(s);
      stream_.publish_state(s);
    }
    std::this_thread::sleep_for(std::chrono::duration_cast<clock::duration>(period));
  }
}

void Supervisor::sampler_loop() {                 // fleshed out in Tasks 6-9
  const auto period = std::chrono::duration<double>(1.0/cfg_.sampler_hz);
  while (running_.load(std::memory_order_acquire)) {
    std::this_thread::sleep_for(std::chrono::duration_cast<clock::duration>(period));
  }
}

// Stubs — filled in by later tasks.
GoalResponse   Supervisor::on_trajectory_goal(const TrajectoryGoal&){ return GoalResponse::kReject; }
void           Supervisor::on_trajectory_accepted(const GoalId&, const TrajectoryGoal&){}
CancelResponse Supervisor::on_trajectory_cancel(const GoalId&){ return CancelResponse::kReject; }
GainsResult    Supervisor::on_set_gains(const GainsRequest&){ return {}; }
ArmState       Supervisor::on_query_state(){ ArmState s; state_snap_.load(s); return s; }
}  // namespace kinova::interface
```

- [ ] **Step 5: Register source + run + commit**

Add `src/interface/supervisor.cpp` to `KINOVA_LIB_SOURCES` in `CMakeLists.txt`.
Run: `scratchpad/abra_test.sh 'Supervisor.StartStopClean'` → PASS.
```bash
git add include/kinova_lowlevel/interface/supervisor.h src/interface/supervisor.cpp CMakeLists.txt tests/interface/supervisor_test.cpp
git commit -m "feat(interface): Supervisor skeleton — lifecycle, threads, initial mode, query_state/pump"
```

---

### Task 5: State pump publishes ArmState from live feedback

**Files:**
- Test: `tests/interface/supervisor_test.cpp`
- (Implementation already in Task 4's `pump_loop`; this task verifies it against a seeded arm.)

- [ ] **Step 1: Write the failing test**

```cpp
TEST(Supervisor, PumpPublishesArmStateFromFeedback) {
  SupFix f;
  f.init.q = JointVec::Constant(0.25);
  // reseed sim with a non-zero q so the tap snapshots it:
  f.sim = SimTransport(f.init);
  f.sup.start(); f.run_rt();
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  f.sup.stop(); f.teardown();
  ASSERT_GT(f.be.state_count(), 0u);
  EXPECT_NEAR(f.be.last_state().q[0], 0.25, 1e-6);        // q flowed feedback->pump->StreamPort
  // query_state returns the same latest snapshot:
  EXPECT_NEAR(f.sup.on_query_state().q[0], 0.25, 1e-6);
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `scratchpad/abra_test.sh 'Supervisor.PumpPublishesArmStateFromFeedback'`
Expected: FAIL if the fixture reseed pattern needs adjustment, or PASS if Task 4's pump already satisfies it. If it fails for a reason other than the assertion (e.g. `SimTransport` not reassignable), make the fixture construct `sim` from `init` before wiring the tap — adjust the fixture so `init.q` is set before `SimTransport sim{init}`.

- [ ] **Step 3: Make it pass**

No new production code expected (pump implemented in Task 4). If the fixture can't reseed after construction, refactor `SupFix` to take `q0` in its constructor and set `init.q` first. Keep the change test-only.

- [ ] **Step 4: Run to verify pass**

Run: `scratchpad/abra_test.sh 'Supervisor.PumpPublishesArmStateFromFeedback'` → PASS.

- [ ] **Step 5: Commit**

```bash
git add tests/interface/supervisor_test.cpp
git commit -m "test(interface): supervisor pump publishes ArmState + query_state reflects live q"
```

---

### Task 6: Accept and execute a position-mode goal to completion

**Files:**
- Modify: `src/interface/supervisor.cpp` (`on_trajectory_goal`, `on_trajectory_accepted`, `sampler_loop`)
- Test: `tests/interface/supervisor_test.cpp`

**Interfaces:**
- Consumes: `TrajectoryExecutor::submit/tick`, `ExecStatus`.
- Produces: a working accept→submit→sample→settle path for a single position goal. Feedback published each tick; on time-based completion, `settle(id, {SUCCESSFUL,…})`.

- [ ] **Step 1: Write the failing test**

```cpp
static interface::Trajectory ramp7(double from,double to,double dur){
  interface::Trajectory t; JointVec a=JointVec::Constant(from), b=JointVec::Constant(to);
  t.points = {{a,0.0},{b,dur}}; return t; }

TEST(Supervisor, PositionGoalRunsToCompletionAndSettlesSuccess) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::TrajectoryGoal g;
  g.trajectory = ramp7(0.0, 0.05, 0.4);
  g.control_mode = interface::ControlModeKind::kPosition;
  g.preemption   = interface::Preemption::kLatestWins;
  g.path_tolerance = JointVec::Constant(-1.0);       // guard off for this test (sim is static echo)
  interface::GoalId id{}; id[0]=1;
  ASSERT_EQ(f.sup.on_trajectory_goal(g), interface::GoalResponse::kAccept);
  f.sup.on_trajectory_accepted(id, g);
  std::this_thread::sleep_for(std::chrono::milliseconds(800));   // > duration + settle
  f.sup.stop(); f.teardown();
  ASSERT_EQ(f.be.result_count(), 1u);
  EXPECT_EQ(f.be.last_result().error_code, interface::result_code::kSuccessful);
  EXPECT_EQ(f.be.last_result_id()[0], 1);
  EXPECT_GT(f.be.feedback_count(), 0u);              // add feedback_count() to FakeBackend
}
```

(Add `size_t feedback_count() const` to `FakeBackend`.)

- [ ] **Step 2: Run to verify it fails**

Run: `scratchpad/abra_test.sh 'Supervisor.PositionGoalRunsToCompletionAndSettlesSuccess'`
Expected: FAIL — `on_trajectory_goal` currently rejects (stub returns kReject).

- [ ] **Step 3: Implement accept + sampler execution**

```cpp
// on_trajectory_goal (backend thread): fast pre-check only, no executor mutation.
GoalResponse Supervisor::on_trajectory_goal(const TrajectoryGoal& g){
  if (g.trajectory.points.empty()) return GoalResponse::kReject;                 // INVALID_GOAL
  const uint8_t want = (g.control_mode==ControlModeKind::kImpedance)?1:0;
  if (in_flight_.load() && want != atomic_mode_.load()) return GoalResponse::kReject; // mode-change-while-moving
  return GoalResponse::kAccept;
}
void Supervisor::on_trajectory_accepted(const GoalId& id, const TrajectoryGoal& g){
  std::lock_guard<std::mutex> l(q_mtx_); inbox_.push_back({id, g, false});
}
```

Flesh out `sampler_loop`:
```cpp
void Supervisor::sampler_loop(){
  const auto period = std::chrono::duration<double>(1.0/cfg_.sampler_hz);
  const auto t0 = clock::now();
  GoalId active_id{}; bool have_active=false;
  while (running_.load(std::memory_order_acquire)) {
    // 1) drain inbox (only this thread touches traj_)
    for (;;) {
      Inbound in; { std::lock_guard<std::mutex> l(q_mtx_); if (inbox_.empty()) break; in=inbox_.front(); inbox_.pop_front(); }
      if (in.cancel) continue;                                   // cancel handled in Task 9
      // (mode switching handled in Task 8; Task 6 assumes same/position mode)
      const SubmitResult sr = traj_->submit(in.goal.trajectory, in.goal.control_mode,
                                            in.goal.preemption, in.goal.path_tolerance);
      if (sr != SubmitResult::kAccepted) {
        TrajectoryResult r; r.error_code=result_code::kInvalidGoal; r.error_string="rejected by executor";
        action_.settle(in.id, r); continue;
      }
      if (have_active && in.goal.preemption==Preemption::kLatestWins) {          // preempted the old goal
        TrajectoryResult r; r.error_code=result_code::kPreempted; action_.settle(active_id, r);
      }
      active_id = in.id; have_active = true;
      in_flight_.store(true);
    }
    // 2) tick the active trajectory
    if (traj_->is_active()) {
      JointFeedback fb; JointVec q = JointVec::Zero(); if (snap_.load(fb)) q = fb.q;
      const ExecStatus st = traj_->tick(secs_since(t0), q);
      TrajectoryFeedback fbk; fbk.actual=q; fbk.fraction_complete=st.fraction; action_.publish_feedback(active_id, fbk);
      if (st.promoted) { /* Task 9 */ }
      if (st.completed && have_active) {
        TrajectoryResult r;
        r.error_code = (st.error_code==ExecStatus::kPathToleranceViolated)
                       ? result_code::kPathToleranceViolated : result_code::kSuccessful;
        action_.settle(active_id, r); have_active=false; in_flight_.store(false);
      }
    }
    std::this_thread::sleep_for(std::chrono::duration_cast<clock::duration>(period));
  }
}
```

- [ ] **Step 4: Run to verify pass**

Run: `scratchpad/abra_test.sh 'Supervisor.PositionGoalRunsToCompletionAndSettlesSuccess'` → PASS.

- [ ] **Step 5: Commit**

```bash
git add src/interface/supervisor.cpp tests/interface/supervisor_test.cpp tests/interface/fake_backend.h
git commit -m "feat(interface): supervisor executes a position goal end-to-end, settles SUCCESSFUL"
```

---

### Task 7: Divergence abort surfaces PATH_TOLERANCE_VIOLATED

**Files:**
- Test: `tests/interface/supervisor_test.cpp` (logic already in Task 6's completion branch)

- [ ] **Step 1: Write the failing test**

```cpp
TEST(Supervisor, DivergenceAbortSettlesPathToleranceViolated) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::TrajectoryGoal g;
  g.trajectory = ramp7(0.0, 0.5, 2.0);               // moves 0.5 rad; SimTransport never moves
  g.path_tolerance = JointVec::Constant(0.2);        // guard ON -> must trip
  interface::GoalId id{}; id[0]=2;
  ASSERT_EQ(f.sup.on_trajectory_goal(g), interface::GoalResponse::kAccept);
  f.sup.on_trajectory_accepted(id, g);
  std::this_thread::sleep_for(std::chrono::milliseconds(600));
  f.sup.stop(); f.teardown();
  ASSERT_EQ(f.be.result_count(), 1u);
  EXPECT_EQ(f.be.last_result().error_code, interface::result_code::kPathToleranceViolated);
  EXPECT_FALSE(f.sup.on_query_state().fault);         // divergence is not a hardware fault
}
```

- [ ] **Step 2: Run to verify it fails or passes**

Run: `scratchpad/abra_test.sh 'Supervisor.DivergenceAbortSettlesPathToleranceViolated'`
Expected: PASS if Task 6's completion branch already maps `kPathToleranceViolated`. If it FAILS, the sampler must feed real `q_meas` (it does) and map the code (it does) — debug via systematic-debugging skill. Treat a pass here as confirmation, not a no-op: it proves the guard runs through the supervisor's live `q`.

- [ ] **Step 3: (only if failing) fix the mapping**

No new code expected. If failing, verify `snap_.load(fb)` returns the static echo `q≈0` while `desired` ramps, so `|error|` crosses 0.2 — matching the merged `trajectory_run` behavior.

- [ ] **Step 4: Commit**

```bash
git add tests/interface/supervisor_test.cpp
git commit -m "test(interface): supervisor surfaces PATH_TOLERANCE_VIOLATED on live-feedback divergence"
```

---

### Task 8: Mode-change-at-rest + impedance selection

**Files:**
- Modify: `src/interface/supervisor.cpp` (inbox drain: mode switch)
- Test: `tests/interface/supervisor_test.cpp`

**Interfaces:**
- Produces: on an accepted goal whose `control_mode` differs from the active one AND nothing is in flight — rebuild `traj_` against the new mode's sink, push gains (impedance), `request_mode`, wait `mode_settle_s`, then submit. A differing-mode goal while in flight was already rejected at `on_trajectory_goal`.

- [ ] **Step 1: Write the failing tests**

```cpp
TEST(Supervisor, RejectsModeChangeWhileInFlight) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::TrajectoryGoal g1; g1.trajectory=ramp7(0.0,0.05,1.0); g1.path_tolerance=JointVec::Constant(-1.0);
  g1.control_mode=interface::ControlModeKind::kPosition;
  interface::GoalId id1{}; id1[0]=1;
  ASSERT_EQ(f.sup.on_trajectory_goal(g1), interface::GoalResponse::kAccept);
  f.sup.on_trajectory_accepted(id1, g1);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));       // now in flight
  interface::TrajectoryGoal g2 = g1; g2.control_mode=interface::ControlModeKind::kImpedance;
  EXPECT_EQ(f.sup.on_trajectory_goal(g2), interface::GoalResponse::kReject);   // mode change mid-motion
  f.sup.stop(); f.teardown();
}

TEST(Supervisor, SwitchesToImpedanceAtRest) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::TrajectoryGoal g; g.trajectory=ramp7(0.0,0.03,0.3); g.path_tolerance=JointVec::Constant(-1.0);
  g.control_mode=interface::ControlModeKind::kImpedance;
  g.has_gains=true; g.gains.kq=JointVec::Constant(60.0); g.gains.zeta=0.6;
  g.gains.torque_limit=(JointVec()<<39,39,39,39,9,9,9).finished();
  interface::GoalId id{}; id[0]=9;
  ASSERT_EQ(f.sup.on_trajectory_goal(g), interface::GoalResponse::kAccept);
  f.sup.on_trajectory_accepted(id, g);
  std::this_thread::sleep_for(std::chrono::milliseconds(900));       // settle + duration
  f.sup.stop(); f.teardown();
  ASSERT_EQ(f.be.result_count(), 1u);
  EXPECT_EQ(f.be.last_result().error_code, interface::result_code::kSuccessful);
}
```

- [ ] **Step 2: Run to verify they fail**

Run: `scratchpad/abra_test.sh 'Supervisor.RejectsModeChangeWhileInFlight:Supervisor.SwitchesToImpedanceAtRest'`
Expected: `RejectsModeChangeWhileInFlight` may already PASS (pre-check exists); `SwitchesToImpedanceAtRest` FAILS (no mode switch — traj_ still bound to position, wrong mode adopted).

- [ ] **Step 3: Implement the mode switch in the inbox drain**

Replace the "(mode switching handled in Task 8…)" comment in `sampler_loop`'s drain with:
```cpp
if (in.goal.control_mode != active_mode_kind_) {
  // Only reachable when NOT in flight (on_trajectory_goal rejects otherwise).
  if (in.goal.control_mode == ControlModeKind::kImpedance) {
    if (in.goal.has_gains) { JointImpedanceParams p; p.Kq=in.goal.gains.kq; p.zeta=in.goal.gains.zeta;
                             p.torque_limit=in.goal.gains.torque_limit; imp_.set_gains(p); }
    exec_.request_mode(&imp_); traj_.emplace(imp_);
    active_mode_kind_=ControlModeKind::kImpedance; atomic_mode_.store(1);
  } else {
    exec_.request_mode(&pos_); traj_.emplace(pos_);
    active_mode_kind_=ControlModeKind::kPosition; atomic_mode_.store(0);
  }
  std::this_thread::sleep_for(                                    // let the RT loop adopt + on_enter settle
      std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(cfg_.mode_settle_s)));
}
```

- [ ] **Step 4: Run to verify pass**

Run: `scratchpad/abra_test.sh 'Supervisor.*'` → all PASS.

- [ ] **Step 5: Commit**

```bash
git add src/interface/supervisor.cpp tests/interface/supervisor_test.cpp
git commit -m "feat(interface): supervisor mode-change-at-rest + impedance gains selection"
```

---

### Task 9: Preemption + cancel result semantics

**Files:**
- Modify: `src/interface/supervisor.cpp` (`on_trajectory_cancel`, promotion handling in `sampler_loop`)
- Test: `tests/interface/supervisor_test.cpp`

**Interfaces:**
- Produces: `on_trajectory_cancel` enqueues a cancel that aborts the active goal (settle `PREEMPTED`); on `st.promoted` the finished goal settles `SUCCESSFUL` and the queued id becomes active. Latest-wins preemption already settles the old goal (Task 6).

- [ ] **Step 1: Write the failing test**

```cpp
TEST(Supervisor, LatestWinsPreemptionSettlesOldGoalPreempted) {
  SupFix f; f.sup.start(); f.run_rt();
  interface::TrajectoryGoal g; g.trajectory=ramp7(0.0,0.1,2.0); g.path_tolerance=JointVec::Constant(-1.0);
  interface::GoalId a{}; a[0]=1; interface::GoalId b{}; b[0]=2;
  f.sup.on_trajectory_goal(g); f.sup.on_trajectory_accepted(a, g);
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  f.sup.on_trajectory_goal(g); f.sup.on_trajectory_accepted(b, g);   // latest-wins preempt
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  f.sup.stop(); f.teardown();
  // first settled result must be goal 'a' with PREEMPTED
  bool saw_preempt_a=false;
  for (auto& pr : f.be.all_results()) if (pr.first[0]==1) saw_preempt_a = (pr.second.error_code==interface::result_code::kPreempted);
  EXPECT_TRUE(saw_preempt_a);
}
```

(Add `std::vector<std::pair<GoalId,TrajectoryResult>> all_results() const` to `FakeBackend`.)

- [ ] **Step 2: Run to verify it fails/passes**

Run: `scratchpad/abra_test.sh 'Supervisor.LatestWinsPreemptionSettlesOldGoalPreempted'`
Expected: PASS if Task 6's preempt branch fired; if the queued-id bookkeeping for promotion is missing it still passes (latest-wins path is Task 6). Keep this test as the regression guard.

- [ ] **Step 3: Implement cancel + promotion bookkeeping**

```cpp
CancelResponse Supervisor::on_trajectory_cancel(const GoalId& id){
  std::lock_guard<std::mutex> l(q_mtx_); inbox_.push_back({id, {}, true}); return CancelResponse::kAccept;
}
```
In the drain loop, handle `in.cancel`: if there is an active goal, settle it `PREEMPTED`, clear active, `in_flight_.store(false)`, and submit an empty/hold so the executor stops (or track a `cancel_requested` the tick loop honors). In the tick block, when `st.promoted`, settle the just-finished `active_id` as `SUCCESSFUL` and swap in the queued goal's id (track `queued_id_` set when a `kQueue` submit is accepted).

- [ ] **Step 4: Run to verify pass**

Run: `scratchpad/abra_test.sh 'Supervisor.*:Executor*'` → all PASS.

- [ ] **Step 5: Commit**

```bash
git add src/interface/supervisor.cpp tests/interface/supervisor_test.cpp tests/interface/fake_backend.h
git commit -m "feat(interface): supervisor preemption/cancel/promotion result semantics"
```

---

### Task 10: RT-safety with the Supervisor in the loop

**Files:**
- Modify: `tests/rt_safety_test.cpp` (add a supervisor-in-the-loop variant)
- Test: same file

**Interfaces:**
- Consumes: existing `rt_safety_test` harness patterns.
- Produces: a test that runs `RtExecutor` (SimTransport, via FeedbackTap) for a steady-state window with the Supervisor's sampler + pump active and a position goal executing, asserting **zero major page faults** and **zero dropped samples**.

- [ ] **Step 1: Write the failing test**

```cpp
// tests/rt_safety_test.cpp (append; reuse the file's existing includes/patterns)
TEST(RtSafety, SupervisorInLoopNoMajorFaultsSteadyState) {
  using namespace kinova; using namespace kinova::interface;
  Dynamics dyn{URDF_PATH}, pump_dyn{URDF_PATH};
  JointFeedback init; SimTransport sim{init};
  Seqlock<JointFeedback> snap; FeedbackTap tap{sim, snap};
  SampleRing ring{1u<<16};
  JointPositionMode pos{dyn}; JointImpedanceMode imp{dyn};
  RtExecutor exec{tap, ring, {1000.0, Pacing::kSleepSpin, {}}};
  FakeBackend be; Supervisor sup{pos, imp, exec, snap, pump_dyn, be, be};
  std::atomic<bool> stop{false};
  sup.start();
  std::thread rt([&]{ exec.run(stop); });
  TrajectoryGoal g; g.trajectory = ramp7(0.0, 0.2, 3.0); g.path_tolerance=JointVec::Constant(-1.0);
  GoalId id{}; id[0]=1; sup.on_trajectory_goal(g); sup.on_trajectory_accepted(id, g);
  ResourceUsage before = read_usage_of_rt_thread();       // use the harness's existing measurement
  std::this_thread::sleep_for(std::chrono::seconds(2));    // steady-state window
  ResourceUsage after = read_usage_of_rt_thread();
  stop = true; rt.join(); sup.stop();
  EXPECT_EQ(after.majflt - before.majflt, 0u);
  EXPECT_EQ(ring.dropped(), 0u);
}
```

(Match the exact measurement helper the existing `rt_safety_test.cpp` uses for major faults on the RT thread; mirror `RtSafety.JointImpedanceModeNoMajorFaultsSteadyState`.)

- [ ] **Step 2: Run to verify it fails, then passes**

Run: `scratchpad/abra_test.sh 'RtSafety.SupervisorInLoopNoMajorFaultsSteadyState'`
Expected: compiles and PASSES (no production change — this is a gate). If it drops samples or faults, that is a real RT regression: stop and use systematic-debugging; do not weaken the assertion.

- [ ] **Step 3: Commit**

```bash
git add tests/rt_safety_test.cpp
git commit -m "test(rt): RtSafety supervisor-in-the-loop — zero major faults / drops in steady state"
```

---

### Task 11: b1 build integration — install/export + package.xml

**Files:**
- Create: `cmake/kinova_lowlevelConfig.cmake.in`
- Create: `package.xml`
- Modify: `CMakeLists.txt` (BUILD_INTERFACE/INSTALL_INTERFACE include dirs; `install(TARGETS … EXPORT)`; generate + install the config)
- Test: a throwaway downstream `find_package` consumer (built by hand in the step; not added to `unit_tests`)

**Interfaces:**
- Produces: an installed `kinova_lowlevel` target exported as `kinova_lowlevel::kinova_lowlevel`, discoverable via `find_package(kinova_lowlevel CONFIG)`; a root `package.xml` (`<build_type>cmake</build_type>`) so colcon can build the core when vendored into the ROS2 workspace. Default sim build/test unchanged.

- [ ] **Step 1: Make the library's include dirs install-safe**

In `CMakeLists.txt`, replace:
```cmake
target_include_directories(kinova_lowlevel PUBLIC include)
```
with:
```cmake
target_include_directories(kinova_lowlevel PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
```

- [ ] **Step 2: Verify the default build still works (regression gate)**

Run: `scratchpad/abra_test.sh`
Expected: full `ctest` PASS — the generator-expression change must not affect the in-tree build.

- [ ] **Step 3: Add install + export + config template**

Create `cmake/kinova_lowlevelConfig.cmake.in`:
```cmake
@PACKAGE_INIT@
include(CMakeFindDependencyMacro)
find_dependency(pinocchio)
include("${CMAKE_CURRENT_LIST_DIR}/kinova_lowlevelTargets.cmake")
```
Append to `CMakeLists.txt` (after the library target, guarded so it is always on for plain builds):
```cmake
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
install(TARGETS kinova_lowlevel EXPORT kinova_lowlevelTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT kinova_lowlevelTargets NAMESPACE kinova_lowlevel::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/kinova_lowlevel)
configure_package_config_file(cmake/kinova_lowlevelConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/kinova_lowlevelConfig.cmake
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/kinova_lowlevel)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/kinova_lowlevelConfig.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/kinova_lowlevel)
```

- [ ] **Step 4: Add the root `package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>kinova_lowlevel</name>
  <version>0.1.0</version>
  <description>Kinova Gen3 low-level RT control driver (ROS-agnostic core).</description>
  <maintainer email="swapnil.pande98@gmail.com">Swapnil Pande</maintainer>
  <license>Proprietary</license>
  <buildtool_depend>cmake</buildtool_depend>
  <depend>pinocchio</depend>
  <depend>eigen</depend>
  <export><build_type>cmake</build_type></export>
</package>
```

- [ ] **Step 5: Verify install + a downstream find_package (on abra)**

```bash
# on abra, in the build dir the deploy script created:
cmake --install /tmp/kinova-build/build --prefix /tmp/kinova-install
# tiny consumer:
mkdir -p /tmp/consumer && cd /tmp/consumer
printf 'cmake_minimum_required(VERSION 3.16)\nproject(c CXX)\nfind_package(kinova_lowlevel CONFIG REQUIRED)\nadd_executable(c main.cpp)\ntarget_link_libraries(c kinova_lowlevel::kinova_lowlevel)\n' > CMakeLists.txt
printf '#include "kinova_lowlevel/interface/supervisor.h"\nint main(){ return 0; }\n' > main.cpp
cmake -S . -B b -DCMAKE_PREFIX_PATH="/tmp/kinova-install;/usr/local/lib/python3.10/dist-packages/cmeel.prefix" && cmake --build b
```
Expected: the consumer configures (finds `kinova_lowlevelConfig.cmake`) and compiles against the installed headers/target. Capture the success in the commit message.

- [ ] **Step 6: Commit**

```bash
git add CMakeLists.txt cmake/kinova_lowlevelConfig.cmake.in package.xml
git commit -m "build: install/export kinova_lowlevel (b1) + root package.xml for colcon (build_type cmake)"
```

---

## Self-review notes (author)

- **Spec coverage:** Component 1 ports → Tasks 1–2; Component 2 action *content* (goal fields) → `TrajectoryGoal` Task 1 (the ROS `.action` itself is Plan 3); Component 3 sampler (sample/complete/diverge/preempt/mode-at-rest) → Tasks 6–9; Component 4 telemetry (state stream + action feedback) → Tasks 4–6 (full-rate SysID log is out of scope, unchanged `SampleRing`); Component 6 threading/single-writer/per-thread Dynamics/FeedbackTap/lifecycle → Tasks 4–9; RT-safety gate → Task 10; build integration (b1) → Task 11. `Ros2Backend` (Component 5) is Plan 3, a separate repo.
- **Deferred within Plan 2 (surfaced, not hidden):** `goal_tolerance`/`goal_time_tolerance` are carried in the value type but only *informational* in v1 (the executor completes on time; a `GOAL_TOLERANCE_VIOLATED` end-check can be added when a physics sim exists). `sender_id` is carried but arbitration policy is deferred by spec. `set_gains` **service defaults** (`on_set_gains`) is a thin stub returning accepted; per-goal gains are authoritative (spec §Gains authority).
- **Concurrency invariant:** only the sampler thread touches `traj_`/`set_target`; backend callbacks mutate only the mutex-guarded `inbox_` and atomics. The pre-check atomics (`in_flight_`, `atomic_mode_`) are advisory; `TrajectoryExecutor::submit` is the authority (returns `kRejectedModeChangeWhileMoving`), and the sampler settles a rejected submit.
```
