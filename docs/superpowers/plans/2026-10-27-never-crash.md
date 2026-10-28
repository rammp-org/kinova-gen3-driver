# The driver never crashes: implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** No arm-side event (fault, forced out of low-level, disconnect, arm off at start, RPC timeout) terminates `kinova_gen3_node`. Faults are reported and wait for an explicit clear. An unreachable arm is retried forever.

**Architecture:** A new core unit `ArmLink` owns the link lifecycle (`kConnecting` / `kRunning` / `kFaulted`) on the thread that used to call `RtExecutor::run()` directly, and calls `run()` while `kRunning`. `KortexTransport` classifies KORTEX errors into driver exception types at the boundary. The executor can exit on a fault seen in feedback (opt-in). The Supervisor listens for link-state changes and halts or rejects work while the arm is unavailable. The ROS node swaps `exec.run()` for `ArmLink::run()` and adds `/clear_faults` and link state in `/diagnostics`.

**Tech Stack:** C++17, gtest, KORTEX 2.8.0 (behind `KINOVA_ENABLE_KORTEX`), ROS2 (rclcpp, std_srvs, diagnostic_updater).

**Spec:** `docs/superpowers/specs/2026-10-27-never-crash-design.md`

## Global Constraints

- Nothing in the steady-state RT cycle may allocate, lock or block; `RtSafety*` stays green.
- KORTEX types never leave `kortex_transport.cpp` / `kortex_cyclic.h`.
- SI/radians inside; no new unit conversions outside `Transport`.
- `ExecutorConfig::exit_on_fault` defaults to **false**; `ArmLink` requires it true.
- Fault confirm: **3** consecutive good exchanges with `fault || !low_level`.
- Retry period **1 s**; faulted feedback poll **100 Hz**; health poll **every 0.5 s**, **3** consecutive failures = link lost.
- Every RPC in `release_low_level()` and `arm_in_fault()` is bounded at **500 ms**.
- Halt reason text: `"halted: arm fault or connection lost"`.
- The only exception `ArmLink::run()` lets out is `std::logic_error` (actuator-count mismatch from `connect()`).
- Driver work on branch `feat/96-never-crash` off `origin/dev`; ROS work on `feat/96-never-crash` off kinova-gen3-ros2 `origin/dev`. Stage explicit paths only.

## Review Focus

1. **Reclaim drives back to an old target.** After a reconnect or clear, the first adopted mode must be `JointPositionMode` seeded from fresh feedback, and no trajectory/stream from before survives. Pinned by Task 5 `ReconnectHoldsWhereTheArmIsNow`.
2. **A stop signal in each state.** `run()` must return (and `safe_shutdown()` must run) whether stop arrives in `kConnecting`, `kFaulted` or `kRunning`. Pinned by Task 3 `StopReturnsFromEveryState`.
3. **Clear requested while the fault persists.** Must stay `kFaulted` with reason `"fault persists after clear"`, never enter low-level. Pinned by Task 3 `ClearWhileStillFaultedStaysFaulted`.
4. **Disconnect while faulted.** The fault latch must survive the reconnect. Pinned by Task 3 `FaultSurvivesAReconnect`.
5. **Stale state published while unreachable.** The pump must not publish in `kConnecting`. Pinned by Task 5 `NoStatePublishedWhileConnecting`.

---

### Task 1: Transport surface for faults and release

**Files:**
- Modify: `include/kinova_lowlevel/transport.h`, `include/kinova_lowlevel/joint_types.h` (`JointFeedback`), `include/kinova_lowlevel/feedback_tap.h`, `include/kinova_lowlevel/gripper_controller.h`, `include/kinova_lowlevel/sim_transport.h`, `src/sim_transport.cpp`, `tests/rt_executor_miss_test.cpp` (stub the two new methods on `DroppingTransport`)
- Test: `tests/sim_transport_test.cpp`, `tests/feedback_tap_test.cpp`, `tests/gripper_controller_test.cpp`

**Interfaces:**
- Produces:
  - `class ArmRejected : public std::runtime_error` in `transport.h`: the arm refused a request (KORTEX error frame / `KDetailedException`).
  - `virtual void release_low_level() = 0;`: actuators POSITION + base single-level, session kept; no-op when not in low-level.
  - `virtual bool arm_in_fault() = 0;`: base state query over the control channel; throws `TransportTimeout` when unanswered.
  - `bool JointFeedback::low_level = true;`: false when the base reports it is not in low-level servoing.
  - `SimTransport` knobs: `void fail_connects(int n)` (next n `connect()` throw `std::runtime_error("sim: arm unreachable")`); `void set_arm_fault(bool)` (feedback `fault` and `arm_in_fault()` both report it; `clear_faults()` resets it unless `set_fault_sticky(true)`); `void set_fault_sticky(bool)`; `void set_low_level_feedback(bool)`; `void throw_on_exchange(std::function<void()>)` (called at the top of each `exchange()`, may throw); `void fail_health(int n)` (next n `arm_in_fault()` throw `TransportTimeout`); counters `int connects, releases, shutdowns, low_level_entries, clears`.

- [ ] **Step 1: Write failing tests**
  - `FeedbackTap.ForwardsReleaseAndFaultQuery` and `GripperController.ForwardsReleaseAndFaultQuery`: wrap a `SimTransport`, call `release_low_level()` / `arm_in_fault()` through the decorator, assert `sim.releases == 1` and the fault answer matches `set_arm_fault(true)`.
  - `SimTransport.InjectedFaultShowsInFeedbackAndQuery`: `set_arm_fault(true)`, `exchange` → `fb.fault == true`, `arm_in_fault() == true`; `clear_faults()` → both false.
  - `SimTransport.StickyFaultSurvivesClear`.
  - `SimTransport.FailConnectsThrowsThenSucceeds`: `fail_connects(2)`; two throws, third returns; `connects == 3`.
  - `SimTransport.LowLevelFeedbackKnob`: default `fb.low_level == true`; after `set_low_level_feedback(false)`, false.
- [ ] **Step 2: Run** `build.sh sim --gtest_filter='SimTransport*:FeedbackTap*:GripperController*'`. Expected: compile failure on the missing members.
- [ ] **Step 3: Implement.** Add the types and pure virtuals; forward in both decorators; implement in `SimTransport` (`release_low_level` and `safe_shutdown` count; `set_servoing_low_level` counts `low_level_entries`); stub on `DroppingTransport` (`release_low_level` no-op, `arm_in_fault` → false).
- [ ] **Step 4: Run** the full suite. Expected: all pass (the known `Supervisor.StreamingAPoseIntoPositionModeIsAccepted` flake excepted).
- [ ] **Step 5: Commit.**

### Task 2: Executor exits on a confirmed fault (opt-in)

**Files:**
- Modify: `include/kinova_lowlevel/rt_executor.h`, `src/rt_executor.cpp`
- Test: `tests/rt_executor_miss_test.cpp`

**Interfaces:**
- Consumes: `JointFeedback::low_level`, `ArmRejected` (Task 1).
- Produces:
  - `bool ExecutorConfig::exit_on_fault = false;`
  - `class ArmFault : public std::runtime_error` in `rt_executor.h`.
  - `inline constexpr int kFaultConfirmExchanges = 3;`
  - `const ExecutorConfig& RtExecutor::config() const noexcept;`

- [ ] **Step 1: Write failing tests** (extend `DroppingTransport` with an `on_fb` hook that edits `fb` on a successful exchange):
  - `RtExecutorFault.ThreeFaultyExchangesThrowArmFault`: `exit_on_fault=true`, fault from call 5 on → `run()` throws `ArmFault`; `calls == 5 + 3` (the seed included in the count as the fake defines it; assert the exact count the fake produces).
  - `RtExecutorFault.TwoFaultyThenHealthyDoesNotThrow`: fault on calls 5,6 only → run returns on stop.
  - `RtExecutorFault.LeftLowLevelCountsAsFault`: `low_level=false` from call 5 → `ArmFault`, message contains `"low-level"`.
  - `RtExecutorFault.OffByDefaultKeepsTodaysBehaviour`: `exit_on_fault=false`, fault forever → returns on stop, sample flag bit 2 set.
  - `RtExecutorFault.ArmRejectedPropagates`: `exchange` throws `ArmRejected` → `run()` throws `ArmRejected`.
  - `RtExecutorFault.RequestedModeSurvivesAcrossRuns`: `request_mode(&m)`; first `run()` throws `ArmFault` before adopting (fault from the seed on, so `misses==0` but the throw comes first) or, simpler, call `request_mode` after a first `run()` returned; a second `run()` adopts it (`m.entered_frame != UINT64_MAX`).
- [ ] **Step 2: Run** `--gtest_filter='RtExecutor*'`. Expected: FAIL (compile).
- [ ] **Step 3: Implement.** Counter of consecutive good exchanges with `fb.fault || !fb.low_level`, reset on a healthy good exchange, untouched on a miss; on reaching `kFaultConfirmExchanges` with `exit_on_fault`, `snprintf` a message (`"RtExecutor: arm fault: actuator/base fault"` or `"RtExecutor: arm fault: base left low-level servoing"`), print it to stderr, and throw `ArmFault`. The seed read's fault doesn't count (it belongs to `ArmLink`'s pre-check). No allocation before the throw.
- [ ] **Step 4: Run** `--gtest_filter='RtExecutor*:RtSafety*'`. Expected: PASS.
- [ ] **Step 5: Commit.**

### Task 3: `ArmLink`

**Files:**
- Create: `include/kinova_lowlevel/arm_link.h`, `src/arm_link.cpp`, `tests/arm_link_test.cpp`
- Modify: `CMakeLists.txt` (library source + test source)

**Interfaces:**
- Consumes: Task 1 Transport surface + SimTransport knobs; Task 2 `ArmFault`, `CommsLost`, `RtExecutor::config()`.
- Produces:
```cpp
enum class LinkState { kConnecting, kRunning, kFaulted };
const char* to_string(LinkState);
struct LinkStatus { LinkState state = LinkState::kConnecting; std::string reason; uint64_t faults = 0, reconnects = 0; };
class LinkListener { public: virtual ~LinkListener() = default; virtual void on_link_state(LinkState) = 0; };
struct ArmLinkConfig { double retry_period_s = 1.0, fault_poll_hz = 100.0, health_period_s = 0.5; int health_max_failures = 3; };
class ArmLink {
 public:
  ArmLink(Transport&, RtExecutor&, LinkListener* listener, ArmLinkConfig = {});  // throws std::invalid_argument unless exec.config().exit_on_fault
  void run(std::atomic<bool>& stop);   // returns on stop with the transport shut down; rethrows std::logic_error only
  bool request_clear();                // true iff state is kFaulted; the clear happens on run()'s thread
  LinkStatus status() const;           // mutex-guarded copy
};
```

The algorithm (`run`), since the tests below pin outcomes, not order:
```
state ← kConnecting; connected ← false; latched ← false
loop until stop:
  if !connected: try connect(); connected ← true (count reconnects after the first)
                 catch logic_error: safe_shutdown(); rethrow
                 catch any: reason ← what(); safe_shutdown(); sleep(retry, stop); continue
  try:
    if !latched and arm_in_fault(): latch("arm reports a fault")
    if latched:
      state ← kFaulted; release_low_level()
      wait: poll receive() at fault_poll_hz (errors ignored); health arm_in_fault() each health_period
            (health_max_failures consecutive throws → link lost); request_clear → clear
      on clear: clear_faults(); if arm_in_fault(): reason ← "fault persists after clear"; continue (stay faulted)
                else latched ← false
    set_servoing_low_level(); state ← kRunning; exec.run(stop); break
  catch CommsLost / TransportTimeout / link lost: best-effort safe_shutdown(); connected ← false; state ← kConnecting; sleep(retry, stop)
  catch std::exception / (...): latch(what() or "unknown exception"); best-effort release_low_level()
finally: best-effort safe_shutdown()
```
Listener and stderr log fire on each state *change* only. `latch()` bumps `faults` and sets the reason. Connect failures log the first time, on a changed reason, and every 30th attempt. All sleeps are 10 ms slices checking `stop`.

- [ ] **Step 1: Write failing tests** (real `RtExecutor` at 10 kHz, `exit_on_fault=true`, `SimTransport`, a recording listener; tests stop the link from a helper thread when a predicate on `status()` holds, with a 5 s deadline that fails the test):
  - `ArmLink.RejectsAnExecutorWithoutExitOnFault`.
  - `ArmLink.RetriesConnectUntilTheArmAnswers`: `fail_connects(3)`, `retry_period_s=0.01` → reaches `kRunning`; `connects == 4`; listener saw `[kConnecting, kRunning]`; `reason` while connecting was `"sim: arm unreachable"`.
  - `ArmLink.CommsLossReconnectsWithoutAClear`: running, then `throw_on_exchange` throws `TransportTimeout` repeatedly for a while → state goes `kConnecting` → back to `kRunning` with no `request_clear`; `reconnects == 1`; `shutdowns >= 1`.
  - `ArmLink.FaultWaitsForAnExplicitClear`: running, `set_arm_fault(true)` → `kFaulted`, `releases >= 1`, stays faulted 100 ms; `request_clear()` true → `kRunning`; `clears == 1`; `faults == 1`.
  - `ArmLink.ArmRejectedIsAFault` and `ArmLink.UnknownExceptionIsAFault` (reason contains the `what()`).
  - `ArmLink.ArmAlreadyFaultedAtStartWaits`: `set_arm_fault(true)` before `run` → `kFaulted`, `low_level_entries == 0`.
  - `ArmLink.ClearWhileStillFaultedStaysFaulted`: sticky fault; clear → still `kFaulted`, reason `"fault persists after clear"`, `low_level_entries` unchanged.
  - `ArmLink.FaultSurvivesAReconnect`: faulted, `fail_health(3)` → `kConnecting` → back to `kFaulted` (not running) without a clear.
  - `ArmLink.RequestClearIsRefusedUnlessFaulted`.
  - `ArmLink.StopReturnsFromEveryState`: three sub-cases (stop while `fail_connects(1000)`, while faulted, while running); `run` returns within 1 s; `shutdowns >= 1`.
  - `ArmLink.ConfigErrorEscapes`: a transport whose `connect()` throws `std::invalid_argument` → `run` throws `std::invalid_argument`.
- [ ] **Step 2: Run** `--gtest_filter='ArmLink*'`. Expected: FAIL (missing header).
- [ ] **Step 3: Implement** `src/arm_link.cpp` per the algorithm.
- [ ] **Step 4: Run** `--gtest_filter='ArmLink*'` 5× (`--gtest_repeat=5`). Expected: PASS every time.
- [ ] **Step 5: Commit.**

### Task 4: `KortexTransport` classification, release, health

**Files:**
- Modify: `src/kortex_transport.cpp`, `src/kortex_cyclic.h`, `include/kinova_lowlevel/kortex_transport.h`
- Test: `tests/kortex_cyclic_test.cpp` (KORTEX build only)

**Interfaces:**
- Consumes: Task 1 surface.
- Produces: `KortexTransport::release_low_level()`, `KortexTransport::arm_in_fault()`. Behavioural changes listed below.

- [ ] **Step 1: Write the failing test.** In `kortex_cyclic_test.cpp`, change `KortexCyclic.FailedLocalSendIsARuntimeErrorAndTheRouterStaysUsable` to expect `TransportTimeout` (rename it `FailedLocalSendIsALostFrameAndTheRouterStaysUsable`). If the existing fixture can feed an error frame back, add `KortexCyclic.ErrorFrameIsArmRejected`; if it can't without a fake router server, skip that test and note it.
- [ ] **Step 2: Run** `build.sh kortex --gtest_filter='Kortex*'`. Expected: FAIL.
- [ ] **Step 3: Implement.**
  - `kortex_cyclic_call`: `send()` throwing a non-KORTEX `std::runtime_error` → `TransportTimeout`; `get()` throwing `KDetailedException` → `ArmRejected(what)`.
  - A file-local `template <class F> auto rpc(F&&)` used for every TCP RPC on the run path (`SetControlMode`, `SetServoingMode`, `GetArmState`, `GetActuatorCount`, `ClearFaults`): `KDetailedException` → `ArmRejected`; `std::runtime_error` whose `what()` starts with `"timeout detected"` (KORTEX 2.8.0 RPC timeout text) → `TransportTimeout`.
  - `connect()`: drop the `ClearFaults()` call; actuator-count mismatch throws `std::invalid_argument`.
  - `release_low_level()`: the revert half of today's `safe_shutdown()`, every RPC with `RouterClientSendOptions{false, 0, 500}`, stop at the first failure, keep the 500 ms settle only if a revert RPC was sent; sets `low_level_ = false`. `safe_shutdown()` = `release_low_level()` + today's teardown.
  - `arm_in_fault()`: `base->GetArmState({false,0,500})`, `== ARMSTATE_IN_FAULT`.
  - `fill_feedback`: also `fault |= base().fault_bank_a() || base().fault_bank_b() || base().active_state() == ARMSTATE_IN_FAULT`; `fb.low_level = base().active_state() == ARMSTATE_SERVOING_LOW_LEVEL`.
  - `set_servoing_low_level()`: `cmd_.clear_interconnect()` before the seed; after the seed Refresh, `set_actuator_modes(all kPosition)`.
- [ ] **Step 4: Run** `build.sh kortex` (all tests). Expected: PASS; the KORTEX-enabled apps link.
- [ ] **Step 5: Commit.**

### Task 5: Supervisor listens to the link

**Files:**
- Modify: `include/kinova_lowlevel/interface/value_types.h` (`HaltReason::kArmUnavailable`), `include/kinova_lowlevel/interface/supervisor.h`, `src/interface/supervisor.cpp`
- Test: `tests/interface/supervisor_test.cpp`

**Interfaces:**
- Consumes: `LinkListener`, `LinkState` (Task 3).
- Produces: `class Supervisor : …, public kinova::LinkListener`; `void on_link_state(kinova::LinkState) override;`; `bool arm_ready() const;`. Default ready = true, so a Supervisor without `ArmLink` behaves as today.

- [ ] **Step 1: Write failing tests** (existing fixture):
  - `SupervisorLink.GoalsAndStreamsRejectedWhileNotReady`: `on_link_state(kFaulted)` → `on_trajectory_goal` → `kReject`; `on_stream_open` refused with message `"arm not ready"`.
  - `SupervisorLink.LinkDownHaltsTheActiveGoal`: accepted goal running → `on_link_state(kConnecting)` → the backend sees the goal settled with `kHalted`, `"halted: arm fault or connection lost"`.
  - `SupervisorLink.LinkDownClosesTheStream`: open stream → `kFaulted` → `stream_is_open() == false`.
  - `SupervisorLink.ReconnectHoldsWhereTheArmIsNow`: run in impedance with a goal, link down, move `SimTransport`'s q, link back up (`kRunning`) and let the executor run → the active mode is position and its command equals the *new* q.
  - `SupervisorLink.NoStatePublishedWhileConnecting`: count `publish_state` calls on the fake backend over 100 ms in `kConnecting` → 0; in `kFaulted` → > 0.
- [ ] **Step 2: Run** `--gtest_filter='SupervisorLink*'`. Expected: FAIL.
- [ ] **Step 3: Implement.** `on_link_state(s)`: `s == kRunning` → `ready_ = true`; otherwise, if it was ready: `ready_ = false`, `active_mode_kind_ = kPosition`, `exec_.request_mode(&pos_)`, `on_halt(kArmUnavailable)`. Store `link_state_` (atomic) for the pump. The goal/stream checks read `ready_`.
- [ ] **Step 4: Run** the full suite. Expected: PASS.
- [ ] **Step 5: Commit.**

### Task 6: Driver docs + changelog

**Files:** `docs/reference/api.md` (ArmLink, LinkState, the new Transport methods, `exit_on_fault`, `ArmFault`, `ArmRejected`), `docs/integration-runbook.md` (fault → clear, arm off at boot, cable pull), `CHANGELOG.md` (Unreleased), `docs/superpowers/specs/2026-10-27-never-crash-design.md` status line, this plan's checkboxes.

- [ ] Write; `uv run --with mkdocs --with mkdocs-material mkdocs build --strict` passes; commit.

### Task 7: ROS node uses `ArmLink`

Repo: kinova-gen3-ros2, branch `feat/96-never-crash` off `origin/dev`.

**Files:**
- Modify: `kinova_gen3_ros2/src/bringup_node.cpp`, `kinova_gen3_ros2/src/ros2_backend.cpp` + its header (link-status source for diagnostics), `kinova_gen3_ros2/package.xml` + `CMakeLists.txt` (`std_srvs`)
- Test: `kinova_gen3_ros2/test/ros2_backend_test.cpp`

**Interfaces:**
- Consumes: `ArmLink`, `LinkStatus`, `to_string(LinkState)`, `Supervisor` as `LinkListener`.
- Produces: `void Ros2Backend::set_link_status_source(std::function<kinova::LinkStatus()>)`; service `/clear_faults` (`std_srvs/srv/Trigger`).

- [ ] **Step 1: Write failing tests**: `Ros2Backend` diagnostics with a status source returning each state: `kRunning` → OK; `kFaulted` → ERROR, message contains the reason; `kConnecting` → STALE, message `"waiting for the arm: <reason>"`; values `link_state`, `reason`, `faults`, `reconnects` present.
- [ ] **Step 2: Implement.** bringup: `exit_on_fault=true` in the executor config; `ArmLink link(tap, exec, &sup)`; `sup.start()` before `link.run(g_stop)`, inside `try { … } catch (const std::logic_error& e) { RCLCPP_FATAL; rc = 1; }`; drop the direct `connect`/`set_servoing_low_level`. `/clear_faults` on its own callback group: `request_clear()`; false → `success=false`, message `"not faulted (<state>)"`; true → wait ≤ 5 s polling `status()` every 50 ms for `state != kFaulted` or the reason to change to `"fault persists after clear"`; respond with the resulting state and reason.
- [ ] **Step 3: Build and test** with colcon against the driver branch (`CORE_REF=feat/96-never-crash` for the image build; locally, an overlay of the driver worktree). Expected: tests pass. If no local ROS build is possible, say so in the PR and rely on CI.
- [ ] **Step 4: Docs**: `docs/interface.md` (`/clear_faults`, the Arm diagnostics states), `docs/on-robot-runbook.md` + `docs/troubleshooting.mdx` (fault → controller → clear; arm off at boot), `CHANGELOG.md`.
- [ ] **Step 5: Commit**, open PRs in both repos (not merged).
