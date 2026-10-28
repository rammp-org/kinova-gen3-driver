# The driver never crashes — design

Status: implemented 2026-10-27 on `feat/96-never-crash` (sim-tested; on-arm validation pending). Tracks #96 (items 4–7), #59, and option 1 of #100.

## Why

The arm driver is the floor everything else stands on, and today it dies on
ordinary events. The node (`kinova_gen3_node` in kinova-gen3-ros2) exits on:

- **the arm faulting** (protective stop, e-stop pressed). The base drops out of
  low-level servoing and the next cyclic call or `SetControlMode` throws
  `KDetailedException(WRONG_SERVOING_MODE)` (#59);
- **something else forcing it out of low-level** (web app, another client), with
  the same exception;
- **the arm disconnecting**: #95 survives a lost frame but throws `CommsLost`
  after ~60 ms of silence, and nothing catches it;
- **the arm being off when the driver starts**: `connect()` throws once and the
  process exits (#100);
- **a startup RPC timing out** (`SetServoingMode` right after a previous run,
  #59 comment).

Nothing restarts the node. Every one of these ends with someone ssh-ing in. And
on the exits that reach `std::terminate`, `safe_shutdown()` never runs, so the
arm is not reverted either.

## Goals

1. No arm-side event terminates the process: fault, forced out of low-level,
   disconnect, arm off at startup, RPC timeout, any exception out of the RT loop.
2. **Fault → report → wait for an explicit clear.** The driver never clears an
   arm fault on its own (this includes the `ClearFaults()` that `connect()` does
   silently today).
3. **Arm not reachable → retry forever**, logging that it is waiting. Covers the
   arm being off at boot, a cable pull, and #100's ~4 s link renegotiation.
4. While the driver is not commanding, the arm is handed back to the base
   (actuators POSITION, single-level servoing), whenever the arm can still be
   reached.
5. Faults and link state are visible from ROS, and a fault can be cleared from
   ROS.

## Non-goals

- A dedicated Xbox-controller handover. The operator's existing routine — fault
  the arm, drive it with the controller, bring it back — is what this supports:
  while faulted the base owns the arm (so the controller works), `/joint_states`
  keeps publishing, and `/clear_faults` takes control back. A one-button
  handover without faulting the arm is out of scope (Swapnil: "definitely not a
  high priority").
- Auto-clearing faults, or resuming an interrupted goal after any recovery.
- Making the dev/benchmark apps under `apps/` survive faults. They are
  attended tools; they keep today's behaviour.
- #100 option 2 (waiting on a host-tuning marker).
- #96 item 7 (watchdogs counting time lost to misses). Passing an inflated `dt`
  would also step `JointVelocityMode`'s `q_ref` and the position rate limiter
  after a dropout, the step the executor skips `compute()` to avoid. Doing it
  right means a separate elapsed-time value on `ControlMode`, a wider change.
  The bound #96 states (~80 ms per window, Supervisor wall-clock deadline as
  backstop) stands. Deferred.
- A stop signal is not guaranteed prompt while `connect()` is blocked against an
  arm that is off. Nothing needs reverting in `kConnecting`, so a SIGKILL after
  docker's 10 s grace is harmless.

## Anti-goals (a failure even if nothing crashes)

- **The arm moves on its own after recovery.** After reconnect or clear, the arm
  must hold where it *is now*, never drive back to a target from before the
  outage. The operator may have moved it with the controller in between.
- The arm drops (zero torque into actuators left in TORQUE mode) during
  re-entry.
- Stale joint state published as if it were live while the arm is unreachable.
- Any allocation, lock or blocking call added to the steady-state RT cycle
  (`rt_safety_test` stays the gate).
- A fault that is swallowed: every exit from the commanding state is logged,
  counted and visible in diagnostics with its reason.

## Behaviour

Three link states:

| State | Meaning | Arm | `/joint_states` | Goals / streams |
|---|---|---|---|---|
| `kConnecting` | No session with the arm. Retrying `connect()` every 1 s, forever. | unreachable | not published | rejected |
| `kRunning` | Low-level servoing, RT loop commanding. | driver | live, 1 kHz source | accepted |
| `kFaulted` | Fault latched, waiting for an explicit clear. Arm handed back to the base. | base (controller / web app work) | live, ~100 Hz polled | rejected |

Transitions:

- **start** → `kConnecting`.
- `kConnecting` → connect succeeds → ask the base its state over TCP
  (`GetArmState`). `ARMSTATE_IN_FAULT` → `kFaulted`. Otherwise enter low-level, set every actuator to POSITION, then
  `kRunning`.
- `kRunning` → **comms loss** (`CommsLost`, or a timeout entering low-level) →
  best-effort revert + teardown → `kConnecting`. **No clear needed**: when the
  link comes back the driver re-enters low-level and holds where the arm is
  (my call — see Decisions).
- `kRunning` → **arm fault** (fault bits in feedback, base state `IN_FAULT`, or
  base no longer reporting `SERVOING_LOW_LEVEL`) or **arm rejected a command**
  (`KDetailedException`) or **any other exception** from the loop → revert to
  base control, keep the session → `kFaulted`, reason recorded.
- `kFaulted` → explicit clear request → `ClearFaults()` → `GetArmState` no
  longer `IN_FAULT` → enter low-level → `kRunning`. If the fault persists, stay `kFaulted`
  and say so.
- `kFaulted` → the `GetArmState` health poll (2 Hz) fails 3 times in a row →
  teardown → `kConnecting`,
  **fault stays latched**: on reconnect it goes back to `kFaulted`, not
  `kRunning`.
- Any state → stop signal → full `safe_shutdown()` and return.

The single exception that still exits: a robot whose actuator count doesn't
match this build (`kNumJoints`). That is a configuration error, not an arm
event; retrying forever would hide it. It stays a loud startup failure.

On every exit from `kRunning`, the Supervisor:

- settles the active and queued trajectory goals as halted ("halted: arm fault
  or connection lost");
- closes any open stream;
- stops stamping gripper commands;
- re-requests position mode, so the next `run()` adopts `JointPositionMode` and
  seeds it from fresh feedback. `on_enter` already drops every old target
  (`joint_position_mode.cpp`), which is what makes "holds where it is now" true.

## Design

### Core library (kinova-gen3-driver)

**New unit `ArmLink`** (`include/kinova_lowlevel/arm_link.h`). It owns the link
lifecycle above and runs on the caller's thread, the thread that today calls
`exec.run()`. It owns no thread of its own. It calls `RtExecutor::run()` while
`kRunning`, so the executor stays the only thread-aware unit, and it does
connect, backoff, revert and fault polling *outside* the RT loop. Interface,
roughly:

```cpp
enum class LinkState { kConnecting, kRunning, kFaulted };
struct LinkStatus { LinkState state; std::string reason; uint64_t faults, reconnects; };
class LinkListener { public: virtual void on_link_state(LinkState) = 0; };

class ArmLink {
 public:
  ArmLink(Transport&, RtExecutor&, LinkListener*, ArmLinkConfig = {});
  void run(std::atomic<bool>& stop);   // returns only on stop, transport shut down
  bool request_clear();                // any thread; false unless kFaulted
  LinkStatus status() const;           // any thread; mutex-guarded copy
};
```

`run()` throws only for a configuration error (`std::logic_error` family, i.e.
the actuator-count mismatch). Everything else is caught, classified, reported.

**Exception classification** (thrown by `KortexTransport`, caught by `ArmLink`):

| Thrown | From | Meaning | `ArmLink` does |
|---|---|---|---|
| `TransportTimeout` (exists) | cyclic call, and now a failed local `sendto` | lost frame | executor survives isolated ones (#95) |
| `CommsLost` (exists) | executor, after sustained misses | link down | → `kConnecting` |
| `ArmFault` (new, in `rt_executor.h`) | executor with `exit_on_fault`, fault in feedback on 3 consecutive good exchanges, or not low-level on 50 | arm faulted / left low-level | → `kFaulted` |
| `ArmRejected` (new, in `transport.h`) | `KortexTransport`, wrapping `KDetailedException` | arm refused a command | → `kFaulted` |
| `std::logic_error` | `connect()` actuator-count mismatch (changed from `runtime_error` to `std::invalid_argument`) | wrong robot | rethrown, process exits |
| anything else | anywhere | unknown | → `kFaulted`, reason = `what()` |

`KDetailedException` is caught *inside* `KortexTransport`, so no KORTEX type
crosses the boundary. A failed `sendto` (`std::runtime_error` from
`TransportClientUdp::send`) becomes a `TransportTimeout` at the cyclic-call
site, since on a congested NIC it is a lost frame (#96).

**Fault detection in the RT loop.** `JointFeedback` gains one POD field,
`bool low_level = true`. `KortexTransport::fill_feedback` sets it from
`base().active_state() == ARMSTATE_SERVOING_LOW_LEVEL`, and folds the base's
own fault banks and `ARMSTATE_IN_FAULT` into `fault` (today only actuator
banks). The default is `true`, so `SimTransport` and every test fake stay
"healthy" untouched. With the new `ExecutorConfig::exit_on_fault` (default
**false**, so the attended apps keep today's behaviour: teleop's clear packet
still clears and resumes in place), the executor throws `ArmFault` after
`fault` on 3 consecutive good exchanges, or `!low_level` on 50 (the base's
reported state may trail the switch into low-level by a few frames). `ArmLink`'s constructor
throws `std::invalid_argument` if handed an executor without it, so the ROS node
can't forget it (3 ms at 1 kHz: a single garbled frame doesn't
take the arm down; a latched fault does). Counting is a couple of integer ops,
with no allocation until the throw, which ends the loop anyway.

**`Transport::arm_in_fault()`** (new pure virtual): asks the base over TCP
(`GetArmState`, 500 ms bound) whether it is `ARMSTATE_IN_FAULT`. TCP RPCs answer
in every servoing mode, so the fault check at connect, the clear confirmation and
the faulted-state health poll don't depend on UDP cyclic feedback outside
low-level (unverified). `SimTransport` answers from its injected fault.

**`Transport::release_low_level()`** (new pure virtual). Reverts actuators to
POSITION and the base to single-level servoing, keeps the session. No-op if not
in low-level. `KortexTransport::safe_shutdown()` becomes `release_low_level()` +
teardown. Every RPC in the revert is bounded at 500 ms (KORTEX defaults to
3000 ms, which makes a dead-TCP revert take ~24 s), and the revert stops at the
first timeout. A dead link doesn't answer the next six either. Decorators
(`FeedbackTap`, `GripperController`) forward it. `SimTransport` implements it.

**`KortexTransport::connect()`** stops calling `ClearFaults()`. Clearing is now
explicit only. **`set_servoing_low_level()`** ends by setting all 7 actuators to
POSITION (pumped, via `set_actuator_modes`). After a cable pull the actuators
may still be in TORQUE mode, and the first commands of a new run carry zero
torque. On the first start this is redundant but harmless.

**`clear_faults()`** is unchanged. It re-enters low-level only if the transport
is still in it, which is what teleop relies on. `ArmLink` always calls it after
`release_low_level()`, so for `ArmLink` it is just the `ClearFaults()` RPC, and
`ArmLink` does the re-entry itself.

**Gripper on re-entry.** `set_servoing_low_level()` clears the interconnect
submessage from the retained command (`cmd_`), so the first frames after a
reconnect or clear leave the gripper **uncommanded**: the same state as first
startup. The Supervisor's halt already stops `GripperController` stamping. Without
this, the retransmitted submessage would drive the gripper back to its pre-outage
target, the anti-goal.

**Faulted-state feedback.** In `kFaulted`, `ArmLink` polls `receive()`
(`RefreshFeedback`) at 100 Hz so `/joint_states` stays live. Its failures only
cost joint state, never the state machine: liveness comes from the
`GetArmState` poll.

**Supervisor** implements `LinkListener`:

- leaving `kRunning`: store `active_mode_kind_ = kPosition`,
  `exec_.request_mode(&pos_)`, then `on_halt(HaltReason::kArmUnavailable)`
  (new enum value), and mark the arm not ready;
- entering `kRunning`: mark ready;
- `on_trajectory_goal` rejects and `on_stream_open` refuses ("arm not ready")
  while not ready;
- the pump publishes `ArmState` only when the feedback snapshot is new since its
  last publish, so nothing goes out while `kConnecting`, and in `kFaulted` only
  while `ArmLink`'s polling gets answers (implemented with `Seqlock::version()`;
  originally specced as a `kConnecting` check, tightened after review);
- gripper setpoints are dropped while not ready, and a goal accepted after the
  drop is settled halted (review findings).

`SimTransport` gains test knobs: fail the next N `connect()`s, inject a fault
into feedback, throw `ArmRejected` / `TransportTimeout` from `exchange()`. Tests
drive every row of the state table above through `ArmLink` +
`RtExecutor` + `SimTransport`, and the Supervisor's halt/not-ready behaviour
through its existing fixture.

### ROS wrapper (kinova-gen3-ros2)

- `bringup_node.cpp`: replace `tap.connect(); tap.set_servoing_low_level(); …
  exec.run(g_stop)` with `ArmLink link(tap, exec, &sup); link.run(g_stop)`.
  `sup.start()` moves before it (the Supervisor runs fine with the arm not
  ready). A `try/catch` around `link.run()` logs a configuration error as
  FATAL and exits non-zero.
- **`/clear_faults`** (`std_srvs/srv/Trigger`): calls `link.request_clear()`,
  waits up to 5 s for the state to leave `kFaulted`, and answers with success
  and the resulting state/reason. Own callback group so it never blocks state
  publishing. Not gated by the arbitration token (my call).
- **Diagnostics** ("Arm" entry, already on `/diagnostics`): `kRunning` → OK,
  `kFaulted` → ERROR with the reason, `kConnecting` → STALE "waiting for the
  arm at <ip>" with the reason of the last failed attempt. Values: `link_state`,
  `reason`, `faults`, `reconnects`.
- Docs: the on-robot runbook and troubleshooting pages get the fault/clear and
  arm-off workflow; the interface page lists `/clear_faults`.
- Pin: the ROS PR builds against the driver branch via `CORE_REF` until a
  driver release exists; repinning `kinova_gen3.repos` is part of the release,
  not this change.

### rammp-deployments

No change required. Add `/clear_faults` to the sheppy manifest's service list
for the arm node, if the manifest enumerates services; it enumerates topics
today.

## Decisions

Swapnil's:

- Faults are reported and wait for an explicit clear.
- An unreachable arm is retried forever.
- The Xbox handover is low priority. Supporting the fault-then-clear routine is
  enough.
- Changes go into every repo that needs them.

My calls (flagged to Swapnil 2026-10-27, to confirm):

1. A reconnect after comms loss resumes without a clear (holds where the arm
   is). Only a fault needs a clear.
2. `connect()`'s silent `ClearFaults()` is removed. An arm already faulted at
   startup waits for a clear.
3. `/clear_faults` is `std_srvs/Trigger`, not gated by arbitration. The fault is
   reported through `/diagnostics`. Standard types only, so
   rammp-interfaces-ros2 is untouched.
4. Unknown exceptions become a fault (wait for clear), not an exit. Only the
   actuator-count mismatch exits.
5. A fault requires 3 consecutive faulty frames; leaving low-level, 50.
   Leaving low-level counts as a fault.
7. Fault-exit is opt-in on the executor (`exit_on_fault`); the apps keep
   today's behaviour, `ArmLink` requires it.
8. Re-entry leaves the gripper uncommanded, as at startup.
9. #96 item 7 (dt catch-up) deferred: done right it widens `ControlMode`.
10. No bounded reachability probe before `connect()`; a slow stop while the
    arm is off is accepted.
6. Connect retry is a fixed 1 s, not exponential: the arm is on a dedicated
   link, and a short fixed period gets back fastest after a 4 s renegotiation.

## Implementation notes (2026-10-27)

- Setting every actuator to POSITION on re-entry lives in `ArmLink` (after
  `set_servoing_low_level()`), not inside `set_servoing_low_level()`: the teleop
  app's in-place `clear_faults()` re-enters through it while its loop keeps
  commanding torque, and must keep doing so.
- `KortexTransport`'s TCP-RPC classification (`ArmRejected` / `TransportTimeout`)
  has no unit test: there is no fake TCP router. The cyclic-call
  classification is tested through the SDK's real `RouterClient`.

## Not verified (needs the arm)

- That `RefreshFeedback` answers in single-level servoing and while the base is
  in fault. If it doesn't, `/joint_states` goes quiet in `kFaulted`; the state
  machine and the clear don't depend on it.
- What the base reports in `active_state` during normal low-level running
  (assumed `ARMSTATE_SERVOING_LOW_LEVEL`), and during a protective stop. If the
  assumption is wrong, every start ends in `kFaulted` 50 ms in, with the reason
  naming the reported state: loud on the first attended run, and the check is
  one line to drop.
- What the 2F-85 does when left uncommanded after a re-entry (expected: the
  same as at first startup).
- Whether the arm's own watchdog leaves low-level or faults when cyclic commands
  stop, and in which actuator mode it leaves the actuators.
- That `ClearFaults()` + `SetServoingMode(LOW_LEVEL)` re-enters cleanly after an
  e-stop release.
- How long `TransportClientTcp::connect` blocks against an arm that is off.

Attended validation (with Swapnil): arm off at start then powered on; cable pull
while holding; e-stop → controller drive → `/clear_faults`; web app forcing
single-level while the driver runs; `rt_safety_test` and a 1 kHz benchmark
before/after on abra.
