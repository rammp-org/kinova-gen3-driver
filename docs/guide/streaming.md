# Streaming — low-latency setpoints

The command tier (`CommandSink`) is built for trajectories: plan a path, hand it
to the executor, let it run to completion. Teleop and reactive control need the
opposite shape — a setpoint arriving every cycle from an external source, with
no plan to execute and no "done" to reach. That is the streaming tier,
`StreamSink`.

This layer lives in `kinova::interface::StreamingSession`, driven through
`Supervisor`. It knows nothing about IK, gravity, or gains — it is pure
lifecycle logic over injected time, which is what makes it testable with no
robot and no clock.

## Opening a session

A stream is not implicit in sending setpoints — it is a resource you open and
must close, the same shape as arbitration's grant/revoke:

```cpp
StreamOpenRequest r;
r.kind         = SetpointKind::kJointPosition;
r.control_mode = ControlModeKind::kImpedance;
r.timeout_s    = 0.1;
StreamOpenResult res = stream_sink.on_stream_open(r);
```

Opening is explicit for three reasons:

- **The (setpoint kind, control mode) pair is fixed for the session's
  lifetime.** The driver switches into the requested control mode *before*
  admitting the first setpoint, so no setpoint can land mid-switch. Changing
  what you're streaming means closing and reopening.
- **The deadline is mandatory, not per-message.** One `timeout_s` governs the
  whole session — see [below](#the-deadline-one-value-two-enforcement-levels).
- **It reserves the arm.** Only one session may be open at a time, and opening
  requires ownership just like a trajectory goal — see
  [Arbitration](arbitration.md).

## The valid-pair table, as it stands today

Not every (setpoint kind, control mode) combination is executable. `pair_supported`
is the single source of truth, checked at open — nothing is silently degraded
into a mode that can't drive it:

| Setpoint kind    | Control mode | Supported? |
|---|---|---|
| joint position    | position     | yes |
| joint position    | impedance    | yes |
| EE pose           | impedance    | yes — resolved by `JointImpedanceMode`'s in-loop IK |
| EE pose           | position     | **not yet** — position mode has no IK path |
| joint torque      | torque       | yes |
| joint velocity    | any          | **not yet** — no `JointVelocityMode` |
| EE twist          | any          | **not yet** — no `JointVelocityMode` |

The three "not yet" rows are refused loudly at `on_stream_open` — that is the
table doing its job, not a bug or an oversight. Both are a separate follow-on
plan: `JointVelocityMode` unlocks the velocity/twist rows, and a position-mode
IK path unlocks EE-pose-into-position. Until then, requesting one of those
pairs gets `StreamOpenResult{accepted=false, error_code=result_code::kStreamRejected}`
with a message naming the unsupported pair.

`timeout_s <= 0` is also refused at open, independent of the pair — see below.

## A setpoint is a command, not an increment

Every setpoint (`JointSetpoint`, `PoseSetpoint`, `TwistSetpoint`) carries an
**absolute** target, never a delta. Two consequences follow directly from that:

- **Re-sending the same setpoint is a no-op**, not a double-application. A
  client that isn't sure whether its last message landed can just send it
  again.
- **Dropping an intermediate setpoint is correct, not lossy.** The setpoint
  publish path is a single-writer double-buffer, latest-wins: if setpoints 41
  and 42 both arrive before the RT thread's next cycle, only 42 is ever read.
  There is nothing to reconcile — 42 already says where the arm should be,
  independent of 41 ever having existed.

This is why the streaming tier can stay allocation-free and lock-free end to
end: there is no queue to buffer, because there is nothing an unqueued message
would have contributed.

## The deadline: one value, two enforcement levels

`StreamOpenRequest::timeout_s` is a single number, but two independent things
watch it, at two different rates, and each owns a different job:

- **`StreamingSession` (sampler thread, ~250 Hz)** owns the stream's
  *lifecycle*. Once `expired()` is true, `Supervisor::close_stream()` tears the
  session down: it marks the session closed (so no further setpoint is
  admitted), latches the arm at its last-good **measured** q, and hands each
  mode's watchdog back to its own configured default. This runs at sampler
  rate because closing a session is not a per-cycle concern.
- **`CommandWatchdog` (RT thread, 1 kHz)**, armed by `set_command_timeout` with
  the same `timeout_s`, owns making the *output* safe every single cycle in
  between. It is pure staleness **detection** — a lock-free counter bumped by
  every setter and ticked once per RT cycle, no clock call, no allocation. The
  **response** to staleness belongs to the mode, not the watchdog:

  | Mode | On staleness |
  |---|---|
  | `JointTorqueMode` | ramps `tau_ff` to zero over `cmd_decay_s`, reverting to gravity-compensation hold |
  | `JointPositionMode` | freezes the reference at measured q |
  | `JointImpedanceMode` | freezes the reference at measured q, **latched** until a fresh command arrives (disarming the watchdog cannot resurrect a target nobody is maintaining) |

  So the arm is never left chasing a stale target for longer than one control
  cycle, even though the session that requested the stream may not tear down
  for up to `timeout_s` more.

Each mode also has its own default timeout (`JointTorqueParams::cmd_timeout_s`,
`JointPositionParams::cmd_timeout_s`, `JointImpedanceParams::cmd_timeout_s` — all
`0.0`, i.e. disabled, except torque's `0.1`) that applies whenever the mode is
*not* backing an open stream. `set_command_timeout(s)` with `s >= 0` arms the
watchdog with `s`; `s < 0` restores that default. `Supervisor::close_stream()`
calls it with a negative value on teardown for exactly this reason — closing a
stream must hand the mode back to its own supervision, not silently disable its
watchdog by writing zero over whatever the caller configured.

## Mutual exclusion with trajectory goals

A stream and a trajectory goal can never run at once, and each side refuses
the other loudly rather than queuing or interleaving. It takes **three** checks,
not two, because the two accept-time ones are each one-sided:

- `on_stream_open` rejects with `kStreamRejected` if a trajectory goal is
  currently in flight.
- `on_trajectory_goal` rejects if a stream is currently open.
- The sampler re-checks when it **drains** an accepted goal, and settles it
  `INVALID_GOAL` if a stream has opened in the meantime.

The third check closes a real window. A goal becomes "in flight" when the
sampler drains the inbox, not when the backend accepts it, so an accepted goal
sits queued for up to one sampler period (4 ms at the default 250 Hz) while
`on_stream_open` still sees no goal in flight and admits the session. Without
the drain-time re-check the sampler would then rebind the trajectory executor
and tick that goal into the very sink the backend thread is streaming into —
two writers on one double buffer, which is precisely what the exclusivity
invariant exists to rule out (setpoints are written **directly** from the
backend thread, and that is only sound because the sampler writes no targets
while a session is open).

The practical consequence for a client: opening a stream immediately after
submitting a goal is not a way to preempt it. The goal is refused, not
interrupted mid-motion, and the client sees `INVALID_GOAL` with
`"a streaming session opened before this goal could start"`. To hand over
deliberately, cancel the goal (or let it finish) and then open the stream.

The reasoning is the same in both directions: a trajectory goal and a stream
both want to be the one thing writing the active mode's target every cycle,
and the executor has no way to arbitrate between "keep interpolating toward
the next waypoint" and "the last setpoint I got said something else." Rather
than defining an interleaving, the driver picks one owner at a time and makes
the other wait for it to finish or close.

## See also

- [Arbitration](arbitration.md) — the token that gates who may open a stream
  at all; `Arbiter` decorates `StreamSink` the same way it decorates
  `CommandSink`.
- [Control Modes](control-modes.md) — what each mode does with a target once
  admitted.
- [API Reference](../reference/api.md#streaming-interfacestreaming_sessionh-interfaceportsh) —
  the port and value types themselves.
