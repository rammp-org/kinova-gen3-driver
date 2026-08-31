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
| EE pose           | position     | yes — resolved by `JointPositionMode`'s in-loop IK |
| joint torque      | torque       | yes |
| joint velocity    | velocity     | yes — native `set_velocity_target`, pass-through-then-limit |
| EE twist          | velocity     | yes — resolved by `JointVelocityMode`'s damped-least-squares twist map |

Every pair above is now backed by a real control path. An unsupported
`(kind, control mode)` combination — e.g. joint velocity into impedance mode —
is still refused loudly at `on_stream_open`, with
`StreamOpenResult{accepted=false, error_code=result_code::kStreamRejected}` and
a message naming the unsupported pair; `pair_supported` in
[`interface/ports.h`](../reference/api.md#bool-pair_supportedsetpointkind-controlmodekind)
is the single source of truth for exactly which combinations those are.

`timeout_s <= 0` is also refused at open, independent of the pair — see below.

## `JointVelocityMode` specifics

Streaming into velocity mode inherits two things worth knowing before you rely
on it:

- **It is stiff and does not yield to contact.** `JointVelocityMode` commands
  every actuator in `kVelocity` and lets the actuator's own servo close the
  loop — there is no compliance term, and none is planned for this mode. Push
  on the arm while it tracks a stream and it will not spring back or soften; it
  keeps commanding the velocity you asked for. Want compliance, stream into an
  impedance mode instead.
- **A stale stream commands zero, not the last-known velocity.** Holding the
  last velocity while the stream is silent would keep the arm travelling
  toward nothing. So staleness (per the deadline mechanics
  [above](#the-deadline-one-value-two-enforcement-levels)) zeros the commanded
  velocity and **latches** — exactly like `JointImpedanceMode`'s freeze,
  disarming the watchdog cannot resurrect a target nobody is maintaining.

### The first twist setpoint can swing the elbow

`JointVelocityMode` resolves the redundant DOF with a **null-space posture
bias** toward `q_rest` — without it the elbow wanders wherever the twist stream
happens to drag it. Two consequences a client must plan for:

- **The bias is applied as a step, not a ramp.** Unlike `JointTorqueMode` and
  both impedance modes, this mode has no entry ramp. Open a twist session with
  the arm far from `q_rest` and send a *zero* twist, and the elbow starts moving
  immediately — motion the client did not ask for and is not commanding. If that
  matters, bring the arm near `q_rest` before opening the session, set
  `posture_gain = 0` and own the redundant DOF yourself, or expect the swing.
- **A large posture error slows task tracking.** The posture term and the task
  term are summed before `limit()`, and `limit()` scales the *sum* uniformly.
  A big posture correction therefore eats headroom that would otherwise go to
  the twist you asked for. The direction of the achieved twist is preserved —
  uniform scaling is what guarantees that — so it is not wrong, it is slow.

`posture_gain` defaults to `0.15` (matching `DiffIkParams`) rather than
something livelier for exactly this reason: against a posture error that can
reach ~π, a gain of 0.5 asks for ~1.6 rad/s of null-space velocity, over the
URDF cap, as an unramped step.

The EE-twist path additionally saturates by **scaling, not clamping**: when the
damped-least-squares solve would ask a joint to exceed its velocity cap, every
joint's commanded velocity is scaled down **uniformly** so the fastest joint
just reaches its limit, then a hard per-joint clamp backstops the rare edge
case (e.g. a zero entry in `max_qd`). A naive per-joint clamp would silently
rotate the commanded EE twist the moment any one joint saturates — the exact
thing a mode named "velocity" must not do to a twist target. Uniform scaling
keeps the achieved twist pointing the same direction as the commanded one,
just shorter.

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
  | `JointVelocityMode` | commands **zero** velocity, **latched** — holding the last velocity would keep the arm travelling toward nothing |

  So the arm is never left chasing a stale target for longer than one control
  cycle, even though the session that requested the stream may not tear down
  for up to `timeout_s` more.

Each mode also has its own default timeout (`JointTorqueParams::cmd_timeout_s`,
`JointPositionParams::cmd_timeout_s`, `JointImpedanceParams::cmd_timeout_s`,
`JointVelocityParams::cmd_timeout_s` — all `0.0`, i.e. disabled, except torque's
`0.1`) that applies whenever the mode is
*not* backing an open stream. `set_command_timeout(s)` with `s >= 0` arms the
watchdog with `s`; `s < 0` restores that default. `Supervisor::close_stream()`
calls it with a negative value on teardown for exactly this reason — closing a
stream must hand the mode back to its own supervision, not silently disable its
watchdog by writing zero over whatever the caller configured.

## When the pose path cannot solve

The deadline is not the only thing that ends a session. Streaming an **EE pose**
into `JointPositionMode` runs in-loop IK every cycle, and a pose the solver
cannot reach to tolerance for longer than `JointPositionParams::ik_fault_s`
(default `0.1 s`) is a fault. Two things then happen, at two different rates,
the same split the deadline uses:

- **The mode, at 1 kHz**, freezes the reference at the measured configuration
  immediately. Position mode is stiff: holding a stale reference while the
  client believes it is tracking is exactly the silent divergence this driver
  exists to fail loud on.
- **The sampler, on its next tick**, closes the session — with
  `StreamCloseCause::kIkFault`, distinct from `kDeadlineExpired`. That
  distinction is the point: a lapsed deadline means the client went quiet and
  should re-open; an IK fault means the driver could not solve for what was
  being asked, and re-opening the same session will reproduce it.

The latch is re-armed on teardown, so reconnecting works — but reconnecting
without changing the target does not. Check that the poses being streamed are
actually reachable. The most common cause is not a wild target but a **marginal**
one: a pose a fraction of a millimetre outside the workspace, typically produced
by taking the arm's current pose at or near full extension and offsetting it
outward. `ik_fault_s <= 0` disables the fault entirely, at the cost of the arm
silently sitting frozen while the client streams into the void.

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
