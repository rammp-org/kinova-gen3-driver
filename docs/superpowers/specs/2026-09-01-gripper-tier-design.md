# Gripper Tier — Design Spec

## Goal

Give the 2F-85 the same treatment the arm has: commands that carry everything the
hardware accepts, state that reports everything the hardware sends, and a place in
the ownership and halt machinery. Today the gripper is a single `float` stamped into
`JointCommand` by an app-local decorator, with speed and force pinned to constants.

## Dependencies

- `2026-08-25-arm-arbitration-design.md` — the token this tier reuses.
- `2026-08-26-streaming-setpoints-design.md` — the stateless-setpoint rule this tier follows.
- `kinova_arm_ros2`'s `2026-08-31-kinova-arm-description-design.md` — publishes gripper
  joint state into `/joint_states`, and is **blocked on this spec** for velocity. See
  the correction below: it is not blocked on force, because force feedback does not exist.

## Out of scope

- **The EE wrench on `/ee_state`.** Different feedback path, different blocker (KORTEX's
  tool-frame offset). Its own spec.
- **Calibrating grip force to Newtons.** This spec publishes a normalized effort and the
  raw current it derives from; the calibration is separate work that needs a scale and a
  fixture.
- **The ROS2 surface itself.** Sketched at the end so the downstream repo knows what to
  build against; `kinova_arm_ros2` writes its own spec, as it did for the streaming tier.
- **Multi-gripper support.** One interconnect gripper, motor 0. The Gen3 has one.

## The hardware, established from the SDK

Read out of `kortex_api_2.8.0_aarch64` rather than assumed, because two widely-held
beliefs about this gripper turn out to be wrong.

**There are two control paths and only one is available to us.** The high-level
`Base::SendGripperCommand` requires `SINGLE_LEVEL_SERVOING`, which is incompatible with
the `LOW_LEVEL_SERVOING` our 1 kHz torque control needs. It is closed to us regardless of
merit. The low-level cyclic path — the gripper riding inside the same `BaseCyclic` frame
at `interconnect.gripper_command.motor_cmd(0)` — is what this driver already uses.

**`GripperMode` has no force mode.** The entire enum is:

```
UNSPECIFIED_GRIPPER_MODE = 0,
GRIPPER_SPEED            = 2,
GRIPPER_POSITION         = 3,
```

`GRIPPER_FORCE` does not appear anywhere in the SDK include tree. (Value `1` is missing,
so it plausibly existed once.) No force *servo* exists on this hardware by any path.

**The cyclic messages are asymmetric:**

| direction | fields |
|---|---|
| `MotorCommand` | `position`, `velocity`, `force`, `motor_id` |
| `MotorFeedback` | `position`, `velocity`, `current_motor`, `voltage`, `temperature_motor`, `motor_id` |

All command fields are percent, 0..100.

> **Correction to `2026-08-31-kinova-arm-description-design.md`.** That spec states
> *"`GripperCyclicMessage`'s motor feedback already carries `velocity()` and `force()` on
> the wire, and core reads only `position()`"*, and lists filling in gripper velocity and
> effort as a small future core change. **Velocity is correct; force is not.**
> `MotorFeedback` has no `force` field. Its plan to replace the NaN effort in
> `/joint_states` cannot be carried out as written — see "Gripper state" below for what
> replaces it.

**`force` is a ceiling, not a setpoint.** It limits motor current. The gripper closes at
`velocity` toward `position` and stalls when it reaches the limit. "Grasp with 15 N" is
not expressible; "close, but do not push harder than X" is.

**The linkage is underactuated.** One motor, a four-bar, five mimic joints. Free-space
closing keeps the fingertips parallel; on contact the linkage deviates so it can wrap.
Two consequences already recorded downstream and restated here because they bound what
this tier can promise: TF through the fingertips is not trustworthy *during* a grasp, and
percent is not a calibrated aperture — the four-bar makes aperture-vs-angle non-linear,
so 50% is not 42.5 mm.

## Approved decisions

### 1. The gripper rides the arm's token

No separate ownership. Whoever holds the arm holds the gripper.

**Why.** The alternative — an independently checked-out gripper — doubles the arbitration
surface and immediately raises questions with no good answers: what happens when the arm
is revoked but the gripper is not, and who is responsible for an object held by a gripper
whose arm now belongs to someone else. One physical machine, one holder.

**Consequence.** `Arbiter` decorates `GripperSink` exactly as it decorates `CommandSink`
and `StreamSink`, checking the same token against the same generation. No new arbitration
code, one more decorated port.

### 2. Commands carry position, speed and force, and are stateless

Every command carries all three. Speed and force have defaults; a caller that only cares
about position sets one field.

```
position   0..1   0 = open, 1 = closed     (required)
speed      0..1   fraction of max          (default 1.0)
force      0..1   fraction of max, a CAP   (default 0.5)
```

The defaults are today's hardcoded constants (`kGripperVelocityPct = 100`,
`kGripperForcePct = 50`), so nothing about existing behaviour changes for a caller that
ignores the new fields.

**Stateless, not sticky.** Speed and force do not persist between commands. This follows
the streaming tier's existing rule — *"a setpoint is a command and never an increment, so
re-sending is idempotent and dropping intermediate ones is correct."* Sticky force would
make the gripper the one place in the driver where a dropped message changes the meaning
of the next one, and turns "why is it gripping softly?" into archaeology. The cost is that
a force-varying client re-sends three floats instead of one.

A caller that sets `force` once and then streams position-only commands gets the default
back. That is the intended, documented behaviour.

### 3. Force feedback is normalized effort plus raw current, never Newtons

`MotorFeedback` has no force. It has `current_motor`, which is proportional to motor
torque and therefore to grip force, but is uncalibrated.

**Report `effort` as a fraction of maximum, derived from current, and publish
`current_motor` in amps alongside it.**

**Why not Newtons.** A number labelled Newtons that is wrong by an unknown factor is worse
than no number, because it propagates silently into whatever a policy learns from it. This
is the same judgement `kinova_arm_ros2`'s `EeState.msg` already makes about the tool
wrench: *"publishing a zero or a frame-mismatched value would be worse than publishing
nothing."*

**Why normalized rather than raw-only.** Commanding force as a fraction and reading it
back as a fraction makes the two directly comparable — *commanded 0.50, measured 0.47* —
without inventing a unit. Publishing the raw current too means nothing is lost, and a
future calibration can be fitted from logged data rather than requiring a fresh
experiment.

`effort_max_current_a` is the normalizing constant, a parameter with a documented
provenance rather than a magic number. Until measured it is set from the gripper's rated
stall current and marked as such.

### 4. A topic for streaming. ~~An action for grasping.~~ (CUT)

**Topic** — a setpoint, fire-and-forget, no completion. This is all a policy running at
100 Hz needs, and it mirrors the streaming tier.

**Action — `Grasp`** — close until the fingers either reach the target or stall on an
object, and report which happened. This is what a task orchestrator wants ("close on the
cup, tell me when you have it") and it means stall detection is written once, in the
driver, instead of in every caller.

**Why not action-only.** A 100 Hz streaming client cannot use an action. The arm already
learned this, which is why its streaming tier exists alongside its action tier.

**CUT, 2026-09-01.** The `Grasp` action and its stall detection are dropped from scope.
The action's whole justification was writing stall detection once in the driver rather
than in every caller; without it, `Grasp` is `MoveTo` plus a completion callback, which
the topic already provides. The gripper tier is therefore **topic-only**, and decision 4
reduces to its first half.

The effort-threshold scheme originally specified here would not have worked anyway, and
the hardware run is what showed it: a sustained grasp reports effort of about **0.05**,
down in the noise, while the *transient* squeeze hits 1.0. A detector keyed on "effort
exceeds a floor" would have fired during the close and gone quiet exactly when the object
was held.

**If a grasp primitive is wanted later, the measurement points at a much simpler one.**
Position alone separates the cases cleanly: commanded fully closed, the gripper settled at
**0.8333** on an object and **0.9912** on empty air. "Stopped short of the commanded
target" needs no effort threshold, no tuned floor, and no second condition. Whoever picks
this up should start there rather than from what this section originally proposed.

### 5. On halt, the gripper holds

When the arm halts — e-stop, revoked ownership, operator request — the gripper **stops
being commanded** and keeps whatever grip it has.

**Why.** E-stop means stop moving, and opening is itself a motion. Anything held stays
held rather than being dropped from wherever the arm happened to be, which for a loaded
utensil or a heavy part is its own hazard. The 2F-85 is effectively self-locking, so
ceasing to command it *is* holding it — this decision costs nothing to implement and adds
no motion to the halt path.

**The accepted cost, stated plainly:** if the gripper is closed on something it should not
be, e-stop will not release it. Releasing requires a deliberate open command after the
halt clears. Anyone standing up this system needs to know that, so it goes in the guide,
not just here.

**Consequence for `on_halt`.** The gripper's halt handling is a deliberate no-op in the
command path. `GripperController` stops stamping; nothing else happens. The reason this is
worth a decision rather than an omission is that a future reader will otherwise "fix" it.

## Components

### `GripperController` — a `Transport` decorator

Promotes `teleop_socket_server.cpp`'s app-local `GripperInjector` into the library.

Decorates `Transport`, stamping `JointCommand`'s gripper fields on the way past. Non-RT
setters publish through a single-writer double-buffer with a release store; the RT thread
reads one snapshot per cycle, exactly as the modes do.

```cpp
class GripperController : public Transport {
 public:
  explicit GripperController(Transport& inner);
  // Non-RT, one writer. Latest wins; every call carries all three fields.
  void set_target(const GripperCommand& c) noexcept;
  // Non-RT, halt path: stop stamping. The hardware self-locks, so the grip holds.
  void release() noexcept;
  // ... Transport passthrough ...
};
```

**Why a decorator and not a `ControlMode`.** Modes are mutually exclusive; making the
gripper a mode would mean giving up arm control to move the gripper. The gripper is not a
control law — it has no feedback term and no RT computation. It is a field on the outgoing
frame, which is exactly what a `Transport` decorator is for, and it is how this repo
already handles cross-cutting concerns (`FeedbackTap` decorates `Transport`, `Arbiter`
decorates `CommandSink`).

**Why not in `RtExecutor`.** That unit's job is "owns the RT thread and knows nothing
else." Every gripper feature would grow it.

**Consequence — the gripper is orthogonal to control modes.** You can grip during a
trajectory, during an impedance hold, during a velocity stream. Nothing about the gripper
touches mode switching, and the RT-safe surface does not widen.

**The lazy-allocation note carries over.** `KortexTransport` allocates the interconnect
submessage on the first commanded cycle — one heap allocation on the RT path, deliberately
accepted so the gripper stays limp at startup rather than being actuated by a seeded
default. That tradeoff is unchanged here and is still pending hardware validation.

### `JointFeedback::gripper` widens

From a bare `float` to a fixed-size POD, filled by `KortexTransport` from `MotorFeedback`:

```cpp
struct GripperFeedback {
  float position = 0.0f;    // 0 (open) .. 1 (closed)
  float velocity = 0.0f;    // normalized; sign and scale TO CONFIRM — see below
  float effort   = 0.0f;    // 0..1, clamp(|current| / effort_max_current_a, 0, 1)
  float current  = 0.0f;    // amps, raw, as reported
  bool  present  = false;   // false when no interconnect gripper is attached
};
```

`present` replaces the current silent behaviour of leaving position at 0 when no gripper
is attached — which is indistinguishable from an attached, fully-open gripper. Fail loud,
never silent mis-mapping.

### `GripperSink` — a driving port

Peer to `CommandSink` and `StreamSink` in `interface/ports.h`.

```cpp
class GripperSink {
 public:
  virtual ~GripperSink() = default;
  virtual void         on_gripper_command(const GripperCommand&) = 0;
  virtual GraspResponse on_grasp_goal(const GraspGoal&)          = 0;
  virtual void         on_grasp_cancel(const CancelRequest&)     = 0;
  virtual GripperState on_query_gripper() const                  = 0;
};
```

`Supervisor` implements it, writes through `GripperController`, and runs the grasp
lifecycle in its sampler. `Arbiter` decorates it with the same token check it already
applies to the other two sinks.

## RT-safety

The gripper adds nothing to the RT path except a fixed-size struct copy in the decorator's
`exchange`. No allocation, no lock, no blocking call. `rt_safety_test` gains a case that
runs a mode with the gripper decorator in the chain and a live command, asserting zero
major page faults and zero dropped samples — the same shape as the existing mode cases.

Stall detection runs in the sampler thread at 250 Hz and touches no RT state; it reads the
same `Seqlock<JointFeedback>` snapshot the pump already reads.

## Testing

Everything here is testable without a robot. `SimTransport` gains gripper feedback that
echoes the commanded position through a first-order lag, so stall detection and the grasp
lifecycle can be tested deterministically by making the echo stop short of the target.

The tests that matter:

- **The decorator stamps what was set**, and stamps nothing before the first command.
- **Defaults land**: a command with only position set produces speed 1.0 and force 0.5 on
  the wire.
- **Statelessness**: setting force, then sending a position-only command, produces the
  default force — the behaviour decision 2 chose, pinned so it cannot drift.
- **`present` is false** with no gripper attached, and position 0 is not mistaken for open.
- **Halt stops stamping** and does not open the gripper.
- **Grasp completes `OBJECT_GRASPED`** when the sim stalls short of the target, and
  `REACHED_TARGET` when it does not — the distinction the action exists to make.
- **Grasp cancel** mid-close settles the goal and leaves the gripper where it is.
- **Arbitration**: a gripper command with a stale token is refused and counted.

## Measured on the arm (2026-09-01)

`gripper_check` ran an open → close → open cycle at `force = 1.0`, `speed = 0.5`, both on
empty air and closing on a compliant object. Three results, two of which overturn
assumptions above.

- **`kGripperMaxCurrentA` is 1.0 A, not the datasheet's 0.8.** Peak grip current measured
  1.00 A. At 0.8 the two highest samples computed 1.20 and 1.25 and were **clamped to
  exactly 1.0000** — effort was silently saturating during an ordinary grasp. Updated.
- **A grasp spikes then settles.** Current climbs to the peak as the fingers close on the
  object, then drops to roughly **0.05 A** and holds there. A sustained grasp reports a
  *small* effort, not a large one. Any stall detection that keys off "high effort means
  holding something" is wrong; it must key off position-stopped plus the transient.
  Closing on **empty air** is different again: the gripper reaches its commanded position
  and stops driving entirely, reporting **zero** current.
- **`MotorFeedback::velocity` is not a measurement, so `GripperFeedback` no longer
  carries a velocity field at all — it was REMOVED rather than documented.** It is the commanded speed echoed
  back while the gripper considers itself moving, and 0 otherwise — and it is unsigned.
  While the fingers were being physically stopped, the position increments shrank while
  this field held exactly the commanded 0.5000. Across runs, `|velocity| / |dpos/dt|` was
  0.625 on air versus 1.060 on the object at the same commanded speed; a real measurement
  would give one constant.

  A field carrying no information the caller does not already have is worse than no
  field: it looks like a measurement, so eventually someone publishes it as one. The
  struct now documents its own absence so nobody helpfully adds it back.

  **This retires the plan to fill in `/joint_states` velocity from it.** Publishing a
  setpoint echo into a measurement field is the mistake decision 3 refuses to make for
  force. `kinova_arm_ros2`'s description spec should keep gripper velocity as NaN, or
  publish the derivative of position and say so. Flagged there as well.

## Open questions — tune on hardware

Named here rather than buried as constants, because all three need the arm to settle:

- ~~**`stall_pos_eps` / `stall_hold_s` / `stall_effort_min`.**~~ MOOT — stall detection is
  cut (decision 4). Nothing tunes these because nothing reads them.
- ~~**`effort_max_current_a`.**~~ MEASURED — see above. 1.0 A. Re-measure if grasps clamp again.
- **Whether commanded and measured force are actually the same scale.** "Directly
  comparable — commanded 0.50, measured 0.47" (see decision 3) holds only if
  `effort_max_current_a` equals the motor current the hardware produces at
  `force = 100%`. Those are two independently-guessed maxima today — `SimTransport`
  hard-codes the identity (`effort = force` when blocked), so sim will agree with this
  spec by construction, but hardware may not. Same bench session as
  `effort_max_current_a` above: command a range of `force` values against a fixed
  stall and log the resulting current to see whether the two scales actually match.
- ~~**The units and sign of `MotorFeedback::velocity`.**~~ ANSWERED — see above. Unsigned,
  and an echo of the command rather than a measurement. The field is gone. Original note: Presumed percent-of-max and signed
  by direction, by symmetry with the command field — but unlike the rest of this spec, that
  is inference from the field name, not something read off the SDK, which carries no units
  in the generated header. One `--csv` capture of an open-close cycle settles it. If it
  turns out unsigned, `GripperFeedback::velocity` carries magnitude only and says so.
- **Whether the first-cycle allocation should be pre-seeded.** Inherited, still open, still
  wants an attended bring-up.

## What the ROS2 surface will need

Sketched so `kinova_arm_ros2` can plan against it; that repo writes its own spec.

- `GripperCommand.msg` — `position`, `speed 1.0`, `force 0.5`, `token`. ROS 2 `.msg`
  supports field defaults, so the three-field-or-one-field ergonomics survive the IDL.
- `GripperState.msg` — position, velocity, effort, current, present, stamp.
- `Grasp.action` — goal (target position, speed, force), result (outcome enum, final
  position and effort), feedback (current position and effort).
- **`/joint_states` gets position and velocity**, both now honest. **Effort stays NaN**:
  `sensor_msgs/JointState.effort` is documented as N·m or N, and putting a 0..1 fraction
  there is precisely the mis-labelling decision 3 rejects. The normalized effort lives on
  `/gripper_state`, where its units can be stated.
- The controller registry gains gripper rows the same way it gained velocity rows, and
  `available()` keeps asking core rather than declaring.

## Decomposition

1. **Core plumbing** — `GripperFeedback`, `KortexTransport` reading all four fields,
   `GripperController` promoted into the library, `SimTransport` gripper echo. Testable
   and useful on its own: it fixes the `present` mis-mapping and unblocks `/joint_states`
   velocity immediately.
2. **The interface tier** — `GripperSink`, `Supervisor` implementation, `Arbiter`
   decoration. Smaller than originally scoped: the grasp lifecycle and stall detection
   are cut (decision 4), so this is the command path, the state path, and the halt hook.
3. **The ROS2 surface** — downstream, its own spec.

## Reference

- Arbitration: `2026-08-25-arm-arbitration-design.md`
- Streaming setpoints: `2026-08-26-streaming-setpoints-design.md`
- Downstream description package: `kinova_arm_ros2` `2026-08-31-kinova-arm-description-design.md`
- SDK: `kortex_api_2.8.0_aarch64`, `messages/GripperCyclicMessage.pb.h`, `messages/Base.pb.h`
