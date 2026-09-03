# Gripper

The driver commands and reads back the Robotiq 2F-85 mounted on the Gen3's
wrist. This page is the user-facing story: what you can ask the gripper to do,
what it tells you back, and the two places where the honest answer is not the
obvious one. For exact signatures see the
[API Reference](../reference/api.md#gripper-gripper_typesh-gripper_controllerh-interfaceportsh);
for ownership mechanics see [Arbitration](arbitration.md).

## Not a control mode

The gripper is **not** a `ControlMode`. Modes are mutually exclusive — adopting
one gives up whichever was running — and giving up arm control just to move the
gripper would be absurd. Instead `GripperController` decorates `Transport` and
stamps the gripper fields onto every outgoing frame, the same shape `FeedbackTap`
uses to tap feedback and `Arbiter` uses to gate a `CommandSink`.

That makes the gripper **orthogonal** to whatever mode is driving the arm: you
can open or close it during a trajectory, mid impedance-hold, or while a
velocity stream is live. Nothing about the gripper touches mode switching, and
it adds nothing to the RT-safe surface beyond a fixed-size struct copy.

## The command: position, speed, force

Every gripper command carries three fields, all normalized `0..1`:

| Field | Meaning |
|---|---|
| `position` | `0` = open, `1` = closed |
| `speed` | fraction of max closing speed |
| `force` | fraction of max — see below |

**`force` is a ceiling on motor current, not a force setpoint.** The gripper
closes at `speed` toward `position` and stalls once motor current reaches the
`force` limit — that is the entire law. There is no force servo on this
hardware by any path: the SDK's `GripperMode` enum has no force mode at all,
and the high-level API that would host one needs a servoing level
(`SINGLE_LEVEL_SERVOING`) that is incompatible with the `LOW_LEVEL_SERVOING`
this driver's 1 kHz torque control requires. "Grasp with 15 N" is not
expressible on this arm; "close, but do not push harder than X" is.

**Speed and force do not persist between commands.** Every command is a
complete setpoint, following the same rule the streaming tier uses for its
setpoints — an absolute target, never a delta, so a dropped message can never
change the meaning of the next one. Concretely: set `force` once and then send
position-only commands, and you get the *default* force back (`1.0` speed,
`0.5` force), not your earlier value. A caller that wants a non-default force
on every command has to say so on every command.

**Percent is not aperture.** The 2F-85's linkage is underactuated — one motor
driving a four-bar through five mimic joints — which makes aperture-vs-angle
non-linear. `position = 0.5` is not "fingers 42.5 mm apart"; treat `position`
as a normalized command, not a length.

## The state: position, effort, current, present

Feedback mirrors the command's shape minus force, plus a presence flag:

| Field | Meaning |
|---|---|
| `position` | `0` (open) .. `1` (closed), measured |
| `effort` | `0..1`, derived from motor current — see below |
| `current` | amps, raw, exactly as reported |
| `present` | `false` when no interconnect gripper is attached |

`present` matters because, without it, a missing gripper and a fully-open one
both report `position = 0` — a silent mis-mapping the rest of the driver
refuses to allow elsewhere, so it doesn't allow it here either.

**There is no velocity field.** The hardware's feedback message does carry
one, but it was measured on the arm to be the *commanded* speed echoed back —
unsigned, identical whether opening or closing, and it kept reporting the
commanded value even while the fingers were being physically stopped by an
object. A field that carries no information you don't already have is worse
than no field, because it looks like a measurement — so core removed it rather
than publish a setpoint echo into a slot that ought to hold one.

The linkage's underactuation has a second consequence beyond non-linear
aperture: because the four-bar deviates from its free-space geometry on
contact so the fingers can wrap an object, **the mimic model is a free-space
approximation** — TF computed through the fingertips is not trustworthy
*during* a grasp. Fine for planning the approach; don't trust it once the
fingers are loaded.

## Reading effort honestly

`effort` is `|current| / kGripperMaxCurrentA`, a fraction of the gripper's
measured peak current — not Newtons, and not comparable across grippers. Read
literally, it produces a result that looks backwards until you've seen the
trace:

- **A grasp spikes, then settles.** Closing on a compliant object, current
  climbed to **1.00 A** (full scale, `effort = 1.0`) while the fingers closed,
  then dropped to about **0.05 A** and held there for as long as the grip was
  maintained. A *sustained* grasp reports a **small** effort, not a large one.
- **Closing on empty air reports zero.** The gripper reaches the commanded
  position, stops driving entirely, and current falls to `0` — it does not
  hold against the force cap the way it does on contact.

Anything that keys off "high effort means holding something" is backwards: the
high number is the brief squeeze during closure, and the number that persists
while an object is actually held is the low one.

## Ownership rides the arm's token

The gripper has no ownership of its own — `GripperSink` is gated by the same
token as `CommandSink` and `StreamSink`. Whoever holds the arm holds the
gripper; there is no separate grant to request and nothing to reconcile if the
two ever disagreed, because they can't. See
[Arbitration](arbitration.md#the-token-lifecycle) for how a token is granted
and revoked.

## On halt, the gripper holds

**On halt — ownership revoked, e-stop, operator request — the gripper stops
being commanded and keeps whatever grip it has.** E-stop means stop moving,
and opening the gripper is itself a motion, so halting does not open it.

State this plainly because it has a real cost: **if the gripper is closed on
something it should not be, e-stop will not release it.** Releasing needs a
deliberate open command sent after the halt clears. Anyone standing this
system up needs to know that before they rely on e-stop to make a bad grasp
let go.

The mechanism behind "holds" is worth knowing too, because it is not what it
sounds like. `KortexTransport` keeps a persistent cyclic command and
retransmits it whole, every cycle, regardless of whether the driver's own
`active` flag is set — this is **retransmission, not cessation**. So a gripper
stalled at e-stop does not go idle; it stays **energized, driven against the
commanded current cap**, continuously re-commanded toward the same target for
as long as the halt lasts — not merely held by the self-locking linkage. The
2F-85's linkage *is* self-locking, so nothing about whether the grip holds
depends on this — but it is a duty-cycle and thermal consideration for a long
halt, not a free lunch.

## No grasp primitive

There is no `Grasp` action and no stall detection. The gripper tier is
topic-only: send a setpoint, read state back, decide for yourself whether you
got what you asked for.

If you want to build one, the measurement points at the simplest possible
check. Commanding the gripper fully closed (`position = 1.0`), it settles at
**`0.8333`** when closing on an object and at **`0.9912`** on empty air. "The
gripper stopped short of the commanded target" separates the two cases with no
effort threshold at all — effort doesn't help here, since a sustained grasp's
effort is low (see above) and would need a floor tuned against noise. Start
from position, not effort.

## See also

- [Arbitration](arbitration.md) — the token the gripper rides, and what
  revocation and e-stop do to it.
- [Control Modes](control-modes.md) — why the gripper is deliberately not one
  of these.
- [API Reference](../reference/api.md#gripper-gripper_typesh-gripper_controllerh-interfaceportsh) —
  `GripperCommand`, `GripperFeedback`, `GripperController`, `GripperSink`,
  `GripperSetpoint`, `GripperState`.
