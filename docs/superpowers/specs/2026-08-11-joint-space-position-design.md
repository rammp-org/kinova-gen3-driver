# Joint-Space Position Control — Design

**Date:** 2026-08-11
**Status:** implemented
**Branch:** `feat/joint-position-mode`

## Why

The first transport layer needs targets to test against. Joint-space impedance
covers the compliant case; this covers the rigid one. Between them, a caller can
either ask for a configuration and get it, or ask for a configuration and get a
spring toward it.

Scope is deliberately narrow: this exists so other code has something to drive.
It is not a trajectory generator and does not try to be.

## Interface

New `JointTargetSink` (`include/kinova_lowlevel/joint_target_sink.h`), the
joint-space counterpart of `PoseTargetSink`:

```cpp
class JointTargetSink {
 public:
  virtual void set_target(const JointVec& q_d) noexcept = 0;
};
```

Same threading contract as `PoseTargetSink`: one non-RT supervisor thread,
publishing via double-buffer + release-store so the RT reader never sees a torn
target.

A caller that already knows the configuration it wants should not have to invent
a Cartesian pose and pay for an IK solve to express it. That is the whole reason
this seam exists rather than reusing `PoseTargetSink`.

## The mode

`JointPositionMode` implements `ControlMode` + `JointTargetSink`.
`required_modes()` is all `kPosition`; `compute()` emits `out.mode = kPosition`
and `out.position = q_ref_`.

It runs **no dynamics** — no gravity term, no mass matrix, no IK. The actuator's
own servo closes the loop. Measured at `compute p50 = 128 ns` in sim, against a
1 ms budget.

What it owns is the reference integrator, and every stage of it is a guard:

```
q_ref  <- rate limit toward target      bounded by max_ref_speed·dt
       -> leash against measured q      bounded by max_following_error
       -> wrap (continuous joints)      kept in (-pi, pi] like the feedback
       -> clamp to position limits
```

### Why each guard is there

**Rate limit.** Position mode has no compliance. A target that teleports drags
the arm across the whole gap as fast as the joint can move. Default 0.5 rad/s,
below the URDF limits of 1.40 (proximal) / 1.22 (wrist) — a deliberate safety
choice for a mode with nothing to absorb a mistake, printed at startup and
settable. Finite requests are clamped **down** to the URDF value and non-finite
ones seeded from it, so no configuration can ask for more than the hardware is
rated for. Negative requests clamp to zero rather than reversing, which also
keeps `std::clamp`'s `lo <= hi` precondition true.

**Following-error leash.** A no-op whenever the arm is tracking; it only bites
when the arm cannot follow. Without it a blocked arm lets the reference march all
the way to the target while the arm sits still, and the arm snaps across the
entire accumulated gap the instant it comes free. Default 0.35 rad, `<= 0`
disables.

**Continuous-joint wrap.** This is the j3 bug in a new place. Reference and
measurement both live in (−π, π], so a target on the far side of the wrap reads
as a ~2π error and the reference walks most of a full turn the wrong way. The
difference is wrapped before the rate limit, and the command is wrapped after, so
it stays in the representation the transport reports measurements in.

**Position-limit clamp.** Applied to the target first, so the reference settles
exactly on the stop instead of carrying a permanent standing error against an
unreachable setpoint — and again to the output, because the leash rewrites
`q_ref` relative to the *measured* position and could otherwise push it out of
range.

**Zeroed torque and velocity.** `RtExecutor` reuses one `JointCommand` across
mode changes, so a previous mode's setpoints would still be sitting in those
fields. The transport ignores them in `kPosition` today; that is not a reason to
leave stale setpoints where something later could act on them.

## What was deliberately left out

- **No `teleop_socket_server` integration.** That server speaks Cartesian poses
  end-to-end, joint targets do not belong in its protocol, and it is being
  replaced by the transport layer shortly. No protocol version bump, so
  `teleop_protocol.h` ↔ `protocol.py` parity is untouched.
- **No `forbidden_modes()`.** `RtExecutor` holds exactly one active mode and sets
  all 7 actuators to that mode's `required_modes()` on every swap
  (`src/rt_executor.cpp:98-106`), so a torque/position conflict cannot arise. A
  safety check nothing can trigger reads as protection without being any.
  Recorded as [issue #7](https://github.com/rammp-org/kinova-gen3-driver/issues/7)
  for when a hybrid per-joint mode makes it real.
- **No trajectory generation.** Rate limiting is the whole of the motion
  shaping. Anything smoother belongs above this layer.

## Testing

15 unit tests plus an allocation-freedom case in `rt_safety_test.cpp`. Each was
watched failing against a deliberately unguarded stub first — the joint-limit,
wrap, and leash tests all failed on assertions with the exact wrong values
(`3.41 vs 2.41`, `-3 vs 3`, `1 vs 0.35`) before the guards existed.

`apps/joint_position_check.cpp` is the hardware harness; the procedure is
[`docs/integration/joint_position_hardware_check.md`](../../integration/joint_position_hardware_check.md).

## Known unknown

`KortexTransport::write_command` sends `rad_to_deg(cmd.position)` unwrapped while
feedback is wrapped to (−π, π], so continuous joints past half a turn would be
commanded as **negative** degrees. `kPosition` is the first code path that ever
sends our own wrapped angle back to the arm — torque mode passes through the raw
KORTEX value and bypasses the wrap entirely. KORTEX reports positions in
[0, 360); whether it accepts a negative setpoint is unverified and not settleable
in sim. Step 2 of the hardware check is built around answering it.
