# Velocity-mode probe — does the arm honour `kVelocity`?

!!! warning "Temporary"
    This procedure and the `velocity_probe` app it drives are **bring-up
    scaffolding**. Both get deleted once the question below has an answer on the
    real arm. Do not build tooling on top of them.

**Attended procedure. One joint, tiny speed, short duration.**

## Why this exists

`KortexTransport` maps `ActuatorMode::kVelocity` to
`ActuatorConfig::ControlMode::VELOCITY` and writes `JointCommand::velocity` — but
**no `ControlMode` in this driver has ever used that path**, and it cannot be
settled in simulation: `SimTransport` is a static echo with no plant, so a
velocity command produces no motion there by construction.

It matters because `joint_position_mode.h` records that Kinova's own
`ros2_kortex` driver computes a velocity command and then declines to send it:

> `// Velocity command interface not implemented properly in the kortex api`

That comment is about the velocity field in *position* servoing — a different
path — but it establishes that velocity commands in the KORTEX API are a known
trouble spot.

The [streaming-setpoints design](../superpowers/specs/2026-08-26-streaming-setpoints-design.md)
puts an entire control mode (`JointVelocityMode`) on top of `kVelocity`. If the
firmware ignores or faults on it, that design needs rethinking. **So this probe
runs before that mode is written, not after.**

## Safety posture

Velocity mode has **no compliance**. The actuator's own servo chases the
commanded speed at full authority and will keep chasing it into an obstruction —
there is no spring to absorb a mistake, and unlike torque mode there is no
per-joint torque ceiling bounding the outcome.

The probe is built accordingly:

- **One joint**, defaulting to the wrist (`j6`) — the lightest and least able to
  damage anything if a direction is wrong.
- **0.05 rad/s for 2 s**, which is about 0.1 rad (≈5.7°) of travel.
- The command is **ramped in and out** over 0.5 s rather than stepped.
- The app **refuses** `--qd` above 0.3 rad/s or `--duration` above 10 s. It
  exists to answer yes/no, not to move the arm.

## Procedure

**1. Read-only first.** Connects, reads, commands nothing, never enters
low-level servoing:

```sh
./build/velocity_probe --ip 192.168.1.10 --dry-run
```

Confirm the reported `q` matches where the arm visibly is, and note the
predicted end position for the joint under test.

**2. Run the probe.** Hand on the e-stop:

```sh
./build/velocity_probe --ip 192.168.1.10
```

**3. Read the verdict.** The app prints one of:

| Verdict | Meaning |
|---|---|
| `TRACKS` | measured `qd` follows the command and `q` moved — velocity mode is honoured |
| `IGNORES` | measured `qd` stayed ~0 and `q` did not move — the command was accepted and silently dropped |
| `FAULTS` | the transport reported a fault |
| `PARTIAL` | it moved, but not as commanded — read the printed numbers |

## What each outcome means for the design

- **`TRACKS`** — `JointVelocityMode` is buildable as specified. Proceed with
  Plan 2.
- **`IGNORES`** — the worst outcome, because it is silent. The streaming tier's
  twist path needs redesigning; the torque-domain alternative
  (`tau = g(q) + Kd*(qd_des - qd)`) becomes the route, at the cost of the strict
  mode contract the design chose.
- **`FAULTS`** — at least it is loud. Capture the fault and check whether it is
  the mode switch or the command itself.
- **`PARTIAL`** — most likely a units or sign problem in the transport's
  `rad_to_deg` conversion on the velocity field. Compare commanded against
  measured before concluding anything about firmware.

## Note on `--sim`

Running with `--sim` exercises the plumbing only and will **always** report
`IGNORES`, because the sim transport has no plant. That is expected and is not a
result about the hardware. Use it to confirm the app runs and parses, nothing
more.
