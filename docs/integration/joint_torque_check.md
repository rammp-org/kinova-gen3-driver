# Joint-torque check — feedforward and the staleness watchdog

**Attended procedure.** Companion to
[`grav_comp_static_check.md`](grav_comp_static_check.md), which covers the
zero-feedforward case.

## Why this exists

`benchmark_grav_comp` runs `JointTorqueMode` with `tau_ff` never set — i.e.
gravity-compensation hold. What it does **not** exercise is the thing the mode
exists for:

- `set_torque()` publishing a real feedforward on top of gravity comp, and
- the **staleness watchdog** decaying that feedforward to zero when the commands
  stop.

The streaming-setpoint tier will lean on that watchdog as its safe-stop for
torque streaming, so it is worth proving on hardware before anything is built on
top of it.

## Safety posture

Torque mode sits between the other two.

**It is compliant** — you are only ever *adding* torque on top of gravity
compensation, so the arm still yields when pushed. That is unlike position mode,
which chases its command at full actuator authority.

**But there is no spring pulling it back.** Unlike impedance mode there is no
reference to converge on: a sustained feedforward keeps accelerating the joint
for as long as you command it. What bounds the outcome is the per-joint clamp —
`(39, 39, 39, 39, 9, 9, 9)` N·m by default — and the wrist's **9 N·m** is the
number that matters, since a scalar limit sized for the proximal joints would
overrun it fourfold.

The harness refuses a `--tau` above half the target joint's clamp. Defaults are
one joint, **1.0 N·m** on the wrist — roughly a tenth of its rating.

## Procedure

**1. Read-only.** Connects, reads, commands nothing, never enters low-level
servoing. Confirms gravity torque and the clamp for every joint:

```sh
./build/joint_torque_check --ip 192.168.1.10 --dry-run
```

**2. Gravity-comp hold.** No feedforward at all — this should behave exactly
like the static check:

```sh
./build/joint_torque_check --ip 192.168.1.10 --tau 0
```

The arm holds itself up and yields when pushed. Nothing should drift.

**3. The real check.** 1 N·m on the wrist for 2 s, then the publisher stops and
the watchdog takes over:

```sh
./build/joint_torque_check --ip 192.168.1.10 --joint 6 --tau 1.0
```

## What to check

The app prints a 20 Hz trace of measured torque and position for the joint under
test, marked `[commanding]` or `[watchdog]`, and then tells you what to look
for:

- **During `[commanding]`** — measured torque on the joint sits above its
  gravity-only value by roughly the feedforward you asked for.
- **At the `PUBLISHER STOPPED` line** — that excess **ramps** away within about
  `cmd_timeout_s + cmd_decay_s` (0.15 s at the defaults) and does **not** snap.
  A hard drop to zero would mean the decay ramp is not working.
- **After the decay** — the arm holds under gravity compensation alone and the
  position stops drifting.
- **Throughout** — the arm still yields when you push it by hand. If it has gone
  stiff, something has put the actuators in the wrong mode.

## Note on `--sim`

`SimTransport` is a static echo: measured torque never responds and the arm never
moves. A `--sim` run proves the app parses, runs, and shuts down cleanly —
nothing more. The trace will be flat and that is expected.
