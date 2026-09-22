# Velocity hold check — does the integrated reference hold, freeze, and track?

!!! warning "Temporary"
    This procedure and the `velocity_hold_check` app it drives are **bring-up
    scaffolding** for the 1.1.1 change to `JointVelocityMode`
    ([#34](https://github.com/rammp-org/kinova-gen3-driver/issues/34)). Both get
    deleted once the three questions below have answers on the real arm.

**Attended procedure. One joint, small speeds, short jogs, ramped.**

## Why this exists

From 1.1.1 `JointVelocityMode` integrates the commanded velocity into a
position reference and runs the actuators in position mode, because the
actuator's own velocity servo let joint 2 creep under gravity at a zero
command. The unit tests prove the integrator, the leash, the wrap and the
watchdog in simulation. Three things only hardware can settle:

1. **Does a zero command hold?** Joint 2 crept ~0.038 rad/s before.
2. **Does a dead stream stop the arm where it is?** The mode freezes the
   reference at the *measured* position and latches; on hardware that has to
   show as no travel after the last setpoint.
3. **Does the 0.1 rad leash stay dormant while tracking?** It is a constant
   chosen with no data on the position loop's following lag. If it bites during
   a normal jog it caps that joint and, in a twist, rotates the achieved
   direction; then it must become a parameter (a MINOR change), not a constant.

The app uses the library's `JointVelocityMode` itself and publishes setpoints
from a non-RT thread at 100 Hz, exactly as the streaming tier does.

## Safety posture

Velocity mode is stiff. The position servo tracks the reference at full
authority; what bounds it is the URDF velocity cap on the integrated velocity
and the 0.1 rad leash on the reference. The app refuses `--qd` above 1.0 rad/s,
jogs longer than 5 s or further than 1.5 rad, and holds longer than 120 s.
Defaults are the wrist joint at 0.05 rad/s.

## Procedure

**1. Read-only.** Connects, reads, commands nothing, never enters low-level
servoing:

```sh
./build/velocity_hold_check --ip 192.168.1.10 --dry-run
```

**2. The hold.** Streams zeros for 10 s, then for the shape of the original
incident, 60 s:

```sh
./build/velocity_hold_check --ip 192.168.1.10 --phase hold --hold-s 10
./build/velocity_hold_check --ip 192.168.1.10 --phase hold --hold-s 60
```

Verdict `HOLDS` if every joint stayed within 5 mrad. The per-joint drift and
joint 2's rate are printed either way.

**3. The stale path.** Ramps the wrist up to `--qd`, then stops publishing
mid-motion with a 0.2 s watchdog:

```sh
./build/velocity_hold_check --ip 192.168.1.10 --phase stale
./build/velocity_hold_check --ip 192.168.1.10 --phase stale --qd 0.3
```

Verdict `FREEZES` if the joint travelled under 20 mrad after the last
setpoint.

**4. Tracking at speed.** Jogs the wrist out and back at `--qd`, escalating:

```sh
./build/velocity_hold_check --ip 192.168.1.10 --phase track --qd 0.1
./build/velocity_hold_check --ip 192.168.1.10 --phase track --qd 0.3
./build/velocity_hold_check --ip 192.168.1.10 --phase track --qd 0.6 --jog-s 3
```

Verdict `TRACKS` if the mean measured velocity on the flat part is above 90% of
the command and the estimated lead (integral of the command minus the travel)
stays under 0.08 rad. `LEASHED` means the reference hit the leash while
tracking: the leash needs to become a parameter before 1.1.1 ships.

## Results, 2026-09-22 (Gen3 7-DOF + 2F-85, Jetson AGX Orin, RT core 11)

Arm at `q_rest`, wrist joint, 100 Hz setpoints, 0.2 s watchdog.

| Phase | Result | Verdict |
|---|---|---|
| hold, 10 s | peak excursion 0.2 mrad (joint 2); rate 0.00001 rad/s | HOLDS |
| hold, 60 s | joint-2 drift 0.1 mrad; every other joint 0.0 | HOLDS |
| stale, 0.05 rad/s | tracking at 0.054 rad/s at the cut; 12.2 mrad of travel after the last setpoint, i.e. the 0.2 s watchdog window | FREEZES |
| track, 0.1 rad/s | measured 100% / 100%; lead 0.007 / 0.002 rad | TRACKS |
| track, 0.3 rad/s | measured 100% / 100%; lead 0.019 / 0.006 rad | TRACKS |
| track, 0.6 rad/s | measured 100% / 100%; lead 0.038 / 0.012 rad | TRACKS |
| track, 1.0 rad/s | measured 100% / 100%; lead 0.071 / 0.022 rad | TRACKS |

The lead on the outbound leg grows linearly with speed at roughly **70 ms** of
following lag. Extrapolated to the 1.22 rad/s URDF cap that is ~0.087 rad,
inside the 0.1 rad leash but with little margin, and only the unloaded wrist
was measured. A loaded joint near its cap may leash; that is the case for
making the leash a `JointVelocityParams` field in 1.2.0.

One run (the first 1.0 rad/s attempt) aborted before entering servoing with an
uncaught `timeout detected: BaseClient::SetServoingMode`, started about one
second after the previous run's shutdown. The arm did not move; the retry
passed. Same class as
[#59](https://github.com/rammp-org/kinova-gen3-driver/issues/59) /
[#65](https://github.com/rammp-org/kinova-gen3-driver/issues/65).

Not covered here: twist streaming. The DLS map is unchanged by 1.1.1, but a
twist session on the arm should confirm the integrated path feels right.

## Note on `--sim`

Running with `--sim` exercises the plumbing only. The sim transport has no
plant, so `hold` trivially holds, `stale` is inconclusive and `track` lags.
That is expected and is not a result about the hardware.
