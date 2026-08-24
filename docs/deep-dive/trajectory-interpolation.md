# Trajectory interpolation

`interface::sample(tr, t)` turns a discrete `Trajectory` into the continuous
joint target `q_d(t)` that the executor pushes to the active control mode every
tick. How it interpolates between waypoints decides whether the arm moves
smoothly or judders.

## Why the order matters

A motion planner does not hand back a bare list of positions. cuRobo emits a
time-parameterised trajectory carrying **positions, velocities and
accelerations** at every point. If the driver keeps only the positions and
joins them with straight lines, the commanded velocity is piecewise constant: it
*steps* at every waypoint.

A representative cuRobo plan is 144 points over 2.88 s — about 20 ms of spacing.
Linear interpolation therefore puts a velocity discontinuity roughly every 20 ms,
about **50 per second**, and the 1 kHz loop faithfully commands every one of
them. That is the jerk reported in issue #13.

The same plans looked smooth through `ros2_kortex` for a concrete reason:
`ros2_control`'s `joint_trajectory_controller` picks its interpolation from what
the trajectory actually carries — cubic when velocities are present, quintic when
accelerations are too. Routing the same plan through this driver used to
downgrade it to linear. `sample()` now makes the same choice.

## The three orders

`Trajectory` carries two flags that select the order. They describe the whole
trajectory, not individual points:

| `has_velocities` | `has_accelerations` | interpolation | continuity |
| --- | --- | --- | --- |
| `false` | — | linear | C0 — velocity steps at every knot |
| `true` | `false` | cubic Hermite | C1 — velocity continuous |
| `true` | `true` | quintic Hermite | C2 — acceleration continuous |

Accelerations are honoured only together with velocities.

!!! warning "A zero `qd` is not the same as 'no velocity'"
    `JointWaypoint::qd` and `qdd` default to zero, so the flags — not the
    values — are what mark a profile as present. If a caller left the flags
    false but the fields zero, cubic interpolation would read those zeros as
    *"come to a complete stop at every waypoint"* and ease in and out of each
    one — far worse than the linear path it replaced. Populate the flags and
    the fields together, or neither.

Positions-only trajectories keep the exact pre-existing linear behaviour, so
`trajectory_run`'s two-waypoint goals and any other hand-built trajectory are
unaffected.

## The math

Both Hermite forms work on one segment `[a, b]`, with span `T = b.t_s - a.t_s`
and normalised parameter `u = (t - a.t_s) / T ∈ [0, 1]`. Derivatives are scaled
to the unit segment — `v = qd·T`, `A = qdd·T²` — so the polynomial is written in
`u` while the boundary conditions are stated in real time.

**Cubic Hermite** matches position and velocity at both ends:

```
q(u) = h₀₀·a.q + h₁₀·v₀ + h₀₁·b.q + h₁₁·v₁

h₀₀ =  2u³ − 3u² + 1      h₁₀ =  u³ − 2u² + u
h₀₁ = −2u³ + 3u²          h₁₁ =  u³ −  u²
```

At `u = 0` the slope is `v₀` and at `u = 1` it is `v₁`. Since consecutive
segments share the waypoint's `qd`, the velocity leaving one segment equals the
velocity entering the next — the discontinuity is gone by construction.

**Quintic Hermite** additionally matches acceleration at both ends. With
`d = b.q − a.q`:

```
q(u) = a.q + v₀u + ½A₀u² + c₃u³ + c₄u⁴ + c₅u⁵

c₃ =  10d − 6v₀ − 4v₁ − 1.5A₀ + 0.5A₁
c₄ = −15d + 8v₀ + 7v₁ + 1.5A₀ −     A₁
c₅ =   6d − 3v₀ − 3v₁ − 0.5A₀ + 0.5A₁
```

Degenerate segments (duplicate timestamps, `T ≤ 0`) return the left waypoint
rather than dividing by zero.

## Cost

`sample()` does **not** run on the RT thread. It is called from
`TrajectoryExecutor::tick()`, which the supervisor drives from `sampler_loop` at
`SupervisorConfig::sampler_hz` (250 Hz), and which `trajectory_run` drives from
its publisher thread at the same rate. The 1 kHz thread only reads the
double-buffered `JointTarget` those threads publish — it never interpolates.

That is worth stating precisely, because it sets what this function is allowed
to do: `sample()` is not bound by the RT contract, and a future change here does
not need the RT-safety argument. What it *is* bound by is the sampler's own
period; the arithmetic was therefore measured rather than assumed — 200k calls
against a 144-point / 2.88 s plan, on the isolated core:

| order | p50 | p99 | p99.9 |
| --- | --- | --- | --- |
| linear | 64 ns | 224 ns | 288 ns |
| cubic Hermite | 64 ns | 128 ns | 160 ns |
| quintic | 96 ns | 160 ns | 192 ns |

The worst case, quintic, costs about **32 ns more per sample than linear** —
some 0.0008% of the sampler's 4 ms period. The math is fixed-size `JointVec`
arithmetic on the stack: no allocation, no locks. The RT contract is unaffected
either way, and `RtSafety` (which runs the supervisor in the loop) still reports
zero major faults and zero dropped samples in steady state.

One consequence of the 250 Hz sampler worth knowing when reading the smoothness
claim: the reference is republished every 4 ms and then slew-limited by
`max_ref_speed`, so the C1 continuity a velocity profile buys is real but is
delivered through that 4 ms staircase rather than a fresh polynomial evaluation
every millisecond.

## What the profile is used for

Interpolation is only half the value of carrying `qd`/`qdd`. The other half is
**feedforward**: telling the control law what motion it is supposed to be
producing, instead of making it infer that from tracking error.

`sample_target()` returns the reference together with the derivatives *of the
very polynomial `sample()` evaluates*, so the feedforward always describes the
curve actually being commanded, and `JointTargetSink` carries all three to the
mode. Modes only see derivatives when the trajectory really had them, so the
teleop and Cartesian paths are unchanged.

### Joint impedance: damp the velocity error, not the velocity

The torque law used to damp against measured velocity alone:

```
tau = g(q) + Kq·clamp(q_d − q) − Dq·qd
```

That damper pulls against the arm *precisely because it is moving as
commanded*. The cost is a standing, speed-proportional lag which this repo's
own gains documentation states exactly:

```
lag = 2·zeta·qd·sqrt(M/Kq)
```

At `zeta = 0.5` that is `qd·sqrt(M/Kq)` — for joint 1 elbow-up (M ≈ 0.573,
Kq = 80) about **0.085 rad for every rad/s of commanded speed**, so roughly 10°
of lag at 2 rad/s. It is systematic, not noise, and it grows with speed. The
0.35 rad path tolerance the planned-move actions use is padding for it.

Damping the velocity *error* removes it, and the planned acceleration is added
as an inverse-dynamics term so the spring is not left to generate it out of
tracking error:

```
tau = g(q) + Kq·clamp(q_d − q) − Dq·(qd − qd_ref) + M(q)·qdd_d
```

`qd_ref` is measured as the **achieved** reference velocity — how far `q_d`
actually moved this cycle — not copied from the planner. When the reference
rate limiter clamps, we then feed forward the motion we are really commanding
rather than the motion we were asked for, and the limiter also bounds the
feedforward for free.

`M(q)` is already computed every cycle to derive the damping, so the inertial
term is one matvec. Measured over 100k cycles on the isolated core,
`compute()` costs **3040 ns p50 with or without feedforward** — the difference
does not clear the noise floor of the dynamics call that dominates it.

### Joint position: a feedforward the firmware may or may not want

`JointPositionMode` computes the same achieved reference velocity and exposes
it as `last_ref_velocity()`, but **sends it only when
`JointPositionParams::velocity_feedforward` is enabled, which it is not by
default.**

!!! warning "Kinova's own driver tried this and disabled it"
    `ros2_kortex` computes exactly this velocity command and then deliberately
    does not send it (`kortex_driver/src/hardware_interface.cpp`):

    ```cpp
    base_command_.mutable_actuators(i)->set_position(cmd_degrees_tmp_);
    // Velocity command interface not implemented properly in the kortex api
    // base_command_.mutable_actuators(i)->set_velocity(cmd_vel_tmp_);
    ```

    So the vendor's position is that per-cycle velocity alongside position is
    not properly supported by the KORTEX API. Treat `velocity_feedforward` as
    unlikely to help on this hardware until someone demonstrates otherwise on a
    real arm; the flag exists to make that experiment possible, not because the
    path is expected to work.

Whether the firmware treats the velocity field as a feedforward or as a *limit*
in POSITION servoing is not documented either way, and being wrong about it at
1 kHz is a hardware event. So the value is computed and loggable first, and sent
only when someone opts in with the arm attended.

None of this touches the impedance path, which is where the feedforward
actually pays: there we command **torque** and evaluate the control law
ourselves, so no firmware interpretation is involved.

When the flag is off, `JointCommand::velocity_active` stays false and
`KortexTransport` writes nothing to that field — an explicit zero is *not* a
safe stand-in for "no feedforward", since a zero may read as a velocity limit
of zero. The command stays byte-identical to before.

## Where the profile comes from

The ROS2 frontend's `to_trajectory_goal` mapping copies `velocities` and
`accelerations` out of each `trajectory_msgs/JointTrajectoryPoint` and sets the
flags — but only when **every** point carries a correctly sized profile. A
partial profile is treated as absent, which degrades to the next order down
rather than mixing conventions mid-trajectory.
