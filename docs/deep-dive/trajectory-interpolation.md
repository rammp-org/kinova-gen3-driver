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

## Where the profile comes from

The ROS2 frontend's `to_trajectory_goal` mapping copies `velocities` and
`accelerations` out of each `trajectory_msgs/JointTrajectoryPoint` and sets the
flags — but only when **every** point carries a correctly sized profile. A
partial profile is treated as absent, which degrades to the next order down
rather than mixing conventions mid-trajectory.
