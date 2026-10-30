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

Duplicate timestamps never produce a zero-span segment: the segment search
brackets every query with `a.t_s ≤ t < b.t_s`, so it skips the zero-width
pair and the reference steps from the first duplicate's position to the
last's at that instant. The `T ≤ 0` guard in `sample_target` is defensive;
no query reaches it.

## Hermite is not hull-preserving

Linear interpolation never leaves the interval between a segment's two
endpoint positions. Cubic and quintic Hermite can: the curve follows the
endpoint tangents `v = qd·T` (and `A = qdd·T²`), and nothing confines it to
`[min(a.q, b.q), max(a.q, b.q)]`. `sample()` can therefore command a joint
position outside the segment its waypoints bound, a configuration the
planner may not have collision-checked. Both control modes clamp to joint limits
downstream, so this is not a limits hazard. The risk is leaving the corridor
the planner checked.

It happens in two cases:

- **Direction reversals.** When a joint turns around between two waypoints,
  the plan's own curve peaks between them, and the interpolant follows it.
- **Derivatives that don't match the timestamps.** If a plan is re-timed
  (timestamps stretched by `k`) without recomputing its derivatives, every
  tangent is `k` times too long and every `A` is `k²` times too large, and
  the curve swings past its endpoints. A realistic source is a client that
  stretches the timestamps but leaves `qd` and `qdd` as they were
  (RAMMP-CuRobo's `scaled_msg` rescales all three). Compressing the
  timestamps (`k < 1`) makes the tangents too short instead. In the measurement below that leaves the hull by less
  than the consistent plan does (8.5e-5 rad cubic, 1.0e-4 rad quintic at
  ×0.5), but the result no longer follows the plan's path either.

**The path-tolerance guard will not report it.** `TrajectoryExecutor::tick`
compares the measured `q` against `sample_target(...).q`, the interpolated
value, not against the planner's waypoints. If the interpolant overshoots, the
guard measures tracking error relative to the overshoot.

Measured by
`TrajectorySample.HermiteCanLeaveTheWaypointHullWhenDerivativesMismatchTimestamps`:
the worst excursion beyond the endpoint hull, on any joint, scanning each
segment at 1/200 of its span. The plan is 144 points over 2.88 s (20 ms, a
cuRobo-sized plan) of per-joint sines: amplitudes 0.5–1.1 rad, peak
velocity up to 2.75 rad/s, peak acceleration up to 6.9 rad/s². Every joint
reverses at least once. The excursion at a reversal is about
`qdd·T²/8`, so it grows with acceleration and with the square of the
spacing even when the derivatives are consistent. Downsampling a plan while
keeping its derivatives is therefore safe in the sense that the interpolant
still follows the plan's own curve, but that curve leaves the hull further:
keeping every 5th point of this plan (100 ms) gives 3.9e-3 rad quintic.

| plan | cubic | quintic |
| --- | --- | --- |
| derivatives consistent | 2.1e-4 rad (0.012°) | 2.1e-4 rad (0.012°) |
| re-timed ×2, derivatives kept | 4.8e-4 rad (0.027°) | 3.4e-4 rad (0.020°) |
| re-timed ×4, derivatives kept | 1.2e-3 rad (0.066°) | 3.2e-4 rad (0.018°) |
| re-timed ×10, derivatives kept | 5.8e-3 rad (0.33°) | 3.2e-2 rad (1.8°) |
| re-timed ×20, derivatives kept | 5.8e-2 rad (3.3°) | 0.11 rad (6.5°) |

With consistent derivatives the excursion equals the analytic curve's own
peak between samples (2.1e-4 rad), so Hermite adds nothing measurable on top
of the plan. With mismatched derivatives it grows fast. Quintic tolerates a
moderate mismatch better than cubic, then overtakes it between ×4 and ×10 as
its `k²` acceleration error dominates. At ×10, which is a 0.1 speed scale
applied to the timestamps only, the quintic reference leaves the checked
corridor by 1.8° on the worst joint.

**Re-timing a plan without recomputing its derivatives is unsafe.** If you
stretch the timestamps by `k`, divide `qd` by `k` and `qdd` by `k²`, or drop
the profile flags and accept linear interpolation. To slow a goal down,
use `speed_scale` or the runtime override (see [Speed scale](#speed-scale)).
They dilate the executor's clock and leave the plan untouched, so they are
safe. `submit()` does not check for this case.

## Cost

`sample()` does **not** run on the RT thread. It is called from
`TrajectoryExecutor::tick()`, which the supervisor drives from `sampler_loop` at
`SupervisorConfig::sampler_hz` (1 kHz), and which `trajectory_run` drives from
its publisher thread at its own `--tick-rate` (250 Hz by default). The RT thread only reads the
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
some 0.0032% of the sampler's 1 ms period. The math is fixed-size `JointVec`
arithmetic on the stack: no allocation, no locks. The RT contract is unaffected
either way, and `RtSafety` (which runs the supervisor in the loop) still reports
zero major faults and zero dropped samples in steady state.

The sampler used to run at 250 Hz. The reference was then republished every 4 ms
and slew-limited by `max_ref_speed`, so the C1 continuity a velocity profile buys
arrived through a 4 ms staircase. That was measurable on the arm: a looped cuRobo
joint tour showed ~15% less velocity ripple and ~20% less jerk at 1 kHz. The
sampler is still a separate `sleep_for` thread, not synchronised with the RT
cycle, so the staircase is shorter rather than gone; evaluating the trajectory
inside the RT loop would remove it.

## Speed scale

`TrajectoryGoal::speed_scale` and `Supervisor::set_speed_override` slow a goal
down by dilating `TrajectoryExecutor`'s own clock: each tick accumulates
`traj_t += dt_wall * scale` instead of `traj_t += dt_wall`, then calls
`sample_target(tr, traj_t)` exactly as before. The sampler never sees the
scale — the path it interpolates is byte-for-byte the plan the goal carried.
Running the same geometric path at a slower clock means every derivative the
planner's polynomial carries scales with the clock rate: commanded velocity
scales with `s`, acceleration with `s²`, straight out of the chain rule for a
time-reparameterised curve. Since the v1.3 feedforward port the executor is
the one place that applies it: the tick scales the sampled `qd` by the
effective scale and `qdd` by its square before handing the target to
`JointTargetSink`, so the damper and `M(q)·qdd` term always describe the
slowed curve actually being commanded. (`sample_target` is the single
evaluator — position and derivatives come from one set of Hermite
coefficients; `sample()` is just its position.)

The effective scale is `min(goal_scale, override_scale)`, clamped to
`[kMinSpeedScale, 1.0]` (`kMinSpeedScale = 0.01` — zero would stop the
trajectory clock and hang the goal forever) — whichever of the goal's own
request and the operator's runtime override is slower wins, so an operator can
only ever slow a goal down, never speed one up past what it asked for. Both
`speed_scale` and the runtime override are **refused**, not clamped, below
this floor — see `api.md` — so `effective_scale()`'s internal clamp is a
belt-and-braces guarantee, not something a caller can actually reach.

Both dials are exposed on `trajectory_run` for demonstrating this on a bench:
`--speed-scale S` sets the goal's own scale at `submit()`, and `--scale-at T:S`
changes the runtime override mid-flight at `t = T`, printing the applied scale
as it ramps. The second is the one that shows the slew limiter below, since a
goal-level scale is in force from the first tick and so never ramps.

A change in the effective scale **mid-goal** is slew-limited at
`kScaleSlewPerSec` (2.0/s) rather than applied instantly, so the *commanded
velocity* never steps the way a bare change of clock rate would. That
guarantee is scoped to mid-goal changes: at a goal **boundary** — a fresh
`submit()`, including a `kLatestWins` preemption — the applied scale latches
to the new goal's own scale immediately, on its very first tick, including
*upward*. A goal submitted while the override is already down starts at that
slower scale from its first tick. A step in the
commanded reference at a goal boundary is inherent to switching trajectories
at all, scale aside — the slew limit only governs a change of scale *within*
one running goal.

**This is not `max_ref_speed`.** `max_ref_speed` rate-limits how fast
`JointPositionMode` may move its reference *toward* whatever `q_d(t)` the
executor just handed it — a per-cycle clamp on the mode's own output, with no
knowledge that a trajectory or a plan exists above it. Turning that down while
the executor keeps sampling the plan at its normal rate makes the reference
fall behind the sample the executor is publishing; the gap between
`q_desired` and where the reference actually is grows without bound, and that
manufactured gap is exactly what trips `PATH_TOLERANCE_VIOLATED`. Slowing a
goal down is a `speed_scale`/override problem, not a `max_ref_speed` one:
dilate the clock feeding `sample()`, don't throttle the mode consuming its
output.

## Where the profile comes from

The ROS2 frontend's `to_trajectory_goal` mapping copies `velocities` and
`accelerations` out of each `trajectory_msgs/JointTrajectoryPoint` and sets the
flags — but only when **every** point carries a correctly sized profile. A
partial profile is treated as absent, which degrades to the next order down
rather than mixing conventions mid-trajectory.
