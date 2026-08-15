# JTC parity audit — `TrajectoryExecutor` vs `joint_trajectory_controller`

**Date:** 2026-08-15
**Why this exists:** the core must build **without ROS**, so the trajectory
execution layer cannot be `ros2_control`'s `joint_trajectory_controller` (JTC)
and has to be re-implemented here. That is a deliberate architectural choice,
not duplication by accident — which means the bar is *deliberate* parity: we
should know exactly which of JTC's behaviours we have, which we have chosen not
to have, and which we are missing by oversight.

Audited against `ros-controls/ros2_controllers`, `humble`:
`joint_trajectory_controller/src/joint_trajectory_controller.cpp`,
`.../trajectory.cpp`, `.../joint_trajectory_controller_parameters.yaml`.

Ours: `include/kinova_lowlevel/interface/trajectory_executor.h`,
`src/interface/trajectory_executor.cpp`, `src/interface/supervisor.cpp`, and
the ROS2 mapping in `kinova_arm_ros2`.

## Matrix

| JTC behaviour | ours | verdict |
| --- | --- | --- |
| Linear / cubic Hermite / quintic interpolation, selected by what the trajectory carries | `sample()` | **at parity** (issue #13, PR #14) |
| Derivatives of the interpolant available for feedforward | `sample_target()` | **ahead** — JTC computes them but Kinova's HW interface discards them |
| Per-joint path tolerance during motion | `path_tol_`, position only | **partial** — JTC also tolerances velocity and acceleration |
| Per-joint **goal** tolerance at the end of the trajectory | field exists, **never read** | **GAP — P0** |
| **goal_time** tolerance | field exists, **never read** | **GAP — P0** |
| Completion means *arrived*, not merely *time elapsed* | `elapsed >= dur` only | **GAP — P0** |
| Joint **name** mapping / reordering | positional indexing; `joint_names` never read | **GAP — P0 (safety)** |
| Hold position on abort / tolerance violation | executor drops the goal; mode holds its last reference | at parity in effect |
| Splice a replacing trajectory from the current state | restarts from the new trajectory's own first point | **GAP — P1** |
| Angle wraparound in the error computation | `wrap_to_pi` in both modes | **at parity** |
| Trajectory start time (`header.stamp`) | always starts immediately | gap — P2 |
| `cmd_timeout` (stale trajectory) | none | gap — P2 |
| `stopped_velocity_tolerance` | none | gap — P2 |
| Partial-joints goals | not supported | **deliberate non-goal** — a 7-DOF arm command should be complete |
| PID + `ff_velocity_scale` closed-loop adapter | n/a | **deliberate non-goal** — we command torque and run our own impedance law, which is strictly richer than a PID on position error |
| `open_loop_control`, `allow_integration_in_goal_trajectories` | n/a | deliberate non-goal |

## The two that matter

### P0 — goal completion is purely time-based

`TrajectoryExecutor::tick()` completes on `elapsed >= dur` and reports
`kSuccessful` unconditionally. `TrajectoryGoal::goal_tolerance` and
`goal_time_tolerance_s` are populated by the ROS mapping and then **read by
nothing** — dead fields that look live.

So a goal reports SUCCESSFUL when the *clock* runs out, regardless of where the
arm actually is. A trajectory that was tracking badly but stayed inside the
(generous, 0.35 rad) path tolerance still reports success. JTC instead checks
the goal tolerance once past the last point, and only then succeeds; if the arm
is outside it, it waits, and aborts with `GOAL_TOLERANCE_VIOLATED` once
`goal_time` is exceeded.

This is the "Related, not covered here" note in issue #13, and it is worse than
it first looks: **the caller cannot tell a successful move from a failed one**,
which is exactly the class of silent failure this repo's "fail loud" posture
exists to prevent.

JTC reference: `joint_trajectory_controller.cpp:254-291` (tolerance checks),
`:363-391` (success vs `GOAL_TOLERANCE_VIOLATED`).

### P0 — joint names are ignored, so a reordered goal is silently mis-mapped

The core never reads `joint_names` — `to_trajectory_goal` copies
`p.positions[i]` straight into `w.q[i]` by index. A `JointTrajectory` whose
`joint_names` are in any order other than `joint_1..joint_7` is therefore
mapped onto the **wrong joints**, silently, and executed.

Nothing currently exercises this — cuRobo emits `joint_1..7` in order — which
is precisely what makes it a latent hazard rather than a visible bug. JTC
handles it by building a mapping vector and remapping every point
(`joint_trajectory_controller.cpp:1393-1440`).

This directly violates the project's own rule: *"Fail loud at startup, never
silent mis-mapping."* Even if we decide not to support reordering, we must
**reject** a goal whose joint names do not match, rather than execute it
against the wrong joints.

### P1 — a replacing trajectory does not start from where the arm is

On `kLatestWins` the executor adopts the new trajectory and restarts its clock,
sampling from the new trajectory's first point. If the arm is not already at
that point, the reference jumps. JTC splices: it holds the state the arm was
actually in and blends from there
(`set_point_before_trajectory_msg`, `joint_trajectory_controller.cpp:208-213`).

The rate limiter in both modes bounds the resulting jump, so this is not a
safety issue today — it is a motion-quality one, and it will matter more as
preemption gets used (teleop, reactive replanning).

## Recommended order

1. Reject goals whose `joint_names` do not match the expected order (fail loud,
   cheap, closes a silent-mis-mapping hazard). Optionally remap afterwards.
2. Implement real goal semantics: honour `goal_tolerance` and
   `goal_time_tolerance_s`, add `kGoalToleranceViolated` handling, and make
   completion mean *arrived*. This makes `ExecuteJointTrajectory`'s existing
   `GOAL_TOLERANCE_VIOLATED = -5` code reachable — it is currently declared but
   never returned.
3. Splice on preemption.
4. The P2 items, only if a caller actually needs them.

Items 1 and 2 are the ones that change what a caller can *trust*. Everything
below them is refinement.
