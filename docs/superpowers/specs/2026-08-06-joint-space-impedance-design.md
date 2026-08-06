# Joint-space impedance with in-loop IK — design

**Date:** 2026-08-06
**Status:** approved
**Branch:** `feat/joint-space-impedance`

## Problem

`CartesianImpedanceMode` commands `tau = g + Jt F` on a 7-DOF arm against a 6-DOF
task. The remaining DOF is not commanded — it is only softly shaped by a
null-space term (posture bias, manipulability ascent). During Quest teleop the
arm therefore drifts into awkward elbow configurations and, near joint limits or
singularities, into states the operator cannot recover from by moving the
controller.

The null-space term cannot fix this. It is a *torque* applied in the null space,
so it biases the free DOF but never constrains it. Nothing bounds where that DOF
ends up.

## Approach

Resolve redundancy at the **reference** level instead of the torque level. Solve
IK for the commanded pose to obtain a full 7-DOF reference configuration `q_d`,
then run an independent impedance spring on every joint:

```
q_d   <- DiffIk(target_pose, warm_start = q_d)     # reference config, integrated
q_ref  = clamp(q_d, q +/- leash)                   # per-joint spring saturation
tau    = g(q) + ramp * ( Kq.(q_ref - q) - Dq.qd )
tau    = clamp(tau, +/- tau_lim_i)                 # per-joint
```

Every joint has a commanded position, so there is no free DOF. Which arm
configuration the robot adopts is decided deterministically by the IK's
redundancy resolution (posture bias + limit avoidance) rather than by whatever
the torque field drifts toward.

The tradeoff accepted here: Cartesian compliance is no longer exact. The
end-effector stiffness becomes `J^-T Kq J^-1`, i.e. configuration-dependent and
not diagonal in the task frame. For teleop that is the right trade — predictable
posture beats an exactly-shaped task-space ellipsoid.

## Components

### 1. `DiffIkSolver` — `include/kinova_lowlevel/diff_ik.h`, `src/diff_ik.cpp`

Bounded damped-least-squares Gauss-Newton, warm-started from the previous
reference, RT-safe (fixed-size stack scratch, no heap allocation after
construction).

```
repeat up to max_iters, stop when |e_pos| < pos_tol and |e_rot| < rot_tol:
    e  = pose_error(target, fk(q))
    clamp |e_pos| <= max_pos_err, |e_rot| <= max_rot_err
    J  = jacobian(q)
    A  = J Jt + lambda^2 I                                   # 6x6, fixed-size LDLT
    dq = Jt A^-1 e  +  N * [ k_rest (q_rest - q) + k_lim grad_limit(q) ]
    N  = I - Jt A^-1 J
    dq = clamp(dq, +/- max_joint_step)
    q  = clamp(q + dq, q_lower + margin, q_upper - margin)
```

- The 6x6 LDLT solve is the same alloc-free pattern already used for the
  null-space projector in `src/cartesian_impedance_mode.cpp`.
- `grad_limit(q)` is zero except within `limit_margin` of a hard stop, where it
  ramps linearly to 1.0 pushing inward. Continuous joints contribute zero.
- Error clamping (`max_pos_err` / `max_rot_err`) is what makes an unreachable or
  teleported target safe: the reference walks toward it at a bounded pace instead
  of producing an enormous first step.
- Warm-starting is what gives solution continuity. Consecutive teleop targets are
  millimeters apart, so the solve typically converges in one or two iterations.

Kept as a standalone unit (not folded into the mode) so it is testable without an
RT loop.

### 2. `Dynamics::joint_limits(JointVec& lower, JointVec& upper)`

Joint limits are read from the URDF, not hardcoded. Gen3 joints 1/3/5/7 are
`continuous`; Pinocchio packs those as (cos, sin) with `nq == 2` and reports
meaningless [-1, 1] configuration-space limits, so they are reported as
+/-infinity and never clamped. Joints 2/4/6 are revolute and get their real
limits (+/-2.41, +/-2.66, +/-2.23 rad).

Deriving from the model rather than hardcoding is consistent with the existing
`nv != kNumJoints` startup guard: a mismatched URDF must fail loudly or behave
correctly, never silently clamp to wrong values.

### 3. `JointImpedanceMode` — `include/kinova_lowlevel/joint_impedance_mode.h`, `src/joint_impedance_mode.cpp`

Presents the same seam as `CartesianImpedanceMode`: `on_enter` / `compute` /
`set_target(Pose)` / `set_gains`, with the same double-buffer + atomic-index
publication for non-RT setters (single writer, RT reader takes one snapshot per
cycle).

Three decisions worth recording:

**`q_d` is seeded from measured `q` at `on_enter` only, then integrated
open-loop.** Re-seeding from the measured configuration each cycle would collapse
the spring: the error would always be near zero and the mode would degenerate to
rigid tracking, losing compliance and fighting any contact. The reference is a
virtual arm the real one is spring-coupled to.

**Leash (`max_tracking_error`, default 0.35 rad).** Caps `|q_ref - q|` per joint,
so spring torque saturates at `Kq * leash` while gravity compensation passes
through untouched. The existing scalar `torque_limit` clamp cannot do this — it
clamps the *total* torque, so under high load it eats into the gravity term and
the arm sags. Applied to the torque computation only; `q_d` itself is not
modified, so pushing the arm away does not corrupt the IK reference.

**Per-joint torque limits, default `(39, 39, 39, 39, 9, 9, 9)` N.m.** The URDF
gives joints 5-7 an effort limit of 9 N.m. The existing scalar
`CartesianImpedanceParams::torque_limit = 39.0` exceeds that by 4x. With stiff
joint gains this matters, so the new mode is per-joint. The Cartesian mode's
scalar is deliberately left alone (out of scope) but is a known latent issue.

`max_ref_speed` (rad/s) additionally caps how far `q_d` may move per cycle, so a
teleop pose jump ramps in rather than slamming.

Defaults, conservative for a first hardware run:

| Param | Default | Unit |
|---|---|---|
| `Kq` | 100, 100, 100, 100, 40, 40, 40 | N.m/rad |
| `Dq` | 12, 12, 12, 12, 5, 5, 5 | N.m.s/rad |
| `torque_limit` | 39, 39, 39, 39, 9, 9, 9 | N.m |
| `max_tracking_error` | 0.35 | rad |
| `max_ref_speed` | 1.0 | rad/s |
| `gain_ramp_s` | 0.5 | s |

### 4. `PoseTargetSink` — `include/kinova_lowlevel/pose_target_sink.h`

Three-line abstract base declaring `virtual void set_target(const Pose&)`. Both
impedance modes implement it so `teleop_socket_server` can hold one pointer
regardless of which mode is active. One-line change to `CartesianImpedanceMode`.

### 5. Server integration — `--joint-impedance`

The Python supervisor requires **no changes**. POSE_TARGET, REHOME, FREEZE and
the gripper path all work identically because the new mode exposes the same
`set_target(Pose)` seam.

New flags: `--joint-impedance`, `--jkp`, `--jkd`, `--leash`, `--ref-speed`,
`--ik-iters`, `--ik-posture-gain`, `--ik-qrest`. `--jkp` / `--jkd` accept either
a single scalar (applied to all joints) or 7 comma-separated values.

`teleop_loop.py:167` sends SET_GAINS at startup. In joint mode the Cartesian
`Kx`/`Dx`/null-space fields are meaningless. The server honors `torque_limit` and
`gain_ramp_s` from the packet and prints a **one-time warning to stderr** naming
the ignored fields. Silently dropping operator-supplied gains is the kind of
footgun that costs an hour of confused hardware debugging.

Cartesian impedance remains the default so nothing that works today regresses.

## Testing

`tests/diff_ik_test.cpp`
- converges to a reachable pose from a perturbed seed (pose matches; `q` need not)
- respects hard joint limits under a target that pulls past them
- posture bias moves the configuration while task error stays converged
- unreachable target stays bounded, finite, NaN-free
- warm start from the exact solution is a no-op

`tests/joint_impedance_mode_test.cpp`
- requires torque mode; passes measured `q` through to `out.position`
- at `q == q_d`, `qd == 0` gives gravity only
- matches an independently computed control law
- per-joint torque clamping (asserts the 9 N.m wrist limit specifically)
- leash saturates the spring term but leaves gravity intact
- entry ramp scales the spring but not gravity
- `max_ref_speed` bounds reference motion on a teleported target

`tests/rt_safety_test.cpp`
- new `JointImpedanceMode` case asserting zero major faults in steady state.
  This is what proves the in-loop IK does not allocate.

Build and `ctest` run on abra (aarch64 Jetson) via `local_tools/build_on_abra.sh`;
this project cannot build on macOS.

## Out of scope

- Changing `CartesianImpedanceParams::torque_limit` to per-joint (noted above).
- Any protocol version bump or Python-side change.
- Nullspace/manipulability behavior of the existing Cartesian mode.
