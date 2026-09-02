# Deep Dive: The Velocity-Mode Twist Map

This page derives the damped-least-squares (DLS) twist map used by
`JointVelocityMode::set_twist_target`, explains why the damping is scheduled on
manipulability rather than fixed, and works through the projector-free
null-space posture form actually implemented in `solve_twist`. For the
conceptual overview and the stiffness/staleness contract see the
[Control Modes guide](../guide/control-modes.md) and the
[Streaming guide](../guide/streaming.md#jointvelocitymode-specifics); for exact
signatures, the [API Reference](../reference/api.md#jointvelocitymode-joint_velocity_modeh).

The canonical design record is the spec and plan under `docs/superpowers/`.

## Why velocity mode needs a different safety story

Every other control law in this driver commands **torque**. A bad solve there
still produces a *bounded* torque, clamped per joint before it reaches the
actuator — the worst case is a hard shove, not an unbounded motion. Velocity
mode commands `ActuatorMode::kVelocity` directly: whatever `solve_twist`
computes goes straight to the joint servo, with no dynamics standing between
the number and the motor. That single fact drives every design choice below —
in particular, damping near a singularity is not a refinement here, it is what
keeps the *solve* well-conditioned rather than returning an enormous `qd` in
the direction the arm has just lost.

!!! note "What actually bounds the command"

    Damping is about **conditioning**, not about bounding. What bounds the
    command is `limit()`: a uniform scale so the fastest joint just reaches its
    `max_qd` cap, followed by a hard per-joint clamp, applied on **every** cycle
    regardless of `w`. And because `w_threshold` sits at roughly one tenth of a
    nominal `w`, `limit()` is in practice the *primary* bound across most of the
    near-singular band — an undamped solve there runs straight into the clamp.

    The behaviour you observe near a singularity is therefore the EE **slowing
    down**: the uniform scale shrinks the whole command while preserving its
    direction. It is not tracking degrading in one direction. If the tool feels
    sluggish, that is `limit()` scaling, and reaching for `dls_damping_max` will
    make it worse, not better.

    Both halves are pinned by test:
    `JointVelocityModeTwist.DampingNotTheLimiterIsWhatBoundsTheSolveNearASingularity`
    and `…TheDampingRampIsWhatSeparatesTheseTwoSolves`, which measure a
    near-singular pose (`w = 0.00031`, inside the ramp) with the schedule on and
    off: flat, the solve pins a joint exactly at its 1.3963 rad/s URDF cap;
    ramped, it stays at 0.31 rad/s and gives up tracking instead.

## The DLS twist map

Given the measured configuration `q` and a commanded EE twist `V` (linear
velocity stacked on angular velocity, base frame), the task is to find a
joint-velocity command `qd` such that `J(q) · qd ≈ V`, where `J` is the 6×7
frame Jacobian (`jacobian()`, `LOCAL_WORLD_ALIGNED`, same convention as
[Cartesian impedance](impedance.md#frames)).

`J` is 6×7 — more columns than rows — so the system is underdetermined; a
6-DOF task on a 7-DOF arm has a whole line of joint-velocity solutions for any
achievable `V`, differing only in what they do with the null-space (redundant)
direction. The **damped least-squares** solution is the minimum-norm task
solution plus a Tikhonov regularizer:

```
qd_task = Jᵀ (J Jᵀ + λ²I)⁻¹ V
```

### Where this comes from

Minimizing `‖J·qd − V‖² + λ²‖qd‖²` over `qd` (task tracking, regularized by
command magnitude) and setting the gradient to zero gives the normal equations

```
(JᵀJ + λ²I) qd = Jᵀ V
```

Applying the push-through identity `Jᵀ(JJᵀ + λ²I) = (JᵀJ + λ²I)Jᵀ` lets this be
solved as a 6×6 system instead of a 7×7 one — the same trick used in
[Cartesian impedance's nullspace projector](impedance.md#damping-for-singularity-robustness),
here applied to the primal task solve rather than a projector:

```
qd = Jᵀ (J Jᵀ + λ²I)⁻¹ V
```

`A = J Jᵀ + λ²I` is 6×6, symmetric, and — because of the `λ²I` term — strictly
positive definite for **any** `J`, including a rank-deficient one at a
singularity. That is what makes the fixed-size Cholesky solve below always
well-defined, with no special-casing for "is this Jacobian singular right
now?"

At `λ = 0` this is the Moore–Penrose pseudo-inverse solution — exact task
tracking, minimum joint-velocity norm. `λ > 0` trades a small amount of task
tracking error for a bounded solution as `J` loses rank.

## Damping scheduled on manipulability

A single fixed `λ` is either too small to bound the solution near a
singularity or too large to track accurately everywhere else. Instead `λ` is
**scheduled** on Yoshikawa's manipulability measure

```
w = sqrt(det(J Jᵀ))
```

— a scalar that is large in well-conditioned configurations and goes to
**exactly zero** at a true kinematic singularity (`J` loses rank, `det(JJᵀ) =
0`). Below a threshold `w_threshold`, damping ramps from the baseline
`dls_damping` up to `dls_damping_max` quadratically in how far past the
threshold the arm is:

```
r = 1 − w / w_threshold           # 0 at the threshold, 1 exactly at a singularity
λ = dls_damping + (dls_damping_max − dls_damping) · r²
```

The quadratic ease keeps damping near-baseline (and tracking near-exact) until
the arm is genuinely close to degenerate, then rises steeply right as it
matters — a linear schedule would cost tracking accuracy across a much wider
band of otherwise-fine configurations for the same worst-case bound.

### Measured, not guessed

`w_threshold`'s default (`0.0033`) is grounded in two measurements on this
URDF (`JointVelocityModeTwist.ManipulabilityIsLowerAtTheSingularity`):

| Configuration | `w = sqrt(det(J Jᵀ))` |
|---|---|
| `nominal_q()` — elbow-up, comfortably posed | **0.0325** |
| `straight_q()` — arm fully extended | **0.0000** — a true singularity, not a near one |

`straight_q()` is the classic wrist/elbow singularity for this arm: fully
extended, the wrist axes align and the Jacobian loses rank exactly, not
approximately — `det(J Jᵀ)` prints to six decimal places as zero. With that
range established, `w_threshold` is set to roughly one tenth of the nominal
value: comfortably inside "normal operation" territory so ordinary teleop
motion never triggers extra damping, while still leaving headroom before the
arm reaches the actual singularity.

`w` is **unit-mixed and scale-dependent** — `J` stacks linear (m/rad) on angular
(rad/rad) rows, so `det(J Jᵀ)` has no clean physical dimension and its magnitude
depends on the arm's link lengths and on where the EE frame sits. Changing the
URDF or the EE frame — the 2F-85 gripper model moves the EE frame — changes the
numbers in the table above, and `w_threshold` must be re-derived from a fresh
measurement rather than carried over.

### Getting `w` for free

Manipulability is `sqrt(det(J Jᵀ))`, and `solve_twist` needs a factorization of
`J Jᵀ` (or the damped version of it) anyway to solve the DLS system. Rather
than compute `det` separately, `solve_twist` factors the **undamped** `A = J
Jᵀ` first with a Cholesky decomposition, reads `det(A)` off the product of the
diagonal factor (`LLT`/`LDLT` hand you the determinant as a byproduct of the
factorization, at no extra cost over the factorization itself), then adds
`λ²I` to the diagonal and re-factors for the actual damped solve. Two
factorizations, not two independent computations of "condition of J" and "the
solve" — `w` is a side effect of work the solve needs regardless.

## The projector-free null-space form

The Gen3's redundant DOF (7 joints, 6-D task) needs somewhere to go, or the
elbow drifts while the tool tracks the twist perfectly — the same problem
[Cartesian impedance's nullspace term](impedance.md#nullspace-posture-redundancy-resolution)
solves for torque. Here the secondary objective is a posture-seeking joint
velocity

```
b = posture_gain · (q_rest − q)     # per joint, wrapped to (−π, π] on continuous joints
```

and the textbook way to add it without disturbing the task is to project it
through the null-space projector `N = I − Jᵀ(JJᵀ+λ²I)⁻¹J` (7×7) and add `N·b`
to the task solution. `solve_twist` never forms `N`:

```
qd = qd_task + b − Jᵀ (J Jᵀ + λ²I)⁻¹ (J b)
   = qd_task + N·b
```

Expanding `N·b = b − Jᵀ(JJᵀ+λ²I)⁻¹J·b` shows the two forms are identical —
the projector-free version just reorders the arithmetic so that `J` and the
already-factored `(JJᵀ+λ²I)` (its Cholesky factor is already sitting in memory
from the task solve) act on the 7×1 vector `b`, rather than materializing the
7×7 matrix `N` and multiplying it out. Same result, no 7×7 temporary, and the
6×6 factorization is **reused** — `ldlt_.solve(J * bias_)` is one more
triangular solve against the same factor, not a second decomposition. This is
the same "solve, don't invert-and-multiply" discipline the RT-safety section
below depends on.

Continuous joints (`std::isfinite(lo) == false` and `std::isfinite(hi) ==
false` on the URDF, i.e. joints 1/3/5/7) take the posture error's **short**
way around: `wrap_to_pi(q_rest[i] − q[i])`, not the raw difference. Without
this, a rest angle just past ±π drives the joint most of a turn the wrong way
to reach it — a plain subtraction has no notion of "the long way round" on an
unbounded joint.

## Real-time safety

`compute()` — and therefore `solve_twist` — runs on the 1 kHz RT thread and
must never allocate, lock, or block.

- **No heap.** Every quantity is fixed-size: `Jacobian6` (6×7), the 6×6 damped
  matrix `A_`, `Vector6`, `JointVec` (7×1). The two decompositions
  (`ldlt_.compute(A_)`, once undamped for manipulability and once damped for
  the solve) reuse a single preallocated `Eigen::LDLT<Matrix<double,6,6>>`
  member — `compute()` on a fixed-size Eigen matrix factors in place, no
  dynamic storage. `Dynamics` preallocates Pinocchio's `Data` in its
  constructor, so `jacobian()` allocates nothing per call.
- **Empirically verified.** `RtSafety.JointVelocityModeTwistNoMajorFaultsSteadyState`
  drives the mode through the executor on the sim transport with a live,
  non-zero twist target — so the Jacobian, both LDLT factorizations, and the
  null-space solve run every single cycle of the measured window, not a
  zero-target short-circuit — and asserts **zero major page faults** and zero
  dropped telemetry samples in steady state.
- **Measured cost, on a Jetson AGX Orin (`benchmark_joint_velocity --sim
  --rate 1000 --duration 5`):**

  | | `--kind joint` (native, pass-through) | `--kind twist` (DLS + null-space) |
  |---|---|---|
  | `compute_ns` p50 | 64 ns | 1024 ns |
  | `compute_ns` p99 | 256 ns | 4096 ns |
  | `compute_ns` max | 1440 ns | 17279 ns |
  | overruns / faults / dropped | 0 / 0 / 0 | 0 / 0 / 0 |

  The DLS + null-space solve costs roughly **~1 µs** more per cycle at the
  median and **~4 µs** at p99 than the native pass-through path — both
  comfortably inside the 1 ms budget at 1 kHz, with no overruns, faults, or
  dropped samples in either mode. This closes the Open Decision on measuring
  `JointVelocityMode`'s per-cycle cost from the `docs/superpowers/` design spec
  for this plan.

## Saturation preserves direction

Once `qd` is computed (task plus posture), `limit()` scales it **uniformly** —
finding the single scale factor that brings the fastest-saturating joint down
to its `max_qd` cap, and applying that same factor to every joint — rather
than clamping each joint independently. A per-joint clamp would silently
rotate the achieved EE twist the moment any one joint saturates, which is
exactly the failure mode a mode named "velocity" must not have: the operator
commanded a *direction*, and saturation should make it slower, never sideways.
A hard per-joint clamp still runs afterward as a backstop for degenerate
inputs (e.g. a zero entry in `max_qd`, where the scale factor is undefined),
but in the normal case it is a no-op — the uniform scale already brought every
joint inside its cap.

## See also

- [Control Modes guide](../guide/control-modes.md) — conceptual overview.
- [Streaming guide](../guide/streaming.md#jointvelocitymode-specifics) — the
  stiffness and staleness contract for streamed velocity/twist setpoints.
- [API Reference](../reference/api.md#jointvelocitymode-joint_velocity_modeh) —
  exact signatures.
- [Deep Dive: Cartesian Impedance](impedance.md) — the sibling derivation for
  the torque-domain damped pseudo-inverse and its null-space projector.
