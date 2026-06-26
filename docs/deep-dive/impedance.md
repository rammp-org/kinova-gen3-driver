# Deep Dive: Cartesian Impedance

This page derives the Cartesian impedance control law as implemented in
`CartesianImpedanceMode`, explains the frame conventions that make it consistent,
the nullspace redundancy resolution, the safety ramp, and the real-time-safety
design. For the conceptual overview see the
[Control Modes guide](../guide/control-modes.md); for signatures, the
[API Reference](../reference/api.md).

The canonical design record is the spec and plan under `docs/superpowers/`.

## The control law

Per cycle, given joint position `q` and velocity `q̇`, a target tool pose `x_d`,
and gain matrices `Kx`, `Dx`:

```
x          = fk(q)                        # current tool pose  (SE(3))
J          = jacobian(q)                  # 6×7, LOCAL_WORLD_ALIGNED
ẋ          = J · q̇                        # tool spatial velocity (6×1)
e          = pose_error(x_d, x)           # decoupled SE(3) error (6×1)
F          = Kx ∘ e  −  Dx ∘ ẋ            # task wrench (6×1)
τ_active   = Jᵀ · F  +  τ_null            # joint torque from task + nullspace
τ          = g(q)  +  ramp(t) · τ_active  # gravity ALWAYS full
τ_out      = clamp(τ, ±τ_limit)
```

`∘` is elementwise (diagonal gains). `g(q)` is the generalized gravity torque.
This is the classic **operational-space impedance** law in its `Jᵀ`-stiffness
form: a diagonal spring–damper in task space, mapped to joint torques by the
Jacobian transpose, plus full gravity compensation. v1 does **not** shape the
apparent inertia (no `Λ = (J M⁻¹ Jᵀ)⁻¹` term) — that would require the mass
matrix and is a documented future extension; the `Jᵀ` form is the robust,
standard starting point.

## The pose error

`pose_error(x_d, x)` returns the **decoupled geometric error**

```
e = [ p_d − p ;  rotvec(R_d · R⁻¹) ]
```

— position difference in the world frame, stacked with the orientation error as a
**rotation vector** (axis·angle) of the relative rotation `R_d R⁻¹`. The
implementation takes the relative quaternion, flips it to the `w ≥ 0` hemisphere
(shortest geodesic), and converts to axis·angle. It is singularity-free for
orientation errors below π.

We deliberately use this decoupled error rather than the full `se(3)` `log6`
(which couples translation and rotation through the left Jacobian). The decoupled
form pairs naturally with a **diagonal** world-frame stiffness — independent XYZ
and rotational gains, which is what users expect to dial — and it keeps the error
metric a small Eigen-only helper, leaving `Dynamics` a pure rigid-body query unit.

## Frames — why the pieces line up {#frames}

Everything is expressed in **world-aligned axes at the tool origin**, and that
consistency is what makes the law correct without stray rotations:

- `jacobian()` uses Pinocchio's **`LOCAL_WORLD_ALIGNED`** reference frame, so
  `ẋ = J q̇` is the tool's spatial velocity (linear velocity of the origin and
  angular velocity) expressed in world-aligned axes.
- `fk()` returns the tool pose in the world frame, and `pose_error` returns the
  position error in the world frame and the orientation error as a world-frame
  rotation vector.
- The diagonal gains `Kx`, `Dx` are therefore world-frame gains: `Kx[0..2]` act
  on world X/Y/Z translation, `Kx[3..5]` on world rotation about X/Y/Z.

Because `F` lives in the same frame the Jacobian maps to/from, `Jᵀ F` is the
correct joint torque for that wrench — no extra frame transform is needed. (If you
ever wanted stiffness defined in the *tool* frame instead of world-aligned, that's
a small variant: rotate `e` and `ẋ` into the tool frame, apply the gains, rotate
`F` back — or use `LOCAL` Jacobian throughout.)

### Validating it numerically

A frame or sign mistake in `fk`/`jacobian` would be invisible to a test that
recomputes the law with the same primitives. The anchor is the **finite-difference
Jacobian test**: it perturbs each joint, recomputes `fk`, and compares the
numerical `Δpose/Δq` (linear part from position, angular part from the rotation
vector of the relative rotation) against the analytic Jacobian column — to 1e-4,
independent of Pinocchio's conventions. That test pins both `fk` and `jacobian`.

## Nullspace posture (redundancy resolution)

The Gen3 has 7 DOF but a tool pose is 6 DOF, leaving a **1-DOF redundancy** (the
elbow can swing while the tool stays put). Left alone, the elbow drifts. We add a
secondary joint-space posture torque

```
τ0 = nullspace_kp · (q_rest − q)  −  nullspace_kd · q̇
```

(a PD law pulling toward the rest posture captured at entry) and **project it into
the nullspace of the task** so it produces no tool wrench:

```
τ_null = N · τ0,        N = I − Jᵀ (Jᵀ)⁺
```

### Why `N` annihilates the task

With `Jᵀ` of full column rank (rank 6), the Moore–Penrose pseudo-inverse is
`(Jᵀ)⁺ = (J Jᵀ)⁻¹ J`, so

```
J · N = J − J Jᵀ (J Jᵀ)⁻¹ J = J − J = 0.
```

Thus any secondary torque mapped through `N` lies in `null(J)` and produces **zero
task-space motion/wrench** — the elbow moves toward `q_rest` without disturbing the
tool. `N` is also a true projector (`N² = N`). The unit test
`ProjectorAnnihilatesTaskSpace` checks both `‖J N‖ ≈ 0` and `‖N² − N‖ ≈ 0` with the
**undamped** projector (exact to machine precision at a non-singular pose);
`PostureTorqueProducesNoTaskWrench` checks the live damped path produces `‖J·Δτ‖ ≈ 0`.

### Damping for singularity robustness

Near a kinematic singularity `J Jᵀ` becomes ill-conditioned and the exact inverse
blows up. We use a **damped (Levenberg–Marquardt) pseudo-inverse**:

```
(Jᵀ)⁺ = (J Jᵀ + λ²I)⁻¹ J,     λ = pinv_damping (default 1e-3)
```

The `λ²I` keeps `J Jᵀ + λ²I` strictly positive definite for any `J`, so the
fixed-size LDLT solve is always well-defined. Damping makes the projector
*approximate* (`J N = λ²(J Jᵀ + λ²I)⁻¹ J = O(λ²)` instead of exactly 0) — an
intentional trade of a tiny task-space leak for boundedness near singularities.
The `--dry-run` tool reports the Jacobian condition number so you can see when
you're approaching one.

## The entry ramp — gravity is always full

Entering impedance with full stiffness against a small initial error would jolt
the arm. The active wrench (task term + nullspace) is therefore **ramped in**:

```
ramp(t) = min(1, t_since_enter / gain_ramp_s)        # 0 → 1 over gain_ramp_s
τ = g(q) + ramp(t) · τ_active
```

Crucially the ramp multiplies **only** `τ_active`; gravity `g(q)` is applied in
full from the very first cycle. Ramping the whole torque (gravity included) would
make the arm sag during the fade-in. The ramp uses elapsed-at-start-of-cycle then
advances by `dt`, so the first cycle after `on_enter` is pure gravity comp, and
`gain_ramp_s ≤ 0` short-circuits to full immediately. `on_enter` resets the ramp,
so re-entering the mode re-ramps cleanly.

The per-joint clamp is applied **after** the ramp. Saturation can break the exact
nullspace orthogonality — that's an accepted trade: safety (a hard torque bound)
takes precedence over the projection.

## Real-time safety

`compute()` runs on the 1 kHz RT thread and must never allocate, lock, or block.

- **No heap.** Every quantity is fixed-size: `Jacobian6` (6×7), `Vector6`,
  `JointVec` (7×1), and the nullspace temporaries `J Jᵀ` (6×6), `(Jᵀ)⁺` (6×7),
  `N` (7×7). The damped pseudo-inverse uses Eigen's **fixed-size LDLT**
  (`Matrix<double,6,6>::ldlt().solve(...)`), whose internal storage is stack-
  resident — no dynamic allocation. `Dynamics` preallocates Pinocchio's `Data` in
  its constructor, so `gravity`/`fk`/`jacobian` allocate nothing per call.
- **Empirically verified.** `RtSafety.ImpedanceModeNoMajorFaultsSteadyState` runs
  the mode (nullspace on) through the executor on the sim transport and asserts
  **zero major page faults** in the steady-state window and zero dropped telemetry
  — i.e. the whole FK + Jacobian + gravity + LDLT path touches no new memory once
  warmed up.
- **Measured cost:** p50 ≈ 2 µs, p99 ≈ 4 µs per cycle on a Jetson AGX Orin — well
  inside the 1 ms budget.

### Redundant forward passes

`fk()`, `jacobian()`, and `gravity()` each run their own kinematic-tree pass, so
calling all three per cycle repeats the forward pass 2–3×. At 7-DOF and the cost
above this is comfortably affordable, so the methods stay **granular** (clean,
independently testable). A fused single-pass `evaluate(q, q̇) → {pose, J, g}` is a
documented future optimization if profiling ever demands it.

## Live targets & gains without races

`set_target` and `set_gains` are called from a non-RT supervisor thread while
`compute()` reads on the RT thread. They publish via **lock-free double-buffers**:
the writer fills the inactive slot, then flips an atomic index with a release
store; the reader acquire-loads the index and takes a **value snapshot** of the
slot. `compute()` copies the gains snapshot once at the top of the cycle rather
than holding a reference, so a burst of `set_gains` calls can never tear a read
mid-cycle.

One subtlety: the "is there an external target?" flag is written by both
`set_target` (non-RT, sets true) and `on_enter` (RT, sets false on entry). Both are
release-stores to an `atomic<bool>`, and the target payload is always published
(release) *before* the flag flips, so the reader's acquire-load sees a consistent
target. The race is benign: at worst the loop uses the entry-hold pose for one
extra cycle. This is exactly the seam a future front-end/IPC server plugs into,
where the single non-RT writer is the server thread.

## See also

- [Control Modes guide](../guide/control-modes.md) — conceptual overview & tuning.
- [API Reference](../reference/api.md) — exact signatures.
- `docs/superpowers/specs/2026-06-16-cartesian-impedance-controller-design.md` —
  the design spec.
- `docs/superpowers/plans/2026-06-16-cartesian-impedance-controller.md` — the
  task-by-task implementation plan.
