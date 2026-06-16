# Cartesian Impedance Controller — Design Spec

**Date:** 2026-06-16
**Status:** Approved for planning
**Scope:** A Cartesian (task-space) impedance `ControlMode` for the Gen3 7-DOF,
plus the `Dynamics` extensions it needs (forward kinematics + frame Jacobian).
Everything stays behind the existing `ControlMode` / `Dynamics` interfaces — no
changes to `RtExecutor` or `Transport`.

## Goal

Give application developers a compliant task-space controller: the arm behaves
like a 6-DOF spring-damper at its tool frame. Push it and it springs back toward
a commanded pose; release and it settles. This is the headline new control mode
that a (later) front-end API will expose.

## Out of scope (explicitly deferred)

- **Front-end / web server / IPC** — separate sub-project, designed later. This
  spec only produces a `ControlMode` and the math it needs.
- **Inertia shaping** (desired apparent mass via `Λ = (J M⁻¹ Jᵀ)⁻¹`) — would
  need the mass matrix. v1 uses the robust `Jᵀ`-stiffness form. Future extension.
- **Other modes** (joint impedance, high-speed velocity) — slot in later as
  additional `ControlMode`s with no interface change.

## Approved design decisions

1. **Cartesian (task-space) impedance**, not joint-space.
2. **Controlled frame defaults to `gen3_end_effector_link`** (Kinova's tool/flange
   frame), configurable by name and validated at construction.
3. **Nullspace posture term on by default** (the 7-DOF arm has a 1-DOF
   redundancy under a 6-DOF task; without it the elbow drifts).
4. **No inertia shaping in v1.**
5. **Decoupled geometric pose error** (position in world + orientation as a
   rotation vector), not the full se(3) `log6` — pairs naturally with diagonal
   world-frame stiffness and keeps the error metric out of `Dynamics`.

---

## Component 1 — `Dynamics` extension

### Governing invariant

`Dynamics` remains the **sole owner of Pinocchio**: the only translation unit
that includes a Pinocchio header. Its public surface speaks only our own
Eigen-based value types. Eigen is already a public dependency (`joint_types.h`);
Pinocchio stays behind the pimpl.

### New value types — `include/kinova_lowlevel/cartesian_types.h` (Eigen-only)

```cpp
using Vector6   = Eigen::Matrix<double, 6, 1>;             // spatial: [vx vy vz | wx wy wz]
using Jacobian6 = Eigen::Matrix<double, 6, kNumJoints>;    // 6×7 frame Jacobian
struct Pose { Eigen::Vector3d p; Eigen::Quaterniond R; };  // SE(3), fixed-size, no heap
```

POD/fixed-size, no Pinocchio types — consumers never pull in Pinocchio.

### New public methods on `Dynamics` (`gravity` unchanged)

```cpp
explicit Dynamics(const std::string& urdf_path,
                  const std::string& ee_frame = "gen3_end_effector_link");

Pose fk(const JointVec& q);                          // pose of the controlled frame
void jacobian(const JointVec& q, Jacobian6& J_out);  // out-param: caller owns storage
```

`jacobian` uses an out-param (like `gravity`) to fill caller-owned, pre-allocated
storage — no allocation in the RT path. `fk` returns a fixed-size `Pose` by value.

### Internal refactor — one config-packing helper

The flat-angle → Pinocchio-config packing (the `cos/sin` continuous-joint
handling currently inlined in `gravity()` at `dynamics.cpp:34-39`) is extracted
into one private method:

```cpp
void Impl::pack(const JointVec& q);   // fills this->qcfg from q, handling nqs==2 joints
```

`gravity`, `fk`, and `jacobian` all call `pack(q)` first. The subtle
continuous-joint packing then lives in exactly one place; the three methods can
never disagree about the configuration. The existing continuous-joint
round-trip test keeps this green.

### Implementation per method

- **`fk`:** `pack(q)` → `forwardKinematics(model, data, qcfg)` →
  `updateFramePlacement(model, data, frame_id_)` → read `data.oMf[frame_id_]`
  (`SE3`) → copy `.translation()` and `Quaterniond(.rotation())` into our `Pose`.
- **`jacobian`:** `pack(q)` →
  `computeFrameJacobian(model, data, qcfg, frame_id_, LOCAL_WORLD_ALIGNED, J_out)`.
  **`LOCAL_WORLD_ALIGNED`** expresses the spatial velocity in world-aligned axes
  at the EE origin — the same frame the diagonal world-frame stiffness `Kx`
  lives in, so `ẋ = J·qd` and `tau = Jᵀ·F` are consistent without extra rotations.

**Efficiency note:** `fk`, `jacobian`, and `gravity` each run a kinematic-tree
pass, so calling all three per cycle repeats the forward pass 2–3×. At 7-DOF that
is single-digit µs (gravity alone measured ~2 µs), well inside the 1 kHz budget.
Methods stay **granular** (clean, independently testable). A fused
`evaluate(q, qd) → {pose, J, g}` single-pass method is a documented future
optimization *if* the benchmark shows it matters — YAGNI for now.

### Constructor footgun guard

After the existing `nv == kNumJoints` check, resolve
`frame_id_ = model.getFrameId(ee_frame)` and **throw if `!model.existFrame(ee_frame)`**
— same hard-fail philosophy as the `nv` guard. A typo'd or missing frame fails
loudly at startup, never silently controls the wrong point.

### RT-safety

Every Pinocchio algorithm used (`computeGeneralizedGravity`, `forwardKinematics`,
`updateFramePlacement`, `computeFrameJacobian`) operates on the pre-allocated
`Data` — no heap allocation after the ctor. New members are `frame_id_` (int)
and the reused `qcfg`; outputs are fixed-size. The RT-invariant is preserved and
covered by the RT-safety test (below).

---

## Component 2 — pose-error helper (`cartesian.h`, Eigen-only, no Pinocchio)

```cpp
Vector6 pose_error(const Pose& desired, const Pose& current);
// returns [ p_d − p ;  rotvec(R_d · R⁻¹) ]   — decoupled geometric error
```

Orientation error is the rotation vector (axis·angle) of the relative rotation
`R_d · R⁻¹`, computed via Eigen quaternion → `AngleAxis`. Singularity-free for
orientation errors below π. Lives with the control law, independently testable,
and swappable without touching `Dynamics`.

---

## Component 3 — `CartesianImpedanceMode : ControlMode`

### Parameters

```cpp
struct CartesianImpedanceParams {
  Vector6  Kx = (Vector6() << 300,300,300, 30,30,30).finished();   // N/m | N·m/rad
  Vector6  Dx = (Vector6() << 35,35,35, 5,5,5).finished();         // N·s/m | N·m·s/rad
  double   nullspace_kp   = 5.0;     // joint-space posture stiffness (N·m/rad)
  double   nullspace_kd   = 1.0;     // joint-space posture damping (N·m·s/rad)
  bool     nullspace_on   = true;
  double   torque_limit   = 39.0;    // per-joint clamp (N·m)
  double   gain_ramp_s    = 0.5;     // ramp gains 0→full over this window on entry
};
```

(Default gain values are starting points to be tuned on hardware; the static
sim tests assert *behavior/sign*, not specific numbers.)

### Live setpoint / gains (RT-safe publish)

The mode holds a target pose and a posture, settable from a non-RT thread:

```cpp
void set_target(const Pose& x_d);          // desired tool pose
void set_gains(const CartesianImpedanceParams& p);
```

These publish a new parameter block via an atomic-pointer swap into a
double-buffer (same lock-free pattern `RtExecutor` uses for mode handoff);
`compute()` reads the current block once per cycle. This is the seam the future
server plugs into; until then the demo app / tests call it directly. No
allocation in `compute()`.

### `on_enter(fb)`

- Capture the **current** tool pose `fk(fb.q)` as the initial target (the arm
  holds where it is — never jumps).
- Capture the current joint configuration as the nullspace rest posture.
- Start the gain ramp at 0 (gains scale from 0→1 over `gain_ramp_s`) to avoid a
  torque jolt at entry.

### `compute(fb, dt, out)` — the control law

```
x   = dyn.fk(fb.q)
J   = dyn.jacobian(fb.q)          // 6×7, LOCAL_WORLD_ALIGNED
xd  = J * fb.qd                    // 6×1 spatial velocity
e   = pose_error(x_d, x)           // 6×1 decoupled error
F   = Kx.cwiseProduct(e) + Dx.cwiseProduct(-xd)        // 6×1 task wrench
tau = Jᵀ * F + dyn.gravity(fb.q)
if (nullspace_on)
    tau += N * ( nullspace_kp*(q_rest - fb.q) - nullspace_kd*fb.qd )
tau *= ramp(dt)                    // entry ramp 0→1
out.mode = kTorque
out.torque = clamp(tau, ±torque_limit)
```

- **Nullspace projector** `N = I − Jᵀ (Jᵀ)⁺`, with `(Jᵀ)⁺` a **damped**
  (Levenberg-Marquardt) pseudo-inverse so the term stays bounded near kinematic
  singularities. The pseudo-inverse is Eigen-only and computed in the mode
  (fixed-size, no heap).
- `required_modes()` → all 7 actuators in `kTorque`.
- Position passthrough: as with gravity-comp, the command also carries the
  measured position so the robot's low-level safety can hold on a following-error
  fault (matches the existing torque-mode convention in `GravityCompTorqueMode`).

### Safety

- Entry gain ramp (no jolt).
- Per-joint torque clamp.
- Damped pseudo-inverse bounds the nullspace term near singularities.
- Default target = entry pose, so the controller is a compliant *hold* until a
  client commands otherwise — the safe default.

---

## Component 4 — demo / benchmark app

A small app (`apps/benchmark_cartesian_impedance.cpp`, mirroring
`benchmark_grav_comp.cpp`) to exercise the mode in sim and (later, attended) on
hardware:

- `--sim` drives `SimTransport`; `--ip` is the real path (compiled only with
  KORTEX, never run unattended).
- Holds the entry pose with compliance (zero-displacement impedance — push the
  arm, it springs back).
- CLI knobs for `Kx`/`Dx`/nullspace gains/torque limit; reuses the existing
  telemetry + RT-tuning machinery and reports the same cycle/compute histograms
  (so we benchmark impedance compute cost the same way as gravity-comp).
- `--dry-run` read-only variant (as gravity-comp has) prints `fk(q)`, the
  Jacobian condition number, and the would-be wrench without commanding torque,
  for pre-torque validation against the real arm.

---

## Testing (all sim / no robot)

- **FK:** at `neutral()` and a couple of known configs, assert the tool pose
  against hand-verified Pinocchio reference values.
- **Jacobian:** validate analytic `J` against a **finite-difference** of `fk`
  (perturb each joint, compare `Δpose/Δq` columns) — catches frame/reference and
  packing errors.
- **pose_error:** identical poses → zero; pure translation and pure rotation
  cases → expected vector; rotation error sign/axis correct.
- **CartesianImpedanceMode behavior:** at the target pose with zero velocity the
  task wrench is ~0 (only gravity in `tau`); a known pose displacement produces a
  restoring wrench of the correct sign on the expected axes; nonzero velocity
  produces a damping wrench opposing motion.
- **Nullspace:** `N·Jᵀ ≈ 0` (projector annihilates task-space directions);
  nullspace term does not perturb the commanded task wrench.
- **Entry:** `on_enter` sets target to the entry pose (error ≈ 0 on the first
  cycle); ramp scales torque from ~0 upward.
- **RT-safety:** run `RtExecutor` + `CartesianImpedanceMode` on `SimTransport`;
  assert zero major page faults in steady state and zero dropped telemetry
  samples (extends the existing RT-safety test).

## File touch list

- **New:** `include/kinova_lowlevel/cartesian_types.h`,
  `include/kinova_lowlevel/cartesian.h` + `src/cartesian.cpp` (pose_error),
  `include/kinova_lowlevel/cartesian_impedance_mode.h` +
  `src/cartesian_impedance_mode.cpp`,
  `apps/benchmark_cartesian_impedance.cpp`, tests under `tests/`.
- **Modified:** `include/kinova_lowlevel/dynamics.h` + `src/dynamics.cpp`
  (fk/jacobian/pack/frame ctor), `CMakeLists.txt` (new sources + test + app).
- **Unchanged:** `transport.*`, `rt_executor.*`, `control_mode.h`, `telemetry*`.
