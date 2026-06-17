# Control Modes

This driver separates **communication** (`Transport`) from **computation**
(`ControlMode`). A control mode is a pure, RT-safe control law: each cycle the
`RtExecutor` hands it the latest joint feedback and a `dt`, and it fills a
`JointCommand`. Swapping the control law never touches the transport or the RT
machinery — modes are the unit you add to give the arm a new behavior.

Two modes ship today:

| Mode | Header | What it does |
|---|---|---|
| `GravityCompTorqueMode` | `gravity_comp_mode.h` | Cancels gravity so the arm is weightless/back-drivable. |
| `CartesianImpedanceMode` | `cartesian_impedance_mode.h` | Makes the tool behave like a 6-DOF spring-damper about a target pose. |

Everything is in SI units / radians. Conversions to KORTEX's degrees/N·m happen
only at the transport boundary.

---

## The `ControlMode` interface

```cpp
class ControlMode {
 public:
  virtual ActuatorModes required_modes() const = 0;             // per-joint actuator mode
  virtual void on_enter(const JointFeedback&) = 0;              // called once when adopted
  virtual void compute(const JointFeedback& fb, double dt_s,    // RT-safe, every cycle
                       JointCommand& out) = 0;
  virtual void on_exit() = 0;
};
```

- **`required_modes()`** tells the transport which actuator mode each joint needs
  (e.g. all `kTorque`). The executor sets these before the loop starts.
- **`on_enter(fb)`** runs once, on the RT thread, when the executor adopts the
  mode — the place to capture the entry state (both modes here use it to "hold
  where you are").
- **`compute(fb, dt_s, out)`** is the control law. It **must be RT-safe**: no heap
  allocation, no locks, no blocking. All scratch is preallocated.
- Both shipped modes also write `out.position = fb.q` (position passthrough), so
  the robot's low-level safety can fall back to a position hold if it ever flags a
  following-error fault while in torque mode.

A mode is handed to the executor with `RtExecutor::request_mode(&mode)`; the swap
is an atomic-pointer handoff adopted at a cycle boundary. The caller owns the
mode and must keep it alive while it's active.

---

## `Dynamics` — the rigid-body math the modes use

`Dynamics` is the **only** unit that includes Pinocchio. It loads the URDF once,
preallocates, and exposes three RT-safe queries plus model introspection:

```cpp
explicit Dynamics(const std::string& urdf_path,
                  const std::string& ee_frame = "gen3_end_effector_link");

void gravity(const JointVec& q, JointVec& tau_out);   // generalized gravity torque
Pose fk(const JointVec& q);                            // pose of ee_frame (world frame)
void jacobian(const JointVec& q, Jacobian6& J_out);    // 6x7, LOCAL_WORLD_ALIGNED
int nv() const;  int nq() const;
```

- **End-effector frame.** `ee_frame` defaults to `gen3_end_effector_link` (the
  Kinova tool flange). It is **validated at construction** — an unknown frame
  throws (no silent mis-control). Pass a different frame name to control a
  different point.
- **Frame convention.** `jacobian()` uses Pinocchio's **`LOCAL_WORLD_ALIGNED`**
  reference frame: the spatial velocity `J·q̇` is expressed in **world-aligned
  axes at the EE origin**. `fk()` returns the EE pose in the world frame. This is
  the same frame the Cartesian stiffness gains live in, so the impedance law needs
  no extra rotations.
- **Continuous joints.** The Gen3's continuous joints are packed faithfully as
  `(cos θ, sin θ)` internally (`nq > nv`); the public API stays a flat 7-vector of
  angles. There is no wide-limit-revolute hack.
- **Footgun guard.** The ctor throws if the URDF's `nv != 7` (wrong model) — fail
  loud at startup rather than silently mis-map joints.
- **Cost.** `fk`, `jacobian`, and `gravity` each run one kinematic pass, so a mode
  that calls all three repeats the forward pass 2–3× per cycle. At 7-DOF this is
  single-digit µs (see the benchmark below), well inside the 1 kHz budget. A fused
  single-pass `evaluate()` is a documented future optimization if profiling ever
  demands it.

### Pose error helper

`pose_error()` (in `cartesian.h`, Eigen-only — no Pinocchio) gives the
**decoupled geometric SE(3) error** used by the impedance law:

```cpp
Vector6 pose_error(const Pose& desired, const Pose& current);
//   [ p_d − p ;  rotvec(R_d · R⁻¹) ]
```

Position error in the world frame; orientation error as a rotation vector (axis·
angle) of the relative rotation, shortest-path. Singularity-free for orientation
errors below π. It pairs naturally with diagonal world-frame stiffness — i.e.
independent XYZ + rotational gains.

---

## `GravityCompTorqueMode`

```cpp
struct GravityCompParams { double scale = 1.0; double damping = 0.0; double torque_limit = 39.0; };
GravityCompTorqueMode(Dynamics& dyn, GravityCompParams p = {});
```

Law: `tau = scale·gravity(q) − damping·q̇`, clamped to `±torque_limit`.

- `scale = 1.0` fully cancels gravity (arm floats); `scale = 0.5` half-compensates
  (the arm sags gently — useful as a first cautious hardware check of the torque
  sign/magnitude before trusting full compensation).
- `damping` adds light joint-velocity damping to take the edge off.
- Requires a URDF that matches the **mounted hardware** including any tool —
  running the bare-arm URDF with a gripper attached under-compensates and the arm
  sags. Use `models/gen3_7dof_2f85.urdf` for the Robotiq 2F-85.

---

## `CartesianImpedanceMode`

Makes the tool frame behave like a 6-DOF spring-damper about a target pose: push
the arm and it springs back; release and it settles. This is the headline mode
application developers build on.

### The control law

```
x         = fk(q)                                  // current tool pose
J         = jacobian(q)                            // 6x7, LOCAL_WORLD_ALIGNED
ẋ         = J · q̇                                  // tool spatial velocity
e         = pose_error(x_d, x)                      // decoupled SE(3) error
F         = Kx ∘ e − Dx ∘ ẋ                         // 6-D task wrench
tau_active = Jᵀ · F + nullspace_term               // (nullspace optional)
tau        = gravity(q) + ramp · tau_active         // gravity ALWAYS full
out.torque = clamp(tau, ±torque_limit)
```

The wrench is a diagonal spring (`Kx`) plus damper (`Dx`) in world-aligned task
axes, mapped to joint torques by `Jᵀ`. Gravity compensation is added in full and
is **never** scaled by the ramp, so the arm holds itself up from the first cycle.

### Parameters

```cpp
struct CartesianImpedanceParams {
  Vector6 Kx = {300,300,300, 30,30,30};   // stiffness: N/m (xyz) | N·m/rad (rpy)
  Vector6 Dx = {35,35,35,  5,5,5};        // damping:   N·s/m     | N·m·s/rad
  double  nullspace_kp = 5.0;             // posture stiffness (N·m/rad)
  double  nullspace_kd = 1.0;             // posture damping  (N·m·s/rad)
  bool    nullspace_on = true;
  double  pinv_damping = 1e-3;            // λ for the damped pseudo-inverse
  double  torque_limit = 39.0;            // per-joint clamp (N·m)
  double  gain_ramp_s  = 0.5;             // fade-in window for the active wrench
};
```

Defaults are sensible starting points; **tune `Kx`/`Dx` on hardware**. The
`Vector6` layout is `[x y z | rx ry rz]` (translation over rotation).

### Nullspace posture (redundancy resolution)

A 7-DOF arm controlling a 6-DOF task has a 1-DOF redundancy — without handling it,
the elbow drifts. With `nullspace_on` (default), a low-gain joint-space posture
term `nullspace_kp·(q_rest − q) − nullspace_kd·q̇` is projected into the Jacobian
nullspace by `N = I − Jᵀ(Jᵀ)⁺` (a **damped** Levenberg-Marquardt pseudo-inverse,
`pinv_damping = λ`, so it stays bounded near kinematic singularities). The
projection guarantees the posture term produces **no task-space wrench** — it only
moves the redundant elbow toward the rest posture captured at entry. Set
`nullspace_on = false` to disable.

### Entry behavior & safety

- **`on_enter`** captures the current tool pose as the target (the arm **holds
  where it is** — never jumps to some default) and the current joint config as the
  nullspace rest posture.
- **Gain ramp.** The active wrench (stiffness + nullspace) fades in `0→1` over
  `gain_ramp_s`; gravity is full immediately. This avoids a torque jolt at entry
  while never letting the arm sag.
- **Torque clamp.** Every joint is clamped to `±torque_limit`. Saturation can
  break the exact nullspace orthogonality — safety takes precedence over the
  projection.
- **Defaults to a compliant hold**, so the controller is safe-by-default until a
  client commands a new target.

### Live targets & gains (the future-server seam)

```cpp
void set_target(const Pose& x_d) noexcept;          // desired tool pose
void set_gains(const CartesianImpedanceParams& p) noexcept;
```

These are **non-RT setters** meant to be called from a single supervisor thread.
They publish into lock-free double-buffers that `compute()` reads as a value
snapshot once per cycle — so a slow or bursty caller can never stall or tear the
RT loop. This is exactly the seam a future front-end/IPC server plugs into; until
then the demo app and tests call them directly. Until a target is set, the mode
holds the entry pose.

---

## Running the impedance benchmark

`benchmark_cartesian_impedance` runs the mode through the `RtExecutor` and reports
per-cycle timing — the same telemetry machinery as the gravity-comp benchmark. It
is **sim by default** (`--sim`); the real-robot `--ip` path is compiled in only
for the KORTEX build and must never be run unattended.

```sh
# sim (no robot) — what good output looks like
./benchmark_cartesian_impedance --sim --urdf ../models/gen3_7dof_2f85.urdf \
  --rate 1000 --duration 5
```

Useful flags: `--rate`, `--pacing sleepspin|nanosleep`, `--cpu`, `--rt-priority`,
`--duration`, `--csv`, and the gain knobs `--kx-trans`, `--kx-rot`, `--dx-trans`,
`--dx-rot`, `--nullspace-kp`, `--nullspace-kd`, `--gain-ramp-s`, `--pinv-damping`,
`--torque-limit`, `--no-nullspace`.

**Measured compute cost** (sim, Jetson AGX Orin): the full FK + Jacobian + gravity
+ nullspace-projection law runs at **p50 ≈ 2 µs, p99 ≈ 4 µs** per cycle, with
`dropped = 0` and zero major page faults — i.e. it allocates nothing in the RT
loop and fits comfortably in the 1 kHz budget. (The RT-safety test
`RtSafety.ImpedanceModeNoMajorFaultsSteadyState` asserts the zero-allocation
property with the nullspace path active.)

### `--dry-run` (read-only pre-torque validation)

Before commanding any torque on real hardware, `--dry-run` connects and **reads
feedback only** — it never enters low-level servoing and never commands torque. It
prints the tool pose `fk(q)`, the orientation quaternion, and the Jacobian
condition number while you move the arm by pendant, so you can confirm the URDF /
frame match the real arm first.

```sh
./benchmark_cartesian_impedance --ip 192.168.1.10 --dry-run \
  --urdf ../models/gen3_7dof_2f85.urdf --duration 0     # attended only
```

> Hardware runs are **attended-only**. See
> [`integration-runbook.md`](integration-runbook.md) for the full bring-up
> sequence, and start every new arm/tool with `--dry-run` before torque.

---

## Adding a new control mode

Velocity, joint-impedance, and other laws slot in the same way — no transport,
executor, or RT changes:

1. Implement `ControlMode` in a new `*_mode.h`/`.cpp`. Capture entry state in
   `on_enter`; keep `compute()` allocation-free (preallocate all scratch as
   members; use fixed-size Eigen types).
2. Get whatever rigid-body quantities you need from `Dynamics` (extend it if a new
   quantity like the mass matrix is required — it's the one place Pinocchio lives).
3. Add unit tests that recompute the law independently (see
   `cartesian_impedance_mode_test.cpp` — the `MatchesIndependentlyComputedLaw`
   oracle pattern) and extend the RT-safety test to assert zero major faults with
   your mode active.
4. Add the source to `KINOVA_LIB_SOURCES` and the test to `unit_tests` in
   `CMakeLists.txt`.

See the design spec and plan under `docs/superpowers/` for the Cartesian impedance
controller as a worked example.
