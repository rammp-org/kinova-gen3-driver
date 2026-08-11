# Control Modes

A **control mode** is the driver's unit of "what the arm does." It's a pure,
real-time-safe control law: each 1 kHz cycle the executor hands it the latest
joint feedback and a `dt`, and it fills in a joint command. Swapping the control
law never touches the transport or the RT machinery — modes are how you give the
arm a new behavior.

This page explains the three shipped modes conceptually. For exact signatures see
the [API Reference](../reference/api.md); for the impedance math and RT-safety
design see the [Deep Dive](../deep-dive/impedance.md).

## How a mode fits in

Every mode implements the same small interface (`ControlMode`):

- **`required_modes()`** — which actuator mode each joint needs (e.g. all torque).
  The executor applies these before the loop starts.
- **`on_enter(feedback)`** — runs once, on the RT thread, when the mode is adopted.
  Every shipped mode uses it to "hold where you are" by capturing the entry state.
- **`compute(feedback, dt, command)`** — the control law, every cycle. It must be
  RT-safe: no allocation, no locks, no blocking.
- **`on_exit()`** — cleanup hook.

You hand a mode to the executor with `request_mode(&mode)`; it's adopted at the
next cycle boundary (an atomic-pointer swap). You keep ownership — the mode must
outlive its time as the active mode.

Every shipped mode also echoes the measured joint position back in the command
(*position passthrough*), so the robot's own low-level safety can fall back to a
position hold if it ever flags a following-error fault while in torque mode.

The two impedance modes additionally implement `PoseTargetSink` (a single
`set_target(Pose)` method), so anything that drives the arm from a stream of
Cartesian targets — the teleop server, for one — works against either without
knowing which is live.

Everything is **SI units / radians**. Degrees and N·m conversions happen only at
the transport boundary.

## Gravity Compensation — `GravityCompTorqueMode`

Cancels the arm's own weight so it becomes weightless and back-drivable — push it
and it stays where you leave it.

- **Law (conceptually):** command the joint torques that exactly oppose gravity,
  optionally scaled, with light velocity damping, clamped to a torque limit.
- **`scale`** — fraction of gravity to apply. `1.0` floats the arm; `0.5` makes it
  sag gently. Start at `0.5` on new hardware: a gentle, predictable sag confirms
  the torque sign and magnitude before you trust full compensation.
- **`damping`** — adds a little joint-velocity damping to take the edge off.
- **When to use it:** teaching/lead-through, a safe first hardware bring-up, or as
  the gravity term other torque laws build on.

The single biggest correctness requirement is that the **URDF matches the mounted
hardware**, including any tool. The bare-arm model with a gripper attached
under-compensates by the tool's weight and the arm sags — use
`models/gen3_7dof_2f85.urdf` for the Robotiq 2F-85.

## Cartesian Impedance — `CartesianImpedanceMode`

Makes the **tool frame** behave like a 6-DOF spring-damper about a target pose:
push the arm away and it springs back; release and it settles. This is the
headline mode that application developers build on.

- **Law (conceptually):** a stiffness pulls the tool toward the target pose and a
  damper resists tool velocity, producing a 6-D wrench (force + torque) that is
  mapped to joint torques. Gravity compensation is added on top so the arm holds
  itself up. (The exact equations and frames are in the
  [Deep Dive](../deep-dive/impedance.md).)
- **Stiffness `Kx` / damping `Dx`** — 6-vectors laid out `[x y z | rx ry rz]`:
  three translational and three rotational gains, in world-aligned axes at the
  tool. Higher `Kx` = stiffer spring. Defaults are reasonable starting points;
  **tune on hardware.**
- **Holds the entry pose by default.** On entry it captures the current tool pose
  as the target, so the arm holds where it is and never jumps. Call `set_target`
  to move the target (the arm springs toward it) and `set_gains` to retune — both
  are safe to call live from a non-RT thread.
- **Nullspace posture (on by default).** A 7-DOF arm doing a 6-DOF task has one
  redundant degree of freedom — without handling it, the elbow drifts. A gentle
  joint-space posture term holds a rest posture *without disturbing the tool*
  (it's projected into the task's nullspace). Turn it off with `nullspace_on =
  false`; tune with `nullspace_kp` / `nullspace_kd`.
- **Entry ramp.** The spring/damper fade in over `gain_ramp_s` (default 0.5 s) so
  there's no torque jolt at the moment you enter the mode — while gravity is
  applied in full from the very first cycle, so the arm never sags during fade-in.
- **Torque limit.** Every joint is clamped to `±torque_limit` as a hard safety
  bound.
- **When to use it:** compliant interaction, contact tasks, assembly,
  hand-guiding with a virtual fixture, anything where you want the tool to be
  springy about a commanded pose rather than rigidly position-controlled.

### Tuning starting points

| Knob | Default | Effect / how to tune |
|---|---|---|
| `Kx` (trans / rot) | 300 N/m / 30 N·m/rad | Spring stiffness. Raise for tighter tracking, lower for softer compliance. Too high → buzzing/instability. |
| `Dx` (trans / rot) | 35 / 5 | Damping. Raise if the spring oscillates; roughly scale with √`Kx`. |
| `nullspace_kp` / `kd` | 0 / 8 | How firmly the elbow holds its rest posture. `kp = 0` means pure damping — resist null-space motion without snapping back. Keep modest. |
| `gain_ramp_s` | 0.5 s | Longer = gentler entry. |
| `torque_limit` | 39 N·m | Lower it for a more conservative cap during bring-up. |

## Joint-Space Impedance — `JointImpedanceMode`

Same idea as Cartesian impedance, but the spring lives in **joint space**: the
mode solves IK for the commanded tool pose to get a full 7-joint reference
configuration, then runs an independent spring-damper on every joint.

- **Why it exists.** Cartesian impedance commands a 6-DOF task with a 7-DOF arm,
  so one degree of freedom is never commanded — the null-space term biases it but
  cannot constrain it. Over a teleop session the elbow drifts, and near limits or
  singularities it reaches configurations the operator can't undo by moving the
  controller. Solving IK moves redundancy resolution to the *reference*: the arm
  configuration is chosen deterministically instead of drifting.
- **Law (conceptually):** each cycle, a bounded IK step nudges the reference
  configuration toward the commanded pose (warm-started from last cycle, so
  solutions stay continuous). Then per-joint stiffness pulls the measured
  configuration toward that reference and per-joint damping resists joint
  velocity. Gravity compensation is added on top, always in full.
- **Damping is derived, not dialed in.** `Dq_i = 2·zeta·sqrt(Kq_i · M_ii(q))`,
  using the joint-space mass matrix. A flat damping vector cannot be right at more
  than one configuration: on this arm the effective inertia at joint 1 swings ~38x
  between extended (0.015 kg·m²) and elbow-up (0.573), so any constant is far too
  high at one end and too low at the other. You set a damping *ratio* (`zeta`) and
  it stays honest as the arm moves and as you retune `Kq`.
- **The tradeoff.** Tool-frame stiffness is no longer what you dial in — it
  becomes `J⁻ᵀ Kq J⁻¹`, which varies with configuration and is not diagonal in the
  task frame. For teleop that's usually the right trade: predictable posture beats
  a precisely shaped compliance ellipsoid. For contact-rich tasks where you care
  about *how* the tool yields along each axis, prefer Cartesian impedance.
- **Redundancy resolution.** Inside the IK, a null-space posture bias pulls toward
  `ik.q_rest` (set it to your preferred elbow-up config) and a limit-avoidance
  term pushes away from hard stops; the reference is then hard-clamped to the URDF
  limits. Continuous joints (1/3/5/7) are unlimited and their posture error wraps
  to ±π, so a rest angle near ±π never drives a joint the long way round.
- **Leash (`max_tracking_error`).** Caps the position error the spring sees, so
  spring torque saturates at `Kq × leash`. Unlike the total torque clamp, this
  leaves gravity compensation untouched — the arm can't sag when the spring
  saturates. This is your main "how hard will it shove" safety dial.
- **Reference speed limit (`max_ref_speed`).** Bounds how fast the reference may
  move, so a teleop pose jump (tracking glitch, clutch re-engage) ramps in instead
  of slamming. Per-joint, seeded from the URDF velocity limits — set below hand
  speed it silently accumulates lag that only unwinds when the operator slows.
- **If it feels mushy / disconnected from your hand**, that is velocity-
  proportional tracking lag, not softness. With gravity compensated, holding a
  joint at speed `qdot` needs `Kq·lag = Dq·qdot`, so `lag = 2·zeta·qdot·sqrt(M/Kq)`.
  Reach for `zeta` first (lag scales linearly with it), then `Kq` (only
  `1/sqrt`), then check `max_ref_speed` is not binding.
- **Per-joint torque limits.** Defaults `(39, 39, 39, 39, 9, 9, 9)` N·m, matching
  the URDF — the wrist joints are rated 9 N·m, not 39.
- **When to use it:** VR/Quest teleop and any pose-streaming application where the
  arm keeps wandering into awkward configurations.

### Tuning starting points

| Knob | Default | Effect / how to tune |
|---|---|---|
| `Kq` | 80 ×4, 30 ×3 (N·m/rad) | Joint stiffness. Raise for tighter tracking; too high → buzzing. Wrist joints are weaker, keep them lower. |
| `zeta` | 0.5 | Damping **ratio**, not damping. `1.0` = critically damped; lower = livelier and more overshoot. |
| `max_ref_speed` | URDF velocity limits | Per-joint reference rate cap. Seeded from the model so it can never sit silently below what the hardware can do. |
| `max_tracking_error` | 0.35 rad | The leash. Lower it to make the arm gentler when it's far from the reference. |
| `ik.q_rest` | elbow-up placeholder | **Tune on hardware** — this is the posture the arm defaults to. |
| `ik.posture_gain` | 0.15 | How strongly the elbow returns to `q_rest`. Raise if it still wanders; lower if it fights you. |
| `ik.max_iters` | 4 | IK iterations per cycle. Per-cycle cost is **unmeasured** — see [issue #6](https://github.com/rammp-org/kinova-gen3-driver/issues/6). Nothing has timed this path, in sim or on hardware. |

> The gains above were tuned by feel during live teleop, not measured. Treat them
> as a working starting point rather than characterised values —
> [issue #6](https://github.com/rammp-org/kinova-gen3-driver/issues/6).

## Joint-Space Position — `JointPositionMode`

Commands every actuator in `kPosition` and lets the actuator's own servo close
the loop. Runs **no dynamics at all** — no gravity term, no mass matrix, no IK —
which makes it the cheapest control path in the driver (`compute p50 = 128 ns`
in sim, against a 1 ms budget).

Driven through `JointTargetSink::set_target(const JointVec&)`, the joint-space
counterpart of `PoseTargetSink`: a caller that already knows the configuration it
wants does not have to invent a Cartesian pose and pay for an IK solve.

**There is no compliance.** The arm will not yield to contact — it will push
through until the actuator faults. Use it to move to known configurations and to
exercise a target path; use joint-space impedance when a human is in the loop.

What the mode owns is the reference integrator, and every stage is a guard:
rate limit (`max_ref_speed·dt`), a following-error leash against the *measured*
position, continuous-joint wrapping, then a position-limit clamp.

| Knob | Default | Effect |
|---|---|---|
| `max_ref_speed` | 0.5 rad/s | Deliberately below the URDF limits (1.40 / 1.22). Finite requests are clamped **down** to the URDF value, non-finite ones seeded from it — no config can outrun the hardware rating. |
| `max_following_error` | 0.35 rad | How far the reference may lead the measured position. Bounds the snap when a blocked arm comes free. `<= 0` disables. |
| `q_lower` / `q_upper` | URDF limits | Software position limits. A tighter caller-supplied value survives; non-finite entries are seeded from the model. |

Hardware validation procedure:
[`../integration/joint_position_hardware_check.md`](../integration/joint_position_hardware_check.md).

## Choosing a mode

| You want… | Mode |
|---|---|
| The arm weightless / back-drivable | Gravity compensation |
| A safe, predictable first hardware test | Gravity compensation at `scale 0.5` |
| The tool to be compliant about a target pose | Cartesian impedance |
| Online retargeting / regrasping with compliance | Cartesian impedance + `set_target` |
| Precisely shaped tool-frame compliance for contact | Cartesian impedance |
| Teleop that keeps drifting into awkward elbow poses | Joint-space impedance |
| Every joint constrained, posture fully predictable | Joint-space impedance |
| To move to a known joint configuration, rigidly | Joint-space position |
| A target for another layer to test against | Joint-space position |
| The cheapest possible control path | Joint-space position |

## Adding your own mode

Velocity control, admittance control, and other laws slot in the same way — no
transport, executor, or RT changes:

1. Implement `ControlMode` in a new `*_mode.h` / `.cpp`. Capture entry state in
   `on_enter`; keep `compute()` allocation-free (preallocate all scratch as
   members; use fixed-size Eigen types).
2. Get the rigid-body quantities you need from `Dynamics` (extend it if you need a
   new quantity such as the mass matrix — it's the one place Pinocchio lives).
3. Add a unit test that recomputes the law independently (see the
   `MatchesIndependentlyComputedLaw` oracle pattern) and extend the RT-safety test
   to assert zero major page faults with your mode active.
4. Register the source/test in `CMakeLists.txt`.

See the [Deep Dive](../deep-dive/impedance.md) and the design spec/plan under
`docs/superpowers/` for the Cartesian impedance controller as a worked example.
