# Control Modes

A **control mode** is the driver's unit of "what the arm does." It's a pure,
real-time-safe control law: each 1 kHz cycle the executor hands it the latest
joint feedback and a `dt`, and it fills in a joint command. Swapping the control
law never touches the transport or the RT machinery — modes are how you give the
arm a new behavior.

This page explains the two shipped modes conceptually. For exact signatures see
the [API Reference](../reference/api.md); for the impedance math and RT-safety
design see the [Deep Dive](../deep-dive/impedance.md).

## How a mode fits in

Every mode implements the same small interface (`ControlMode`):

- **`required_modes()`** — which actuator mode each joint needs (e.g. all torque).
  The executor applies these before the loop starts.
- **`on_enter(feedback)`** — runs once, on the RT thread, when the mode is adopted.
  Both shipped modes use it to "hold where you are" by capturing the entry state.
- **`compute(feedback, dt, command)`** — the control law, every cycle. It must be
  RT-safe: no allocation, no locks, no blocking.
- **`on_exit()`** — cleanup hook.

You hand a mode to the executor with `request_mode(&mode)`; it's adopted at the
next cycle boundary (an atomic-pointer swap). You keep ownership — the mode must
outlive its time as the active mode.

Both shipped modes also echo the measured joint position back in the command
(*position passthrough*), so the robot's own low-level safety can fall back to a
position hold if it ever flags a following-error fault while in torque mode.

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
| `nullspace_kp` / `kd` | 5 / 1 | How firmly the elbow holds its rest posture. Keep modest. |
| `gain_ramp_s` | 0.5 s | Longer = gentler entry. |
| `torque_limit` | 39 N·m | Lower it for a more conservative cap during bring-up. |

## Choosing a mode

| You want… | Mode |
|---|---|
| The arm weightless / back-drivable | Gravity compensation |
| A safe, predictable first hardware test | Gravity compensation at `scale 0.5` |
| The tool to be compliant about a target pose | Cartesian impedance |
| Online retargeting / regrasping with compliance | Cartesian impedance + `set_target` |

## Adding your own mode

Velocity control, joint impedance, and other laws slot in the same way — no
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
