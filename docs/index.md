# Kinova Gen3 Low-Level Driver

A durable, maintainable C++ driver for **Kinova Gen3 7-DOF low-level (1 kHz)
control** on a PREEMPT_RT Jetson, behind clean, testable boundaries. It is the
foundational arm driver underneath every higher-level stack we build, so
correctness and deterministic timing outrank feature velocity here.

## What you get today

- **Five control modes** — joint torque (and so gravity compensation), Cartesian
  impedance, joint impedance, joint position, and joint velocity — running in a
  single real-time thread at 1 kHz, all behind one `ControlMode` interface.
- **A gripper controller** for the 2F-85, driven in the same loop.
- **An interface tier** — `Supervisor`, `Arbiter`, `StreamingSession` — giving
  front-ends ports for trajectory goals, streaming setpoints and command
  arbitration, with no framework inside the core. See the
  [arbitration](guide/arbitration.md) and [streaming](guide/streaming.md) guides.
- **Rigid-body dynamics** (gravity, forward kinematics, frame Jacobian, mass
  matrix) via Pinocchio, behind a small Eigen-only interface.
- **Real-time telemetry** — per-cycle timing histograms, drop-don't-block, drained
  off the RT thread.
- **A simulation transport** so the whole stack builds, runs, and is unit-tested
  with no robot attached.

The full-law Cartesian impedance compute (FK + Jacobian + gravity + nullspace
projection) runs at **p50 ≈ 2 µs / p99 ≈ 4 µs** per cycle with zero allocation in
the RT loop.

## Support matrix

The modes are not equally proven, and the difference matters more than the
feature list. This table is the honest version.

| Mode | On hardware | RT cost measured | Known limitations |
|---|---|---|---|
| `JointTorqueMode` — gravity comp | Yes, [procedure](integration/joint_torque_check.md) | Yes — `benchmark_grav_comp` | **Droops and drifts** on the arm rather than holding ([#40](https://github.com/rammp-org/kinova-gen3-driver/issues/40)); entangled with the URDF mass model ([#5](https://github.com/rammp-org/kinova-gen3-driver/issues/5)) |
| `JointTorqueMode` — feedforward | Yes, [procedure](integration/joint_torque_check.md) | Yes — `benchmark_grav_comp` | Compliant by construction: you only ever *add* torque on top of gravity comp |
| `CartesianImpedanceMode` | Yes, via teleop | Yes — `benchmark_cartesian_impedance` | Gains tuned by feel, not characterised |
| `JointImpedanceMode` | Yes, via teleop | **No** — no benchmark harness exists | Compute budget and closed-loop behaviour both unvalidated ([#6](https://github.com/rammp-org/kinova-gen3-driver/issues/6)) |
| `JointPositionMode` | Yes, [procedure](integration/joint_position_hardware_check.md) | Not separately benchmarked | — |
| `JointVelocityMode` | Yes, [probe](integration/velocity_mode_probe.md) | Yes — `benchmark_joint_velocity` | **A zero command does not hold**: joint 2 creeps ~0.038 rad/s under gravity. The actuator's own servo, not this driver ([#34](https://github.com/rammp-org/kinova-gen3-driver/issues/34)). Do not park an arm here and stream zeros |
| `GripperController` | Yes — measured on the arm | In-loop RT-safety case only | Commanded/measured force maxima are independently guessed |
| Interface tier | Yes — [stream check](integration/stream_check.md), and `ExecuteJointTrajectory` end to end | RT-safety cases in loop | Goals complete on elapsed time, not arrival ([#17](https://github.com/rammp-org/kinova-gen3-driver/issues/17)); `joint_names` ignored ([#16](https://github.com/rammp-org/kinova-gen3-driver/issues/16)) |

One thing this table is deliberately not shy about: the URDF masses do not match
the true system ([#5](https://github.com/rammp-org/kinova-gen3-driver/issues/5)),
which weakens every gravity-model number downstream of it.

Nothing above is a reason not to build on this driver — it is a list of what to
check before trusting a particular number, and it is why the API is now stable
even where the characterisation is not. From **1.0.0** on the driver follows
semantic versioning and consumers pin an exact tag; see
[Versioning](https://github.com/rammp-org/kinova-gen3-driver/blob/main/CONTRIBUTING.md#versioning)
for what the public surface covers.

The pages under `integration/` linked above are bring-up and hardware procedures
for whoever is at the arm; they live in the repo rather than in this site's nav.

## Where to go next

| If you want to… | Read |
|---|---|
| Build it and run a controller | [Getting Started](getting-started.md) |
| Understand the control modes conceptually | [Control Modes guide](guide/control-modes.md) |
| Look up an exact type or signature | [API Reference](reference/api.md) |
| Understand the impedance math & RT design | [Deep Dive: Impedance](deep-dive/impedance.md) |
| Bring up a real robot (attended) | [Integration Runbook](integration-runbook.md) |
| Tune the Jetson for steady timing | [Real-Time Tuning](rt-tuning.md) |

## Architecture in one picture

```
main ──▶ RtExecutor ── owns the single RT thread, pacing, mode handoff
            │
   ┌────────┼─────────────┬───────────────┐
   ▼        ▼             ▼               ▼
Transport  ControlMode  Telemetry      rt_system
(comm)     (compute)    (timing)       (sched/affinity)
KORTEX        │
or Sim     Dynamics ── Pinocchio (gravity / fk / jacobian)
```

**Communication** (`Transport`) and **computation** (`ControlMode`) are fully
separated; `RtExecutor` is the only thread-aware unit. A control mode is a pure,
RT-safe control law — the unit you add to give the arm a new behavior. See the
[guide](guide/control-modes.md).

## Status & roadmap

- **Shipped:** gravity-comp + Cartesian impedance modes, the dynamics surface,
  telemetry, RT tuning, sim + (compiled-in) real KORTEX transport.
- **Coming:** a **front-end / IPC server** so non-C++ clients can set targets and
  gains and stream state over a process boundary. The impedance mode's non-RT
  `set_target` / `set_gains` setters are the seam it plugs into. Until then, the
  driver is consumed as a **C++ library** — see [Getting Started](getting-started.md).

> **Safety:** torque control on real hardware is **attended-only**. Never run the
> robot unattended; always validate a new arm/tool read-only (`--dry-run`) before
> commanding torque. See the [Integration Runbook](integration-runbook.md).
