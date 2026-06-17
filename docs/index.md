# Kinova Gen3 Low-Level Driver

A durable, maintainable C++ driver for **Kinova Gen3 7-DOF low-level (1 kHz
torque) control** on a PREEMPT_RT Jetson. It is **benchmarking-first**: the
primary deliverable is characterizing per-cycle compute cost and loop-timing
stability, behind clean, testable boundaries.

## What you get today

- **Two control modes** — gravity compensation and Cartesian (task-space)
  impedance — running in a single real-time thread at 1 kHz.
- **Rigid-body dynamics** (gravity, forward kinematics, frame Jacobian) via
  Pinocchio, behind a small Eigen-only interface.
- **Real-time telemetry** — per-cycle timing histograms, drop-don't-block, drained
  off the RT thread.
- **A simulation transport** so the whole stack builds, runs, and is unit-tested
  with no robot attached.

The full-law Cartesian impedance compute (FK + Jacobian + gravity + nullspace
projection) runs at **p50 ≈ 2 µs / p99 ≈ 4 µs** per cycle with zero allocation in
the RT loop.

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
