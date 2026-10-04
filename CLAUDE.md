# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A layered C++17 low-level control driver for the **Kinova Gen3 7-DOF** arm,
running 1 kHz torque control on a PREEMPT_RT Jetson (aarch64). Control modes
today: **gravity compensation** and **Cartesian (task-space) impedance**.

**This is the foundational arm driver underlying ALL of our arm work.** It began
benchmarking-first (characterizing per-cycle compute cost + loop-timing
stability), and that instrumentation stays — but the mandate has grown: every
higher-level stack (ROS frontends, planners, learned policies) sits on top of
this. Treat it accordingly.

- **Stability and performance are THE metrics.** Correctness and deterministic
  1 kHz timing outrank feature velocity every time. A regression here breaks
  everything downstream. Benchmarking is how we *prove* stability/performance —
  it's the instrument, no longer the goal. Any change to the RT path must be
  measured, not assumed (see the RT-safety invariant below).
- **Documentation is a first-class deliverable.** Because everyone builds on
  this, the docs are load-bearing. We will stand up a hosted docs site — keep the
  docs buildable and coherent as you go (see **Documentation** below).

Read `README.md` for the full narrative (design rationale, RT tuning, KORTEX SDK
install). Read `docs/index.md` and `docs/superpowers/specs|plans/` for design +
implementation-plan detail. This file is the fast path.

## Build & test

**Only builds on Linux/aarch64 (Jetson).** It CANNOT build on macOS — KORTEX and
the RT syscalls are Linux-only; clang errors on a Mac are expected noise. The
default build is **sim-only** and needs no KORTEX SDK.

```sh
cmake -S . -B build \
  -DCMAKE_PREFIX_PATH=/usr/local/lib/python3.10/dist-packages/cmeel.prefix \
  && cmake --build build -j && ctest --test-dir build --output-on-failure
```

`CMAKE_PREFIX_PATH` must point at the pip/cmeel prefix that holds
`pinocchioConfig.cmake` (`<prefix>/lib/cmake/pinocchio/`); adjust for the Python
version.

- **All tests are one gtest binary** (`unit_tests`), registered as a single ctest
  test. Run a subset with the gtest filter:
  `./build/unit_tests --gtest_filter='CartesianImpedance*'`.
- Tests run **without a robot** (SimTransport). The URDF path is injected at
  compile time via `-DURDF_PATH=…` (see `CMakeLists.txt`) — tests don't take a
  path argument.
- Real-robot build is opt-in: `-DKINOVA_ENABLE_KORTEX=ON
  -DKORTEX_HW_DIR=<aarch64 kortex SDK>`. Do not enable-and-connect to hardware
  unattended (`docs/integration-runbook.md`).

Sim benchmarks:
```sh
./build/benchmark_grav_comp --sim --urdf models/gen3_7dof.urdf --rate 1000 --duration 5
./build/benchmark_cartesian_impedance --sim --urdf models/gen3_7dof_2f85.urdf --rate 1000 --duration 5
```

## Architecture — the boundaries that matter

Seven decoupled units; the whole design is about keeping three concerns from
leaking into each other. When editing, preserve these invariants:

- **`Transport` (interface) is the ONLY unit that includes KORTEX.** Concretes:
  `SimTransport` (fake robot, CI) and `KortexTransport` (real, pimpl'd so KORTEX
  headers never enter the public interface). `kortex_transport.cpp` is only
  compiled when `KINOVA_ENABLE_KORTEX=ON`.
- **`Dynamics` is the ONLY unit that includes Pinocchio.** Loads the URDF once,
  pre-allocates `Data`; RT-safe `gravity(q)`, `fk(q)`, `jacobian(q)`. Continuous
  (unbounded) joints are packed as `(cos θ, sin θ)` *inside* Dynamics — no
  wide-limit-revolute hack. The public interface stays a flat 7-vector of angles.
- **`ControlMode` (interface) is the compute boundary** —
  `required_modes()/on_enter/compute(fb,dt,out)/on_exit`. `compute` MUST be
  RT-safe: no allocation, no locks, no blocking. Concretes:
  `JointTorqueMode`, `CartesianImpedanceMode`. Gravity compensation is not its
  own mode — it's `JointTorqueMode` with `tau_ff` never set. New laws (e.g.
  velocity) slot in as new implementations.
- **`RtExecutor` is the ONLY thread-aware unit.** It owns the single RT thread
  and per cycle: pace → `exchange` → check faults → `compute` → push a
  `CycleSample`. Neither `Transport` nor `ControlMode` knows a thread exists.
  **Mode switch = atomic-pointer swap adopted at a cycle boundary**
  (`request_mode`); the executor stores a raw pointer and never owns the mode —
  the ControlMode must outlive the executor.
- **`Telemetry`**: lock-free SPSC `SampleRing`, **drop-don't-block**. The RT
  producer never blocks/allocates; a full ring drops and bumps a visible counter.
  All formatting/CSV/histogram work happens on a non-RT drain thread.
- **`rt_system`**: `mlockall`, `SCHED_FIFO`, core affinity, `/dev/cpu_dma_latency`
  pin — startup/shutdown only. Runs unprivileged after a one-time
  `scripts/rt_grant_once.sh`; degrades to `SCHED_OTHER` without it.
- **`joint_types`/`units`**: fixed-size 7-DOF POD value types, alloc-free.
  `kNumJoints=7`. No KORTEX/Pinocchio types leak here.

## Conventions

- **SI / radians everywhere internally.** KORTEX speaks degrees / N·m; the single
  unit conversion happens only at the `Transport` boundary.
- **Fail loud at startup, never silent mis-mapping.** `Dynamics` throws if URDF
  `nv != 7`; `KortexTransport` validates actuator count on connect. Keep this
  posture.
- **Nothing in the RT path (`compute`, executor cycle) may allocate, lock, or
  block.** The `rt_safety_test` asserts zero major page faults + zero dropped
  samples in steady state — a change that regresses this fails CI.
- Two models: `gen3_7dof.urdf` (bare arm) and `gen3_7dof_2f85.urdf` (with 2F-85
  gripper payload — used by tests and the impedance benchmark).

## Stability & performance (the bar)

This driver underlies everything, so hold a high bar and prove it:

- **Never regress the RT contract silently.** No allocation, lock, or blocking
  call in `compute` or the executor cycle. `rt_safety_test` (zero major page
  faults, zero dropped samples in steady state) is the gate — if you touch the RT
  path, run it and read the result, don't assume.
- **Measure timing changes on real numbers.** Any change plausibly affecting
  per-cycle cost or jitter should be run through the benchmark (`--csv`) and the
  before/after percentiles (p50/p99/p99.9/max, overruns, faults) compared.
  "Looks fine" is not evidence for a driver this much depends on it.
- **Prefer boring, correct, and instrumented** over clever. New behavior lands as
  a new `ControlMode`/`Transport` implementation behind the existing boundaries,
  not by widening the RT-safe surface. Fail loud at startup rather than degrade
  silently at 1 kHz.

## Documentation

Docs are a first-class deliverable and headed for a hosted site — keep them
buildable and current.

- Pages are plain Markdown under `docs/`, wired into an **mkdocs (Material)** nav
  in `mkdocs.yml`. They also render on GitHub, so the generator is swappable.
- Preview / build the site:
  ```sh
  pip install mkdocs mkdocs-material   # once
  mkdocs serve      # live preview at http://127.0.0.1:8000
  mkdocs build      # static site into ./site
  ```
- **When you change behavior, update the docs in the same change** and add new
  pages to the `nav:` in `mkdocs.yml` (an orphaned page won't appear on the
  site). The layers: `guide/` (how to use a control mode), `reference/api.md`
  (interfaces), `deep-dive/` (the math/derivations), `rt-tuning.md` +
  `integration-runbook.md` (operations). Put new content in the matching layer.
- `docs/superpowers/specs|plans/` are design + implementation-plan records — keep
  them in step with what's actually implemented (the git history shows plans
  being reconciled to the code, not left to drift).
