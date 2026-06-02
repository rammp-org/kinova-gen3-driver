# Kinova Gen3 Low-Level Control Driver — Design

**Date:** 2026-06-02
**Status:** Approved (design); implementation in progress
**Target hardware:** Kinova Gen3 7-DOF, NVIDIA Jetson (`abra`), Linux 5.15-rt-tegra (PREEMPT_RT), aarch64

## 1. Goal & framing

Build a **durable, maintainable C++ driver for Gen3 low-level control**. The supported control modes over time are torque (with gravity comp), impedance, and high-speed velocity control. Public-facing interfaces (ROS, websockets, …) are deliberately **deferred** — this work implements the *functionality* behind a clean library boundary so a frontend is "just another consumer" later.

**The single most important deliverable is benchmarking:** characterizing per-cycle compute cost and loop-timing stability on the PREEMPT_RT Jetson.

This is **not green-field**. A validated single-file prototype already exists at `~/rtos_testing/cpp/grav_comp_test.cpp` on `abra` (mirrors `kortex_hardware/src/Gen3Robot.cpp`). It proves the full KORTEX low-level handshake, 1 kHz torque gravity-comp loop, continuous-joint dynamics handling, and `mlockall`+`SCHED_FIFO` timing. **This project refactors that proven prototype into a layered, instrumented, testable library** — keeping the hard-won correct sequences, adding architecture and benchmarking.

## 2. Scope (first deliverable)

- RT control-loop **skeleton** over the KORTEX low-level cyclic API.
- **One real mode:** gravity-comp torque (realistic per-cycle compute for benchmarking; foundation for impedance).
- **Heavy instrumentation:** live console stats, latency histogram, CSV-to-disk, CPU/thread introspection.
- Mode/control abstraction designed in so impedance + velocity slot in later as new `ControlMode` implementations.

**Out of scope now:** impedance & velocity modes (designed-for, not built), live cross-mode switching across differing actuator modes, any public frontend (ROS/websocket), config-file framework.

## 3. Decisions (the "why")

| Decision | Choice | Rationale |
|---|---|---|
| Substrate | KORTEX C++ low-level cyclic API | Only path to low-level control on Gen3; prototype already links the vendored static lib |
| Dynamics | Pinocchio + faithful 7-DOF URDF | Fast analytical RNEA/CRBA; gravity now, mass matrix/Coriolis for impedance later |
| Continuous joints | Modeled faithfully; **cos/sin packing inside `Dynamics`** | NO wide-limit-revolute hack — that risks phantom joint-limit/windup failures. Easy interface (flat angle vector), faithful implementation. See [[no-silent-footgun-hacks]] |
| Units | SI/radians internal; convert only at the Transport boundary | KORTEX speaks deg/N·m; one conversion site |
| Loop architecture | Single RT thread (A); comm & compute as separate functional units | Simplest correct; at 1 kHz the UDP round-trip + sub-ms compute fit easily. Functional separation keeps A→A′(async)→B(pipeline) a localized change |
| Pacing | Sleep-then-spin (proven) + pure `clock_nanosleep(ABSTIME)` as alternative | Prototype's hybrid gives lower jitter; benchmark both |
| Build | Library core + separate app target; native on Jetson, cross-compile kept open | Frontend-agnostic; no hardcoded paths, deps via `find_package`/cache vars |

## 4. Architecture — seven units

```
        ┌───────────────┐
main ──▶ │  RtExecutor   │  owns RT thread, timing, mode-switch handoff
        └───────┬───────┘
         ┌──────┼───────────────┬────────────────┐
         ▼      ▼               ▼                ▼
   ┌──────────┐ ┌────────────┐ ┌──────────┐ ┌───────────┐
   │ Transport│ │ControlMode │ │ Telemetry│ │ rt_system │
   │  (comm)  │ │ (compute)  │ │          │ │ (sched..) │
   └──────────┘ └─────┬──────┘ └──────────┘ └───────────┘
   KORTEX-only        ▼
                ┌──────────┐
                │ Dynamics │  Pinocchio-only
                └──────────┘
   all share ──▶ joint_types  (POD, fixed-size 7-DOF, alloc-free)
```

- **`joint_types`** — fixed-size SI value types: `JointFeedback{q,qd,tau,current}`, `JointCommand{setpoints + mode tag}`, `ActuatorMode` enum. `kNumJoints=7` compile-time const. No KORTEX/Pinocchio types leak.
- **`Transport` (interface)** — ONLY code that includes KORTEX/protobuf. Lifecycle (`connect`, `set_servoing_low_level`, `set_actuator_modes` with the *pumping* sequence, `safe_shutdown`) + cyclic ops split so blocking-vs-async lives here: `send()`→`RefreshCommand`, `receive()`→`RefreshFeedback`, `exchange()`→`Refresh`. Concrete: `KortexTransport`, protobuf messages pre-allocated/reused; does deg↔rad + frame_id/command_id bookkeeping.
- **`ControlMode` (interface)** — `required_modes()`, `on_enter(fb)`, `compute(fb, dt, out)` (RT-safe, no alloc), `on_exit()`. Concrete: `GravityCompTorqueMode`.
- **`Dynamics`** — ONLY Pinocchio code. Loads URDF once, pre-allocates `Data`. `gravity(q, tau_out)` now; `mass_matrix`/`nonlinear`/`inverse_dynamics` later. Public interface takes flat 7-vector of joint angles; internally maps to Pinocchio `nq` config via faithful cos/sin packing (ported from prototype's `compute_gravity`).
- **`RtExecutor`** — owns RT thread; per cycle: pick up mode-switch → `exchange` → check faults → `compute` → push timing sample. Paces to next boundary. The ONE place that decides thread layout (the A→B seam). Mode switch = atomic pointer swap of pre-built modes at a cycle boundary.
- **`Telemetry`** — lock-free SPSC ring buffer (RT producer, drop-don't-block + drop counter), drained by non-RT thread feeding console/CSV/histogram. RT side only reads clocks + pushes.
- **`rt_system`** — `mlockall`, `SCHED_FIFO`, core affinity, `getrusage`/page-fault/ctx-switch + applied-policy introspection. Startup/shutdown only.

## 5. RT executor, timing & threading

Three threads: **RT loop** (`SCHED_FIFO`, pinned, `mlockall`'d — the only RT thread), **Supervisor** (normal: setup/teardown, mode requests, fault response), **Telemetry drain** (normal; NOT on the RT core).

Cycle (four timestamps → full decomposition):
```
sleep→boundary; t0=now (wake jitter=t0-boundary)
maybe_switch_mode(); transport.exchange(cmd,fb); t1=now (comm cost)
check_faults(fb); mode.compute(fb,dt,cmd); t2=now (compute cost)
telemetry.push({boundary,t0,t1,t2,flags})   // total cycle vs period
if (boundary missed) { overruns++; resync }
```
RT thread: no alloc, no I/O, no locks. **Pacing** defaults to prototype's sleep-then-spin (coarse `clock_nanosleep` + busy-wait); pure `clock_nanosleep(ABSTIME)` selectable for comparison.

**Torque entry** (no-jump): seed cyclic with current positions, switch actuators to TORQUE with `Refresh` *pumping* between each `SetControlMode`, first torque = `gravity(q)`, position passthrough each cycle. **Safe exit:** revert actuators to POSITION, `SINGLE_LEVEL_SERVOING`, close sessions. Missed deadlines are operational risk (robot watchdog), not just a metric.

## 6. Benchmarking & telemetry

`CycleSample` ~32B POD: `cycle_index, wake_jitter_ns, comm_ns, compute_ns, cycle_ns, flags`. Lock-free SPSC ring, preallocated, drop-don't-block with visible drop counter. Clock = `clock_gettime(CLOCK_MONOTONIC)` (vDSO); measure clock overhead itself first.

Four consumers: (1) **live console** (~1 Hz): rate, cycle min/mean/p50/p99/p99.9/max, jitter, compute & comm min/mean/max, overruns, drops; (2) **CSV** per-cycle, buffered off RT thread; (3) **latency histogram** (hand-rolled log buckets); (4) **CPU/thread introspection** one-shot start/end: applied sched policy/prio/affinity, page-fault delta (≈0 proves `mlockall`), **involuntary ctx switches** (`ru_nivcsw` — top preemption signal). Per-cycle CPU time is an opt-in flag (adds a 2nd in-loop clock read); default is coarse `getrusage` deltas. Run bounded by duration or cycle count.

## 7. Dynamics & gravity-comp mode

`Dynamics`: `buildModel(urdf)` + pre-alloc `Data`; `gravity(q,tau_out)` → `computeGeneralizedGravity`. Continuous joints packed via `nqs[jid]==2` → (cos,sin), bounded → angle (ported, validated). Model fidelity (incl. end-effector payload inertia) governs grav-comp quality — payload is an explicit input.

`GravityCompTorqueMode`: `required_modes()`=all TORQUE; `compute` sets `out.torque = gravity(fb.q)` + measured-position passthrough + torque clamp (default 39 N·m). Optional joint-damping `−b·q̇` (default `b=0`) as explicit stability knob. Scale knob (default 1.0) retained from prototype.

## 8. Build, deps, layout

C++17, CMake. Library target `kinova_lowlevel` + separate `apps/benchmark_grav_comp`.
- **KORTEX**: vendored static lib via `KORTEX_HW_DIR` (default `~/rtos_testing/cpp/kortex_hardware`); include dirs + `libKortexApiCpp.a` + `dl` + `pthread` (mirror prototype CMake).
- **Pinocchio**: `find_package(pinocchio REQUIRED)`; `CMAKE_PREFIX_PATH` → pip `cmeel.prefix` (`/usr/local/lib/python3.10/dist-packages/cmeel.prefix`). Pulls Eigen 3.4, Boost, urdfdom (all present).
- **GoogleTest** (present) for unit tests.
- No absolute paths; stub `cmake/aarch64-toolchain.cmake` checked in but unused by default native build.

```
include/kinova_lowlevel/  joint_types.h transport.h control_mode.h dynamics.h
                          rt_executor.h telemetry.h rt_system.h
src/                      kortex_transport.cpp dynamics.cpp rt_executor.cpp
                          telemetry.cpp rt_system.cpp gravity_comp_mode.cpp
apps/                     benchmark_grav_comp.cpp
models/                   gen3_7dof.urdf            (from ~/rtos_testing/7dof.urdf)
tests/                    *_test.cpp
cmake/                    FindKortex.cmake aarch64-toolchain.cmake
scripts/                  rt_setup notes, plot_csv.py
```

Dev loop: author in this repo on the Mac → rsync to `abra` → build + unit/SimTransport tests there (Kinova powered off → no live-robot runs).

## 9. Testing strategy

- **`SimTransport`** fake robot (implements `Transport`): runs the whole executor+mode+dynamics+telemetry pipeline with no robot, models round-trip latency, CI-able.
- **Unit (no robot, CI):** `Dynamics` gravity at known poses + continuous-joint config round-trip; deg↔rad round-trips; SPSC ring correctness + drop counting under load; `GravityCompTorqueMode.compute` = gravity + position passthrough.
- **RT-safety (no robot):** loop on `SimTransport` asserts zero steady-state page faults / zero heap alloc in hot path.
- **Integration (robot, MANUAL, with user present):** static-pose gravity vs measured torques; hold-position; live timing benchmark. Built now, **run together when the user is back** (robot is off).
- Built **test-first** (TDD).

## 10. Open items / to confirm with user (robot powered off)

1. Confirm `~/rtos_testing/7dof.urdf` is the correct 7-DOF model and includes the actual end-effector payload inertia.
2. Robot IP (prototype default `192.168.1.10`), credentials.
3. Run the integration suite together once the Kinova is on.
