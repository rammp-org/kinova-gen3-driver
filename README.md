# Kinova Gen3 Low-Level C++ Driver

A durable, maintainable C++ driver for **Kinova Gen3 7-DOF low-level control**.

## Purpose

This is the functional core of a low-level control stack for the Gen3, built
**benchmarking-first**: the primary deliverable is characterizing per-cycle
compute cost and 1 kHz loop-timing stability on the PREEMPT_RT Jetson, not a
user-facing API.

Supported control today: **torque with gravity compensation**. Designed so
**impedance** and **high-speed velocity** modes slot in later as new
`ControlMode` implementations. Public frontends (ROS, websockets, …) are
deliberately deferred — they become "just another consumer" of this library.

It is a layered, instrumented, testable refactor of a validated single-file
prototype (`grav_comp_test.cpp`), preserving the hard-won-correct KORTEX
handshake and dynamics sequences.

## Architecture — seven units

```
        ┌───────────────┐
main ──▶ │  RtExecutor   │  owns the RT thread, timing, mode-switch handoff
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

| Unit | Responsibility |
|---|---|
| `joint_types` / `units` | Fixed-size SI/radian POD value types (`JointFeedback`, `JointCommand`, `ActuatorMode`, `kNumJoints=7`); deg↔rad + `wrap_to_pi`. No KORTEX/Pinocchio types leak. |
| `Transport` (interface) | The comm boundary — the ONLY unit that includes KORTEX. Lifecycle + cyclic `exchange`/`send`/`receive`. Concretes: `SimTransport` (fake robot, CI) and `KortexTransport` (real Gen3 handshake, pimpl). |
| `ControlMode` (interface) | The compute boundary — `required_modes()`, `on_enter`, RT-safe `compute(fb, dt, out)`, `on_exit`. Concrete: `GravityCompTorqueMode`. |
| `Dynamics` | The ONLY unit that includes Pinocchio. Loads the URDF once, pre-allocates `Data`; `gravity(q, tau_out)` now, mass-matrix/Coriolis later. |
| `Telemetry` | Lock-free SPSC `SampleRing` (drop-don't-block) drained off the RT thread into `NanoHistogram` + `TelemetrySink` (console/CSV). |
| `rt_system` | `mlockall`, `SCHED_FIFO`, core affinity, and `getrusage` introspection. Startup/shutdown only. |
| `RtExecutor` | Owns the single RT thread; per cycle paces → `exchange` → check faults → `compute` → push a `CycleSample`. Mode switch = atomic-pointer swap at a cycle boundary. |

**Key boundary:** communication (`Transport`) and computation (`ControlMode`)
are fully separated; `RtExecutor` is the *only* thread-aware unit. Neither the
transport nor the control law knows how many threads exist or how they are
scheduled.

## Key design decisions (with rationale)

| Decision | Choice | Rationale |
|---|---|---|
| Threading | **Single RT thread**; comm & compute as separate functional units | Simplest correct design — at 1 kHz the UDP round-trip + sub-ms compute fit easily in one thread. The functional separation means moving later to a pipelined/async layout (A→pipeline) is a **localized change inside `RtExecutor`** that doesn't touch `Transport` or `ControlMode`. |
| Dynamics | **Pinocchio + faithful continuous-joint handling** | Fast analytical RNEA/CRBA. Continuous (unbounded) joints are packed as `(cos θ, sin θ)` *inside* `Dynamics` — **no wide-limit-revolute hack**. The hack risks phantom joint-limit / windup failures; modeling the joints faithfully keeps the public interface a simple flat angle vector while staying correct. |
| Units | **SI / radians internally; convert only at the Transport boundary** | KORTEX speaks degrees / N·m. Doing the one conversion at the single comm site keeps the entire compute path in one consistent unit system. |
| Telemetry | **Drop-don't-block lock-free SPSC ring**, drained off the RT thread | The RT producer must never block or allocate. When the ring is full it drops and bumps a visible drop counter rather than stalling the control loop. All formatting/CSV/histogram work happens on a non-RT drain thread. |
| Pacing | **Sleep-then-spin (default) and pure `clock_nanosleep(ABSTIME)`** — both selectable | The prototype's hybrid sleep-then-spin gives lower jitter; pure `clock_nanosleep` is simpler. Both are built in (`--pacing sleepspin\|nanosleep`) so we can **benchmark both** on real hardware. |
| Footguns | **Hard throws over silent failures** | `Dynamics` throws if the URDF `nv != 7`; `KortexTransport` validates the actuator count on connect. Better to fail loudly at startup than to silently mis-map joints. |

## Build (on the Jetson `abra`)

**This driver builds and runs only on the Jetson `abra`** (Linux 5.15-rt-tegra,
PREEMPT_RT, aarch64). It cannot build on macOS — KORTEX and the RT syscalls are
Linux-only; local clang errors on the Mac are expected noise. Cross-compilation
is left open (a stub `cmake/aarch64-toolchain.cmake` is checked in but unused by
the default native build).

The standard dev loop syncs this repo to `abra` and builds + tests there:

```sh
./scripts/build_on_abra.sh        # rsync + cmake + build + ctest
```

Or manually:

```sh
./scripts/sync_to_abra.sh
ssh abra 'cd ~/kinova-gen3-driver/build && cmake .. \
  -DCMAKE_PREFIX_PATH=/usr/local/lib/python3.10/dist-packages/cmeel.prefix \
  && cmake --build . -j && ctest --output-on-failure'
```

**Dependencies** (all already installed on `abra`):

- **Pinocchio** — pip-installed under the `cmeel.prefix`; point CMake at it with
  `-DCMAKE_PREFIX_PATH=/usr/local/lib/python3.10/dist-packages/cmeel.prefix`
  (`pinocchioConfig.cmake` lives at `<prefix>/lib/cmake/pinocchio/`).
- **Eigen 3.4**, **Boost**, **urdfdom** — pulled in by Pinocchio.
- **GoogleTest** — for the unit tests.
- **KORTEX C++ SDK** — only required for the real-robot build (see below).

## Real robot status — IMPORTANT

**The driver currently builds SIM-ONLY.** The vendored KORTEX C++ SDK on `abra`
(`~/rtos_testing/cpp/kortex_hardware/lib/release/libKortexApiCpp.a`) is
**x86-64 only**. There is **no aarch64 KORTEX C++ lib on the box**, so the real
`KortexTransport` path cannot link into a Jetson (aarch64) binary yet.

By default the build does not compile or link `kortex_transport.cpp`, and the
benchmark app is built with the real `--ip` path `#ifdef`'d out (sim-only).

**To enable the real-robot path later** (after installing an aarch64 build of
the KORTEX C++ SDK):

```sh
cmake .. -DCMAKE_PREFIX_PATH=/usr/local/lib/python3.10/dist-packages/cmeel.prefix \
  -DKINOVA_ENABLE_KORTEX=ON \
  -DKORTEX_HW_DIR=/path/to/aarch64/kortex_hardware
```

With `-DKINOVA_ENABLE_KORTEX=ON`, `KortexTransport` is compiled and linked and
the `--ip` path is active. `KORTEX_HW_DIR` is only required/validated when this
option is ON, so a box without KORTEX headers can still build and run the sim.

> Do not enable this and connect to hardware unattended. See
> [`docs/integration-runbook.md`](docs/integration-runbook.md).

## Run the sim benchmark

```sh
ssh abra 'cd ~/kinova-gen3-driver/build && \
  ./benchmark_grav_comp --sim --urdf ../models/gen3_7dof.urdf \
    --rate 1000 --duration 5 --csv /tmp/bench.csv'
```

Useful flags: `--rate <hz>`, `--pacing sleepspin|nanosleep`, `--cpu <core>`,
`--rt-priority <prio>`, `--duration <s>`, `--csv <path>`, and the gravity-comp
knobs `--scale`, `--damping`, `--torque-limit`.

What the output means:

- **Live console (~1 Hz):** loop rate; `cycle` percentiles (p50/p99/p99.9/max);
  `comm` (transport round-trip) and `compute` (control-law) costs; wake
  `jitter`; `overruns` (missed deadlines); `faults`; `dropped` telemetry samples.
- **Final report:** `introspect()` (applied sched policy / priority / affinity,
  page-fault deltas — `majflt+=0` proves `mlockall`, involuntary context
  switches are the top preemption signal); cycle/compute histograms; total
  dropped samples; per-run page-fault delta.
- **CSV:** one row per cycle, buffered and written off the RT thread.

### First observed sim baseline

Labeled clearly as **SCHED_OTHER, unprivileged, SIM transport, idle Jetson**
(NOT a true RT measurement — see caveat below):

```
rate ≈ 1000 Hz
gravity-comp compute: p50 ≈ 2 µs, p99 ≈ 4 µs, worst ≈ 25 µs
  (the worst-case is the first-cycle Pinocchio warm-up)
overruns = 0, faults = 0, dropped = 0
```

> **Caveat:** these are sim-transport numbers under `SCHED_OTHER` without
> privileges. True RT determinism requires a privileged run
> (`SCHED_FIFO` via `sudo`, or `setcap cap_sys_nice,cap_ipc_lock+ep` on the
> binary) **and** the real robot's UDP round-trip in the `comm` measurement.

## Tests

Unit, SimTransport-integration, and RT-safety tests all run **without a robot**
and are exercised by `ctest`:

```sh
./scripts/build_on_abra.sh                                  # builds + runs ctest
# or, on abra directly:
ssh abra 'cd ~/kinova-gen3-driver/build && ctest --output-on-failure'
```

Coverage:

- **Unit:** `Dynamics` gravity at known poses + continuous-joint config
  round-trip; deg↔rad round-trips; `SampleRing` FIFO/drop/SPSC correctness;
  `GravityCompTorqueMode.compute` = gravity + position passthrough.
- **SimTransport integration:** the whole executor + mode + dynamics + telemetry
  pipeline driven by the fake robot.
- **RT-safety:** runs `RtExecutor` on `SimTransport` and asserts **zero major
  page faults** in steady state and **zero dropped** samples with adequate ring
  capacity.

Robot-in-the-loop tests are documented but **not run unattended** — see
`docs/`.

## Repo layout

```
include/kinova_lowlevel/   joint_types.h units.h transport.h control_mode.h
                           dynamics.h rt_executor.h telemetry.h
                           telemetry_consumers.h rt_system.h
                           sim_transport.h kortex_transport.h gravity_comp_mode.h
src/                       dynamics.cpp telemetry.cpp telemetry_consumers.cpp
                           rt_system.cpp sim_transport.cpp kortex_transport.cpp
                           gravity_comp_mode.cpp rt_executor.cpp
apps/                      benchmark_grav_comp.cpp
models/                    gen3_7dof.urdf
tests/                     *_test.cpp
cmake/                     aarch64-toolchain.cmake (stub, unused by default)
scripts/                   build_on_abra.sh  sync_to_abra.sh
docs/                      integration-runbook.md
                           integration/grav_comp_static_check.md
                           superpowers/{specs,plans}/…   (design + plan)
```
