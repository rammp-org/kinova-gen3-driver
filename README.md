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

## KORTEX C++ SDK (for the real-robot build)

The **default build is sim-only** and needs no KORTEX SDK — `kortex_transport.cpp`
is not compiled and the app's `--ip` path is `#ifdef`'d out. The real-robot path
is opt-in via `-DKINOVA_ENABLE_KORTEX=ON`.

**Architecture matters.** The KORTEX C++ lib vendored in the old prototype
(`~/rtos_testing/cpp/kortex_hardware/`) is **x86-64** and will not link into a
Jetson (aarch64) binary. Kortex `2.3.0` (the prototype's version) publishes *no*
aarch64 C++ build at all. We use **Kortex C++ API 2.8.0 aarch64** — the newest
version that actually builds against our code (the newest overall, 3.4.0, ships
broken headers: its `ClientService.h` includes a `CoreBenchmarker.h` that isn't
in the package).

### Install (on `abra`, no sudo — it's just a user-dir extract)

```sh
ssh abra '
  DEST=~/kortex_api_2.8.0_aarch64
  mkdir -p "$DEST"
  curl -sL "https://artifactory.kinovaapps.com/artifactory/generic-public/kortex/API/2.8.0/linux_aarch64_gcc_7.4.zip" -o /tmp/k.zip
  unzip -q -o /tmp/k.zip -d "$DEST"
'
# yields $DEST/include/{client,client_stubs,common,google,messages} + $DEST/lib/release/libKortexApiCpp.a (AArch64)
```

### Build the real path against it

```sh
ssh abra 'cd ~/kinova-gen3-driver && rm -rf build_kortex && mkdir build_kortex && cd build_kortex && cmake .. \
  -DCMAKE_PREFIX_PATH=/usr/local/lib/python3.10/dist-packages/cmeel.prefix \
  -DKINOVA_ENABLE_KORTEX=ON \
  -DKORTEX_HW_DIR=$HOME/kortex_api_2.8.0_aarch64 \
  && cmake --build . -j'
```

`KORTEX_HW_DIR` is only required/validated when `KINOVA_ENABLE_KORTEX=ON`, so a
box without the SDK still builds the sim. (Symptom of pointing at the x86-64 lib:
`ld: ... Relocations in generic ELF (EM: 62) ... file in wrong format`.)

> **Version vs firmware:** Kinova couples the API version to robot firmware.
> 2.8.0 suits 2.x firmware; confirm the arm's firmware at bring-up (browse to the
> robot IP) before locking it in. For our low-level cyclic use the control API is
> stable across 2.3–2.8, so no control features are lost on 2.x.

> Do not enable this and connect to hardware unattended. See
> [`docs/integration-runbook.md`](docs/integration-runbook.md).

## Real-time tuning — REQUIRED for steady timing

A PREEMPT_RT kernel alone does **not** give a steady loop. `abra` has the RT
kernel and MAXN power, but as of 2026-06-02 the CPU governor (`schedutil`), deep
CPU idle (`c7`, 5 ms exit latency), missing core isolation, and unlocked clocks
will all show up as jitter. **Tune the platform before trusting any timing
numbers.**

**The driver runs without `sudo`.** Scheduling (`SCHED_FIFO`), `mlockall`, core
affinity, and the C-state pin are all done *inside* `rt_system::enable_rt()`;
they only need a **one-time** permission grant (not per-run sudo, and unlike
`setcap` it survives rebuilds):

```sh
sudo ./scripts/rt_grant_once.sh    # once: 'realtime' group + rtprio/memlock limits
                                    # + udev rule for /dev/cpu_dma_latency. Re-login after.
```

After that the driver gets real RT as a normal user — confirm `policy: FIFO` and
`cpu_dma_latency: pinned@0us` in its report. (Without the grant it still runs,
degrading to `SCHED_OTHER`.) Notably the driver pins `/dev/cpu_dma_latency` to
0 µs itself, suppressing the deep `c7` (~5 ms) idle state for our process — the
same trick cyclictest uses.

The remaining knobs are CPU-/system-global (not driver-local) — they need root
or a boot edit and mainly tighten the tail under load:

```sh
sudo ./scripts/rt_setup.sh 11      # governor, clocks, deep-idle, RT throttling (runtime)
```

Core **isolation** (`isolcpus=11 nohz_full=11 rcu_nocbs=11`) is a boot-time
kernel-cmdline change, and you should **measure** the result with `cyclictest`.
The full checklist — privilege model, every setting with rationale, the
bootloader edit, per-run flags, validation, and persistence — is in
[`docs/rt-tuning.md`](docs/rt-tuning.md).

**Validated on `abra`** (see [`docs/rt-validation-results.md`](docs/rt-validation-results.md)):
after tuning + isolating core 11, `cyclictest` worst-case wake latency dropped
from **48 µs → 7 µs** and *held at 7 µs under loadavg ~19.6*; the driver's own
cycle p99.9 = 1 µs with zero steady-state major faults — all without runtime sudo.

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
> privileges, on an **untuned** box. True RT determinism requires the platform
> tuning in [`docs/rt-tuning.md`](docs/rt-tuning.md) (governor, C-states, core
> isolation, clock lock), a privileged run (`SCHED_FIFO` via `sudo` or
> `setcap cap_sys_nice,cap_ipc_lock+ep`), **and** the real robot's UDP round-trip
> in the `comm` measurement.

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
                           rt_grant_once.sh  rt_setup.sh
docs/                      rt-tuning.md  rt-validation-results.md
                           integration-runbook.md
                           integration/grav_comp_static_check.md
                           superpowers/{specs,plans}/…   (design + plan)
```
