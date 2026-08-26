# Getting Started

This guide gets you from a clean checkout to a control mode running against the
**simulation transport** (no robot), then shows how to drive a mode from your own
C++ and how to validate a real arm read-only before any torque.

> The driver is currently consumed as a **C++ library** linked into your own RT
> program. A higher-level front-end/IPC server (so non-C++ clients can drive it
> over a process boundary) is on the [roadmap](index.md#status--roadmap) — when it
> lands, this page will grow a client-API section.

## Prerequisites

- An **aarch64 Jetson** with a **PREEMPT_RT** kernel (developed on Linux
  5.15-rt-tegra). The driver builds natively on the Jetson; it does **not** build
  on macOS (the KORTEX SDK and RT syscalls are Linux-only).
- Dependencies installed on the Jetson:
  - **Pinocchio** (pip/cmeel) — point CMake at its prefix.
  - **Eigen 3.4**, **Boost**, **urdfdom** (pulled in by Pinocchio).
  - **GoogleTest** (for the unit tests).
  - **KORTEX C++ SDK** — only for the real-robot build; see the
    [README](../README.md#kortex-c-sdk-for-the-real-robot-build).

## 1. Build (sim-only, the default)

```sh
cd kinova-gen3-driver
cmake -S . -B build \
  -DCMAKE_PREFIX_PATH=/usr/local/lib/python3.10/dist-packages/cmeel.prefix \
  && cmake --build build -j
ctest --test-dir build --output-on-failure      # unit + sim-integration + RT-safety tests
```

`CMAKE_PREFIX_PATH` points at wherever pip installed Pinocchio's cmeel prefix
(adjust the Python version). The default build is **sim-only** — no KORTEX
required. The real-robot path is opt-in via `-DKINOVA_ENABLE_KORTEX=ON`.

## 2. Run a control mode in sim

Each mode ships a benchmark app that runs it through the `RtExecutor` and reports
per-cycle timing. No robot is involved.

```sh
cd build

# Gravity compensation (JointTorqueMode with no feedforward)
./benchmark_grav_comp --sim --urdf ../models/gen3_7dof_2f85.urdf \
  --ee-frame gen3_end_effector_link --rate 1000 --duration 5

# Cartesian impedance (holds the entry pose; push -> springs back)
./benchmark_cartesian_impedance --sim --urdf ../models/gen3_7dof_2f85.urdf \
  --rate 1000 --duration 5
```

You'll see ~1 Hz console lines (loop rate, cycle/compute percentiles, overruns,
dropped) and a final report. **What good looks like:** rate ≈ 1000 Hz,
`overruns ≈ 0`, `faults = 0`, `dropped = 0`, `majflt+=0`. With the sim transport
the impedance compute reports **p50 ≈ 2 µs**.

> Use `models/gen3_7dof_2f85.urdf` (arm **with** the Robotiq 2F-85 gripper as a
> fixed payload). Running the bare-arm URDF with a gripper mounted
> under-compensates gravity and the arm sags. Match the URDF to the hardware.

## 3. Drive a mode from your own C++

A control mode plugs into the `RtExecutor`. Minimal program (sim transport):

```cpp
#include <atomic>
#include <thread>
#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/cartesian_impedance_mode.h"
#include "kinova_lowlevel/rt_executor.h"
#include "kinova_lowlevel/sim_transport.h"
#include "kinova_lowlevel/telemetry.h"
using namespace kinova;

int main() {
  // 1. Dynamics from the URDF (validates the model + EE frame at construction).
  Dynamics dyn("models/gen3_7dof_2f85.urdf");      // default EE frame: gen3_end_effector_link

  // 2. A transport. SimTransport is a fake robot; swap in KortexTransport(ip) for real hardware.
  JointFeedback init;                               // zero state
  SimTransport transport(init);

  // 3. Telemetry ring (drained on a non-RT thread).
  SampleRing ring(1 << 16);
  std::atomic<bool> draining{true};
  std::thread drain([&]{ CycleSample s; while (draining) { while (ring.pop(s)) {/* log s */} } });

  // 4. The control mode (holds the pose captured at entry, compliantly).
  CartesianImpedanceParams params;                  // sensible defaults; tune Kx/Dx on hardware
  CartesianImpedanceMode mode(dyn, params);

  // 5. The executor owns the RT thread. cpu=-1 => no pinning; priority 80.
  transport.connect();
  transport.set_servoing_low_level();
  RtExecutor ex(transport, ring, { /*rate_hz*/ 1000.0, Pacing::kSleepSpin, /*RtConfig*/ {80, -1, true} });
  ex.request_mode(&mode);

  // 6. Run the loop until stopped (here: a separate thread stops it after 5 s).
  std::atomic<bool> stop{false};
  std::thread timer([&]{ std::this_thread::sleep_for(std::chrono::seconds(5)); stop = true; });
  ex.run(stop);                                     // blocks on this (RT) thread

  transport.safe_shutdown();
  timer.join();
  draining = false; drain.join();
}
```

To **change the target or gains at runtime** (e.g. from a supervisor thread),
call the non-RT setters — they publish lock-free snapshots the RT loop reads each
cycle:

```cpp
Pose target = dyn.fk(some_q);   // or build an SE(3) pose directly
target.p.x() += 0.05;           // shift the tool +5 cm in world X
mode.set_target(target);        // arm springs toward the new pose
mode.set_gains(stiffer_params); // retune online
```

See the [Control Modes guide](guide/control-modes.md) for what each mode does and
the [API Reference](reference/api.md) for exact signatures.

## 4. Before real hardware: validate read-only

Torque control on a real arm is **attended-only**. The impedance app has a
`--dry-run` that **connects and reads feedback only** — it never enters low-level
servoing and never commands torque. Use it to confirm the URDF/frame match the
real arm (move it by pendant and watch the reported pose / Jacobian condition):

```sh
./benchmark_cartesian_impedance --ip 192.168.1.10 --dry-run \
  --urdf ../models/gen3_7dof_2f85.urdf --duration 0
```

Then follow the [Integration Runbook](integration-runbook.md) for the full,
conservative bring-up (low scale, low torque limit, e-stop in reach). Tune the
Jetson first per [Real-Time Tuning](rt-tuning.md).

## Next

- [Control Modes guide](guide/control-modes.md) — gravity-comp and impedance,
  conceptually.
- [API Reference](reference/api.md) — every public type and signature.
- [Deep Dive: Impedance](deep-dive/impedance.md) — the math and the RT-safety
  design.
