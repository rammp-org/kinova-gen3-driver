# Teleop Gripper Command — Design Spec

**Date:** 2026-06-28
**Status:** Approved for planning
**Branch:** `feat/teleop-gripper` (off `feat/teleop-socket-server`)
**Scope:** Wire the gripper through the teleop socket server so the Python
supervisor can command gripper position 0–1 over the existing UDP protocol, and
read back the gripper's actual position. Picks up the gripper work explicitly
deferred by the socket-server spec
([`2026-06-26-teleop-socket-server-design.md`](2026-06-26-teleop-socket-server-design.md),
"Out of scope §5.3").

## Goal

The supervisor already sends a `gripper` field (`f32`, 0–1) in every
`POSE_TARGET` packet and reads a `gripper_state` field in every `FEEDBACK`
packet. Today the server stores the commanded value (`last_gripper`) and echoes
it, but **no command ever reaches the hardware** — the `TODO(gripper)` in
`apps/teleop_socket_server.cpp` marks exactly where the path stops.

After this work:
- A `gripper` value in `POSE_TARGET` drives the physical Robotiq 2F-85 gripper.
- `FEEDBACK.gripper_state` reports the gripper's **actual** position read from the
  robot, not the last command.
- The arm controller (`CartesianImpedanceMode`) is untouched; gripper control is
  orthogonal to it.
- **No change to the wire protocol** — the fields, sizes, and the parity test
  (`20 / 84 / 157 / 28 / 261`) all stay exactly as they are.

## How the gripper is actually driven

On the Gen3, the Robotiq 2F-85 is not a separate connection. In the
`LOW_LEVEL_SERVOING` cyclic loop we already run at 1 kHz, the gripper rides
inside the **same** `BaseCyclic::Command` packet as the arm, in its
`interconnect.gripper_command` sub-message. Each cycle that we want to move the
gripper, we add a motor command with a **position setpoint in percent (0–100)**;
the gripper firmware closed-loops to it. The supervisor speaks 0–1, so the
server maps `0–1 → 0–100`. Velocity and force are sent as fixed safe defaults
(see Component 3) — not exposed over the wire (YAGNI).

The gripper's measured position comes back the same way: in
`BaseCyclic::Feedback`, the `interconnect.gripper_feedback` motor reports its
actual position in percent.

> **Verify-at-implementation note:** the exact KORTEX accessor names
> (`mutable_interconnect()`, `mutable_gripper_command()`, `add_motor_cmd()`,
> `set_position/set_velocity/set_force`, and the feedback side
> `interconnect().gripper_feedback().motor(0).position()`) must be confirmed
> against the real headers on `abra` before claiming the KORTEX build compiles.
> The approach (gripper embedded in the cyclic interconnect message) is the
> documented Gen3 low-level pattern; only the precise getters/setters are to be
> pinned down.

## Guiding principles (consistent with the socket-server spec)

1. **Gripper is orthogonal to arm control.** It flows through a `Transport`
   decorator, not through any `ControlMode`. Any present or future control mode
   gets gripper support for free.
2. **Touch the driver core minimally.** The only edits to existing library files
   are: two fields on `JointCommand`, one field on `JointFeedback`, the
   `KortexTransport` read/write of the interconnect gripper, and the
   `SimTransport` echo. No `RtExecutor` change; no `ControlMode` change.
3. **Mirror existing patterns.** `GripperInjector` is the command-path twin of
   the already-shipped `FeedbackTap` (read-side decorator). Compose the two so
   each decorator has a single responsibility.

## Data flow

```
                          set_gripper(0..1)          stamp into JointCommand
Python supervisor ─UDP─▶ rx thread ─────────▶ GripperInjector ──exchange()──▶ FeedbackTap ──▶ KortexTransport
  POSE_TARGET.gripper                            (atomic target)                                  │ writes interconnect
                                                                                                  │ gripper_command (0..100%)
                                                                                                  ▼
                                                                                            Robotiq 2F-85
                                                                                                  │ gripper_feedback (0..100%)
Python supervisor ◀─UDP─ feedback thread ◀── JointFeedback.gripper ◀── FeedbackTap.snap ◀─────────┘
  FEEDBACK.gripper_state                       (actual position)
```

---

## Component 1 — `JointCommand` / `JointFeedback` fields (`include/kinova_lowlevel/joint_types.h`)

- `JointCommand` gains:
  - `float gripper = 0.0f;` — target position, **0–1** (server-side units; the
    transport converts to percent).
  - `bool gripper_active = false;` — when false, no gripper motor command is
    emitted, so a command with no gripper intent never fights the gripper. The
    teleop path always sets this true once a `POSE_TARGET` has been seen.
- `JointFeedback` gains:
  - `float gripper = 0.0f;` — measured gripper position, **0–1** (transport
    converts from percent on the way in).

These default-initialize, so every existing `JointCommand{}` / `JointFeedback{}`
construction (benchmarks, tests, other modes) keeps compiling unchanged with the
gripper inert.

## Component 2 — `GripperInjector` decorator (`apps/teleop_socket_server.cpp`)

A `Transport` decorator, sibling of `FeedbackTap`, living in the same app file:

- Holds `std::atomic<float> gripper_{0.0f}` and `std::atomic<bool> active_{false}`.
- `set_gripper(float g)` (called by the rx thread): stores the clamped target and
  sets `active_ = true`.
- `exchange(const JointCommand& c, JointFeedback& fb)`: copies `c`, stamps
  `c2.gripper = gripper_`, `c2.gripper_active = active_`, forwards to inner.
  Same for `send()`. (`JointCommand` is small/POD — the copy is RT-safe, no
  alloc.)
- All other `Transport` methods forward verbatim, including `clear_faults()`.

Composition in `main`: `KortexTransport`/`SimTransport` → `GripperInjector` →
`FeedbackTap` → `RtExecutor`. (Injector inside, tap outside; order doesn't
matter functionally since they touch disjoint directions, but this keeps the
"command goes in, feedback comes out" reading.)

## Component 3 — `KortexTransport` gripper read/write (`src/kortex_transport.cpp`)

In `Impl::write_command(const JointCommand& cmd)`, after the per-actuator loop,
when `cmd.gripper_active`:
- Get/create the interconnect gripper command, ensure one motor command exists,
  and set:
  - `position = clamp(cmd.gripper, 0, 1) * 100.0f` (percent),
  - `velocity = kGripperVelocityPct` (default **100%** — full speed),
  - `force = kGripperForcePct` (default **50%** — moderate; conservative for
    bring-up, tune later).
- The two force/velocity constants live in the anonymous namespace next to the
  port constants, documented as tunable.

In `Impl::fill_feedback(JointFeedback& fb)`:
- Read the interconnect gripper motor position (percent) and write
  `fb.gripper = pos_pct / 100.0f`. Guard for the no-gripper case (a Gen3 with no
  interconnect/gripper should report 0 and not crash) — confirm the
  `interconnect().gripper_feedback().motor_size()` guard against the headers.

No change to `exchange`/`send`/`receive` signatures — they already route through
`write_command`/`fill_feedback`.

## Component 4 — `SimTransport` gripper echo (`include/kinova_lowlevel/sim_transport.h`, `src/sim_transport.cpp`)

`SimTransport` already stores `last_cmd_`. On `exchange`/`send`, copy
`last_cmd_.gripper` into `state_.gripper` so the echoed feedback reflects the
last commanded gripper (there is no simulated gripper dynamics — instantaneous
echo is the honest sim behavior). `receive()` returns the same `state_`. This
keeps the `--sim` bring-up path showing a plausible `gripper_state`.

## Component 5 — Server wiring (`apps/teleop_socket_server.cpp`)

- Construct `GripperInjector injector(*base_transport);` then
  `FeedbackTap transport(injector, snapshot);` (replacing the direct
  `FeedbackTap transport(*base_transport, snapshot);`).
- In the `kPoseTarget` handler, replace the `TODO(gripper)` /
  `last_gripper.store(...)` block with a single `injector.set_gripper(pkt.gripper);`.
- Feedback thread: set `pkt.gripper_state = fb.gripper;` (actual measured
  position from the snapshot) instead of `last_gripper`.
- These two changes make the `last_gripper` atomic dead — **delete it** and its
  declaration. The gripper target now lives in the injector; the reported value
  now comes from the feedback snapshot.

## Component 6 — Tests

- **Protocol parity unchanged.** No new protocol test; the existing
  `teleop_protocol_test.cpp` and its size asserts must still pass untouched
  (regression guard that we did NOT alter the wire).
- **`JointCommand`/`JointFeedback` defaults** — extend the existing sim/unit
  coverage minimally: a `SimTransport` test asserting that a commanded
  `gripper` round-trips into `gripper` feedback through `exchange()`, and that
  `gripper_active == false` leaves feedback gripper untouched. Mirror the style
  of `tests/sim_transport_test.cpp`.

## Out of scope (deferred)

- **Gripper velocity/force over the wire.** Hardcoded defaults for now; exposing
  them would need a protocol change, which we are explicitly avoiding.
- **Gripper force/grasp detection feedback** (current, object-detected flags) —
  the protocol has no field for it; not added.
- **Real-arm bring-up.** Live gripper motion and the KORTEX-build run stay
  deferred to an attended, in-person session per the integration runbook, same
  posture as the socket-server spec. This sub-project delivers and verifies the
  sim path + the KORTEX *build*.

## Acceptance criteria

1. **No wire regression** — `teleop_protocol_test` passes unchanged; the five
   size `static_assert`s (`20 / 84 / 157 / 28 / 261`) still hold.
2. **Sim path** (`--sim`) — a `POSE_TARGET` with `gripper = g` results in
   `FEEDBACK.gripper_state == g` (echo through `SimTransport`); `unit_tests`
   (incl. the new sim gripper test) pass via CTest on `abra`.
3. **KORTEX build** — the `KINOVA_ENABLE_KORTEX` build compiles with the
   interconnect gripper read/write against the real headers (accessor names
   confirmed on `abra`).
4. **Real-arm gripper motion** — deferred to an attended session.

## Build / test loop

Same as the socket-server spec — builds and tests run **on the Jetson `abra`**,
not the Mac (KORTEX + Pinocchio are Linux-only). Use the gitignored
`local_tools/` helpers:
- Full build + tests: `bash local_tools/build_on_abra.sh`.
- Fast iteration: `bash local_tools/sync_to_abra.sh && ssh abra 'cd
  ~/kinova-gen3-driver/build && cmake --build . -j unit_tests teleop_socket_server
  && ./unit_tests'`.
