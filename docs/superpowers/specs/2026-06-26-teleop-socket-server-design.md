# Teleop Socket Server — Design Spec

**Date:** 2026-06-26
**Status:** Approved for planning
**Scope:** A UDP socket-server front-end that bridges the Python VR-teleop
supervisor (`kinova-quest-teleop`) to the existing `CartesianImpedanceMode` over
the wire. Adds one executable, one public protocol header, one C++ parity test,
and the single sanctioned library edit (`Transport::clear_faults`). No change to
`RtExecutor`.

This is the **front-end / IPC sub-project** that the Cartesian impedance spec
([`2026-06-16-cartesian-impedance-controller-design.md`](2026-06-16-cartesian-impedance-controller-design.md))
explicitly deferred: *"Front-end / web server / IPC — separate sub-project,
designed later."*

## Goal

Let an external process drive `CartesianImpedanceMode` and read robot feedback
over UDP, **byte-for-byte compatible** with the already-built, already-tested
Python supervisor. The supervisor is the client; this server is the server. The
authoritative wire contract is `kinova_teleop/protocol.py` in the supervisor
repo; the C++ side mirrors it and the two MUST stay identical.

The requirements and a working, size-verified reference implementation live in
the supervisor repo under `docs/driver-frontend/` (`REQUIREMENTS.md`, plus
`reference/teleop_protocol.h.ref`, `reference/teleop_socket_server.cpp.ref`,
`reference/driver-lib-changes.patch`). This spec adopts that reference, adapted
to the actual driver headers (verified to match).

## Guiding principles (repo opinions honored first)

1. **Dynamics stays the sole owner of Pinocchio.** The server uses two
   `Dynamics` instances purely through the public `fk()` API — one for the RT
   mode, one owned by the feedback thread (`fk` is RT-safe but not safe to share
   across threads). No new Pinocchio consumers.
2. **Touch the driver core as little as possible.** Robot feedback escapes the
   1 kHz RT loop via a `Transport` **decorator** (`FeedbackTap`), not by editing
   `RtExecutor`. The decorator snapshots `JointFeedback` into a single-writer /
   single-reader lock-free seqlock at the exact point the loop reads it — **no
   allocation or lock on the RT path.**
3. **Mirror existing patterns.** The executable, its CMake block, and the test
   follow `apps/benchmark_cartesian_impedance.cpp`, its CMake target, and the
   `unit_tests` gtest target respectively, including the
   `KINOVA_ENABLE_KORTEX` / `KINOVA_NO_KORTEX` / `_OS_UNIX` build guards.

## Out of scope (explicitly deferred)

- **Gripper actuation.** The protocol carries a `gripper` field and the server
  stores + echoes the last commanded value (`gripper_state`), but no command
  path exists yet. Wiring `JointCommand`'s gripper field and the cyclic
  `interconnect.gripper_command` is deferred per requirements §5.3.
- **Real-arm bring-up.** Live motion, the KORTEX build run against hardware, and
  the `CLEAR_FAULTS`+`REHOME` recovery drill stay deferred to an **attended,
  in-person** session per the repo's safety posture
  ([`integration-runbook.md`](../../integration-runbook.md)). This sub-project
  delivers and verifies the sim path.
- **Any `RtExecutor` change.** Feedback is tapped via the decorator instead.
- **Generalizing the transport.** `FeedbackTap` is a generic "tap robot state
  out of the RT loop" mechanism and could lift into the library if a second
  consumer ever needs live feedback, but for now it stays in the app file (the
  demo timeline favors a problem-specific server).

## The wire protocol (HARD contract — must match exactly)

UDP, little-endian, fixed-size, packed (`#pragma pack(1)`). The host is
little-endian (x86_64 / aarch64 Jetson), matching Python's `<` struct format.

- Constants: `magic = 0x4B544C50` ("KTLP"), `version = 1`, `NUM_JOINTS = 7`.
- 20-byte header on every packet: `magic u32`, `version u16`, `msg_type u16`,
  `seq u32`, `timestamp_ns u64`.
- `msg_type`: `POSE_TARGET=1, SET_GAINS=2, CONTROL=3, FEEDBACK=4`.

**Total packet sizes are the acceptance check**, each guarded by a
`static_assert`:

| message | total bytes | body (after header) |
|---|---|---|
| POSE_TARGET | **84** | `pos[3] f64`, `quat_wxyz[4] f64`, `gripper f32`, `flags u32` |
| SET_GAINS | **157** | `Kx[6] f64`, `Dx[6] f64`, `nullspace_kp/kd f64`, `pinv_damping f64`, `torque_limit f64`, `gain_ramp_s f64`, `nullspace_on u8` |
| CONTROL | **28** | `command u32`, `control_seq u32` |
| FEEDBACK | **261** | `q[7] f64`, `qd[7] f64`, `tau[7] f64`, `ee_pos[3] f64`, `ee_quat_wxyz[4] f64`, `gripper_state f32`, `fault u8`, `frame_id u64`, `last_control_seq u32` |

Semantics:
- **Quaternions are `w, x, y, z`** (`Eigen::Quaterniond(w,x,y,z)`). Normalize on
  receipt.
- **Positions are meters, in the robot base frame.**
- `flags`: bit0 `ENGAGED`, bit1 `FREEZE`.
- `ControlCmd`: `CLEAR_FAULTS=1, REHOME=2, FREEZE=3, SHUTDOWN=4`.
- `Vector6` gain layout `[x y z | rx ry rz]`, matching `CartesianImpedanceParams`.

---

## Component 1 — `include/kinova_lowlevel/teleop_protocol.h` (new, public)

Adopt `teleop_protocol.h.ref` essentially verbatim: namespace `kinova::teleop`,
the `MsgType` / `ControlCmd` / `TargetFlags` enums, the five `#pragma pack(1)`
structs (`Header`, `PoseTargetPacket`, `GainsPacket`, `ControlPacket`,
`FeedbackPacket`), and the five `static_assert`s
(`20 / 84 / 157 / 28 / 261`). Lives alongside the other public headers so both
the app and the unit test include it. Verified field-for-field against
`protocol.py` (formats `<IHHIQ`, `<3d4dfI`, `<6d6d5dB`, `<II`, `<7d7d7d3d4dfBQI`).

## Component 2 — Driver-library change: runtime `clear_faults()`

The only edit to existing library files. Apply `driver-lib-changes.patch`:

- `Transport` gains a **default-no-op** `virtual void clear_faults() {}` — so
  `SimTransport` inherits the no-op and needs no change.
- `KortexTransport` overrides it: `base->ClearFaults()` then, if it was in
  low-level servoing, re-enter via `set_servoing_low_level()` to re-seed (a
  protective stop can drop the arm out of low-level mode). Wrapped in
  `try/catch`; guarded on `connected_` + `base`.

Verified: the patch's referenced internals (`Impl::connected_`, `Impl::base`,
`Impl::low_level_`, `set_servoing_low_level()`) all exist in
`src/kortex_transport.cpp` as written.

## Component 3 — `apps/teleop_socket_server.cpp` (new executable)

Adopt-and-adapt `teleop_socket_server.cpp.ref`. Internal units, each with one
clear purpose:

- **`Seqlock<T>`** (POD `T`) — single-writer / single-reader lock-free
  latest-value snapshot. Writer (RT thread) does a bounded copy between two
  release-fenced odd/even sequence bumps; reader retries while a write is in
  flight. No alloc, no lock, no blocking on the RT path.
- **`FeedbackTap : Transport`** — decorator wrapping the real transport; forwards
  every call and, in `exchange()` / `receive()`, calls `snap_.store(fb)`. This is
  what lets the feedback thread read robot state with zero change to `RtExecutor`.
- **`pose_from_packet`** — builds a `Pose` from a `PoseTargetPacket`
  (`Quaterniond(w,x,y,z)`, normalized).

Threading model (mirrors `benchmark_cartesian_impedance`):

- **Main thread** — `RtExecutor::run(g_stop)`: owns the 1 kHz RT loop, blocks
  until stop.
- **RX thread** — `recvfrom` with a 200 ms timeout (so it can observe `g_stop`);
  validates `magic`+`version`+min-size, drops malformed; learns the client
  address; dispatches:
  - `POSE_TARGET` → `mode.set_target(...)`, store `gripper`.
  - `SET_GAINS` → fill `CartesianImpedanceParams`, `mode.set_gains(...)`.
  - `CONTROL` → `CLEAR_FAULTS` = `transport.clear_faults()`; `REHOME` =
    `set_target(home_pose)`; `FREEZE` = `set_target(fk(latest q))`; `SHUTDOWN` =
    `g_stop = true`. **Always** echo `control_seq` into `last_control_seq`
    (the supervisor resends until it sees the ACK, even for no-op actions).
  - This is the **only** thread that calls `set_target` / `set_gains` (the mode's
    setters are single-writer lock-free).
- **Feedback thread** — its own `Dynamics dyn_fb`; loop at ~150 Hz (6 ms sleep):
  load the latest snapshot, `fk(q)` → EE pose, fill a `FeedbackPacket`
  (`q/qd/tau/fault/frame_id` from feedback, `ee_pos/ee_quat_wxyz` from `fk`,
  `gripper_state` = last commanded, `last_control_seq` = last ACK), `sendto` the
  last client address.

Startup sequence: `connect()` → `set_servoing_low_level()` → one `receive()` →
`home_pose = dyn_fb.fk(seed_fb.q)` (seeds `REHOME`'s target once). Then construct
the `CartesianImpedanceMode`, hand the `FeedbackTap` to a `RtExecutor`, request
the mode, and `run`. On exit: `safe_shutdown()`, set `g_stop`, join both threads,
close the socket.

CLI mirrors the benchmark: `--sim`, `--ip`, `--urdf`, `--port` (default 9095),
`--rate`, `--cpu`, `--rt-priority`. Raw POSIX sockets
(`<sys/socket.h>`, `<netinet/in.h>`, `<arpa/inet.h>`) — no new dependencies. The
`KortexTransport` include is guarded by `#ifndef KINOVA_NO_KORTEX` so the sim
build needs no KORTEX symbols.

## Component 4 — `tests/teleop_protocol_test.cpp` (new, in `unit_tests`)

A gtest in the existing `unit_tests` target (repo standard). Covers:

- **Sizes** — `EXPECT_EQ(sizeof(...), 20/84/157/28/261)` (runtime mirror of the
  header's compile-time `static_assert`s, so a size regression names itself in
  CTest output).
- **Round-trip** — populate a `PoseTargetPacket` and a `FeedbackPacket`, `memcpy`
  through a raw byte buffer, read back, and `EXPECT_EQ`/`EXPECT_DOUBLE_EQ` every
  field. Confirms the packed layout survives a serialize→deserialize cycle and
  documents the field offsets.

## Component 5 — `CMakeLists.txt`

- New `teleop_socket_server` executable, block copied from
  `benchmark_cartesian_impedance`: `target_link_libraries(... kinova_lowlevel
  Eigen3::Eigen)`, `target_include_directories(... include)`, and the
  `if(KINOVA_ENABLE_KORTEX) ... else() ... KINOVA_NO_KORTEX ...` guard so it
  builds in both sim-only (default, Jetson aarch64) and KORTEX modes.
- Add `tests/teleop_protocol_test.cpp` to the `unit_tests` sources.

---

## Data flow

```
Python supervisor ──UDP:9095──▶ RX thread ──set_target/set_gains──▶ CartesianImpedanceMode
                                                                          │ (RT thread)
                                                       RtExecutor.run ◀────┘
                                                          │ exchange()/receive()
                                                    FeedbackTap.store ──▶ Seqlock
                                                                            │
Python supervisor ◀──UDP FEEDBACK── Feedback thread ◀── load + dyn_fb.fk ──┘
```

## Build / test loop

Builds and tests run **on the Jetson `abra`, not the Mac** (KORTEX + RT APIs are
Linux-only; Pinocchio lives under the Jetson's `cmeel.prefix`). Helper scripts in
the gitignored `local_tools/`:

- Full build + all tests: `bash local_tools/build_on_abra.sh`.
- Fast iteration: `bash local_tools/sync_to_abra.sh && ssh abra 'cd
  ~/kinova-gen3-driver/build && cmake --build . -j unit_tests teleop_socket_server
  && ./unit_tests --gtest_filter="TeleopProtocol*"'`.

## Acceptance criteria

1. **Protocol parity** — the five `static_assert`s compile
   (`20 / 84 / 157 / 28 / 261`); the new gtest passes; the supervisor's
   `pytest tests/test_protocol.py -q` passes (canonical Python sizes).
2. **Sim bring-up** (`--sim`) — the server binds, accepts `POSE_TARGET`, and
   streams parseable `FEEDBACK`. Motion stays frozen (`SimTransport` is
   echo-only; expected). *(Live loopback against the supervisor is a manual
   check, not gated here.)*
3. **No RT regression** — the seqlock adds only a bounded copy on the RT path; no
   alloc/lock. (Quantitative cycle-time parity vs. the impedance benchmark is a
   hardware check, deferred with real-arm bring-up.)
4. **Real-arm** — deferred to an attended session per the integration runbook.

## Verification plan for this sub-project

Build the sim server on `abra`, run `unit_tests` (incl. the new protocol test)
via CTest, and run the supervisor's `pytest tests/test_protocol.py -q`. Do not
run a live socket loopback or any hardware path in this session.
