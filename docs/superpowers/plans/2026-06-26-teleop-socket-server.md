# Teleop Socket Server Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a UDP socket-server executable (`teleop_socket_server`) that bridges the Python VR-teleop supervisor to the existing `CartesianImpedanceMode`, byte-for-byte compatible with `kinova_teleop/protocol.py`.

**Architecture:** One new public protocol header (`teleop_protocol.h`) mirrors the Python wire contract with packed structs + `static_assert` sizes. One new app (`apps/teleop_socket_server.cpp`) runs the 1 kHz RT loop on the main thread, an RX thread that dispatches inbound packets to the mode's single-writer setters, and a feedback thread that streams robot state at ~150 Hz. Robot feedback escapes the RT loop via a `Transport` **decorator** (`FeedbackTap` + a lock-free seqlock) with no change to `RtExecutor`. The one library edit is a default-no-op `Transport::clear_faults()` overridden in `KortexTransport`.

**Tech Stack:** C++17, Eigen 3.4, Pinocchio (FK), raw POSIX UDP sockets (libc), GoogleTest. Builds + tests on the Jetson `abra` (aarch64 / PREEMPT_RT).

## Global Constraints

- **Wire contract is HARD.** `magic = 0x4B544C50` ("KTLP"), `version = 1`, `NUM_JOINTS = 7`. Little-endian, packed (`#pragma pack(1)`). Quaternions serialized **w, x, y, z**. Positions meters, base frame.
- **Exact total packet sizes (the acceptance check):** Header **20**, POSE_TARGET **84**, SET_GAINS **157**, CONTROL **28**, FEEDBACK **261**. Each guarded by a `static_assert`.
- **No allocation or lock on the RT path.** The feedback snapshot is a bounded copy via seqlock only.
- **Dynamics is the sole owner of Pinocchio.** Use it only through the public `fk()`; the feedback thread owns its own `Dynamics` instance (`fk` is RT-safe but not safe to share across threads). All `set_target`/`set_gains` calls come from a **single thread** (the RX thread).
- **Builds in both sim-only (default, `KINOVA_NO_KORTEX`) and KORTEX modes.** `src/kortex_transport.cpp` compiles only when `KINOVA_ENABLE_KORTEX=ON`.
- **Raw POSIX sockets only** (`<sys/socket.h>`, `<netinet/in.h>`, `<arpa/inet.h>`) — no new dependencies.

## Build/test loop (read once)

This project builds and tests **on the Jetson `abra`, not the Mac** (RT APIs + KORTEX SDK + Pinocchio are Linux/aarch64 only). Helper scripts live in the gitignored `local_tools/`.

- **Full build + all tests (default sim build):** `bash local_tools/build_on_abra.sh` — rsyncs to `abra`, configures with the baked-in Pinocchio prefix, builds, runs `ctest --output-on-failure`.
- **Fast single-target iteration:**
  ```sh
  bash local_tools/sync_to_abra.sh && \
  ssh abra 'cd ~/kinova-gen3-driver/build && cmake --build . -j unit_tests teleop_socket_server && ./unit_tests --gtest_filter="<FILTER>"'
  ```
- A test that "fails to compile" counts as a failing test in TDD — that is the expected first state for each new test.
- Commit after each task with the message shown. **Do not push** (the user pushes).

## File structure

| File | Responsibility | New/Mod |
|---|---|---|
| `include/kinova_lowlevel/teleop_protocol.h` | Packed wire structs + enums + `static_assert` sizes. C++ half of `protocol.py`. | New |
| `tests/teleop_protocol_test.cpp` | gtest: size parity + serialize/deserialize round-trip. | New |
| `include/kinova_lowlevel/transport.h` | + default-no-op `virtual void clear_faults() {}` | Mod |
| `include/kinova_lowlevel/kortex_transport.h` | + `void clear_faults() override;` decl | Mod |
| `src/kortex_transport.cpp` | `clear_faults()` impl (KORTEX-only TU) | Mod |
| `tests/sim_transport_test.cpp` | + test that `Transport::clear_faults()` default is a safe no-op | Mod |
| `apps/teleop_socket_server.cpp` | the server: seqlock, FeedbackTap, RX + feedback threads, main | New |
| `CMakeLists.txt` | new `teleop_socket_server` target; add the protocol test source | Mod |

---

## Task 1: Wire-protocol header + parity test

Self-contained: produces the contract header and proves its sizes/layout, independent of the server.

**Files:**
- Create: `include/kinova_lowlevel/teleop_protocol.h`
- Create: `tests/teleop_protocol_test.cpp`
- Modify: `CMakeLists.txt` (add the test source to the `unit_tests` target)

**Interfaces:**
- Produces (consumed by Task 3): namespace `kinova::teleop` with `kMagic`, `kVersion`, `kNumJointsProto`, enums `MsgType {kPoseTarget=1,kSetGains=2,kControl=3,kFeedback=4}`, `ControlCmd {kClearFaults=1,kRehome=2,kFreeze=3,kShutdown=4}`, `TargetFlags`, packed structs `Header`, `PoseTargetPacket`, `GainsPacket`, `ControlPacket`, `FeedbackPacket`, and `bool valid_header(const Header&, MsgType, uint32_t)`.

- [ ] **Step 1: Write the failing test**

Create `tests/teleop_protocol_test.cpp`:

```cpp
#include <gtest/gtest.h>
#include <cstring>
#include "kinova_lowlevel/teleop_protocol.h"

namespace tp = kinova::teleop;

// Sizes are the hard acceptance check; this mirrors the header's compile-time
// static_asserts at runtime so a regression names itself in CTest output. These
// match the Python codec (kinova_teleop/protocol.py / tests/test_protocol.py).
TEST(TeleopProtocol, PacketSizes) {
  EXPECT_EQ(sizeof(tp::Header), 20u);
  EXPECT_EQ(sizeof(tp::PoseTargetPacket), 84u);
  EXPECT_EQ(sizeof(tp::GainsPacket), 157u);
  EXPECT_EQ(sizeof(tp::ControlPacket), 28u);
  EXPECT_EQ(sizeof(tp::FeedbackPacket), 261u);
}

TEST(TeleopProtocol, Constants) {
  EXPECT_EQ(tp::kMagic, 0x4B544C50u);  // "KTLP"
  EXPECT_EQ(tp::kVersion, 1u);
  EXPECT_EQ(tp::kNumJointsProto, 7);
}

// Serialize -> raw bytes -> deserialize must preserve every field bit-for-bit.
// Proves the packed layout survives a memcpy round-trip through a byte buffer
// (the exact path the socket takes).
TEST(TeleopProtocol, PoseTargetRoundTrip) {
  tp::PoseTargetPacket in{};
  in.h.magic = tp::kMagic;
  in.h.version = tp::kVersion;
  in.h.msg_type = static_cast<uint16_t>(tp::MsgType::kPoseTarget);
  in.h.seq = 42;
  in.h.timestamp_ns = 123456789ull;
  in.pos[0] = 0.10; in.pos[1] = -0.20; in.pos[2] = 0.30;
  in.quat_wxyz[0] = 1.0; in.quat_wxyz[1] = 0.0;
  in.quat_wxyz[2] = 0.0; in.quat_wxyz[3] = 0.0;
  in.gripper = 0.5f;
  in.flags = tp::kFlagEngaged | tp::kFlagFreeze;

  unsigned char buf[sizeof(in)];
  std::memcpy(buf, &in, sizeof(in));
  tp::PoseTargetPacket out{};
  std::memcpy(&out, buf, sizeof(out));

  EXPECT_EQ(out.h.magic, in.h.magic);
  EXPECT_EQ(out.h.msg_type, in.h.msg_type);
  EXPECT_EQ(out.h.seq, in.h.seq);
  EXPECT_EQ(out.h.timestamp_ns, in.h.timestamp_ns);
  for (int i = 0; i < 3; ++i) EXPECT_DOUBLE_EQ(out.pos[i], in.pos[i]);
  for (int i = 0; i < 4; ++i) EXPECT_DOUBLE_EQ(out.quat_wxyz[i], in.quat_wxyz[i]);
  EXPECT_FLOAT_EQ(out.gripper, in.gripper);
  EXPECT_EQ(out.flags, in.flags);
}

TEST(TeleopProtocol, FeedbackRoundTrip) {
  tp::FeedbackPacket in{};
  in.h.magic = tp::kMagic;
  in.h.version = tp::kVersion;
  in.h.msg_type = static_cast<uint16_t>(tp::MsgType::kFeedback);
  for (int i = 0; i < 7; ++i) { in.q[i] = i * 0.1; in.qd[i] = -i * 0.2; in.tau[i] = i * 1.5; }
  in.ee_pos[0] = 0.4; in.ee_pos[1] = 0.0; in.ee_pos[2] = 0.5;
  in.ee_quat_wxyz[0] = 0.0; in.ee_quat_wxyz[1] = 1.0;
  in.ee_quat_wxyz[2] = 0.0; in.ee_quat_wxyz[3] = 0.0;
  in.gripper_state = 0.25f;
  in.fault = 1;
  in.frame_id = 99999ull;
  in.last_control_seq = 7;

  unsigned char buf[sizeof(in)];
  std::memcpy(buf, &in, sizeof(in));
  tp::FeedbackPacket out{};
  std::memcpy(&out, buf, sizeof(out));

  for (int i = 0; i < 7; ++i) {
    EXPECT_DOUBLE_EQ(out.q[i], in.q[i]);
    EXPECT_DOUBLE_EQ(out.qd[i], in.qd[i]);
    EXPECT_DOUBLE_EQ(out.tau[i], in.tau[i]);
  }
  for (int i = 0; i < 3; ++i) EXPECT_DOUBLE_EQ(out.ee_pos[i], in.ee_pos[i]);
  for (int i = 0; i < 4; ++i) EXPECT_DOUBLE_EQ(out.ee_quat_wxyz[i], in.ee_quat_wxyz[i]);
  EXPECT_FLOAT_EQ(out.gripper_state, in.gripper_state);
  EXPECT_EQ(out.fault, in.fault);
  EXPECT_EQ(out.frame_id, in.frame_id);
  EXPECT_EQ(out.last_control_seq, in.last_control_seq);
}
```

Then add the source to the `unit_tests` target in `CMakeLists.txt`. Find the `add_executable(unit_tests ...)` list (currently ends with `tests/cartesian_impedance_mode_test.cpp)`) and append the new file:

```cmake
add_executable(unit_tests
    tests/smoke_test.cpp
    tests/cartesian_types_test.cpp
    tests/joint_types_test.cpp
    tests/dynamics_test.cpp
    tests/telemetry_test.cpp
    tests/rt_system_test.cpp
    tests/sim_transport_test.cpp
    tests/gravity_comp_mode_test.cpp
    tests/rt_safety_test.cpp
    tests/cartesian_test.cpp
    tests/cartesian_impedance_mode_test.cpp
    tests/teleop_protocol_test.cpp)
```

- [ ] **Step 2: Run the test — verify it FAILS to compile**

Run:
```sh
bash local_tools/sync_to_abra.sh && \
ssh abra 'cd ~/kinova-gen3-driver/build && cmake .. -DCMAKE_PREFIX_PATH=/usr/local/lib/python3.10/dist-packages/cmeel.prefix && cmake --build . -j unit_tests'
```
Expected: FAIL — `fatal error: kinova_lowlevel/teleop_protocol.h: No such file or directory`.

- [ ] **Step 3: Create the protocol header**

Create `include/kinova_lowlevel/teleop_protocol.h`:

```cpp
#pragma once
// Wire protocol for the teleop socket server. This is the C++ half of the
// contract defined in the Python supervisor's `kinova_teleop/protocol.py`; the
// two MUST stay byte-for-byte identical. Layout is little-endian, packed (no
// padding). Quaternions are serialized w, x, y, z (Eigen Quaterniond order).
// Positions are meters in the robot base frame. Bump kVersion on any change and
// update the Python side; the static_asserts below guard the C++ sizes.
//
// Endianness note: x86_64 and aarch64 (Jetson) are little-endian, matching the
// Python `<` struct format. This header assumes a little-endian host.
#include <cstdint>

namespace kinova::teleop {

inline constexpr uint32_t kMagic = 0x4B544C50;  // "KTLP"
inline constexpr uint16_t kVersion = 1;
inline constexpr int kNumJointsProto = 7;

enum class MsgType : uint16_t {
  kPoseTarget = 1,
  kSetGains = 2,
  kControl = 3,
  kFeedback = 4,
};

enum class ControlCmd : uint32_t {
  kClearFaults = 1,
  kRehome = 2,
  kFreeze = 3,
  kShutdown = 4,
};

enum TargetFlags : uint32_t {
  kFlagNone = 0,
  kFlagEngaged = 1u << 0,
  kFlagFreeze = 1u << 1,
};

#pragma pack(push, 1)

struct Header {
  uint32_t magic;
  uint16_t version;
  uint16_t msg_type;
  uint32_t seq;
  uint64_t timestamp_ns;
};
static_assert(sizeof(Header) == 20, "Header layout must match Python");

struct PoseTargetPacket {
  Header h;
  double pos[3];          // x, y, z [m], base frame
  double quat_wxyz[4];    // w, x, y, z
  float gripper;          // 0=open .. 1=closed
  uint32_t flags;         // TargetFlags
};
static_assert(sizeof(PoseTargetPacket) == 84, "PoseTarget layout mismatch");

struct GainsPacket {
  Header h;
  double Kx[6];           // [x y z | rx ry rz]
  double Dx[6];
  double nullspace_kp;
  double nullspace_kd;
  double pinv_damping;
  double torque_limit;
  double gain_ramp_s;
  uint8_t nullspace_on;
};
static_assert(sizeof(GainsPacket) == 157, "Gains layout mismatch");

struct ControlPacket {
  Header h;
  uint32_t command;       // ControlCmd
  uint32_t control_seq;
};
static_assert(sizeof(ControlPacket) == 28, "Control layout mismatch");

struct FeedbackPacket {
  Header h;
  double q[7];
  double qd[7];
  double tau[7];
  double ee_pos[3];
  double ee_quat_wxyz[4];
  float gripper_state;
  uint8_t fault;
  uint64_t frame_id;
  uint32_t last_control_seq;
};
static_assert(sizeof(FeedbackPacket) == 261, "Feedback layout mismatch");

#pragma pack(pop)

inline bool valid_header(const Header& h, MsgType expected, uint32_t /*size_seen*/) {
  return h.magic == kMagic && h.version == kVersion &&
         h.msg_type == static_cast<uint16_t>(expected);
}

}  // namespace kinova::teleop
```

- [ ] **Step 4: Run the test — verify it PASSES**

Run:
```sh
bash local_tools/sync_to_abra.sh && \
ssh abra 'cd ~/kinova-gen3-driver/build && cmake --build . -j unit_tests && ./unit_tests --gtest_filter="TeleopProtocol*"'
```
Expected: PASS — 4 tests (`PacketSizes`, `Constants`, `PoseTargetRoundTrip`, `FeedbackRoundTrip`).

- [ ] **Step 5: Commit**

```sh
git add include/kinova_lowlevel/teleop_protocol.h tests/teleop_protocol_test.cpp CMakeLists.txt
git commit -m "feat(teleop): wire-protocol header + parity test"
```

---

## Task 2: Runtime `clear_faults()` — the one library edit

**Files:**
- Modify: `include/kinova_lowlevel/transport.h` (add default-no-op virtual)
- Modify: `include/kinova_lowlevel/kortex_transport.h` (add override declaration)
- Modify: `src/kortex_transport.cpp` (implement; KORTEX-only TU)
- Modify: `tests/sim_transport_test.cpp` (test the base default no-op via SimTransport)

**Interfaces:**
- Produces (consumed by Task 3): `virtual void Transport::clear_faults()` — callable on any `Transport&`. `SimTransport` inherits the no-op; `KortexTransport` overrides it.

**Note:** In the default sim build (`KINOVA_ENABLE_KORTEX=OFF`), `src/kortex_transport.cpp` is **not** compiled, so the override is exercised only in the KORTEX build. The sim-build test below targets the `Transport` base-class default reached through `SimTransport`.

- [ ] **Step 1: Write the failing test**

In `tests/sim_transport_test.cpp`, append this test. The file already has `#include "kinova_lowlevel/sim_transport.h"` and `using namespace kinova;` at the top (so `Transport` is available transitively and names are unqualified) — match that style:

```cpp
// SimTransport has no fault concept, so it inherits Transport's default-no-op
// clear_faults(). The teleop server calls clear_faults() on whatever Transport
// it holds; this guards that the sim path is a safe no-op (no throw, state
// unchanged) so protocol bring-up under --sim never trips on it.
TEST(SimTransport, ClearFaultsIsNoOp) {
  JointFeedback init;
  SimTransport t(init);
  Transport& base = t;  // call through the base interface
  EXPECT_NO_THROW(base.clear_faults());
}
```

- [ ] **Step 2: Run the test — verify it FAILS to compile**

Run:
```sh
bash local_tools/sync_to_abra.sh && \
ssh abra 'cd ~/kinova-gen3-driver/build && cmake --build . -j unit_tests'
```
Expected: FAIL — `'class kinova::Transport' has no member named 'clear_faults'`.

- [ ] **Step 3: Add the default-no-op virtual to `Transport`**

In `include/kinova_lowlevel/transport.h`, add after the `safe_shutdown()` pure virtual (currently the last method before the closing brace):

```cpp
  virtual void safe_shutdown() = 0;
  // Runtime fault recovery (e.g. after a protective stop). Default no-op so
  // transports without faults (SimTransport) need not implement it.
  virtual void clear_faults() {}
```

- [ ] **Step 4: Declare the override on `KortexTransport`**

In `include/kinova_lowlevel/kortex_transport.h`, add after `void safe_shutdown() override;`:

```cpp
  void safe_shutdown() override;
  void clear_faults() override;
```

- [ ] **Step 5: Implement `KortexTransport::clear_faults()`**

In `src/kortex_transport.cpp`, add immediately after the `KortexTransport::safe_shutdown()` definition (before the closing `}  // namespace kinova`):

```cpp
void KortexTransport::clear_faults() {
  auto& I = *impl_;
  if (!I.connected_ || !I.base) return;
  try {
    I.base->ClearFaults();
  } catch (...) {
  }
  // A protective stop can drop the robot out of low-level servoing. If we were
  // in low-level mode, re-enter + re-seed so the RT loop can resume commanding.
  if (I.low_level_) {
    try {
      set_servoing_low_level();
    } catch (...) {
    }
  }
}
```

- [ ] **Step 6: Run the test — verify it PASSES (sim build)**

Run:
```sh
bash local_tools/sync_to_abra.sh && \
ssh abra 'cd ~/kinova-gen3-driver/build && cmake --build . -j unit_tests && ./unit_tests --gtest_filter="SimTransport.ClearFaultsIsNoOp"'
```
Expected: PASS.

- [ ] **Step 7: Commit**

```sh
git add include/kinova_lowlevel/transport.h include/kinova_lowlevel/kortex_transport.h src/kortex_transport.cpp tests/sim_transport_test.cpp
git commit -m "feat(transport): runtime clear_faults() (default no-op; KortexTransport override)"
```

---

## Task 3: The `teleop_socket_server` executable + CMake target

**Files:**
- Create: `apps/teleop_socket_server.cpp`
- Modify: `CMakeLists.txt` (new executable target, mirroring `benchmark_cartesian_impedance`)

**Interfaces:**
- Consumes: `kinova::teleop` protocol (Task 1); `Transport::clear_faults()` (Task 2); existing `CartesianImpedanceMode`, `CartesianImpedanceParams`, `Dynamics::fk`, `RtExecutor`, `SimTransport`, `KortexTransport`, `SampleRing`, `Pacing`.
- Produces: the `teleop_socket_server` binary (no downstream code consumes it).

**Note on testing:** the testable logic (the wire layout) is fully covered by Task 1. This task's deliverable is a binary that compiles in the default sim build and launches/binds the UDP port. There is no unit test for `main()`; verification is a build + a manual launch smoke (Step 4) and the cross-language pytest (Step 5).

- [ ] **Step 1: Create the server source**

Create `apps/teleop_socket_server.cpp`:

```cpp
// teleop_socket_server — bridges the Python VR-teleop supervisor to the
// CartesianImpedanceMode seam over UDP. The RT loop (RtExecutor) owns the main
// thread; two helper threads handle the socket:
//
//   * rx thread:       receive POSE_TARGET / SET_GAINS / CONTROL, apply via the
//                      mode's non-RT setters (single writer, as required).
//   * feedback thread: read the latest JointFeedback snapshot, compute EE pose
//                      via its own Dynamics, and stream FEEDBACK back to the
//                      last client address.
//
// Robot feedback is captured without touching the driver core: FeedbackTap wraps
// the Transport and snapshots `fb` at the exact point the RT loop reads it.
//
//   ./teleop_socket_server --sim  --urdf ../models/gen3_7dof_2f85.urdf --port 9095
//   ./teleop_socket_server --ip 192.168.1.10 --urdf ../models/gen3_7dof_2f85.urdf
//
// SimTransport is echo-only (the arm will not move); use it for protocol
// bring-up. Real motion requires the KORTEX build against the arm.
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cstring>
#include <csignal>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <Eigen/Dense>

#include "kinova_lowlevel/cartesian_impedance_mode.h"
#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/rt_executor.h"
#include "kinova_lowlevel/sim_transport.h"
#include "kinova_lowlevel/telemetry.h"
#include "kinova_lowlevel/teleop_protocol.h"
#include "kinova_lowlevel/transport.h"
#ifndef KINOVA_NO_KORTEX
#include "kinova_lowlevel/kortex_transport.h"
#endif

using namespace kinova;
namespace tp = kinova::teleop;

namespace {
std::atomic<bool> g_stop{false};
void on_sigint(int) { g_stop.store(true); }

int64_t ns_now() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return int64_t(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

// Single-writer / single-reader lock-free latest-value snapshot (seqlock). The
// RT thread is the only writer; the feedback thread is the only reader. POD T.
template <class T>
class Seqlock {
 public:
  void store(const T& v) {  // writer (RT thread): bounded, no alloc/lock
    const uint32_t s = seq_.load(std::memory_order_relaxed);
    seq_.store(s + 1, std::memory_order_release);  // mark write in progress (odd)
    std::atomic_thread_fence(std::memory_order_release);
    data_ = v;
    std::atomic_thread_fence(std::memory_order_release);
    seq_.store(s + 2, std::memory_order_release);  // publish (even)
  }
  bool load(T& out) const {  // reader: retry while a write is in flight
    for (int i = 0; i < 16; ++i) {
      const uint32_t s1 = seq_.load(std::memory_order_acquire);
      if (s1 & 1u) continue;
      std::atomic_thread_fence(std::memory_order_acquire);
      out = data_;
      std::atomic_thread_fence(std::memory_order_acquire);
      if (seq_.load(std::memory_order_acquire) == s1) return true;
    }
    return false;
  }

 private:
  std::atomic<uint32_t> seq_{0};
  T data_{};
};

// Transport decorator: forwards everything to the wrapped transport and, on each
// feedback-producing call, publishes the latest JointFeedback into a snapshot.
// This is what lets the feedback thread read robot state without any change to
// RtExecutor or the driver library.
class FeedbackTap : public Transport {
 public:
  FeedbackTap(Transport& inner, Seqlock<JointFeedback>& snap)
      : inner_(inner), snap_(snap) {}
  void connect() override { inner_.connect(); }
  void set_servoing_low_level() override { inner_.set_servoing_low_level(); }
  void set_actuator_modes(const ActuatorModes& m) override {
    inner_.set_actuator_modes(m);
  }
  void exchange(const JointCommand& c, JointFeedback& fb) override {
    inner_.exchange(c, fb);
    snap_.store(fb);
  }
  void send(const JointCommand& c) override { inner_.send(c); }
  void receive(JointFeedback& fb) override {
    inner_.receive(fb);
    snap_.store(fb);
  }
  void safe_shutdown() override { inner_.safe_shutdown(); }
  void clear_faults() override { inner_.clear_faults(); }

 private:
  Transport& inner_;
  Seqlock<JointFeedback>& snap_;
};

Pose pose_from_packet(const tp::PoseTargetPacket& p) {
  Pose x;
  x.p = Eigen::Vector3d(p.pos[0], p.pos[1], p.pos[2]);
  x.R = Eigen::Quaterniond(p.quat_wxyz[0], p.quat_wxyz[1], p.quat_wxyz[2],
                           p.quat_wxyz[3]);
  x.R.normalize();
  return x;
}

}  // namespace

int main(int argc, char** argv) {
  std::string ip;
  std::string urdf = "../models/gen3_7dof_2f85.urdf";
  bool use_sim = false;
  int port = 9095;
  double rate_hz = 1000.0;
  int cpu = -1;
  int rt_priority = 80;
  CartesianImpedanceParams gains;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&](const char* name) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << name << " needs a value\n";
        std::exit(2);
      }
      return argv[++i];
    };
    if (a == "--ip") ip = next("--ip");
    else if (a == "--sim") use_sim = true;
    else if (a == "--urdf") urdf = next("--urdf");
    else if (a == "--port") port = std::stoi(next("--port"));
    else if (a == "--rate") rate_hz = std::stod(next("--rate"));
    else if (a == "--cpu") cpu = std::stoi(next("--cpu"));
    else if (a == "--rt-priority") rt_priority = std::stoi(next("--rt-priority"));
    else {
      std::cerr << "unknown arg: " << a << "\n";
      std::exit(2);
    }
  }

  std::cout << "[teleop-srv] urdf=" << urdf << " rate=" << rate_hz << "Hz port="
            << port << " sim=" << (use_sim ? "yes" : "no") << "\n";

  // Two Dynamics instances: one for the mode (RT thread), one for the feedback
  // thread (fk is RT-safe but not safe to share across threads).
  Dynamics dyn(urdf);
  Dynamics dyn_fb(urdf);

  std::unique_ptr<Transport> base_transport;
  if (use_sim) {
    JointFeedback init;
    base_transport = std::make_unique<SimTransport>(init);
  } else {
#ifndef KINOVA_NO_KORTEX
    if (ip.empty()) {
      std::cerr << "real-robot mode requires --ip <addr> (or pass --sim)\n";
      return 2;
    }
    base_transport = std::make_unique<KortexTransport>(ip);
#else
    std::cerr << "built without KORTEX; only --sim is available\n";
    return 2;
#endif
  }

  Seqlock<JointFeedback> snapshot;
  FeedbackTap transport(*base_transport, snapshot);

  std::signal(SIGINT, on_sigint);

  // --- UDP socket ---------------------------------------------------------
  int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    std::perror("socket");
    return 1;
  }
  sockaddr_in local{};
  local.sin_family = AF_INET;
  local.sin_addr.s_addr = htonl(INADDR_ANY);
  local.sin_port = htons(static_cast<uint16_t>(port));
  if (::bind(sock, reinterpret_cast<sockaddr*>(&local), sizeof(local)) < 0) {
    std::perror("bind");
    return 1;
  }
  timeval rx_timeout{0, 200'000};  // 200 ms so the rx thread can observe g_stop
  ::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &rx_timeout, sizeof(rx_timeout));

  // Shared state between socket threads.
  std::mutex client_mu;
  sockaddr_in client_addr{};
  bool have_client = false;
  std::atomic<float> last_gripper{0.0f};
  std::atomic<uint32_t> last_control_seq{0};

  transport.connect();
  transport.set_servoing_low_level();

  // Seed the home pose (for REHOME) from the current joint state.
  JointFeedback seed_fb;
  transport.receive(seed_fb);
  const Pose home_pose = dyn_fb.fk(seed_fb.q);

  CartesianImpedanceMode mode(dyn, gains);

  // --- rx thread ----------------------------------------------------------
  std::thread rx([&] {
    char buf[512];
    while (!g_stop.load(std::memory_order_acquire)) {
      sockaddr_in src{};
      socklen_t srclen = sizeof(src);
      const ssize_t n = ::recvfrom(sock, buf, sizeof(buf), 0,
                                   reinterpret_cast<sockaddr*>(&src), &srclen);
      if (n < static_cast<ssize_t>(sizeof(tp::Header))) continue;
      tp::Header h;
      std::memcpy(&h, buf, sizeof(h));
      if (h.magic != tp::kMagic || h.version != tp::kVersion) continue;

      {  // remember who to stream feedback to
        std::lock_guard<std::mutex> lk(client_mu);
        client_addr = src;
        have_client = true;
      }

      switch (static_cast<tp::MsgType>(h.msg_type)) {
        case tp::MsgType::kPoseTarget: {
          if (n < static_cast<ssize_t>(sizeof(tp::PoseTargetPacket))) break;
          tp::PoseTargetPacket pkt;
          std::memcpy(&pkt, buf, sizeof(pkt));
          mode.set_target(pose_from_packet(pkt));
          last_gripper.store(pkt.gripper, std::memory_order_relaxed);
          // TODO(gripper): once JointCommand carries a gripper field, forward
          // pkt.gripper to the cyclic interconnect command here.
          break;
        }
        case tp::MsgType::kSetGains: {
          if (n < static_cast<ssize_t>(sizeof(tp::GainsPacket))) break;
          tp::GainsPacket pkt;
          std::memcpy(&pkt, buf, sizeof(pkt));
          CartesianImpedanceParams p;
          for (int k = 0; k < 6; ++k) {
            p.Kx[k] = pkt.Kx[k];
            p.Dx[k] = pkt.Dx[k];
          }
          p.nullspace_kp = pkt.nullspace_kp;
          p.nullspace_kd = pkt.nullspace_kd;
          p.pinv_damping = pkt.pinv_damping;
          p.torque_limit = pkt.torque_limit;
          p.gain_ramp_s = pkt.gain_ramp_s;
          p.nullspace_on = pkt.nullspace_on != 0;
          mode.set_gains(p);
          break;
        }
        case tp::MsgType::kControl: {
          if (n < static_cast<ssize_t>(sizeof(tp::ControlPacket))) break;
          tp::ControlPacket pkt;
          std::memcpy(&pkt, buf, sizeof(pkt));
          switch (static_cast<tp::ControlCmd>(pkt.command)) {
            case tp::ControlCmd::kClearFaults:
              transport.clear_faults();
              break;
            case tp::ControlCmd::kRehome:
              mode.set_target(home_pose);
              break;
            case tp::ControlCmd::kFreeze: {
              JointFeedback fb;
              if (snapshot.load(fb)) mode.set_target(dyn_fb.fk(fb.q));
              break;
            }
            case tp::ControlCmd::kShutdown:
              g_stop.store(true, std::memory_order_release);
              break;
          }
          // Acknowledge regardless so the supervisor's resend-until-ack stops.
          last_control_seq.store(pkt.control_seq, std::memory_order_release);
          break;
        }
        default:
          break;
      }
    }
  });

  // --- feedback thread ----------------------------------------------------
  std::thread feedback([&] {
    // Its own Dynamics for thread-safe fk. ~150 Hz publish.
    while (!g_stop.load(std::memory_order_acquire)) {
      sockaddr_in dest{};
      bool send = false;
      {
        std::lock_guard<std::mutex> lk(client_mu);
        if (have_client) {
          dest = client_addr;
          send = true;
        }
      }
      JointFeedback fb;
      if (send && snapshot.load(fb)) {
        const Pose ee = dyn_fb.fk(fb.q);
        tp::FeedbackPacket pkt{};
        pkt.h.magic = tp::kMagic;
        pkt.h.version = tp::kVersion;
        pkt.h.msg_type = static_cast<uint16_t>(tp::MsgType::kFeedback);
        pkt.h.seq = 0;
        pkt.h.timestamp_ns = static_cast<uint64_t>(ns_now());
        for (int k = 0; k < kNumJoints; ++k) {
          pkt.q[k] = fb.q[k];
          pkt.qd[k] = fb.qd[k];
          pkt.tau[k] = fb.tau[k];
        }
        pkt.ee_pos[0] = ee.p.x();
        pkt.ee_pos[1] = ee.p.y();
        pkt.ee_pos[2] = ee.p.z();
        pkt.ee_quat_wxyz[0] = ee.R.w();
        pkt.ee_quat_wxyz[1] = ee.R.x();
        pkt.ee_quat_wxyz[2] = ee.R.y();
        pkt.ee_quat_wxyz[3] = ee.R.z();
        pkt.gripper_state = last_gripper.load(std::memory_order_relaxed);
        pkt.fault = fb.fault ? 1 : 0;
        pkt.frame_id = fb.frame_id;
        pkt.last_control_seq = last_control_seq.load(std::memory_order_acquire);
        ::sendto(sock, &pkt, sizeof(pkt), 0,
                 reinterpret_cast<sockaddr*>(&dest), sizeof(dest));
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(6));  // ~150 Hz
    }
  });

  // Telemetry ring is required by RtExecutor but not consumed here (timing
  // telemetry is not part of teleop); it drops-don't-block when full.
  SampleRing ring(1 << 16);
  RtExecutor ex(transport, ring,
                {rate_hz, Pacing::kSleepSpin, {rt_priority, cpu, true}});
  ex.request_mode(&mode);

  std::cout << "[teleop-srv] listening; RT loop running. Ctrl-C to stop.\n";
  ex.run(g_stop);  // blocks on the main (RT) thread until stop

  transport.safe_shutdown();
  g_stop.store(true, std::memory_order_release);
  rx.join();
  feedback.join();
  ::close(sock);
  std::cout << "[teleop-srv] stopped.\n";
  return 0;
}
```

- [ ] **Step 2: Add the CMake target**

In `CMakeLists.txt`, after the `benchmark_cartesian_impedance` block (ends at its closing `endif()`, just before the `# --- tests ---` section), add:

```cmake
# teleop_socket_server bridges the Python VR-teleop supervisor to
# CartesianImpedanceMode over UDP. Sim build (default) exercises the protocol;
# the real KORTEX path is compiled in but only run against the arm in person.
add_executable(teleop_socket_server apps/teleop_socket_server.cpp)
target_link_libraries(teleop_socket_server PRIVATE kinova_lowlevel Eigen3::Eigen)
target_include_directories(teleop_socket_server PRIVATE include)
if(KINOVA_ENABLE_KORTEX)
    # Real-robot build: KortexTransport is compiled into the library and the
    # KORTEX include dirs / -D_OS_UNIX are needed where the app references the
    # KortexTransport header. The real --ip path is active (no KINOVA_NO_KORTEX).
    target_include_directories(teleop_socket_server PRIVATE ${KORTEX_INCLUDE_DIRS})
    target_compile_definitions(teleop_socket_server PRIVATE _OS_UNIX)
else()
    # SIM-ONLY build (default). KINOVA_NO_KORTEX #ifdef's out the KortexTransport
    # reference so no KORTEX symbols are demanded -- required on the Jetson, where
    # the vendored libKortexApiCpp.a is x86-64 only and would fail to link into an
    # aarch64 executable.
    target_compile_definitions(teleop_socket_server PRIVATE KINOVA_NO_KORTEX)
endif()
```

- [ ] **Step 3: Build the server (default sim build) — verify it compiles & links**

Run:
```sh
bash local_tools/sync_to_abra.sh && \
ssh abra 'cd ~/kinova-gen3-driver/build && cmake .. -DCMAKE_PREFIX_PATH=/usr/local/lib/python3.10/dist-packages/cmeel.prefix && cmake --build . -j teleop_socket_server'
```
Expected: PASS — links `teleop_socket_server` with no KORTEX symbols demanded. The five `static_assert`s in `teleop_protocol.h` compile (size parity gate).

- [ ] **Step 4: Launch smoke test — verify it binds and runs**

Run (background launch, then stop after a couple seconds):
```sh
ssh abra 'cd ~/kinova-gen3-driver/build && ./teleop_socket_server --sim --urdf ../models/gen3_7dof_2f85.urdf --port 9095 & SRV=$!; sleep 2; kill -INT $SRV; wait $SRV'
```
Expected: prints `[teleop-srv] urdf=... port=9095 sim=yes`, then `[teleop-srv] listening; RT loop running. Ctrl-C to stop.`, and on SIGINT `[teleop-srv] stopped.`. No `socket`/`bind` perror.

- [ ] **Step 5: Commit**

```sh
git add apps/teleop_socket_server.cpp CMakeLists.txt
git commit -m "feat(teleop): UDP socket server bridging the supervisor to CartesianImpedanceMode"
```

---

## Task 4: Full-suite + cross-language parity verification

No new files — this is the green-bar gate for the whole sub-project.

**Files:** none (verification only).

- [ ] **Step 1: Build everything + run the full C++ test suite**

Run:
```sh
bash local_tools/build_on_abra.sh
```
Expected: configure + build clean; `ctest --output-on-failure` reports **all tests passed**, including `unit_tests` (which now contains `TeleopProtocol.*` and `SimTransportTest.ClearFaultsIsNoOp`).

- [ ] **Step 2: Run the supervisor's Python protocol tests (canonical sizes)**

From the supervisor repo on the machine where it lives:
```sh
cd ~/atdev/kinova-quest-teleop && python -m pytest tests/test_protocol.py -q
```
Expected: PASS — confirms the Python codec's `20 / 84 / 157 / 28 / 261` sizes, i.e. both sides agree on the wire contract.

- [ ] **Step 3: Confirm acceptance criteria & record results**

Confirm against the spec's acceptance criteria:
- Protocol parity: `static_assert`s compiled (Step 1 build), C++ `TeleopProtocol.PacketSizes` passed, Python pytest passed.
- Sim bring-up: server binds and runs under `--sim` (Task 3 Step 4).
- No RT regression / real-arm: deferred to an attended hardware session (out of scope here) — note this explicitly in the completion summary, do not claim hardware verification.

No commit (verification only). Report the CTest summary line and the pytest summary line as evidence.

---

## Self-review notes

- **Spec coverage:** protocol header + sizes (Task 1) ✓; `clear_faults` library edit (Task 2) ✓; server with seqlock/FeedbackTap/RX+feedback threads/home-seed/CLI (Task 3) ✓; CMake target + test wiring (Tasks 1 & 3) ✓; build/test/pytest verification (Task 4) ✓. Deferred items (gripper actuation, real-arm motion, RtExecutor changes, transport generalization) carry no task by design — called out in the spec's "Out of scope".
- **Naming consistency:** `kinova::teleop` types, `FeedbackTap`, `Seqlock`, `pose_from_packet`, `clear_faults()`, `teleop_socket_server` used identically across tasks and the server source.
- **No placeholders:** every code step shows complete content; every run step shows the exact command and expected output.
