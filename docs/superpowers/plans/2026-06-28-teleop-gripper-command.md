# Teleop Gripper Command Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let the Python teleop supervisor drive the Robotiq 2F-85 gripper position (0–1) over the existing UDP protocol and read back the gripper's actual position, without touching the arm controller or the wire format.

**Architecture:** Gripper flows as data through the command path: two new fields on `JointCommand` (`gripper`, `gripper_active`) and one on `JointFeedback` (`gripper`). A `GripperInjector` transport decorator (twin of the existing `FeedbackTap`) stamps the rx-thread's target into each `JointCommand` as it passes to the transport. `KortexTransport` writes the cyclic interconnect gripper command and reads the gripper's measured position back; `SimTransport` echoes the command. No `ControlMode`, `RtExecutor`, or protocol change.

**Tech Stack:** C++17, Eigen, KORTEX low-level cyclic API (Linux-only), GoogleTest, CMake. Builds/tests run on the Jetson `abra`, not the Mac.

## Global Constraints

- **No wire-protocol change.** `include/kinova_lowlevel/teleop_protocol.h` and `tests/teleop_protocol_test.cpp` must NOT be edited. The five size `static_assert`s (`20 / 84 / 157 / 28 / 261`) must still hold; the parity test must still pass.
- **Server-side gripper units are 0–1.** Conversion to/from KORTEX percent (0–100) happens only inside `KortexTransport`.
- **`NUM_JOINTS = kNumJoints = 7`.** Unchanged.
- **Build guards:** existing `KINOVA_ENABLE_KORTEX` / `KINOVA_NO_KORTEX` / `_OS_UNIX` guards stay as-is. The sim build (default, no KORTEX) must keep compiling.
- **RT-safety:** no heap allocation, no locking, no blocking on the command path inside `GripperInjector` / `write_command`. `JointCommand` copies are POD-sized and fine.
- **Builds and tests run on `abra`**, e.g. `bash local_tools/sync_to_abra.sh && ssh abra 'cd ~/kinova-gen3-driver/build && cmake --build . -j unit_tests teleop_socket_server && ./unit_tests'`. The Mac cannot build KORTEX or Pinocchio.

---

### Task 1: Gripper fields on `JointCommand` / `JointFeedback`

Add the data-carrying fields and prove `SimTransport` round-trips them. This is the foundation every later task consumes.

**Files:**
- Modify: `include/kinova_lowlevel/joint_types.h`
- Modify: `src/sim_transport.cpp`
- Test: `tests/sim_transport_test.cpp`

**Interfaces:**
- Consumes: nothing (foundation task).
- Produces:
  - `JointCommand.gripper` — `float`, default `0.0f`, units 0–1 (target).
  - `JointCommand.gripper_active` — `bool`, default `false`. When false, no gripper command is emitted downstream.
  - `JointFeedback.gripper` — `float`, default `0.0f`, units 0–1 (measured).
  - `SimTransport`: when `cmd.gripper_active`, echoes `cmd.gripper` into the feedback `gripper` field on `exchange()`/`send()`; leaves it unchanged otherwise.

- [ ] **Step 1: Write the failing test**

Add to `tests/sim_transport_test.cpp`:

```cpp
TEST(SimTransport, EchoesGripperWhenActive) {
  JointFeedback init;
  SimTransport t(init);
  t.connect();
  JointCommand c;
  c.gripper = 0.42f;
  c.gripper_active = true;
  JointFeedback fb;
  t.exchange(c, fb);
  EXPECT_NEAR(fb.gripper, 0.42f, 1e-6f);
}

TEST(SimTransport, LeavesGripperUntouchedWhenInactive) {
  JointFeedback init;
  init.gripper = 0.7f;       // pre-existing measured position
  SimTransport t(init);
  t.connect();
  JointCommand c;
  c.gripper = 0.1f;
  c.gripper_active = false;  // no gripper intent
  JointFeedback fb;
  t.exchange(c, fb);
  EXPECT_NEAR(fb.gripper, 0.7f, 1e-6f);  // unchanged
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `bash local_tools/sync_to_abra.sh && ssh abra 'cd ~/kinova-gen3-driver/build && cmake --build . -j unit_tests && ./unit_tests --gtest_filter="SimTransport.*Gripper*"'`
Expected: COMPILE FAIL — `JointCommand` has no member `gripper` / `gripper_active`, `JointFeedback` has no member `gripper`.

- [ ] **Step 3: Add the struct fields**

In `include/kinova_lowlevel/joint_types.h`, extend the two structs:

```cpp
struct JointFeedback {
  JointVec q   = JointVec::Zero();
  JointVec qd  = JointVec::Zero();
  JointVec tau = JointVec::Zero();
  JointVec current = JointVec::Zero();
  uint64_t frame_id = 0;
  bool fault = false;
  float gripper = 0.0f;   // measured gripper position, 0 (open) .. 1 (closed)
};

struct JointCommand {
  ActuatorMode mode = ActuatorMode::kTorque;
  JointVec position = JointVec::Zero();
  JointVec velocity = JointVec::Zero();
  JointVec torque   = JointVec::Zero();
  float gripper = 0.0f;        // target gripper position, 0 (open) .. 1 (closed)
  bool  gripper_active = false; // when false, no gripper command is emitted
};
```

- [ ] **Step 4: Echo the gripper in `SimTransport`**

In `src/sim_transport.cpp`, in both `exchange()` and `send()`, right after `last_cmd_ = cmd;`, add:

```cpp
  if (cmd.gripper_active) state_.gripper = cmd.gripper;
```

So `exchange()` becomes:

```cpp
void SimTransport::exchange(const JointCommand& cmd, JointFeedback& fb) {
  last_cmd_ = cmd;
  if (cmd.gripper_active) state_.gripper = cmd.gripper;
  if (latency_us_ > 0) {
    const int64_t deadline = ns_now() + int64_t(latency_us_) * 1000LL;
    while (ns_now() < deadline) { /* busy-wait, off-RT friendly */ }
  }
  ++frame_;
  state_.frame_id = frame_;
  fb = state_;
}
```

and `send()` becomes:

```cpp
void SimTransport::send(const JointCommand& cmd) {
  last_cmd_ = cmd;
  if (cmd.gripper_active) state_.gripper = cmd.gripper;
  ++frame_;
  state_.frame_id = frame_;
}
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `ssh abra 'cd ~/kinova-gen3-driver/build && cmake --build . -j unit_tests && ./unit_tests --gtest_filter="SimTransport.*"'`
Expected: PASS — all `SimTransport.*` tests including the two new ones, and the pre-existing echo/round-trip/clear-faults tests still green.

- [ ] **Step 6: Commit**

```bash
git add include/kinova_lowlevel/joint_types.h src/sim_transport.cpp tests/sim_transport_test.cpp
git commit -m "feat(gripper): add gripper fields to JointCommand/JointFeedback; sim echo"
```

---

### Task 2: `GripperInjector` transport decorator + server wiring

Add the decorator that carries the rx-thread's gripper target into the command path, wire it into the server between the base transport and `FeedbackTap`, and report the measured gripper in `FEEDBACK`. This is the sim-complete teleop gripper path.

**Files:**
- Modify: `apps/teleop_socket_server.cpp`

**Interfaces:**
- Consumes: `JointCommand.gripper`, `JointCommand.gripper_active`, `JointFeedback.gripper` (Task 1); existing `FeedbackTap`, `Seqlock<JointFeedback>`, `tp::PoseTargetPacket.gripper`, `tp::FeedbackPacket.gripper_state`.
- Produces: `GripperInjector` — a `Transport` decorator with `void set_gripper(float)` (called by the rx thread) that stamps `gripper`/`gripper_active` into every `JointCommand` forwarded via `exchange()`/`send()`.

- [ ] **Step 1: Add the `GripperInjector` decorator**

In `apps/teleop_socket_server.cpp`, inside the anonymous namespace, immediately after the `FeedbackTap` class (before `pose_from_packet`), add:

```cpp
// Transport decorator: carries the latest gripper target (set by the rx thread)
// and stamps it into each JointCommand on its way to the wrapped transport. Keeps
// gripper control orthogonal to any ControlMode. The atomics make the rx-thread
// write / RT-thread read race-free; the JointCommand copy is POD-sized (no alloc).
class GripperInjector : public Transport {
 public:
  explicit GripperInjector(Transport& inner) : inner_(inner) {}

  // Called by the rx thread when a POSE_TARGET arrives.
  void set_gripper(float g) {
    if (g < 0.0f) g = 0.0f;
    if (g > 1.0f) g = 1.0f;
    gripper_.store(g, std::memory_order_relaxed);
    active_.store(true, std::memory_order_relaxed);
  }

  void connect() override { inner_.connect(); }
  void set_servoing_low_level() override { inner_.set_servoing_low_level(); }
  void set_actuator_modes(const ActuatorModes& m) override {
    inner_.set_actuator_modes(m);
  }
  void exchange(const JointCommand& c, JointFeedback& fb) override {
    inner_.exchange(stamp(c), fb);
  }
  void send(const JointCommand& c) override { inner_.send(stamp(c)); }
  void receive(JointFeedback& fb) override { inner_.receive(fb); }
  void safe_shutdown() override { inner_.safe_shutdown(); }
  void clear_faults() override { inner_.clear_faults(); }

 private:
  JointCommand stamp(const JointCommand& c) {
    JointCommand out = c;
    out.gripper = gripper_.load(std::memory_order_relaxed);
    out.gripper_active = active_.load(std::memory_order_relaxed);
    return out;
  }
  Transport& inner_;
  std::atomic<float> gripper_{0.0f};
  std::atomic<bool> active_{false};
};
```

- [ ] **Step 2: Insert the injector into the transport stack**

In `main`, find:

```cpp
  Seqlock<JointFeedback> snapshot;
  FeedbackTap transport(*base_transport, snapshot);
```

Replace with:

```cpp
  Seqlock<JointFeedback> snapshot;
  GripperInjector injector(*base_transport);
  FeedbackTap transport(injector, snapshot);
```

- [ ] **Step 3: Forward the gripper target from the POSE_TARGET handler**

In the rx thread's `case tp::MsgType::kPoseTarget:` block, replace:

```cpp
          mode.set_target(pose_from_packet(pkt));
          last_gripper.store(pkt.gripper, std::memory_order_relaxed);
          // TODO(gripper): once JointCommand carries a gripper field, forward
          // pkt.gripper to the cyclic interconnect command here.
```

with:

```cpp
          mode.set_target(pose_from_packet(pkt));
          injector.set_gripper(pkt.gripper);
```

- [ ] **Step 4: Report the measured gripper in FEEDBACK**

In the feedback thread, replace:

```cpp
        pkt.gripper_state = last_gripper.load(std::memory_order_relaxed);
```

with:

```cpp
        pkt.gripper_state = fb.gripper;  // actual measured position from snapshot
```

- [ ] **Step 5: Delete the now-dead `last_gripper` atomic**

In `main`, in the "Shared state between socket threads" block, delete the line:

```cpp
  std::atomic<float> last_gripper{0.0f};
```

(It now has no readers or writers. `last_control_seq` above/below it stays.)

- [ ] **Step 6: Build the sim server to verify it compiles and links**

Run: `ssh abra 'cd ~/kinova-gen3-driver/build && cmake --build . -j teleop_socket_server unit_tests && ./unit_tests'`
Expected: PASS — `teleop_socket_server` links; full `unit_tests` suite green (incl. Task 1's sim gripper tests and the untouched `TeleopProtocol*` parity/size tests). No reference to `last_gripper` remains (no unused-variable warning).

- [ ] **Step 7: Commit**

```bash
git add apps/teleop_socket_server.cpp
git commit -m "feat(teleop): GripperInjector decorator; forward gripper cmd, report measured"
```

---

### Task 3: `KortexTransport` interconnect gripper read/write

Drive the physical gripper and read its measured position. This is the KORTEX-only task; it compiles only under `KINOVA_ENABLE_KORTEX` and is verified by the KORTEX build on `abra` (live motion stays deferred to an attended session).

**Files:**
- Modify: `src/kortex_transport.cpp`

**Interfaces:**
- Consumes: `JointCommand.gripper`, `JointCommand.gripper_active`, `JointFeedback.gripper` (Task 1).
- Produces: nothing for later tasks (terminal hardware task).

**Before writing code — confirm the KORTEX accessor names on `abra`.** The plan below uses the documented Gen3 low-level interconnect gripper API, but the exact protobuf getters/setters MUST be confirmed against the installed headers before claiming the build is correct. Run:

```bash
ssh abra 'grep -rn "gripper_command\|gripper_feedback\|GripperCyclic\|add_motor_cmd\|mutable_interconnect" \
  $(find / -path "*BaseCyclic*" -name "*.h" 2>/dev/null | head -5) 2>/dev/null | head -40'
```

Expected: confirm `BaseCyclic::Command` exposes `mutable_interconnect()->mutable_gripper_command()` with `add_motor_cmd()` / `mutable_motor_cmd(i)` whose entries have `set_position/set_velocity/set_force` (percent), and `BaseCyclic::Feedback` exposes `interconnect().gripper_feedback().motor(i).position()` with a `motor_size()`. If any name differs, adapt the code in Steps 2–3 to the confirmed names and note the correction in the commit message.

- [ ] **Step 1: Add the gripper tuning constants**

In `src/kortex_transport.cpp`, in the anonymous `namespace {` block (next to `kTcpPort`/`kUdpPort`), add:

```cpp
// Gripper motor-command defaults (KORTEX units are percent, 0..100). Position is
// commanded per-cycle from JointCommand.gripper; velocity/force are fixed safe
// defaults — full speed, moderate force. Tune force down if it over-grips.
constexpr float kGripperVelocityPct = 100.0f;
constexpr float kGripperForcePct    = 50.0f;
```

- [ ] **Step 2: Write the gripper command in `write_command`**

In `Impl::write_command(const JointCommand& cmd)`, after the per-actuator `for` loop closes (after the loop that ends with `a->set_command_id(frame_id_);`), before the method's closing brace, add:

```cpp
    // Gripper rides inside the same cyclic command, in the interconnect's
    // gripper_command motor message. Position is percent (0..100); we map the
    // server's 0..1 target. Only emitted when the teleop path has set a target.
    if (cmd.gripper_active) {
      auto* gripper = cmd_.mutable_interconnect()->mutable_gripper_command();
      if (gripper->motor_cmd_size() == 0) gripper->add_motor_cmd();
      auto* m = gripper->mutable_motor_cmd(0);
      float pos = cmd.gripper;
      if (pos < 0.0f) pos = 0.0f;
      if (pos > 1.0f) pos = 1.0f;
      m->set_position(pos * 100.0f);
      m->set_velocity(kGripperVelocityPct);
      m->set_force(kGripperForcePct);
    }
```

- [ ] **Step 3: Read the measured gripper in `fill_feedback`**

In `Impl::fill_feedback(JointFeedback& fb)`, before the closing brace (after `fb.fault = fault;`), add:

```cpp
    // Measured gripper position from the interconnect feedback (percent -> 0..1).
    // Guard the no-gripper case (a robot without an interconnect gripper reports
    // zero motors): leave fb.gripper at its default 0.
    const auto& ic = fb_.interconnect();
    if (ic.gripper_feedback().motor_size() > 0) {
      fb.gripper = float(ic.gripper_feedback().motor(0).position()) / 100.0f;
    }
```

- [ ] **Step 4: Build under KORTEX to verify it compiles**

Run: `ssh abra 'cd ~/kinova-gen3-driver/build && cmake -DKINOVA_ENABLE_KORTEX=ON . && cmake --build . -j teleop_socket_server unit_tests && ./unit_tests'`
Expected: PASS — the KORTEX build compiles and links `KortexTransport` with the interconnect gripper read/write; `unit_tests` green (sim tests are KORTEX-independent and must still pass). If the build uses a separate KORTEX build dir per `local_tools/`, use that script instead — confirm which on `abra`.

- [ ] **Step 5: Commit**

```bash
git add src/kortex_transport.cpp
git commit -m "feat(kortex): drive interconnect gripper from JointCommand; read measured position"
```

---

### Task 4: Documentation touch-up

Update the socket-server doc/header comments so the gripper is no longer described as unimplemented. Small, but keeps the docs honest.

**Files:**
- Modify: `apps/teleop_socket_server.cpp` (top-of-file comment, if it claims gripper is unsupported)
- Modify: `docs/` teleop reference page if one mentions the deferred gripper (grep first)

**Interfaces:**
- Consumes: nothing. Produces: nothing.

- [ ] **Step 1: Find any "gripper deferred/unimplemented" prose**

Run: `cd ~/kinova-gen3-driver && grep -rni "gripper" apps/ docs/ --include='*.cpp' --include='*.md' | grep -i "defer\|todo\|not.*implement\|out of scope\|unsupported"`
Expected: a short list (the socket-server design's "Out of scope" note is historical and stays; focus on the app file's top comment and any user-facing reference/runbook page).

- [ ] **Step 2: Update the prose**

For each hit that describes the *current* behavior as gripper-less (not historical design notes), update it to state the gripper is commanded 0–1 via `POSE_TARGET.gripper` and reported via `FEEDBACK.gripper_state`. Leave dated design specs unchanged (they are point-in-time records).

- [ ] **Step 3: Commit**

```bash
git add -A
git commit -m "docs(teleop): gripper command path is implemented"
```

---

## Verification (whole feature, on `abra`)

After all tasks:
- `ssh abra 'cd ~/kinova-gen3-driver/build && cmake --build . -j unit_tests teleop_socket_server && ./unit_tests'` — full suite green, including `SimTransport.*Gripper*` and the untouched `TeleopProtocol*` size/parity tests.
- KORTEX build compiles (Task 3 Step 4).
- **Deferred to an attended in-person session:** live gripper motion against the real arm, per the integration runbook's safety posture.
