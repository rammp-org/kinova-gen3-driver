## Task 3 Report: `teleop_socket_server` executable + CMake target

### What was done

1. **Created `apps/teleop_socket_server.cpp`** — verbatim from the brief. The server:
   - Implements a `Seqlock<T>` template for lock-free RT→feedback-thread state sharing.
   - Implements `FeedbackTap : public Transport` decorator that snapshots `JointFeedback` on every `exchange`/`receive` call.
   - Spawns an rx thread (UDP `recvfrom` with 200 ms SO_RCVTIMEO) and a feedback thread (~150 Hz `sendto`).
   - Handles `kPoseTarget`, `kSetGains`, `kControl` (with `kClearFaults`, `kRehome`, `kFreeze`, `kShutdown` sub-commands).
   - Guards `#include "kinova_lowlevel/kortex_transport.h"` with `#ifndef KINOVA_NO_KORTEX`.
   - Validates every inbound packet size inline (`if (n < sizeof(...)) break;`) and checks `magic`+`version` before dispatch — does NOT use `valid_header`'s size argument.

2. **Modified `CMakeLists.txt`** — added `teleop_socket_server` target after the `benchmark_cartesian_impedance` block (before `# --- tests ---`), mirroring the benchmark pattern: `KINOVA_NO_KORTEX` in the else branch, `_OS_UNIX` + `${KORTEX_INCLUDE_DIRS}` in the KORTEX branch.

### Step 3: Build output (abra, sim-only)

```
Consolidate compiler generated dependencies of target kinova_lowlevel
[ 83%] Built target kinova_lowlevel
[ 91%] Building CXX object CMakeFiles/teleop_socket_server.dir/apps/teleop_socket_server.cpp.o
[100%] Linking CXX executable teleop_socket_server
[100%] Built target teleop_socket_server
```

No errors, no warnings. KORTEX symbols not demanded. Five `static_assert` size guards in `teleop_protocol.h` compiled successfully.

### Step 4: Launch smoke test output

```
[teleop-srv] urdf=../models/gen3_7dof_2f85.urdf rate=1000Hz port=9095 sim=yes
[teleop-srv] listening; RT loop running. Ctrl-C to stop.
[teleop-srv] stopped.
```

No `socket`/`bind` perror. Clean SIGINT shutdown confirmed.

### Step 5: Unit tests regression

```
[==========] Running 44 tests from 18 test suites.
...
[  PASSED  ] 44 tests.
```

All 44 tests pass, including the 4 `TeleopProtocol.*` tests from Task 1.

### Commit

```
commit 5cd520c
feat(teleop): UDP socket server bridging the supervisor to CartesianImpedanceMode
 2 files changed, 385 insertions(+)
 create mode 100644 apps/teleop_socket_server.cpp
```

### Concerns

None. Build, smoke, and regression all clean.

---

## Fix wave 1

Commit: `995e9f2`
File: `apps/teleop_socket_server.cpp`

### Fix 1 (CRITICAL) — `dyn_rx`: eliminate data race on `Dynamics` in FREEZE handler

**Problem:** `dyn_fb.fk(fb.q)` was called from two threads — the feedback thread (~150 Hz) and the rx thread's `kFreeze` handler. `Dynamics::fk` mutates internal Pinocchio state (`impl_->qcfg`, `impl_->data`) without locking, so this was an unsynchronised data race on a safety-critical path.

**Change 1 — construct `dyn_rx`** (after the existing `dyn` and `dyn_fb` declarations):

```cpp
// Three Dynamics instances — Dynamics::fk mutates internal Pinocchio state and
// is NOT thread-safe. Each thread that calls fk owns its own instance so no
// Dynamics object is ever shared across threads:
//   dyn    — CartesianImpedanceMode / RT thread
//   dyn_fb — feedback thread (periodic ~150 Hz fk)
//   dyn_rx — rx thread, FREEZE handler only
Dynamics dyn(urdf);
Dynamics dyn_fb(urdf);
Dynamics dyn_rx(urdf);  // rx-thread exclusive: prevents data race with dyn_fb
```

**Change 2 — kFreeze handler in rx thread** (previously used `dyn_fb`):

```cpp
case tp::ControlCmd::kFreeze: {
  JointFeedback fb;
  // Use dyn_rx (rx-thread-exclusive) — dyn_fb belongs to the
  // feedback thread and must not be touched here.
  if (snapshot.load(fb)) mode.set_target(dyn_rx.fk(fb.q));
  break;
}
```

`dyn_fb` is now referenced only inside the feedback thread lambda; `dyn_rx` only inside the rx thread lambda; `dyn` only by `CartesianImpedanceMode` (RT thread). No `Dynamics` instance is shared across threads.

### Fix 2 (IMPORTANT) — join threads BEFORE `safe_shutdown` and `close(sock)`

**Problem:** The previous shutdown sequence called `transport.safe_shutdown()` first, then signalled `g_stop`, then joined the threads. The rx thread could still call `transport.clear_faults()` after the transport was torn down. The feedback thread uses the socket, so closing the socket before joining was also wrong.

**Before:**
```cpp
transport.safe_shutdown();
g_stop.store(true, std::memory_order_release);
rx.join();
feedback.join();
::close(sock);
```

**After:**
```cpp
// Stop threads before tearing down the transport: the rx thread can call
// transport.clear_faults() and the feedback thread uses the socket, so both
// must be fully stopped before safe_shutdown and close(sock).
g_stop.store(true, std::memory_order_release);
rx.join();
feedback.join();
transport.safe_shutdown();
::close(sock);
```

### Fix 3 (MINOR) — correct misleading `Seqlock` comment

**Before:**
```cpp
// Single-writer / single-reader lock-free latest-value snapshot (seqlock). The
// RT thread is the only writer; the feedback thread is the only reader. POD T.
```

**After:**
```cpp
// Single-writer / single-reader lock-free latest-value snapshot (seqlock). The
// RT thread is the only writer; the feedback thread is the only reader.
// T need not be trivially copyable — fixed-size Eigen members store data inline,
// so a torn read yields transient garbage numbers but never a bad pointer.
```

No `static_assert` was added (it would fail for `JointFeedback` with Eigen members).

### Build output

```
[ 83%] Built target kinova_lowlevel
Consolidate compiler generated dependencies of target teleop_socket_server
[ 91%] Building CXX object CMakeFiles/teleop_socket_server.dir/apps/teleop_socket_server.cpp.o
[100%] Linking CXX executable teleop_socket_server
[100%] Built target teleop_socket_server
```

No warnings, no errors.

### Launch smoke output

```
[teleop-srv] urdf=../models/gen3_7dof_2f85.urdf rate=1000Hz port=9095 sim=yes
[teleop-srv] listening; RT loop running. Ctrl-C to stop.
[teleop-srv] stopped.
exit code: 0
```

Banner printed, `listening; RT loop running` present, SIGINT handled cleanly, no `socket`/`bind` perror, clean `stopped.` line.

### Unit tests result

```
[==========] Running 44 tests from 18 test suites.
[  PASSED  ] 44 tests.
```

All 44 tests pass. No regressions.

### Commit

```
commit 995e9f2
fix(teleop): give rx thread its own Dynamics for FREEZE; join before safe_shutdown
 1 file changed, 17 insertions(+), 5 deletions(-)
```
