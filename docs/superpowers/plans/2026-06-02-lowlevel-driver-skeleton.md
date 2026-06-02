# Gen3 Low-Level Driver Skeleton — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refactor the validated `grav_comp_test.cpp` prototype into a durable, layered, heavily-instrumented C++ library for Kinova Gen3 low-level control, with gravity-comp torque as the first mode and benchmarking as the primary deliverable.

**Architecture:** Seven units behind clean interfaces — `joint_types` (POD), `Transport` (KORTEX-only comm), `ControlMode` (compute), `Dynamics` (Pinocchio-only), `Telemetry` (lock-free), `rt_system` (RT setup), `RtExecutor` (orchestration). Single RT thread (`SCHED_FIFO`, pinned, `mlockall`'d); comm and compute are separate units so the threading model can change later without touching them.

**Tech Stack:** C++17, CMake, KORTEX C++ SDK (vendored static lib), Pinocchio (pip `cmeel.prefix`), Eigen 3.4, GoogleTest. Built/tested on `abra` (Linux 5.15-rt-tegra, aarch64). **Robot is OFF — only build + unit + SimTransport tests run unattended.**

**Reference artifacts on `abra`:**
- `~/rtos_testing/cpp/grav_comp_test.cpp` — the validated single-file prototype (KORTEX handshake, gravity packing, pacing).
- `~/rtos_testing/cpp/CMakeLists.txt` — proven KORTEX + Pinocchio link recipe.
- `~/rtos_testing/cpp/kortex_hardware/` — vendored KORTEX SDK (headers + `lib/release/libKortexApiCpp.a`).
- `~/rtos_testing/7dof.urdf` — Gen3 7-DOF URDF.

---

## Conventions for every task

- **Dev loop:** edit in this repo on the Mac → `scripts/sync_to_abra.sh` rsyncs to `abra:~/kinova-gen3-driver` → build + test over SSH. A task is "done" only when its tests pass *on abra*.
- **Build/test on abra:** `ssh abra 'cd ~/kinova-gen3-driver/build && cmake --build . -j && ctest --output-on-failure'`
- **TDD:** failing test → run-it-fails → minimal impl → run-it-passes → commit.
- **RT-safe** means: no heap allocation, no syscalls (except the cyclic exchange), no locks, no exceptions thrown on the hot path. Asserted mechanically in Task 11.

---

## Public interfaces (the contracts — defined once, referenced by all tasks)

These header contents are authoritative. Tasks implement against exactly these signatures.

### `include/kinova_lowlevel/joint_types.h`
```cpp
#pragma once
#include <array>
#include <cstdint>
#include <Eigen/Core>

namespace kinova {

inline constexpr int kNumJoints = 7;
using JointVec = Eigen::Matrix<double, kNumJoints, 1>;   // SI: rad, rad/s, N·m

enum class ActuatorMode : uint8_t { kPosition, kVelocity, kTorque, kCurrent };

struct JointFeedback {                  // all SI / radians
  JointVec q   = JointVec::Zero();      // position [rad]
  JointVec qd  = JointVec::Zero();      // velocity [rad/s]
  JointVec tau = JointVec::Zero();      // measured torque [N·m]
  JointVec current = JointVec::Zero();  // [A]
  uint64_t frame_id = 0;
  bool fault = false;                   // robot-reported fault flag
};

struct JointCommand {                   // all SI / radians
  ActuatorMode mode = ActuatorMode::kTorque;
  JointVec position = JointVec::Zero(); // setpoint or passthrough [rad]
  JointVec velocity = JointVec::Zero(); // [rad/s]
  JointVec torque   = JointVec::Zero(); // [N·m]
};

constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;

}  // namespace kinova
```

### `include/kinova_lowlevel/dynamics.h`
```cpp
#pragma once
#include <string>
#include "kinova_lowlevel/joint_types.h"

namespace kinova {

// Pinocchio rigid-body dynamics for the Gen3. The ONLY unit that includes
// Pinocchio. Public interface takes a flat angle vector (nv); continuous-joint
// (cos,sin) packing to the nq config vector is handled internally and faithfully.
class Dynamics {
 public:
  explicit Dynamics(const std::string& urdf_path);
  ~Dynamics();
  // RT-safe: no allocation after construction.
  void gravity(const JointVec& q, JointVec& tau_out);
  int nv() const;
  int nq() const;
 private:
  struct Impl;
  Impl* impl_;   // pimpl: keeps Pinocchio headers out of consumers
};

}  // namespace kinova
```

### `include/kinova_lowlevel/transport.h`
```cpp
#pragma once
#include <array>
#include "kinova_lowlevel/joint_types.h"

namespace kinova {

using ActuatorModes = std::array<ActuatorMode, kNumJoints>;

// Comm boundary. The ONLY unit (KortexTransport impl) that includes KORTEX.
class Transport {
 public:
  virtual ~Transport() = default;
  virtual void connect() = 0;
  virtual void set_servoing_low_level() = 0;
  virtual void set_actuator_modes(const ActuatorModes&) = 0; // pumping handled inside
  virtual void exchange(const JointCommand&, JointFeedback&) = 0; // blocking round-trip
  virtual void send(const JointCommand&) = 0;                     // non-blocking
  virtual void receive(JointFeedback&) = 0;
  virtual void safe_shutdown() = 0;   // revert POSITION + SINGLE_LEVEL_SERVOING + close
};

}  // namespace kinova
```

### `include/kinova_lowlevel/control_mode.h`
```cpp
#pragma once
#include "kinova_lowlevel/joint_types.h"
#include "kinova_lowlevel/transport.h"

namespace kinova {

class ControlMode {
 public:
  virtual ~ControlMode() = default;
  virtual ActuatorModes required_modes() const = 0;
  virtual void on_enter(const JointFeedback&) = 0;
  // RT-safe: fills `out` from `fb`. dt_s is the nominal period in seconds.
  virtual void compute(const JointFeedback& fb, double dt_s, JointCommand& out) = 0;
  virtual void on_exit() = 0;
};

}  // namespace kinova
```

### `include/kinova_lowlevel/telemetry.h`
```cpp
#pragma once
#include <atomic>
#include <cstdint>
#include <vector>

namespace kinova {

struct CycleSample {
  uint64_t cycle_index = 0;
  uint32_t wake_jitter_ns = 0;  // t0 - intended_boundary
  uint32_t comm_ns = 0;         // exchange cost
  uint32_t compute_ns = 0;      // control-law cost
  uint32_t cycle_ns = 0;        // full period actual
  uint16_t flags = 0;           // bit0 overrun, bit1 fault
};

// Single-producer (RT thread) / single-consumer (drain thread) lock-free ring.
// push() never blocks; drops + counts when full.
class SampleRing {
 public:
  explicit SampleRing(size_t capacity_pow2);
  bool push(const CycleSample&) noexcept;     // false => dropped
  bool pop(CycleSample&) noexcept;            // false => empty
  uint64_t dropped() const noexcept;
 private:
  std::vector<CycleSample> buf_;
  size_t mask_;
  std::atomic<uint64_t> head_{0};   // producer
  std::atomic<uint64_t> tail_{0};   // consumer
  std::atomic<uint64_t> dropped_{0};
};

}  // namespace kinova
```

(`rt_system.h`, the telemetry consumers, `gravity_comp_mode.h`, and `rt_executor.h` are defined in their own tasks below.)

---

## Task 0: Repo scaffolding, CMake, abra build loop

**Files:**
- Create: `CMakeLists.txt`, `cmake/aarch64-toolchain.cmake`, `scripts/sync_to_abra.sh`, `.gitignore`, `models/gen3_7dof.urdf` (copied from abra), `src/.gitkeep`, `tests/smoke_test.cpp`

- [ ] **Step 1:** Copy the URDF and confirm SDK/Pinocchio locations on abra.
  Run: `ssh abra 'cp ~/rtos_testing/7dof.urdf /tmp/ && ls -la ~/rtos_testing/cpp/kortex_hardware/lib/release/libKortexApiCpp.a && find /usr/local/lib/python3.10/dist-packages/cmeel.prefix -name pinocchioConfig.cmake'`
  Then `scp abra:/tmp/7dof.urdf models/gen3_7dof.urdf`.
  Expected: lib exists; a `pinocchioConfig.cmake` path is printed (record it as `PINOCCHIO_PREFIX` = the dir two levels above the `lib/cmake/pinocchio/` dir).

- [ ] **Step 2:** Write `scripts/sync_to_abra.sh`:
```bash
#!/usr/bin/env bash
set -euo pipefail
REMOTE=${1:-abra}
rsync -az --delete \
  --exclude build --exclude .git \
  "$(dirname "$0")/../" "$REMOTE:~/kinova-gen3-driver/"
echo "synced to $REMOTE:~/kinova-gen3-driver"
```
  `chmod +x scripts/sync_to_abra.sh`.

- [ ] **Step 3:** Write top-level `CMakeLists.txt` (library + app + tests). KORTEX recipe mirrors the prototype; Pinocchio via `find_package`. Key contents:
```cmake
cmake_minimum_required(VERSION 3.16)
project(kinova_lowlevel CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wno-deprecated-declarations -O2)

set(KORTEX_HW_DIR "$ENV{HOME}/rtos_testing/cpp/kortex_hardware" CACHE PATH "KORTEX SDK")
if(NOT EXISTS "${KORTEX_HW_DIR}/include/client/BaseClientRpc.h")
  message(FATAL_ERROR "KORTEX headers not found under ${KORTEX_HW_DIR}; pass -DKORTEX_HW_DIR=")
endif()
set(KORTEX_INCLUDE_DIRS
  ${KORTEX_HW_DIR}/include ${KORTEX_HW_DIR}/include/client
  ${KORTEX_HW_DIR}/include/client_stubs ${KORTEX_HW_DIR}/include/common
  ${KORTEX_HW_DIR}/include/messages ${KORTEX_HW_DIR}/include/google)
set(KORTEX_LIB ${KORTEX_HW_DIR}/lib/release/libKortexApiCpp.a)

find_package(pinocchio REQUIRED)   # needs CMAKE_PREFIX_PATH=<PINOCCHIO_PREFIX>

# Core library (no KORTEX in most units; KORTEX confined to kortex_transport.cpp)
add_library(kinova_lowlevel
  src/dynamics.cpp src/telemetry.cpp src/telemetry_consumers.cpp
  src/rt_system.cpp src/gravity_comp_mode.cpp src/rt_executor.cpp
  src/sim_transport.cpp src/kortex_transport.cpp)
target_include_directories(kinova_lowlevel PUBLIC include PRIVATE ${KORTEX_INCLUDE_DIRS})
target_link_libraries(kinova_lowlevel PUBLIC pinocchio::pinocchio PRIVATE ${KORTEX_LIB} dl pthread)

add_executable(benchmark_grav_comp apps/benchmark_grav_comp.cpp)
target_link_libraries(benchmark_grav_comp PRIVATE kinova_lowlevel)

enable_testing()
find_package(GTest REQUIRED)
add_executable(unit_tests
  tests/smoke_test.cpp tests/joint_types_test.cpp tests/dynamics_test.cpp
  tests/telemetry_test.cpp tests/gravity_comp_mode_test.cpp tests/rt_safety_test.cpp)
target_link_libraries(unit_tests PRIVATE kinova_lowlevel GTest::gtest_main)
target_compile_definitions(unit_tests PRIVATE URDF_PATH="${CMAKE_SOURCE_DIR}/models/gen3_7dof.urdf")
add_test(NAME unit_tests COMMAND unit_tests)
```
  Note: as units are added task-by-task, keep the `add_library`/`unit_tests` source lists in sync — early tasks may temporarily comment out not-yet-created sources.

- [ ] **Step 4:** Write `tests/smoke_test.cpp`:
```cpp
#include <gtest/gtest.h>
TEST(Smoke, Builds) { EXPECT_EQ(1 + 1, 2); }
```
  For Task 0 only, trim `add_library` sources to none (header-only) and `unit_tests` to just `smoke_test.cpp` so the project configures before other files exist. Re-expand in later tasks.

- [ ] **Step 5:** Configure + build + test on abra:
  Run: `./scripts/sync_to_abra.sh && ssh abra 'mkdir -p ~/kinova-gen3-driver/build && cd ~/kinova-gen3-driver/build && cmake .. -DCMAKE_PREFIX_PATH=<PINOCCHIO_PREFIX> && cmake --build . -j && ctest --output-on-failure'`
  Expected: smoke test PASS. This proves the toolchain, Pinocchio discovery, and the sync loop before any real code.

- [ ] **Step 6: Commit** `git add -A && git commit -m "build: CMake skeleton + abra sync loop + URDF"`

---

## Task 1: joint_types + unit conversions

**Files:** Create `include/kinova_lowlevel/joint_types.h` (contract above) + `include/kinova_lowlevel/units.h`; Test `tests/joint_types_test.cpp`.

- [ ] **Step 1: Failing test** `tests/joint_types_test.cpp`:
```cpp
#include <gtest/gtest.h>
#include "kinova_lowlevel/joint_types.h"
#include "kinova_lowlevel/units.h"
using namespace kinova;
TEST(Units, DegRadRoundTrip) {
  JointVec deg; deg << 0, 90, -180, 45, 30, 270, -90;
  JointVec back = rad_to_deg(deg_to_rad(deg));
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(back[i], deg[i], 1e-9);
}
TEST(Units, WrapToPi) {
  EXPECT_NEAR(wrap_to_pi(3.0 * M_PI / 2.0), -M_PI / 2.0, 1e-9);
  EXPECT_NEAR(wrap_to_pi(-3.0 * M_PI), M_PI, 1e-9);
}
```
- [ ] **Step 2: Run, expect FAIL** (units.h missing). `ssh abra '... ctest -R unit_tests'`
- [ ] **Step 3: Implement** `include/kinova_lowlevel/units.h`:
```cpp
#pragma once
#include <cmath>
#include "kinova_lowlevel/joint_types.h"
namespace kinova {
inline JointVec deg_to_rad(const JointVec& d) { return d * kDeg2Rad; }
inline JointVec rad_to_deg(const JointVec& r) { return r * kRad2Deg; }
inline double wrap_to_pi(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}
}  // namespace kinova
```
  Add `joint_types_test.cpp` back into `unit_tests` sources.
- [ ] **Step 4: Run, expect PASS.**
- [ ] **Step 5: Commit** `git commit -am "feat: joint_types + unit conversions"`

---

## Task 2: Dynamics (Pinocchio wrapper)

**Files:** Create `include/kinova_lowlevel/dynamics.h` (contract above) + `src/dynamics.cpp`; Test `tests/dynamics_test.cpp`.

Port the continuous-joint packing from the prototype's `compute_gravity` (the `model.nqs[jid] == 2` → cos/sin branch).

- [ ] **Step 1: Failing test** `tests/dynamics_test.cpp`:
```cpp
#include <gtest/gtest.h>
#include "kinova_lowlevel/dynamics.h"
using namespace kinova;
TEST(Dynamics, LoadsModel) {
  Dynamics dyn(URDF_PATH);
  EXPECT_EQ(dyn.nv(), kNumJoints);
}
TEST(Dynamics, GravityZeroAtConfigAndNonzeroOffAxis) {
  Dynamics dyn(URDF_PATH);
  JointVec q = JointVec::Zero(), tau;
  dyn.gravity(q, tau);                       // finite, no throw
  EXPECT_TRUE(tau.allFinite());
  JointVec q2 = JointVec::Zero(); q2[1] = M_PI / 2.0;  // arm out horizontally
  JointVec tau2; dyn.gravity(q2, tau2);
  EXPECT_GT(tau2.cwiseAbs().maxCoeff(), 1.0); // gravity loads a shoulder joint
}
```
- [ ] **Step 2: Run, expect FAIL** (Dynamics undefined). Add `src/dynamics.cpp` + `dynamics_test.cpp` to CMake lists.
- [ ] **Step 3: Implement** `src/dynamics.cpp`:
```cpp
#include "kinova_lowlevel/dynamics.h"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
namespace kinova {
struct Dynamics::Impl {
  pinocchio::Model model;
  pinocchio::Data data;
  Eigen::VectorXd qcfg;   // nq, preallocated
  explicit Impl(const std::string& urdf) : data() {
    pinocchio::urdf::buildModel(urdf, model);
    data = pinocchio::Data(model);
    qcfg = pinocchio::neutral(model);
  }
};
Dynamics::Dynamics(const std::string& urdf_path) : impl_(new Impl(urdf_path)) {}
Dynamics::~Dynamics() { delete impl_; }
int Dynamics::nv() const { return impl_->model.nv; }
int Dynamics::nq() const { return impl_->model.nq; }
void Dynamics::gravity(const JointVec& q, JointVec& tau_out) {
  auto& m = impl_->model; auto& cfg = impl_->qcfg;
  for (int i = 0; i < m.nv; ++i) {
    int jid = m.getJointId(m.names[i + 1]);
    int qidx = m.idx_qs[jid];
    if (m.nqs[jid] == 2) { cfg[qidx] = std::cos(q[i]); cfg[qidx + 1] = std::sin(q[i]); }
    else { cfg[qidx] = q[i]; }
  }
  const Eigen::VectorXd& g = pinocchio::computeGeneralizedGravity(m, impl_->data, cfg);
  for (int i = 0; i < m.nv; ++i) tau_out[i] = g[i];
}
}  // namespace kinova
```
- [ ] **Step 4: Run, expect PASS.**
- [ ] **Step 5: Commit** `git commit -am "feat: Pinocchio Dynamics with faithful continuous-joint packing"`

---

## Task 3: Telemetry SampleRing (lock-free SPSC)

**Files:** Create `include/kinova_lowlevel/telemetry.h` (contract above) + `src/telemetry.cpp`; Test `tests/telemetry_test.cpp`.

- [ ] **Step 1: Failing test** `tests/telemetry_test.cpp`:
```cpp
#include <gtest/gtest.h>
#include <thread>
#include "kinova_lowlevel/telemetry.h"
using namespace kinova;
TEST(SampleRing, FifoOrder) {
  SampleRing r(8);
  for (uint32_t i = 0; i < 4; ++i) { CycleSample s; s.cycle_index = i; ASSERT_TRUE(r.push(s)); }
  CycleSample out; for (uint32_t i = 0; i < 4; ++i) { ASSERT_TRUE(r.pop(out)); EXPECT_EQ(out.cycle_index, i); }
  EXPECT_FALSE(r.pop(out));
}
TEST(SampleRing, DropsWhenFull) {
  SampleRing r(2);                 // capacity 2 usable (pow2)
  CycleSample s;
  int ok = 0; for (int i = 0; i < 10; ++i) ok += r.push(s) ? 1 : 0;
  EXPECT_GT(r.dropped(), 0u);
  EXPECT_EQ(ok + (int)r.dropped(), 10);
}
TEST(SampleRing, SpscNoLoss) {
  SampleRing r(1024);
  constexpr uint64_t N = 100000;
  std::thread prod([&]{ for (uint64_t i=0;i<N;){ CycleSample s; s.cycle_index=i; if (r.push(s)) ++i; } });
  uint64_t expect=0; CycleSample o;
  while (expect<N){ if (r.pop(o)){ ASSERT_EQ(o.cycle_index,expect); ++expect; } }
  prod.join(); EXPECT_EQ(expect,N);
}
```
- [ ] **Step 2: Run, expect FAIL.** Add `src/telemetry.cpp` + test to CMake.
- [ ] **Step 3: Implement** `src/telemetry.cpp` (round capacity up to power of two; acquire/release indices):
```cpp
#include "kinova_lowlevel/telemetry.h"
namespace kinova {
static size_t next_pow2(size_t n){ size_t p=1; while(p<n) p<<=1; return p; }
SampleRing::SampleRing(size_t cap){ size_t p=next_pow2(cap<2?2:cap); buf_.resize(p); mask_=p-1; }
bool SampleRing::push(const CycleSample& s) noexcept {
  uint64_t h=head_.load(std::memory_order_relaxed), t=tail_.load(std::memory_order_acquire);
  if (h-t >= buf_.size()){ dropped_.fetch_add(1,std::memory_order_relaxed); return false; }
  buf_[h & mask_]=s; head_.store(h+1,std::memory_order_release); return true;
}
bool SampleRing::pop(CycleSample& o) noexcept {
  uint64_t t=tail_.load(std::memory_order_relaxed), h=head_.load(std::memory_order_acquire);
  if (t>=h) return false;
  o=buf_[t & mask_]; tail_.store(t+1,std::memory_order_release); return true;
}
uint64_t SampleRing::dropped() const noexcept { return dropped_.load(std::memory_order_relaxed); }
}  // namespace kinova
```
- [ ] **Step 4: Run, expect PASS.**
- [ ] **Step 5: Commit** `git commit -am "feat: lock-free SPSC SampleRing with drop counting"`

---

## Task 4: Telemetry consumers (stats, histogram, CSV)

**Files:** Create `include/kinova_lowlevel/telemetry_consumers.h` + `src/telemetry_consumers.cpp`; extend `tests/telemetry_test.cpp`.

Header:
```cpp
#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include "kinova_lowlevel/telemetry.h"
namespace kinova {
// Log-bucketed histogram of nanosecond durations.
class NanoHistogram {
 public:
  void add(uint32_t ns) noexcept;
  uint64_t count() const noexcept;
  uint32_t min() const noexcept; uint32_t max() const noexcept;
  double mean() const noexcept;
  uint32_t percentile(double p) const noexcept;   // p in [0,1]
  std::string dump() const;                        // CSV-ish bucket table
 private:
  std::vector<uint64_t> buckets_ = std::vector<uint64_t>(64, 0); // bucket k = [2^k, 2^(k+1))
  uint64_t count_=0; uint64_t sum_=0; uint32_t min_=UINT32_MAX; uint32_t max_=0;
};
// Aggregates a stream of CycleSample into rolling stats + histograms; can write CSV.
class TelemetrySink {
 public:
  explicit TelemetrySink(const std::string& csv_path = "");  // empty => no CSV
  ~TelemetrySink();
  void consume(const CycleSample&);          // called by drain thread
  std::string console_line() const;          // one-line summary since last call
  const NanoHistogram& cycle_hist() const { return cycle_; }
  const NanoHistogram& compute_hist() const { return compute_; }
 private:
  NanoHistogram cycle_, compute_, comm_, jitter_;
  uint64_t overruns_=0, faults_=0, n_=0;
  void* csv_=nullptr;  // FILE*
};
}  // namespace kinova
```

- [ ] **Step 1: Failing test** (append to telemetry_test.cpp):
```cpp
#include "kinova_lowlevel/telemetry_consumers.h"
TEST(NanoHistogram, PercentilesMonotonic) {
  NanoHistogram h; for (uint32_t v=1000; v<=100000; v+=1000) h.add(v);
  EXPECT_EQ(h.count(), 100u);
  EXPECT_LE(h.percentile(0.5), h.percentile(0.99));
  EXPECT_GE(h.max(), h.percentile(0.99));
  EXPECT_LE(h.min(), h.percentile(0.01)+1000);
}
TEST(TelemetrySink, CountsOverruns) {
  TelemetrySink s; CycleSample c; c.flags=1; c.cycle_ns=1100000; c.compute_ns=5000; s.consume(c);
  EXPECT_NE(s.console_line().find("overrun"), std::string::npos);
}
```
- [ ] **Step 2: Run, expect FAIL.** Add sources to CMake.
- [ ] **Step 3: Implement** `src/telemetry_consumers.cpp` — `NanoHistogram` using `add`: bucket = bit-width of ns; track min/max/sum/count; `percentile` walks buckets to the target rank and returns the bucket lower bound. `TelemetrySink::consume` feeds each histogram, counts `flags & 1` overruns / `flags & 2` faults, writes a CSV row via `fprintf` when `csv_` set. `console_line` formats `rate/p50/p99/p999/max/jitter/compute/overrun/drop`. (Full bodies are mechanical given the header; keep them allocation-light but these run off the RT thread so allocation is permitted.)
- [ ] **Step 4: Run, expect PASS.**
- [ ] **Step 5: Commit** `git commit -am "feat: telemetry consumers (histogram, sink, CSV)"`

---

## Task 5: rt_system (scheduling, affinity, introspection)

**Files:** Create `include/kinova_lowlevel/rt_system.h` + `src/rt_system.cpp`; Test: covered indirectly (these are syscalls; assert they return a populated struct, not that privileges succeed).

Header:
```cpp
#pragma once
#include <cstdint>
#include <string>
namespace kinova {
struct RtConfig { int priority=80; int cpu=-1; bool lock_memory=true; };
struct RtReport {                 // what was ACTUALLY applied
  bool mlock_ok=false; int policy=-1; int priority=-1; int cpu=-1; std::string note;
};
RtReport enable_rt(const RtConfig&);            // best-effort; never throws
struct ResourceUsage { uint64_t minflt=0, majflt=0, nvcsw=0, nivcsw=0; };
ResourceUsage read_usage();                     // getrusage(RUSAGE_THREAD)
std::string introspect();                       // applied sched + affinity + usage, human-readable
}  // namespace kinova
```
- [ ] **Step 1: Failing test** in `tests/telemetry_test.cpp` (or new `tests/rt_system_test.cpp`):
```cpp
#include "kinova_lowlevel/rt_system.h"
TEST(RtSystem, ReportPopulated) {
  auto rep = kinova::enable_rt({});   // may fail to set FIFO w/o privilege; must not throw
  EXPECT_GE(rep.policy, 0);
  auto u = kinova::read_usage();      // monotonic counters readable
  EXPECT_GE(u.nivcsw + u.nvcsw, 0u);
}
```
- [ ] **Step 2: Run, expect FAIL.**
- [ ] **Step 3: Implement** `src/rt_system.cpp` porting `enable_rt` from the prototype, plus `sched_setaffinity` when `cpu>=0`, `sched_getscheduler`/`sched_getparam` to fill the report, and `getrusage(RUSAGE_THREAD, …)`.
- [ ] **Step 4: Run, expect PASS.**
- [ ] **Step 5: Commit** `git commit -am "feat: rt_system setup + introspection"`

---

## Task 6: Transport interface + SimTransport

**Files:** Create `include/kinova_lowlevel/transport.h` (contract above) + `include/kinova_lowlevel/sim_transport.h` + `src/sim_transport.cpp`; Test `tests/` (in gravity_comp_mode_test or a sim test).

`sim_transport.h`: `SimTransport : Transport` — holds a `JointFeedback state_`; `exchange` copies `cmd` into an internal "last command", integrates a trivial model (optional: `state_.q += state_.qd*dt`), optionally sleeps `latency_us` to mimic round-trip, fills `fb` from `state_`. Constructor takes `(JointFeedback initial, int latency_us=0)`.

- [ ] **Step 1: Failing test:**
```cpp
#include "kinova_lowlevel/sim_transport.h"
using namespace kinova;
TEST(SimTransport, EchoesAndAdvancesFrame) {
  JointFeedback init; init.q.setConstant(0.1);
  SimTransport t(init);
  t.connect(); t.set_servoing_low_level();
  JointCommand c; c.mode=ActuatorMode::kTorque; JointFeedback fb;
  t.exchange(c, fb);
  EXPECT_NEAR(fb.q[0], 0.1, 1e-9);
  uint64_t f0=fb.frame_id; t.exchange(c, fb); EXPECT_GT(fb.frame_id, f0);
}
```
- [ ] **Step 2: Run, expect FAIL.**
- [ ] **Step 3: Implement** `src/sim_transport.cpp` (all RT-safe; latency via busy-wait, not sleep, so it works off-RT in tests).
- [ ] **Step 4: Run, expect PASS.**
- [ ] **Step 5: Commit** `git commit -am "feat: Transport interface + SimTransport fake robot"`

---

## Task 7: KortexTransport (port the validated handshake)

**Files:** Create `include/kinova_lowlevel/kortex_transport.h` + `src/kortex_transport.cpp`; **build-only** verification (robot off — cannot run).

`kortex_transport.h`: `KortexTransport : Transport`, constructor `(std::string ip, std::string user="admin", std::string password="admin")`. Owns TCP/UDP transports, routers, sessions, `BaseClient`, `BaseCyclicClient`, `ActuatorConfigClient` via RAII (`std::unique_ptr` + a destructor that closes sessions). Holds a reused `k_api::BaseCyclic::Command cmd_` and `frame_id_`.

- [ ] **Step 1:** Port from `~/rtos_testing/cpp/grav_comp_test.cpp`, mapping prototype blocks to interface methods:
  - `connect()` ← the TCP/UDP connect + router + `CreateSession` + `ClearFaults` + actuator-count check block.
  - `set_servoing_low_level()` ← `SetServoingMode(LOW_LEVEL_SERVOING)` + seed (`RefreshFeedback`, build `cmd_` with current positions, one `Refresh`).
  - `set_actuator_modes()` ← per-actuator `SetControlMode` with the **`pump()` Refresh between each call** (critical — keeps cyclic alive during the switch).
  - `exchange()` ← deg→rad on feedback into `JointFeedback` (with `wrap_to_pi`), rad/N·m on command into `cmd_` (set `position` passthrough + `torque_joint`/`velocity`/`position` per `cmd.mode`), bump `frame_id_`/`command_id`, `Refresh(cmd_, 0)`, populate `fb` (q,qd,tau,current,frame_id, fault from base feedback flags).
  - `send()`/`receive()` ← `RefreshCommand`/`RefreshFeedback` variants (same field mapping).
  - `safe_shutdown()` ← revert actuators to POSITION, `SINGLE_LEVEL_SERVOING`, close sessions, disconnect (the prototype's shutdown block).
- [ ] **Step 2: Build only** (no run): `./scripts/sync_to_abra.sh && ssh abra 'cd ~/kinova-gen3-driver/build && cmake --build . -j --target kinova_lowlevel'`
  Expected: compiles + links against `libKortexApiCpp.a`. **Do NOT execute against hardware.**
- [ ] **Step 3: Commit** `git commit -am "feat: KortexTransport (ported low-level handshake; build-verified, robot off)"`

---

## Task 8: GravityCompTorqueMode

**Files:** Create `include/kinova_lowlevel/gravity_comp_mode.h` + `src/gravity_comp_mode.cpp`; Test `tests/gravity_comp_mode_test.cpp`.

Header:
```cpp
#pragma once
#include "kinova_lowlevel/control_mode.h"
#include "kinova_lowlevel/dynamics.h"
namespace kinova {
struct GravityCompParams { double scale=1.0; double damping=0.0; double torque_limit=39.0; };
class GravityCompTorqueMode : public ControlMode {
 public:
  GravityCompTorqueMode(Dynamics& dyn, GravityCompParams p={});
  ActuatorModes required_modes() const override;          // all kTorque
  void on_enter(const JointFeedback&) override;
  void compute(const JointFeedback&, double dt_s, JointCommand&) override;  // RT-safe
  void on_exit() override {}
 private:
  Dynamics& dyn_; GravityCompParams p_; JointVec tau_;
};
}  // namespace kinova
```
- [ ] **Step 1: Failing test:**
```cpp
#include "kinova_lowlevel/gravity_comp_mode.h"
using namespace kinova;
TEST(GravComp, OutputsClampedGravityAndPassthrough) {
  Dynamics dyn(URDF_PATH);
  GravityCompTorqueMode m(dyn, {1.0, 0.0, 39.0});
  JointFeedback fb; fb.q.setZero(); fb.q[1]=M_PI/2; fb.qd.setZero();
  for (auto x : m.required_modes()) EXPECT_EQ(x, ActuatorMode::kTorque);
  JointCommand c; m.on_enter(fb); m.compute(fb, 0.001, c);
  JointVec g; dyn.gravity(fb.q, g);
  for (int i=0;i<kNumJoints;++i){
    EXPECT_LE(std::abs(c.torque[i]), 39.0+1e-9);          // clamped
    if (std::abs(g[i])<39.0) EXPECT_NEAR(c.torque[i], g[i], 1e-6);
  }
  EXPECT_NEAR(c.position[1], fb.q[1], 1e-9);              // position passthrough
  EXPECT_EQ(c.mode, ActuatorMode::kTorque);
}
TEST(GravComp, DampingSubtractsVelocityTerm) {
  Dynamics dyn(URDF_PATH);
  GravityCompTorqueMode m(dyn, {1.0, 2.0, 1e9});          // huge limit, damping=2
  JointFeedback fb; fb.q.setZero(); fb.qd.setConstant(1.0);
  JointCommand c; m.on_enter(fb); m.compute(fb, 0.001, c);
  JointVec g; dyn.gravity(fb.q, g);
  for (int i=0;i<kNumJoints;++i) EXPECT_NEAR(c.torque[i], g[i]-2.0*1.0, 1e-6);
}
```
- [ ] **Step 2: Run, expect FAIL.**
- [ ] **Step 3: Implement** `src/gravity_comp_mode.cpp`: `compute` = `dyn_.gravity(fb.q, tau_); tau_ = p_.scale*tau_ - p_.damping*fb.qd; clamp to ±torque_limit; out.mode=kTorque; out.torque=tau_; out.position=fb.q;` (no allocation — `tau_` is a member).
- [ ] **Step 4: Run, expect PASS.**
- [ ] **Step 5: Commit** `git commit -am "feat: GravityCompTorqueMode"`

---

## Task 9: RtExecutor

**Files:** Create `include/kinova_lowlevel/rt_executor.h` + `src/rt_executor.cpp`. Tested via Task 11 (RT-safety + run-on-Sim).

Header:
```cpp
#pragma once
#include <atomic>
#include "kinova_lowlevel/transport.h"
#include "kinova_lowlevel/control_mode.h"
#include "kinova_lowlevel/telemetry.h"
#include "kinova_lowlevel/rt_system.h"
namespace kinova {
enum class Pacing { kSleepSpin, kClockNanosleep };
struct ExecutorConfig { double rate_hz=1000.0; Pacing pacing=Pacing::kSleepSpin; RtConfig rt{}; };
class RtExecutor {
 public:
  RtExecutor(Transport& t, SampleRing& ring, ExecutorConfig cfg);
  void request_mode(ControlMode* m) noexcept;   // supervisor sets; loop adopts at boundary
  void run(std::atomic<bool>& stop);            // blocks; runs the RT loop on calling thread
 private:
  Transport& t_; SampleRing& ring_; ExecutorConfig cfg_;
  std::atomic<ControlMode*> requested_{nullptr};
};
}  // namespace kinova
```
- [ ] **Step 1:** Implement `src/rt_executor.cpp`:
  - `run`: `enable_rt(cfg_.rt)`; seed `fb` via one `t_.exchange` with a hold command; compute `period_ns`; set `boundary`. Loop until `stop`: pace to `boundary` (sleep-spin or `clock_nanosleep(ABSTIME)`); `t0=now` (jitter); adopt `requested_` if changed (`on_exit` old / `on_enter` new, set actuator modes via `t_.set_actuator_modes(mode->required_modes())` — done off-cycle the first time only); `t_.exchange(cmd,fb)`; `t1`; `mode->compute(fb, dt_s, cmd)`; `t2`; fill `CycleSample` (jitter, comm=t1-t0, compute=t2-t1, cycle, flags overrun|fault); `ring_.push(s)`; advance `boundary`, detect overrun + resync.
  - No allocation in the loop; `cmd`/`fb`/`s` are stack locals reused.
- [ ] **Step 2: Build** target `kinova_lowlevel` on abra. Expected: compiles.
- [ ] **Step 3: Commit** `git commit -am "feat: RtExecutor (single RT thread, two pacing strategies)"`

---

## Task 10: benchmark_grav_comp app

**Files:** Create `apps/benchmark_grav_comp.cpp`.

- [ ] **Step 1:** Implement the app: parse args (`--ip`, `--urdf`, `--sim`, `--rate`, `--cpu`, `--rt-priority`, `--pacing sleepspin|nanosleep`, `--duration`, `--csv`, `--scale`, `--damping`, `--torque-limit`). Build `Dynamics`, a `Transport` (`KortexTransport` or `SimTransport` when `--sim`), `SampleRing`, `TelemetrySink`. Start a drain thread: `pop` samples → `sink.consume` + print `console_line` ~1 Hz. `connect`/`set_servoing_low_level`. Construct `GravityCompTorqueMode`, `RtExecutor`, `request_mode`. Install SIGINT → `stop`. Run for `--duration` (or until SIGINT). On exit: `t.safe_shutdown()`, join drain, print histogram dumps + `introspect()` + dropped count + page-fault delta.
- [ ] **Step 2: Build + run on Sim** (no robot): `ssh abra 'cd ~/kinova-gen3-driver/build && cmake --build . -j && ./benchmark_grav_comp --sim --urdf ../models/gen3_7dof.urdf --rate 1000 --duration 5 --csv /tmp/bench.csv'`
  Expected: ~1000 Hz, prints stats, writes CSV, nonzero histogram, clean shutdown.
- [ ] **Step 3: Commit** `git commit -am "feat: benchmark_grav_comp app (sim-verified)"`

---

## Task 11: RT-safety test (zero alloc / zero page faults on the hot path)

**Files:** Create `tests/rt_safety_test.cpp`.

- [ ] **Step 1: Failing test:** run `RtExecutor` against `SimTransport` for ~2000 cycles on a worker thread; capture `read_usage()` before/after the steady-state window; assert `majflt` delta == 0 and (best-effort) `minflt` delta small/zero after warm-up. Also assert the ring received ~2000 samples and `dropped()==0` with adequate capacity.
```cpp
#include <gtest/gtest.h>
#include <thread>
#include "kinova_lowlevel/rt_executor.h"
#include "kinova_lowlevel/sim_transport.h"
#include "kinova_lowlevel/gravity_comp_mode.h"
#include "kinova_lowlevel/dynamics.h"
using namespace kinova;
TEST(RtSafety, NoMajorFaultsSteadyState) {
  JointFeedback init; init.q.setZero();
  SimTransport t(init); Dynamics dyn(URDF_PATH);
  GravityCompTorqueMode mode(dyn);
  SampleRing ring(8192);
  RtExecutor ex(t, ring, {2000.0, Pacing::kSleepSpin, {/*prio*/0,-1,true}});
  ex.request_mode(&mode);
  std::atomic<bool> stop{false};
  std::thread drain([&]{ CycleSample s; while(!stop.load()) while(ring.pop(s)){} });
  // warm up, then measure
  std::thread loop([&]{ ex.run(stop); });
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  auto u0 = read_usage();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  auto u1 = read_usage();
  stop.store(true); loop.join(); drain.join();
  EXPECT_EQ(u1.majflt - u0.majflt, 0u);
  EXPECT_EQ(ring.dropped(), 0u);
}
```
- [ ] **Step 2: Run, expect FAIL** (until Task 9/6/8 complete) then PASS once wired. Add to `unit_tests`.
- [ ] **Step 3: Commit** `git commit -am "test: RT-safety (zero major faults steady-state on Sim)"`

---

## Task 12: README + integration runbook (prep only — robot off)

**Files:** Create `README.md`, `docs/integration-runbook.md`, `tests/integration/grav_comp_static_check.md`.

- [ ] **Step 1:** Write `README.md`: project purpose; the seven-unit architecture diagram; **key design decisions table** (from spec §3, with rationale — single-RT-thread + comm/compute separation, Pinocchio + faithful continuous joints / no wide-limit hack, SI-at-boundary, drop-don't-block telemetry, sleep-spin vs nanosleep); build instructions (abra: CMAKE_PREFIX_PATH, KORTEX_HW_DIR); how to run the sim benchmark; how to run unit tests.
- [ ] **Step 2:** Write `docs/integration-runbook.md`: the exact sequence to run together when the robot is on — power-on checklist, robot IP confirmation, `e-stop within reach`, first run at low `--scale` (e.g. 0.5) with `--torque-limit` conservative, what good output looks like, abort criteria. Mark clearly: **never run unattended.**
- [ ] **Step 3:** Write `tests/integration/grav_comp_static_check.md`: procedure to compare `Dynamics.gravity(q)` vs the robot's measured joint torques at several static poses (validates URDF inertials incl. payload), and the hold-position test.
- [ ] **Step 4: Commit** `git commit -am "docs: README (design decisions) + integration runbook"`

---

## Self-review notes

- **Spec coverage:** all seven units (Tasks 1–9), benchmark outputs — console/CSV/histogram/introspection (Tasks 4,5,10), SimTransport + TDD + RT-safety (Tasks 6,11), build/cross-compile-open (Task 0), README design decisions + integration prep (Task 12). Open items §10 surfaced in the runbook.
- **Type consistency:** all signatures drawn from the single "Public interfaces" block; `JointVec`, `ActuatorModes`, `CycleSample`, `Pacing`, `GravityCompParams` used identically across tasks.
- **Robot-off safety:** the only hardware-touching unit (Task 7) is build-verified only; everything executable runs on `SimTransport`. Live runs deferred to Task 12's runbook, with the user present.
```
