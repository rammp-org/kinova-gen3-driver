# Cartesian Impedance Controller Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a task-space (Cartesian) impedance `ControlMode` for the Gen3 7-DOF, plus the forward-kinematics and frame-Jacobian queries it needs, all behind the existing `Dynamics` / `ControlMode` interfaces.

**Architecture:** `Dynamics` stays the sole owner of Pinocchio and gains `fk()` + `jacobian()` returning Eigen-only value types. A small Eigen-only `pose_error()` helper computes the decoupled geometric SE(3) error. `CartesianImpedanceMode` implements `tau = gravity + ramp·(Jᵀ·F + nullspace)` with `F = Kx·e − Dx·ẋ`. No changes to `Transport` or `RtExecutor`. The 2F-85 URDF (matching the real hardware) becomes the canonical test model.

**Tech Stack:** C++17, Eigen 3.4, Pinocchio (RNEA/FK/Jacobian), GoogleTest. Builds + tests on the Jetson (aarch64 / PREEMPT_RT).

---

## Build/test loop (read once)

This project builds and tests **on the Jetson, not the Mac** (KORTEX + RT APIs are Linux-only; Pinocchio lives under the Jetson's `cmeel.prefix`). Helper scripts live in the gitignored `local_tools/`.

- **Full build + all tests:** `bash local_tools/build_on_abra.sh` — rsyncs the repo to the Jetson, configures, builds, runs `ctest --output-on-failure`.
- **Fast single-test iteration:**
  ```sh
  bash local_tools/sync_to_abra.sh && \
  ssh abra 'cd ~/kinova-gen3-driver/build && cmake --build . -j unit_tests && ./unit_tests --gtest_filter="<FILTER>"'
  ```
- A test "fails to compile" counts as a failing test in TDD — that is the expected first state for each new test.
- Commit after each task with the message shown. Do not push (the user pushes).

## File structure

| File | Responsibility | New/Mod |
|---|---|---|
| `include/kinova_lowlevel/cartesian_types.h` | Eigen-only value types: `Vector6`, `Jacobian6`, `Pose` | New |
| `include/kinova_lowlevel/dynamics.h` | + `fk()`, `jacobian()`, configurable EE-frame ctor | Mod |
| `src/dynamics.cpp` | frame resolution, `pack()` helper, fk/jacobian impl | Mod |
| `include/kinova_lowlevel/cartesian.h` / `src/cartesian.cpp` | `pose_error()` (decoupled SE(3) error), Eigen-only | New |
| `include/kinova_lowlevel/cartesian_impedance_mode.h` / `src/cartesian_impedance_mode.cpp` | the `ControlMode` | New |
| `apps/benchmark_cartesian_impedance.cpp` | sim/hardware demo + `--dry-run` | New |
| `tests/cartesian_types_test.cpp`, `tests/cartesian_test.cpp`, `tests/cartesian_impedance_mode_test.cpp` | unit tests | New |
| `tests/dynamics_test.cpp`, `tests/rt_safety_test.cpp` | + fk/jacobian + impedance RT-safety tests | Mod |
| `CMakeLists.txt` | new sources/tests/app, switch `URDF_PATH` to 2F-85 | Mod |

---

## Task 1: Cartesian value types + adopt the 2F-85 URDF as canonical

**Files:**
- Create: `include/kinova_lowlevel/cartesian_types.h`
- Create: `tests/cartesian_types_test.cpp`
- Modify: `CMakeLists.txt` (add test source; switch `URDF_PATH`)

- [ ] **Step 1: Create the value-types header**

`include/kinova_lowlevel/cartesian_types.h`:
```cpp
#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "kinova_lowlevel/joint_types.h"

namespace kinova {

// Spatial 6-vector: [vx vy vz | wx wy wz] (linear over angular).
using Vector6 = Eigen::Matrix<double, 6, 1>;

// 6xN frame Jacobian (maps joint velocity -> spatial velocity of the EE frame).
using Jacobian6 = Eigen::Matrix<double, 6, kNumJoints>;

// A rigid-body pose (SE(3)). Fixed-size, no heap — RT-safe to copy.
struct Pose {
  Eigen::Vector3d    p = Eigen::Vector3d::Zero();          // position [m]
  Eigen::Quaterniond R = Eigen::Quaterniond::Identity();   // orientation
};

}  // namespace kinova
```

- [ ] **Step 2: Write the failing test**

`tests/cartesian_types_test.cpp`:
```cpp
#include <gtest/gtest.h>
#include "kinova_lowlevel/cartesian_types.h"
using namespace kinova;

TEST(CartesianTypes, PoseDefaultsToIdentity) {
  Pose x;
  EXPECT_TRUE(x.p.isZero());
  EXPECT_NEAR(x.R.norm(), 1.0, 1e-12);          // unit quaternion
  EXPECT_NEAR(x.R.w(), 1.0, 1e-12);             // identity rotation
}

TEST(CartesianTypes, FixedSizes) {
  EXPECT_EQ(Vector6::RowsAtCompileTime, 6);
  EXPECT_EQ(Jacobian6::RowsAtCompileTime, 6);
  EXPECT_EQ(Jacobian6::ColsAtCompileTime, kNumJoints);
}
```

- [ ] **Step 3: Wire the test in and switch the canonical URDF**

In `CMakeLists.txt`, add `tests/cartesian_types_test.cpp` to the `unit_tests` source list (after `tests/smoke_test.cpp`), and change the URDF define from the bare arm to the 2F-85 model (matches the real hardware; has the `gen3_end_effector_link` frame):
```cmake
target_compile_definitions(unit_tests PRIVATE
    URDF_PATH="${CMAKE_SOURCE_DIR}/models/gen3_7dof_2f85.urdf")
```

- [ ] **Step 4: Run tests**

Run: `bash local_tools/build_on_abra.sh`
Expected: builds clean; **all existing tests still pass** on the 2F-85 model (the gravity tests assert only loose/self-consistent properties), and `CartesianTypes.*` pass.

- [ ] **Step 5: Commit**
```bash
git add include/kinova_lowlevel/cartesian_types.h tests/cartesian_types_test.cpp CMakeLists.txt
git commit -m "feat(cartesian): Eigen-only Pose/Vector6/Jacobian6 types; adopt 2F-85 URDF for tests"
```

---

## Task 2: Dynamics — configurable EE frame + `pack()` refactor

**Files:**
- Modify: `include/kinova_lowlevel/dynamics.h`
- Modify: `src/dynamics.cpp`
- Modify: `tests/dynamics_test.cpp`

- [ ] **Step 1: Write the failing tests**

Append to `tests/dynamics_test.cpp`:
```cpp
TEST(Dynamics, DefaultEeFrameResolvesOn2f85) {
  Dynamics dyn(URDF_PATH);                 // default frame "gen3_end_effector_link"
  EXPECT_EQ(dyn.nv(), kNumJoints);         // ctor did not throw
}

TEST(Dynamics, UnknownFrameThrows) {
  EXPECT_THROW(Dynamics(URDF_PATH, "no_such_frame_xyz"), std::runtime_error);
}
```

- [ ] **Step 2: Run to verify failure**

Run: `bash local_tools/sync_to_abra.sh && ssh abra 'cd ~/kinova-gen3-driver/build && cmake --build . -j unit_tests'`
Expected: **compile error** — `Dynamics` has no 2-arg constructor yet.

- [ ] **Step 3: Update the header**

In `include/kinova_lowlevel/dynamics.h`, add the cartesian-types include, the frame ctor arg, and the two new methods:
```cpp
#pragma once
#include <memory>
#include <string>
#include "kinova_lowlevel/joint_types.h"
#include "kinova_lowlevel/cartesian_types.h"
namespace kinova {
class Dynamics {
 public:
  explicit Dynamics(const std::string& urdf_path,
                    const std::string& ee_frame = "gen3_end_effector_link");
  ~Dynamics();
  void gravity(const JointVec& q, JointVec& tau_out);   // RT-safe
  Pose fk(const JointVec& q);                           // RT-safe: pose of ee_frame
  void jacobian(const JointVec& q, Jacobian6& J_out);   // RT-safe: 6x7, LOCAL_WORLD_ALIGNED
  int nv() const;
  int nq() const;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace kinova
```

- [ ] **Step 4: Update the implementation (frame resolution + `pack()` refactor)**

Rewrite `src/dynamics.cpp` so the config-packing is a single helper and the ctor resolves+validates the frame. (fk/jacobian bodies are added in Tasks 3–4; for now declare them minimally so it compiles — see note.) Replace the file with:
```cpp
#include "kinova_lowlevel/dynamics.h"
#include <cmath>
#include <stdexcept>
#include <string>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
namespace kinova {
struct Dynamics::Impl {
  pinocchio::Model model;
  pinocchio::Data data;
  Eigen::VectorXd qcfg;
  pinocchio::FrameIndex frame_id = 0;
  explicit Impl(const std::string& urdf) {
    pinocchio::urdf::buildModel(urdf, model);
    data = pinocchio::Data(model);
    qcfg = pinocchio::neutral(model);
  }
  // Flat 7-vector of joint angles -> Pinocchio config vector, packing each
  // continuous joint as (cos, sin) (nqs==2). The single source of truth used by
  // gravity/fk/jacobian so they can never disagree about the configuration.
  void pack(const JointVec& q) {
    for (int i = 0; i < model.nv; ++i) {
      int jid = model.getJointId(model.names[i + 1]);
      int qidx = model.idx_qs[jid];
      if (model.nqs[jid] == 2) { qcfg[qidx] = std::cos(q[i]); qcfg[qidx + 1] = std::sin(q[i]); }
      else { qcfg[qidx] = q[i]; }
    }
  }
};
Dynamics::Dynamics(const std::string& urdf_path, const std::string& ee_frame)
    : impl_(std::make_unique<Impl>(urdf_path)) {
  if (impl_->model.nv != kNumJoints) {
    throw std::runtime_error(
        "Dynamics: URDF nv=" + std::to_string(impl_->model.nv) +
        " != kNumJoints=" + std::to_string(kNumJoints) + " (wrong URDF for this build)");
  }
  // Footgun guard: a typo'd/missing EE frame must fail loudly at startup, never
  // silently control the wrong point.
  if (!impl_->model.existFrame(ee_frame)) {
    throw std::runtime_error("Dynamics: EE frame '" + ee_frame + "' not in URDF");
  }
  impl_->frame_id = impl_->model.getFrameId(ee_frame);
}
Dynamics::~Dynamics() = default;
int Dynamics::nv() const { return impl_->model.nv; }
int Dynamics::nq() const { return impl_->model.nq; }
void Dynamics::gravity(const JointVec& q, JointVec& tau_out) {
  impl_->pack(q);
  const Eigen::VectorXd& g =
      pinocchio::computeGeneralizedGravity(impl_->model, impl_->data, impl_->qcfg);
  for (int i = 0; i < impl_->model.nv; ++i) tau_out[i] = g[i];
}
Pose Dynamics::fk(const JointVec& q) {
  (void)q;                       // implemented in Task 3
  return Pose{};
}
void Dynamics::jacobian(const JointVec& q, Jacobian6& J_out) {
  (void)q;                       // implemented in Task 4
  J_out.setZero();
}
}  // namespace kinova
```
> Note: the stub `fk`/`jacobian` bodies keep the build green now; Tasks 3 and 4 replace them and their tests prove them.

- [ ] **Step 5: Run tests**

Run: `bash local_tools/build_on_abra.sh`
Expected: all tests pass, including `Dynamics.DefaultEeFrameResolvesOn2f85` and `Dynamics.UnknownFrameThrows`.

- [ ] **Step 6: Commit**
```bash
git add include/kinova_lowlevel/dynamics.h src/dynamics.cpp tests/dynamics_test.cpp
git commit -m "feat(dynamics): configurable validated EE frame + single pack() helper"
```

---

## Task 3: Dynamics::fk (forward kinematics)

**Files:**
- Modify: `src/dynamics.cpp`
- Modify: `tests/dynamics_test.cpp`

- [ ] **Step 1: Write the failing tests**

Append to `tests/dynamics_test.cpp` (no new includes needed — `Pose`/Eigen come in via `dynamics.h` → `cartesian_types.h`):
```cpp
TEST(DynamicsFk, NeutralPoseIsFiniteUnitQuatInReach) {
  Dynamics dyn(URDF_PATH);
  JointVec q = JointVec::Zero();
  Pose x = dyn.fk(q);
  EXPECT_TRUE(x.p.allFinite());
  EXPECT_NEAR(x.R.norm(), 1.0, 1e-9);              // unit quaternion
  EXPECT_GT(x.p.norm(), 0.05);                     // tip is away from base origin
  EXPECT_LT(x.p.norm(), 1.5);                      // within physical reach
}

TEST(DynamicsFk, BaseYawRotatesTipInPlane) {
  // Rotating joint 0 (base yaw about world Z) by +90deg must rotate the tip
  // position about Z: (x,y,z) -> (-y, x, z). Independent of exact link lengths.
  Dynamics dyn(URDF_PATH);
  JointVec q = JointVec::Zero();
  Pose a = dyn.fk(q);
  q[0] = M_PI / 2.0;
  Pose b = dyn.fk(q);
  EXPECT_NEAR(b.p.x(), -a.p.y(), 1e-6);
  EXPECT_NEAR(b.p.y(),  a.p.x(), 1e-6);
  EXPECT_NEAR(b.p.z(),  a.p.z(), 1e-6);
}
```

- [ ] **Step 2: Run to verify failure**

Run: `bash local_tools/sync_to_abra.sh && ssh abra 'cd ~/kinova-gen3-driver/build && cmake --build . -j unit_tests && ./unit_tests --gtest_filter="DynamicsFk.*"'`
Expected: FAIL — stub `fk` returns identity `Pose`, so `p.norm()` is 0 and the base-yaw test fails.

- [ ] **Step 3: Implement fk**

In `src/dynamics.cpp`, replace the stub `fk` with:
```cpp
Pose Dynamics::fk(const JointVec& q) {
  impl_->pack(q);
  pinocchio::forwardKinematics(impl_->model, impl_->data, impl_->qcfg);
  pinocchio::updateFramePlacement(impl_->model, impl_->data, impl_->frame_id);
  const pinocchio::SE3& M = impl_->data.oMf[impl_->frame_id];
  Pose x;
  x.p = M.translation();
  x.R = Eigen::Quaterniond(M.rotation());
  x.R.normalize();
  return x;
}
```

- [ ] **Step 4: Run tests**

Run: `bash local_tools/sync_to_abra.sh && ssh abra 'cd ~/kinova-gen3-driver/build && cmake --build . -j unit_tests && ./unit_tests --gtest_filter="DynamicsFk.*"'`
Expected: PASS.

- [ ] **Step 5: Commit**
```bash
git add src/dynamics.cpp tests/dynamics_test.cpp
git commit -m "feat(dynamics): forward kinematics fk() for the EE frame"
```

---

## Task 4: Dynamics::jacobian (frame Jacobian)

**Files:**
- Modify: `src/dynamics.cpp`
- Modify: `tests/dynamics_test.cpp`

- [ ] **Step 1: Write the failing test (finite-difference validation)**

Append to `tests/dynamics_test.cpp`:
```cpp
namespace {
// Angular part of a small rotation R_b * R_a^{-1} as a rotation vector.
Eigen::Vector3d rotvec(const Eigen::Quaterniond& Ra, const Eigen::Quaterniond& Rb) {
  Eigen::Quaterniond qe = Rb * Ra.inverse();
  if (qe.w() < 0) qe.coeffs() *= -1.0;     // shortest path
  Eigen::AngleAxisd aa(qe.normalized());
  return aa.angle() * aa.axis();
}
}  // namespace

TEST(DynamicsJacobian, MatchesFiniteDifferenceOfFk) {
  Dynamics dyn(URDF_PATH);
  JointVec q;
  q << 0.1, 0.3, -0.2, 0.8, 0.5, -0.4, 0.2;     // a non-singular pose
  Jacobian6 J; dyn.jacobian(q, J);

  const double eps = 1e-6;
  for (int i = 0; i < kNumJoints; ++i) {
    JointVec qp = q; qp[i] += eps;
    Pose a = dyn.fk(q), b = dyn.fk(qp);
    Eigen::Vector3d dlin = (b.p - a.p) / eps;          // world-frame linear vel
    Eigen::Vector3d dang = rotvec(a.R, b.R) / eps;     // world-frame angular vel
    EXPECT_NEAR((J.block<3,1>(0,i) - dlin).norm(), 0.0, 1e-4) << "lin col " << i;
    EXPECT_NEAR((J.block<3,1>(3,i) - dang).norm(), 0.0, 1e-4) << "ang col " << i;
  }
}
```

- [ ] **Step 2: Run to verify failure**

Run: `bash local_tools/sync_to_abra.sh && ssh abra 'cd ~/kinova-gen3-driver/build && cmake --build . -j unit_tests && ./unit_tests --gtest_filter="DynamicsJacobian.*"'`
Expected: FAIL — stub `jacobian` returns zeros.

- [ ] **Step 3: Implement jacobian**

In `src/dynamics.cpp`, replace the stub `jacobian` with:
```cpp
void Dynamics::jacobian(const JointVec& q, Jacobian6& J_out) {
  impl_->pack(q);
  J_out.setZero();
  // LOCAL_WORLD_ALIGNED: spatial velocity expressed in world-aligned axes at the
  // EE origin — the same frame the Cartesian stiffness gains live in.
  pinocchio::computeFrameJacobian(impl_->model, impl_->data, impl_->qcfg,
                                  impl_->frame_id, pinocchio::LOCAL_WORLD_ALIGNED,
                                  J_out);
}
```

- [ ] **Step 4: Run tests**

Run: `bash local_tools/sync_to_abra.sh && ssh abra 'cd ~/kinova-gen3-driver/build && cmake --build . -j unit_tests && ./unit_tests --gtest_filter="DynamicsJacobian.*"'`
Expected: PASS (analytic Jacobian matches the finite difference of fk within 1e-4).

- [ ] **Step 5: Commit**
```bash
git add src/dynamics.cpp tests/dynamics_test.cpp
git commit -m "feat(dynamics): frame Jacobian jacobian() (LOCAL_WORLD_ALIGNED)"
```

---

## Task 5: pose_error helper (decoupled SE(3) error)

**Files:**
- Create: `include/kinova_lowlevel/cartesian.h`
- Create: `src/cartesian.cpp`
- Create: `tests/cartesian_test.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing tests**

`tests/cartesian_test.cpp`:
```cpp
#include <gtest/gtest.h>
#include <cmath>
#include "kinova_lowlevel/cartesian.h"
using namespace kinova;

TEST(PoseError, IdenticalPosesGiveZero) {
  Pose a;
  a.p = Eigen::Vector3d(0.1, -0.2, 0.3);
  a.R = Eigen::Quaterniond(Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitY()));
  Vector6 e = pose_error(a, a);
  EXPECT_NEAR(e.norm(), 0.0, 1e-12);
}

TEST(PoseError, PureTranslationIsDesiredMinusCurrent) {
  Pose cur, des;
  cur.p = Eigen::Vector3d(0, 0, 0);
  des.p = Eigen::Vector3d(0.05, -0.10, 0.02);
  Vector6 e = pose_error(des, cur);
  EXPECT_NEAR((e.head<3>() - des.p).norm(), 0.0, 1e-12);
  EXPECT_NEAR(e.tail<3>().norm(), 0.0, 1e-12);
}

TEST(PoseError, PureRotationAboutZ) {
  Pose cur, des;
  const double ang = 0.3;
  des.R = Eigen::Quaterniond(Eigen::AngleAxisd(ang, Eigen::Vector3d::UnitZ()));
  Vector6 e = pose_error(des, cur);
  EXPECT_NEAR(e.head<3>().norm(), 0.0, 1e-12);
  EXPECT_NEAR(e[3], 0.0,  1e-9);
  EXPECT_NEAR(e[4], 0.0,  1e-9);
  EXPECT_NEAR(e[5], ang,  1e-9);
}
```

- [ ] **Step 2: Create the header and implementation**

`include/kinova_lowlevel/cartesian.h`:
```cpp
#pragma once
#include "kinova_lowlevel/cartesian_types.h"
namespace kinova {
// Decoupled geometric SE(3) error: [ p_d - p ; rotvec(R_d * R^{-1}) ].
// Singularity-free for orientation errors below pi. Pairs with diagonal
// world-frame stiffness. Eigen-only — no Pinocchio.
Vector6 pose_error(const Pose& desired, const Pose& current);
}  // namespace kinova
```

`src/cartesian.cpp`:
```cpp
#include "kinova_lowlevel/cartesian.h"
namespace kinova {
Vector6 pose_error(const Pose& desired, const Pose& current) {
  Vector6 e;
  e.head<3>() = desired.p - current.p;
  Eigen::Quaterniond qe = desired.R * current.R.inverse();
  if (qe.w() < 0) qe.coeffs() *= -1.0;          // shortest geodesic
  Eigen::AngleAxisd aa(qe.normalized());
  e.tail<3>() = aa.angle() * aa.axis();
  return e;
}
}  // namespace kinova
```

- [ ] **Step 3: Wire into CMake**

In `CMakeLists.txt`: add `src/cartesian.cpp` to `KINOVA_LIB_SOURCES` (after `src/dynamics.cpp`), and add `tests/cartesian_test.cpp` to the `unit_tests` sources.

- [ ] **Step 4: Run tests**

Run: `bash local_tools/sync_to_abra.sh && ssh abra 'cd ~/kinova-gen3-driver/build && cmake --build . -j unit_tests && ./unit_tests --gtest_filter="PoseError.*"'`
Expected: PASS.

- [ ] **Step 5: Commit**
```bash
git add include/kinova_lowlevel/cartesian.h src/cartesian.cpp tests/cartesian_test.cpp CMakeLists.txt
git commit -m "feat(cartesian): decoupled SE(3) pose_error helper"
```

---

## Task 6: CartesianImpedanceMode — core control law

> Implements the task term + gravity + clamp + position passthrough, plus the
> RT-safe live setters. Nullspace term (Task 7) and entry ramp (Task 8) come next.

**Files:**
- Create: `include/kinova_lowlevel/cartesian_impedance_mode.h`
- Create: `src/cartesian_impedance_mode.cpp`
- Create: `tests/cartesian_impedance_mode_test.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Create the header**

`include/kinova_lowlevel/cartesian_impedance_mode.h`:
```cpp
#pragma once
#include <atomic>
#include "kinova_lowlevel/control_mode.h"
#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/cartesian.h"
namespace kinova {

struct CartesianImpedanceParams {
  Vector6 Kx = (Vector6() << 300,300,300, 30,30,30).finished();  // N/m | N·m/rad
  Vector6 Dx = (Vector6() << 35,35,35, 5,5,5).finished();        // N·s/m | N·m·s/rad
  double  nullspace_kp = 5.0;     // joint posture stiffness (N·m/rad)
  double  nullspace_kd = 1.0;     // joint posture damping (N·m·s/rad)
  bool    nullspace_on = true;
  double  pinv_damping = 1e-3;    // Levenberg-Marquardt lambda for the pseudo-inverse
  double  torque_limit = 39.0;    // per-joint clamp (N·m)
  double  gain_ramp_s  = 0.5;     // ramp the active wrench 0->1 over this window on entry
};

// Task-space impedance: tau = gravity + ramp * (Jᵀ F + nullspace),
//   F = Kx ∘ pose_error(x_d, fk(q)) - Dx ∘ (J qd).
// Holds the entry pose by default. Live setters publish via a single-writer
// (non-RT) double-buffer; compute() (RT thread) reads one snapshot per cycle.
class CartesianImpedanceMode : public ControlMode {
 public:
  CartesianImpedanceMode(Dynamics& dyn, CartesianImpedanceParams p = {});
  ActuatorModes required_modes() const override;
  void on_enter(const JointFeedback& fb) override;
  void compute(const JointFeedback& fb, double dt_s, JointCommand& out) override;
  void on_exit() override {}

  // Non-RT setters (call from one supervisor thread).
  void set_gains(const CartesianImpedanceParams& p) noexcept;
  void set_target(const Pose& x_d) noexcept;

 private:
  const CartesianImpedanceParams& params() const noexcept;  // RT-safe snapshot
  Dynamics& dyn_;

  // Gains double-buffer (writer: set_gains; reader: compute). Seeded at ctor.
  CartesianImpedanceParams gains_[2];
  std::atomic<int> gains_active_{0};

  // Target: entry pose (RT-only) + optional external override (writer: set_target).
  Pose entry_pose_;
  Pose ext_target_[2];
  std::atomic<int> ext_active_{0};
  std::atomic<bool> has_ext_target_{false};

  double ramp_elapsed_ = 0.0;

  // Preallocated RT scratch.
  Jacobian6 J_;
  JointVec  g_;
  JointVec  tau_;
};
}  // namespace kinova
```

- [ ] **Step 2: Write the failing tests**

`tests/cartesian_impedance_mode_test.cpp`:
```cpp
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include "kinova_lowlevel/cartesian_impedance_mode.h"
using namespace kinova;

namespace {
JointVec sample_q() {
  JointVec q; q << 0.1, 0.3, -0.2, 0.8, 0.5, -0.4, 0.2; return q;
}
}  // namespace

TEST(CartesianImpedance, RequiresTorqueAndPassesThroughPosition) {
  Dynamics dyn(URDF_PATH);
  CartesianImpedanceParams p; p.gain_ramp_s = 0.0; p.nullspace_on = false;
  CartesianImpedanceMode m(dyn, p);
  JointFeedback fb; fb.q = sample_q(); fb.qd.setZero();
  for (auto x : m.required_modes()) EXPECT_EQ(x, ActuatorMode::kTorque);
  JointCommand c; m.on_enter(fb); m.compute(fb, 0.001, c);
  EXPECT_EQ(c.mode, ActuatorMode::kTorque);
  EXPECT_NEAR((c.position - fb.q).norm(), 0.0, 1e-12);
}

TEST(CartesianImpedance, AtTargetZeroVelGivesGravityOnly) {
  Dynamics dyn(URDF_PATH);
  CartesianImpedanceParams p; p.gain_ramp_s = 0.0; p.nullspace_on = false;
  CartesianImpedanceMode m(dyn, p);
  JointFeedback fb; fb.q = sample_q(); fb.qd.setZero();
  m.on_enter(fb);                       // target := fk(fb.q)  -> zero error
  JointCommand c; m.compute(fb, 0.001, c);
  JointVec g; dyn.gravity(fb.q, g);
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(c.torque[i], g[i], 1e-6);
}

TEST(CartesianImpedance, MatchesIndependentlyComputedLaw) {
  Dynamics dyn(URDF_PATH);
  CartesianImpedanceParams p; p.gain_ramp_s = 0.0; p.nullspace_on = false;
  CartesianImpedanceMode m(dyn, p);
  JointFeedback fb; fb.q = sample_q(); fb.qd.setConstant(0.05);
  m.on_enter(fb);
  Pose tgt = dyn.fk(fb.q); tgt.p.x() += 0.05; tgt.p.z() -= 0.03;   // displace
  m.set_target(tgt);
  JointCommand c; m.compute(fb, 0.001, c);

  Jacobian6 J; dyn.jacobian(fb.q, J);
  Vector6 e = pose_error(tgt, dyn.fk(fb.q));
  Vector6 xd = J * fb.qd;
  Vector6 F = p.Kx.cwiseProduct(e) - p.Dx.cwiseProduct(xd);
  JointVec g; dyn.gravity(fb.q, g);
  JointVec expected = g + J.transpose() * F;
  for (int i = 0; i < kNumJoints; ++i)
    expected[i] = std::clamp(expected[i], -p.torque_limit, p.torque_limit);
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(c.torque[i], expected[i], 1e-6);
}

TEST(CartesianImpedance, TorqueIsClamped) {
  Dynamics dyn(URDF_PATH);
  CartesianImpedanceParams p; p.gain_ramp_s = 0.0; p.nullspace_on = false;
  p.torque_limit = 2.0; p.Kx.setConstant(5000);
  CartesianImpedanceMode m(dyn, p);
  JointFeedback fb; fb.q = sample_q(); fb.qd.setZero();
  m.on_enter(fb);
  Pose tgt = dyn.fk(fb.q); tgt.p += Eigen::Vector3d(0.3, 0.3, 0.3);  // big error
  m.set_target(tgt);
  JointCommand c; m.compute(fb, 0.001, c);
  for (int i = 0; i < kNumJoints; ++i) EXPECT_LE(std::abs(c.torque[i]), 2.0 + 1e-9);
}
```

- [ ] **Step 3: Run to verify failure**

Run: `bash local_tools/sync_to_abra.sh && ssh abra 'cd ~/kinova-gen3-driver/build && cmake --build . -j unit_tests'`
Expected: **compile error** — `CartesianImpedanceMode` not implemented / not in the build.

- [ ] **Step 4: Implement the mode (core law, no nullspace/ramp yet)**

`src/cartesian_impedance_mode.cpp`:
```cpp
#include "kinova_lowlevel/cartesian_impedance_mode.h"
#include <algorithm>

namespace kinova {

CartesianImpedanceMode::CartesianImpedanceMode(Dynamics& dyn, CartesianImpedanceParams p)
    : dyn_(dyn) {
  gains_[0] = p;
  gains_[1] = p;
  J_.setZero();
  g_.setZero();
  tau_.setZero();
}

ActuatorModes CartesianImpedanceMode::required_modes() const {
  ActuatorModes modes; modes.fill(ActuatorMode::kTorque); return modes;
}

const CartesianImpedanceParams& CartesianImpedanceMode::params() const noexcept {
  return gains_[gains_active_.load(std::memory_order_acquire)];
}

void CartesianImpedanceMode::set_gains(const CartesianImpedanceParams& p) noexcept {
  int next = 1 - gains_active_.load(std::memory_order_relaxed);
  gains_[next] = p;
  gains_active_.store(next, std::memory_order_release);
}

void CartesianImpedanceMode::set_target(const Pose& x_d) noexcept {
  int next = 1 - ext_active_.load(std::memory_order_relaxed);
  ext_target_[next] = x_d;
  ext_active_.store(next, std::memory_order_release);
  has_ext_target_.store(true, std::memory_order_release);
}

void CartesianImpedanceMode::on_enter(const JointFeedback& fb) {
  entry_pose_ = dyn_.fk(fb.q);                       // hold where we are
  has_ext_target_.store(false, std::memory_order_release);
  ramp_elapsed_ = 0.0;
}

void CartesianImpedanceMode::compute(const JointFeedback& fb, double /*dt_s*/,
                                     JointCommand& out) {
  const CartesianImpedanceParams& p = params();
  const Pose target = has_ext_target_.load(std::memory_order_acquire)
                          ? ext_target_[ext_active_.load(std::memory_order_acquire)]
                          : entry_pose_;

  Pose x = dyn_.fk(fb.q);
  dyn_.jacobian(fb.q, J_);
  Vector6 xd = J_ * fb.qd;
  Vector6 e  = pose_error(target, x);
  Vector6 F  = p.Kx.cwiseProduct(e) - p.Dx.cwiseProduct(xd);

  dyn_.gravity(fb.q, g_);
  tau_ = g_ + J_.transpose() * F;                    // ramp + nullspace added later

  for (int i = 0; i < kNumJoints; ++i)
    tau_[i] = std::clamp(tau_[i], -p.torque_limit, p.torque_limit);

  out.mode = ActuatorMode::kTorque;
  out.torque = tau_;
  out.position = fb.q;                               // passthrough for following-error hold
}

}  // namespace kinova
```

- [ ] **Step 5: Wire into CMake**

Add `src/cartesian_impedance_mode.cpp` to `KINOVA_LIB_SOURCES` and `tests/cartesian_impedance_mode_test.cpp` to the `unit_tests` sources.

- [ ] **Step 6: Run tests**

Run: `bash local_tools/sync_to_abra.sh && ssh abra 'cd ~/kinova-gen3-driver/build && cmake --build . -j unit_tests && ./unit_tests --gtest_filter="CartesianImpedance.*"'`
Expected: PASS (all four).

- [ ] **Step 7: Commit**
```bash
git add include/kinova_lowlevel/cartesian_impedance_mode.h src/cartesian_impedance_mode.cpp tests/cartesian_impedance_mode_test.cpp CMakeLists.txt
git commit -m "feat(impedance): CartesianImpedanceMode core law + RT-safe live setters"
```

---

## Task 7: Nullspace posture term + damped pseudo-inverse

**Files:**
- Modify: `src/cartesian_impedance_mode.cpp`
- Modify: `include/kinova_lowlevel/cartesian_impedance_mode.h` (add `q_rest_` member)
- Modify: `tests/cartesian_impedance_mode_test.cpp`

- [ ] **Step 1: Write the failing tests**

Append to `tests/cartesian_impedance_mode_test.cpp`:
```cpp
TEST(CartesianImpedanceNullspace, ProjectorAnnihilatesTaskSpace) {
  // Build N = I - Jᵀ (Jᵀ)⁺ the same way the mode does and check J*N ≈ 0 and N idempotent.
  Dynamics dyn(URDF_PATH);
  JointVec q = sample_q();
  Jacobian6 J; dyn.jacobian(q, J);
  Eigen::Matrix<double,6,6> JJt = J * J.transpose();
  JJt.diagonal().array() += 1e-3 * 1e-3;
  Eigen::Matrix<double,6,kNumJoints> JtPinv = JJt.ldlt().solve(J);   // (JJt)^-1 J
  Eigen::Matrix<double,kNumJoints,kNumJoints> N =
      Eigen::Matrix<double,kNumJoints,kNumJoints>::Identity() - J.transpose() * JtPinv;
  EXPECT_NEAR((J * N).norm(), 0.0, 1e-6);          // task rows killed
  EXPECT_NEAR((N * N - N).norm(), 0.0, 1e-6);      // idempotent (damping is tiny)
}

TEST(CartesianImpedanceNullspace, NoEffectAtRestPostureZeroVel) {
  // At q == q_rest (entry config) with zero velocity, the posture torque is 0,
  // so nullspace_on and nullspace_off must agree.
  Dynamics dyn(URDF_PATH);
  JointFeedback fb; fb.q = sample_q(); fb.qd.setZero();

  CartesianImpedanceParams off; off.gain_ramp_s = 0.0; off.nullspace_on = false;
  CartesianImpedanceMode m_off(dyn, off);
  m_off.on_enter(fb);
  JointCommand c_off; m_off.compute(fb, 0.001, c_off);

  CartesianImpedanceParams on = off; on.nullspace_on = true;
  CartesianImpedanceMode m_on(dyn, on);
  m_on.on_enter(fb);
  JointCommand c_on; m_on.compute(fb, 0.001, c_on);

  EXPECT_NEAR((c_on.torque - c_off.torque).norm(), 0.0, 1e-9);
}

TEST(CartesianImpedanceNullspace, PostureTorqueProducesNoTaskWrench) {
  // With nonzero posture error, the EXTRA torque from the nullspace term must lie
  // in null(J): J * (tau_on - tau_off) ≈ 0.  Disable the task term by putting the
  // arm exactly at target (zero error) so the only difference is the posture term.
  Dynamics dyn(URDF_PATH);
  CartesianImpedanceParams off; off.gain_ramp_s = 0.0; off.nullspace_on = false;
  CartesianImpedanceMode m_off(dyn, off);
  CartesianImpedanceParams on = off; on.nullspace_on = true; on.nullspace_kp = 8.0;
  CartesianImpedanceMode m_on(dyn, on);

  JointFeedback enter; enter.q = sample_q(); enter.qd.setZero();
  m_off.on_enter(enter); m_on.on_enter(enter);            // q_rest := sample_q()

  JointFeedback fb; fb.q = sample_q(); fb.q[2] += 0.15;   // posture error, off target
  fb.qd.setZero();
  // keep both at "their target == fk(displaced q)" so the task error is zero:
  m_off.set_target(dyn.fk(fb.q));
  m_on.set_target(dyn.fk(fb.q));

  JointCommand c_off, c_on;
  m_off.compute(fb, 0.001, c_off);
  m_on.compute(fb, 0.001, c_on);

  Jacobian6 J; dyn.jacobian(fb.q, J);
  JointVec dtau = c_on.torque - c_off.torque;
  EXPECT_GT(dtau.norm(), 1e-3);                  // the posture term is actually active
  EXPECT_NEAR((J * dtau).norm(), 0.0, 1e-5);     // ...and produces no task wrench
}
```
> Note: these tests assume torque does not clamp (default `torque_limit = 39` with small `nullspace_kp`); keep posture gains modest so the sum stays unclamped.

- [ ] **Step 2: Run to verify failure**

Run: `bash local_tools/sync_to_abra.sh && ssh abra 'cd ~/kinova-gen3-driver/build && cmake --build . -j unit_tests && ./unit_tests --gtest_filter="CartesianImpedanceNullspace.*"'`
Expected: FAIL — `NoEffectAtRestPostureZeroVel` may pass trivially, but `PostureTorqueProducesNoTaskWrench` FAILS (nullspace term not implemented, `dtau` is ~0 so the `>1e-3` assertion fails).

- [ ] **Step 3: Add `q_rest_` to the header**

In `include/kinova_lowlevel/cartesian_impedance_mode.h`, add a member alongside `entry_pose_`:
```cpp
  JointVec q_rest_ = JointVec::Zero();   // nullspace posture target (set on_enter)
```

- [ ] **Step 4: Implement the nullspace term**

In `src/cartesian_impedance_mode.cpp`, set the rest posture in `on_enter` (add after `entry_pose_ = ...`):
```cpp
  q_rest_ = fb.q;
```
Then in `compute`, replace the `tau_ = g_ + J_.transpose() * F;` line with:
```cpp
  tau_ = g_ + J_.transpose() * F;
  if (p.nullspace_on) {
    // Damped pseudo-inverse of Jᵀ:  (Jᵀ)⁺ = (J Jᵀ + λ²I)⁻¹ J   (6x7).
    // N = I - Jᵀ (Jᵀ)⁺  projects secondary joint torques into null(J), so they
    // produce no task-space wrench. Fixed-size LDLT — no heap allocation.
    Eigen::Matrix<double, 6, 6> JJt = J_ * J_.transpose();
    JJt.diagonal().array() += p.pinv_damping * p.pinv_damping;
    Eigen::Matrix<double, 6, kNumJoints> JtPinv = JJt.ldlt().solve(J_);
    Eigen::Matrix<double, kNumJoints, kNumJoints> N =
        Eigen::Matrix<double, kNumJoints, kNumJoints>::Identity()
        - J_.transpose() * JtPinv;
    JointVec tau0 = p.nullspace_kp * (q_rest_ - fb.q) - p.nullspace_kd * fb.qd;
    tau_ += N * tau0;
  }
```
(The clamp loop and `out.*` assignments stay below, unchanged.)

- [ ] **Step 5: Run tests**

Run: `bash local_tools/sync_to_abra.sh && ssh abra 'cd ~/kinova-gen3-driver/build && cmake --build . -j unit_tests && ./unit_tests --gtest_filter="CartesianImpedance*"'`
Expected: PASS (core law tests from Task 6 and all three nullspace tests).

- [ ] **Step 6: Commit**
```bash
git add include/kinova_lowlevel/cartesian_impedance_mode.h src/cartesian_impedance_mode.cpp tests/cartesian_impedance_mode_test.cpp
git commit -m "feat(impedance): nullspace posture term via damped pseudo-inverse projector"
```

---

## Task 8: Entry gain ramp (active wrench only; gravity always full)

> The ramp scales ONLY the impedance + nullspace torque, never gravity — so the
> arm holds against gravity from the first cycle and compliance fades in smoothly.

**Files:**
- Modify: `src/cartesian_impedance_mode.cpp`
- Modify: `tests/cartesian_impedance_mode_test.cpp`

- [ ] **Step 1: Write the failing tests**

Append to `tests/cartesian_impedance_mode_test.cpp`:
```cpp
TEST(CartesianImpedanceRamp, GravityHeldFullDuringRampButWrenchScales) {
  Dynamics dyn(URDF_PATH);
  CartesianImpedanceParams p; p.gain_ramp_s = 1.0; p.nullspace_on = false;
  CartesianImpedanceMode m(dyn, p);
  JointFeedback fb; fb.q = sample_q(); fb.qd.setZero();
  m.on_enter(fb);
  Pose tgt = dyn.fk(fb.q); tgt.p.x() += 0.05; m.set_target(tgt);

  JointVec g; dyn.gravity(fb.q, g);
  Jacobian6 J; dyn.jacobian(fb.q, J);
  Vector6 e = pose_error(tgt, dyn.fk(fb.q));
  JointVec wrench = J.transpose() * (p.Kx.cwiseProduct(e));   // full active term, qd=0

  // First cycle: elapsed=0 -> ramp 0 -> torque == gravity only.
  JointCommand c0; m.compute(fb, 0.25, c0);
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(c0.torque[i], g[i], 1e-6);

  // After ~0.25s accumulated: ramp ≈ 0.25 -> active term partially applied.
  JointCommand c1; m.compute(fb, 0.25, c1);
  JointVec expected1 = g + 0.25 * wrench;
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(c1.torque[i], expected1[i], 1e-5);
}

TEST(CartesianImpedanceRamp, ZeroRampMeansFullImmediately) {
  Dynamics dyn(URDF_PATH);
  CartesianImpedanceParams p; p.gain_ramp_s = 0.0; p.nullspace_on = false;
  CartesianImpedanceMode m(dyn, p);
  JointFeedback fb; fb.q = sample_q(); fb.qd.setZero();
  m.on_enter(fb);
  Pose tgt = dyn.fk(fb.q); tgt.p.x() += 0.05; m.set_target(tgt);
  JointCommand c; m.compute(fb, 0.001, c);
  JointVec g; dyn.gravity(fb.q, g);
  Jacobian6 J; dyn.jacobian(fb.q, J);
  Vector6 e = pose_error(tgt, dyn.fk(fb.q));
  JointVec expected = g + J.transpose() * (p.Kx.cwiseProduct(e));
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(c.torque[i], expected[i], 1e-5);
}
```
> Note: the ramp uses time accumulated at the *start* of each cycle, then advances by `dt` — so the first `compute` after `on_enter` sees ramp 0. The Task-6 `MatchesIndependentlyComputedLaw` test uses `gain_ramp_s = 0.0`, so it is unaffected.

- [ ] **Step 2: Run to verify failure**

Run: `bash local_tools/sync_to_abra.sh && ssh abra 'cd ~/kinova-gen3-driver/build && cmake --build . -j unit_tests && ./unit_tests --gtest_filter="CartesianImpedanceRamp.*"'`
Expected: FAIL — without a ramp the first cycle applies the full wrench, not gravity-only.

- [ ] **Step 3: Implement the ramp**

In `src/cartesian_impedance_mode.cpp` `compute`, split gravity from the active term. Replace the block:
```cpp
  dyn_.gravity(fb.q, g_);
  tau_ = g_ + J_.transpose() * F;
  if (p.nullspace_on) {
    ...
    tau_ += N * tau0;
  }
```
with:
```cpp
  dyn_.gravity(fb.q, g_);

  JointVec tau_active = J_.transpose() * F;
  if (p.nullspace_on) {
    Eigen::Matrix<double, 6, 6> JJt = J_ * J_.transpose();
    JJt.diagonal().array() += p.pinv_damping * p.pinv_damping;
    Eigen::Matrix<double, 6, kNumJoints> JtPinv = JJt.ldlt().solve(J_);
    Eigen::Matrix<double, kNumJoints, kNumJoints> N =
        Eigen::Matrix<double, kNumJoints, kNumJoints>::Identity()
        - J_.transpose() * JtPinv;
    JointVec tau0 = p.nullspace_kp * (q_rest_ - fb.q) - p.nullspace_kd * fb.qd;
    tau_active += N * tau0;
  }

  const double ramp = (p.gain_ramp_s <= 0.0)
                          ? 1.0
                          : std::min(1.0, ramp_elapsed_ / p.gain_ramp_s);
  tau_ = g_ + ramp * tau_active;       // gravity always full; compliance fades in
  ramp_elapsed_ += dt_s;
```
Also change the `compute` signature to use the `dt_s` parameter (it is currently named but unused — drop any `/*dt_s*/` comment so it reads `double dt_s`).

- [ ] **Step 4: Run tests**

Run: `bash local_tools/sync_to_abra.sh && ssh abra 'cd ~/kinova-gen3-driver/build && cmake --build . -j unit_tests && ./unit_tests --gtest_filter="CartesianImpedance*"'`
Expected: PASS (all impedance tests, including ramp).

- [ ] **Step 5: Commit**
```bash
git add src/cartesian_impedance_mode.cpp tests/cartesian_impedance_mode_test.cpp
git commit -m "feat(impedance): entry gain ramp on the active wrench (gravity always full)"
```

---

## Task 9: RT-safety test for the impedance mode

**Files:**
- Modify: `tests/rt_safety_test.cpp`

- [ ] **Step 1: Write the failing test**

Append to `tests/rt_safety_test.cpp` (add `#include "kinova_lowlevel/cartesian_impedance_mode.h"` at the top with the other includes):
```cpp
TEST(RtSafety, ImpedanceModeNoMajorFaultsSteadyState) {
  JointFeedback init; init.q.setZero();
  SimTransport t(init);
  Dynamics dyn(URDF_PATH);
  CartesianImpedanceMode mode(dyn);              // defaults; nullspace on
  SampleRing ring(8192);
  RtExecutor ex(t, ring, {2000.0, Pacing::kSleepSpin, {0, -1, true}});

  std::atomic<bool> stop{false};
  std::atomic<uint64_t> majflt_delta{~0ull};
  std::thread drain([&] { CycleSample s; while (!stop.load()) { while (ring.pop(s)) {} } });

  std::thread loop([&] {
    ex.request_mode(&mode);
    std::atomic<bool> warm_stop{false};
    std::thread warm_watch([&] {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      warm_stop.store(true);
    });
    ex.run(warm_stop);
    warm_watch.join();

    ResourceUsage u0 = read_usage();
    ex.request_mode(&mode);
    std::atomic<bool> measure_stop{false};
    std::thread measure_watch([&] {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      measure_stop.store(true);
    });
    ex.run(measure_stop);
    measure_watch.join();

    ResourceUsage u1 = read_usage();
    majflt_delta.store(u1.majflt - u0.majflt);
    stop.store(true);
  });

  loop.join();
  drain.join();
  EXPECT_EQ(majflt_delta.load(), 0u);
  EXPECT_EQ(ring.dropped(), 0u);
}
```

- [ ] **Step 2: Run to verify it builds and passes**

Run: `bash local_tools/sync_to_abra.sh && ssh abra 'cd ~/kinova-gen3-driver/build && cmake --build . -j unit_tests && ./unit_tests --gtest_filter="RtSafety.ImpedanceModeNoMajorFaultsSteadyState"'`
Expected: PASS — zero new major page faults in steady state (proves fk/jacobian/pseudo-inverse allocate nothing after warm-up), zero dropped samples.

> If this FAILS on major faults, the culprit is a hidden heap allocation in
> `compute()` (e.g. a dynamic-size Eigen temporary). All temporaries here are
> fixed-size; investigate before proceeding — do not relax the assertion.

- [ ] **Step 3: Commit**
```bash
git add tests/rt_safety_test.cpp
git commit -m "test(impedance): RT-safety (zero major faults) for CartesianImpedanceMode"
```

---

## Task 10: Demo / benchmark app (`benchmark_cartesian_impedance`)

**Files:**
- Create: `apps/benchmark_cartesian_impedance.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Create the app**

`apps/benchmark_cartesian_impedance.cpp` — mirrors `benchmark_grav_comp.cpp` (reuse its CLI/telemetry/RT scaffolding) but runs `CartesianImpedanceMode`, holds the entry pose, and has a read-only `--dry-run`:
```cpp
// benchmark_cartesian_impedance — runs CartesianImpedanceMode through the
// RtExecutor at a fixed rate and reports timing. SIM by default (--sim); the
// real KortexTransport path is compiled only with KORTEX and never run
// unattended. Holds the pose captured at entry (compliant hold: push the arm
// and it springs back). --dry-run is READ-ONLY: prints fk(q), Jacobian
// condition number, and the would-be task wrench WITHOUT commanding torque.
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "kinova_lowlevel/cartesian_impedance_mode.h"
#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/rt_executor.h"
#include "kinova_lowlevel/rt_system.h"
#include "kinova_lowlevel/sim_transport.h"
#include "kinova_lowlevel/telemetry.h"
#include "kinova_lowlevel/telemetry_consumers.h"
#include "kinova_lowlevel/transport.h"
#ifndef KINOVA_NO_KORTEX
#include "kinova_lowlevel/kortex_transport.h"
#endif

using namespace kinova;
namespace {
std::atomic<bool> g_stop{false};
void on_sigint(int) { g_stop.store(true); }
}  // namespace

int main(int argc, char** argv) {
  std::string ip;
  std::string urdf = "../models/gen3_7dof_2f85.urdf";
  std::string csv_path;
  bool use_sim = false, dry_run = false;
  double rate_hz = 1000.0, duration_s = 10.0;
  int cpu = -1, rt_priority = 80;
  CartesianImpedanceParams p;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&](const char* n) -> std::string {
      if (i + 1 >= argc) { std::cerr << n << " needs a value\n"; std::exit(2); }
      return argv[++i];
    };
    if (a == "--ip") ip = next("--ip");
    else if (a == "--sim") use_sim = true;
    else if (a == "--dry-run") dry_run = true;
    else if (a == "--urdf") urdf = next("--urdf");
    else if (a == "--rate") rate_hz = std::stod(next("--rate"));
    else if (a == "--cpu") cpu = std::stoi(next("--cpu"));
    else if (a == "--rt-priority") rt_priority = std::stoi(next("--rt-priority"));
    else if (a == "--duration") duration_s = std::stod(next("--duration"));
    else if (a == "--csv") csv_path = next("--csv");
    else if (a == "--kx-trans") p.Kx.head<3>().setConstant(std::stod(next("--kx-trans")));
    else if (a == "--kx-rot") p.Kx.tail<3>().setConstant(std::stod(next("--kx-rot")));
    else if (a == "--dx-trans") p.Dx.head<3>().setConstant(std::stod(next("--dx-trans")));
    else if (a == "--dx-rot") p.Dx.tail<3>().setConstant(std::stod(next("--dx-rot")));
    else if (a == "--nullspace-kp") p.nullspace_kp = std::stod(next("--nullspace-kp"));
    else if (a == "--torque-limit") p.torque_limit = std::stod(next("--torque-limit"));
    else if (a == "--no-nullspace") p.nullspace_on = false;
    else { std::cerr << "unknown arg: " << a << "\n"; std::exit(2); }
  }

  std::cout << "[imp] urdf=" << urdf << " rate=" << rate_hz << "Hz sim="
            << (use_sim ? "yes" : "no") << " dry_run=" << (dry_run ? "yes" : "no")
            << "\n";

  Dynamics dyn(urdf);

  std::unique_ptr<Transport> transport;
  if (use_sim) {
    JointFeedback init; transport = std::make_unique<SimTransport>(init);
  } else {
#ifndef KINOVA_NO_KORTEX
    if (ip.empty()) { std::cerr << "real-robot mode requires --ip (or --sim)\n"; return 2; }
    transport = std::make_unique<KortexTransport>(ip);
#else
    std::cerr << "built without KORTEX; only --sim is available\n"; return 2;
#endif
  }
  Transport& t = *transport;
  std::signal(SIGINT, on_sigint);

  // --- dry-run: READ-ONLY, never commands torque -----------------------------
  if (dry_run) {
    t.connect();
    std::cout << "[dry-run] READ-ONLY — NO torque commanded.\n";
    JointFeedback fb; Jacobian6 J;
    const auto start = std::chrono::steady_clock::now();
    auto last = start - std::chrono::seconds(1);
    while (!g_stop.load(std::memory_order_acquire)) {
      t.receive(fb);
      Pose x = dyn.fk(fb.q);
      dyn.jacobian(fb.q, J);
      Eigen::JacobiSVD<Eigen::Matrix<double, 6, kNumJoints>> svd(J);
      double cond = svd.singularValues()(0) /
                    std::max(1e-12, svd.singularValues()(svd.singularValues().size() - 1));
      auto now = std::chrono::steady_clock::now();
      if (now - last >= std::chrono::milliseconds(500)) {
        std::printf("pos[m]=(%+.3f %+.3f %+.3f)  quat=(%+.3f %+.3f %+.3f %+.3f)  Jcond=%.1f\n",
                    x.p.x(), x.p.y(), x.p.z(), x.R.w(), x.R.x(), x.R.y(), x.R.z(), cond);
        last = now;
      }
      if (duration_s > 0.0 &&
          std::chrono::duration<double>(now - start).count() >= duration_s) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    t.safe_shutdown();
    std::cout << "[dry-run] done — no torque was ever commanded.\n";
    return 0;
  }

  SampleRing ring(1 << 16);
  TelemetrySink sink(csv_path);
  std::atomic<bool> draining{true};
  std::thread drain([&] {
    CycleSample s; auto last = std::chrono::steady_clock::now();
    while (draining.load(std::memory_order_acquire)) {
      while (ring.pop(s)) sink.consume(s);
      auto now = std::chrono::steady_clock::now();
      if (now - last >= std::chrono::seconds(1)) {
        std::cout << sink.console_line() << " dropped=" << ring.dropped() << "\n";
        last = now;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    while (ring.pop(s)) sink.consume(s);
  });

  t.connect();
  t.set_servoing_low_level();
  CartesianImpedanceMode mode(dyn, p);
  RtExecutor ex(t, ring, {rate_hz, Pacing::kSleepSpin, {rt_priority, cpu, true}});
  ex.request_mode(&mode);

  std::thread watchdog;
  if (duration_s > 0.0) {
    watchdog = std::thread([&] {
      auto deadline = std::chrono::steady_clock::now() +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(duration_s));
      while (!g_stop.load(std::memory_order_acquire)) {
        if (std::chrono::steady_clock::now() >= deadline) { g_stop.store(true); break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    });
  }

  ResourceUsage usage_before = read_usage();
  ex.run(g_stop);
  ResourceUsage usage_after = read_usage();
  t.safe_shutdown();

  if (watchdog.joinable()) watchdog.join();
  draining.store(false, std::memory_order_release);
  drain.join();

  const auto& ch = sink.cycle_hist();
  const auto& mh = sink.compute_hist();
  std::cout << "\n==== impedance benchmark report ====\n" << introspect() << "\n";
  std::cout << "cycle_ns   n=" << ch.count() << " p50=" << ch.percentile(0.50)
            << " p99=" << ch.percentile(0.99) << " p99.9=" << ch.percentile(0.999)
            << " max=" << ch.max() << "\n";
  std::cout << "compute_ns n=" << mh.count() << " p50=" << mh.percentile(0.50)
            << " p99=" << mh.percentile(0.99) << " p99.9=" << mh.percentile(0.999)
            << " max=" << mh.max() << "\n";
  std::cout << "dropped=" << ring.dropped() << "  majflt+="
            << (usage_after.majflt - usage_before.majflt) << "\n";
  std::cout << "====================================\n";
  return 0;
}
```
> If `sink.console_line()`, `sink.cycle_hist()`, `introspect()`, or the `ExecutorConfig`/`RtConfig` brace-init in `benchmark_grav_comp.cpp` differ from the above, match the EXACT names/forms used there — copy them verbatim from `apps/benchmark_grav_comp.cpp`. The `#include <csignal>` is needed for `std::signal`; add it if the copied scaffolding doesn't.

- [ ] **Step 2: Wire into CMake**

In `CMakeLists.txt`, after the `benchmark_grav_comp` target block, add a parallel block:
```cmake
add_executable(benchmark_cartesian_impedance apps/benchmark_cartesian_impedance.cpp)
target_link_libraries(benchmark_cartesian_impedance PRIVATE kinova_lowlevel Eigen3::Eigen)
target_include_directories(benchmark_cartesian_impedance PRIVATE include)
if(KINOVA_ENABLE_KORTEX)
    target_include_directories(benchmark_cartesian_impedance PRIVATE ${KORTEX_INCLUDE_DIRS})
    target_compile_definitions(benchmark_cartesian_impedance PRIVATE _OS_UNIX)
else()
    target_compile_definitions(benchmark_cartesian_impedance PRIVATE KINOVA_NO_KORTEX)
endif()
```

- [ ] **Step 3: Build and smoke-run in sim**

Run:
```sh
bash local_tools/sync_to_abra.sh && ssh abra 'cd ~/kinova-gen3-driver/build && cmake --build . -j benchmark_cartesian_impedance && \
  ./benchmark_cartesian_impedance --sim --urdf ../models/gen3_7dof_2f85.urdf --rate 1000 --duration 3'
```
Expected: runs ~3 s, prints ~1 Hz console lines and a final report with `dropped=0`, `majflt+=0`, and a `compute_ns` p50 in the low-µs range (fk+jacobian+gravity+pseudo-inverse). No crash.

- [ ] **Step 4: Commit**
```bash
git add apps/benchmark_cartesian_impedance.cpp CMakeLists.txt
git commit -m "feat(app): benchmark_cartesian_impedance demo + read-only --dry-run"
```

---

## Final verification

- [ ] **Run the full suite:** `bash local_tools/build_on_abra.sh` — expect every test green (existing + `CartesianTypes`, `Dynamics*`, `PoseError`, `CartesianImpedance*`, `RtSafety.*`).
- [ ] Confirm the impedance `compute_ns` cost from the Task 10 run is documented in the commit/PR description (this is the headline benchmark deliverable).
- [ ] Hardware run is **attended-only** and out of scope for this plan (see `docs/integration-runbook.md`); the `--dry-run` is the pre-torque validation step to use first.
```
