# Joint-Space Impedance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a joint-space impedance control mode that solves IK for the commanded EE pose and servos all 7 joints, eliminating the uncommanded redundant DOF that makes Quest teleop drift into unrecoverable arm configurations.

**Architecture:** A standalone `DiffIkSolver` (bounded damped-least-squares Gauss-Newton, warm-started, RT-safe) integrates a reference configuration `q_d` toward the commanded pose, resolving redundancy via null-space posture bias and joint-limit avoidance. `JointImpedanceMode` runs a per-joint spring from `q_d` to the measured `q` on top of full gravity compensation. Both modes implement a shared `PoseTargetSink` seam so `teleop_socket_server` can select between them with a flag and needs no Python-side changes.

**Tech Stack:** C++17, Eigen (fixed-size, no heap in the RT path), Pinocchio (FK / Jacobian / gravity), GoogleTest, CMake. Target: Linux aarch64 (Jetson "abra").

## Global Constraints

- **Cannot build on macOS.** All builds and tests run on abra via `local_tools/build_on_abra.sh`. Never claim a build or test result that was not produced on abra.
- **RT-safety:** everything reachable from `compute()` must be allocation-free after construction. Fixed-size Eigen types only; no `VectorXd`, no `std::vector`, no `auto` binding an Eigen expression template that outlives its operands.
- **Non-RT setters** are called from exactly one supervisor thread and publish via the existing double-buffer + atomic-index pattern (see `src/cartesian_impedance_mode.cpp`). RT reader takes one snapshot per cycle.
- **No wire-protocol change.** `kVersion` stays 1; `include/kinova_lowlevel/teleop_protocol.h` is not modified. The Python supervisor is not touched.
- **Units:** SI throughout — rad, rad/s, N·m, m.
- **`Dynamics` is not thread-safe.** `fk`/`jacobian`/`gravity` mutate internal Pinocchio state. Never call them from a second thread against the same instance.
- Follow the surrounding comment style: explain *why*, especially for RT-safety and memory-ordering decisions.

---

### Task 1: Joint limits from the URDF

**Files:**
- Modify: `include/kinova_lowlevel/dynamics.h`
- Modify: `src/dynamics.cpp`
- Test: `tests/dynamics_test.cpp`

**Interfaces:**
- Consumes: nothing.
- Produces: `void Dynamics::joint_limits(JointVec& lower, JointVec& upper) const;` — per-joint position limits in rad, `±std::numeric_limits<double>::infinity()` for continuous joints.

- [ ] **Step 1: Write the failing test** — append to `tests/dynamics_test.cpp`

```cpp
TEST(Dynamics, JointLimitsMatchUrdf) {
  Dynamics dyn(URDF_PATH);
  JointVec lo, hi;
  dyn.joint_limits(lo, hi);
  // Gen3 joints 1,3,5,7 (indices 0,2,4,6) are `continuous` in the URDF. Pinocchio
  // packs those as (cos,sin) and reports a meaningless [-1,1] config-space limit,
  // so they must come back as infinite and never be clamped.
  for (int i : {0, 2, 4, 6}) {
    EXPECT_TRUE(std::isinf(lo[i])) << "joint index " << i;
    EXPECT_TRUE(std::isinf(hi[i])) << "joint index " << i;
    EXPECT_LT(lo[i], 0.0);
    EXPECT_GT(hi[i], 0.0);
  }
  EXPECT_NEAR(lo[1], -2.41, 1e-6);  EXPECT_NEAR(hi[1], 2.41, 1e-6);
  EXPECT_NEAR(lo[3], -2.66, 1e-6);  EXPECT_NEAR(hi[3], 2.66, 1e-6);
  EXPECT_NEAR(lo[5], -2.23, 1e-6);  EXPECT_NEAR(hi[5], 2.23, 1e-6);
}
```

Add `#include <cmath>` and `#include <limits>` to the test file if not present.

- [ ] **Step 2: Run to verify it fails**

`./local_tools/build_on_abra.sh abra` — expected: compile error, `joint_limits` is not a member of `Dynamics`.

- [ ] **Step 3: Declare it** in `include/kinova_lowlevel/dynamics.h`, after `jacobian`:

```cpp
  // Per-joint position limits [rad] from the URDF. Continuous joints report
  // ±infinity: Pinocchio packs them (cos,sin) and their config-space limits are
  // [-1,1], which is meaningless as a joint angle. Read-only (model only, never
  // `data`) so it is safe to call while another thread uses fk/jacobian.
  void joint_limits(JointVec& lower, JointVec& upper) const;
```

- [ ] **Step 4: Implement it** in `src/dynamics.cpp` (add `#include <limits>`):

```cpp
void Dynamics::joint_limits(JointVec& lower, JointVec& upper) const {
  const pinocchio::Model& m = impl_->model;
  constexpr double kInf = std::numeric_limits<double>::infinity();
  // Same name->joint->config-index walk as Impl::pack, so limits can never
  // disagree with the configuration packing about which joint is which.
  for (int i = 0; i < m.nv; ++i) {
    const int jid = m.getJointId(m.names[i + 1]);
    const int qidx = m.idx_qs[jid];
    if (m.nqs[jid] == 2) {
      lower[i] = -kInf;
      upper[i] = kInf;
    } else {
      lower[i] = m.lowerPositionLimit[qidx];
      upper[i] = m.upperPositionLimit[qidx];
    }
  }
}
```

- [ ] **Step 5: Run to verify it passes** — `./local_tools/build_on_abra.sh abra`, expect `unit_tests` PASS.

- [ ] **Step 6: Commit**

```bash
git add include/kinova_lowlevel/dynamics.h src/dynamics.cpp tests/dynamics_test.cpp
git commit -m "feat(dynamics): expose per-joint URDF position limits"
```

---

### Task 2: `PoseTargetSink` seam

**Files:**
- Create: `include/kinova_lowlevel/pose_target_sink.h`
- Modify: `include/kinova_lowlevel/cartesian_impedance_mode.h`

**Interfaces:**
- Consumes: `Pose` from `cartesian_types.h`.
- Produces: `class PoseTargetSink` with `virtual void set_target(const Pose&) noexcept = 0;`. `CartesianImpedanceMode` now derives from it.

- [ ] **Step 1: Create the header**

```cpp
#pragma once
#include "kinova_lowlevel/cartesian_types.h"
namespace kinova {

// Non-RT seam for publishing a Cartesian target to a control mode. Every mode the
// teleop server can drive implements it, so the server holds a single pointer and
// does not care which impedance law is running underneath.
//
// Contract: callable from ONE non-RT supervisor thread concurrently with the RT
// thread's compute(). Implementations publish via a double-buffer + release-store
// so the RT reader always observes a whole target, never a torn one.
class PoseTargetSink {
 public:
  virtual ~PoseTargetSink() = default;
  virtual void set_target(const Pose& x_d) noexcept = 0;
};

}  // namespace kinova
```

- [ ] **Step 2: Make `CartesianImpedanceMode` implement it** — in `include/kinova_lowlevel/cartesian_impedance_mode.h`, add `#include "kinova_lowlevel/pose_target_sink.h"`, change the class head to:

```cpp
class CartesianImpedanceMode : public ControlMode, public PoseTargetSink {
```

and mark the existing declaration:

```cpp
  void set_target(const Pose& x_d) noexcept override;
```

`src/cartesian_impedance_mode.cpp` needs no change — the definition already matches.

- [ ] **Step 3: Build to verify nothing regressed** — `./local_tools/build_on_abra.sh abra`, expect all existing tests PASS.

- [ ] **Step 4: Commit**

```bash
git add include/kinova_lowlevel/pose_target_sink.h include/kinova_lowlevel/cartesian_impedance_mode.h
git commit -m "refactor(modes): PoseTargetSink seam shared by teleop-drivable modes"
```

---

### Task 3: `DiffIkSolver`

**Files:**
- Create: `include/kinova_lowlevel/diff_ik.h`, `src/diff_ik.cpp`
- Test: `tests/diff_ik_test.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `Dynamics::joint_limits` (Task 1), `pose_error` from `cartesian.h`.
- Produces:
  - `struct DiffIkParams` — fields `max_iters`, `pos_tol`, `rot_tol`, `damping`, `max_pos_err`, `max_rot_err`, `max_joint_step`, `posture_gain`, `q_rest`, `limit_gain`, `limit_margin`, `limit_clamp_margin`, `q_lower`, `q_upper`.
  - `struct IkResult { double pos_err; double rot_err; int iters; bool converged; };`
  - `class DiffIkSolver` — `DiffIkSolver(Dynamics&, DiffIkParams)`, `IkResult solve(const Pose& target, JointVec& q)`, `void set_params(const DiffIkParams&) noexcept`, `const DiffIkParams& params() const noexcept`.

- [ ] **Step 1: Write the header** `include/kinova_lowlevel/diff_ik.h`

```cpp
#pragma once
#include <limits>
#include "kinova_lowlevel/cartesian.h"
#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/joint_types.h"
namespace kinova {

struct DiffIkParams {
  int    max_iters      = 4;      // hard cap: this runs inside the 1 kHz cycle
  double pos_tol        = 1e-4;   // m
  double rot_tol        = 1e-3;   // rad
  double damping        = 1e-3;   // Levenberg-Marquardt lambda for the DLS inverse
  // Per-iteration clamp on the task error fed to the solve. This is what makes an
  // unreachable or teleported target safe: the reference walks toward it at a
  // bounded pace instead of producing one enormous step.
  double max_pos_err    = 0.05;   // m
  double max_rot_err    = 0.20;   // rad
  double max_joint_step = 0.05;   // rad, per joint per iteration
  // Null-space posture bias. Decides the redundant DOF deterministically instead
  // of letting it drift wherever the pose trajectory drags it.
  double posture_gain   = 0.15;
  JointVec q_rest =               // elbow-up home — TUNE ON HARDWARE
      (JointVec() << 0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57).finished();
  // Null-space push away from hard stops, active only inside limit_margin.
  double limit_gain     = 0.5;
  double limit_margin   = 0.25;   // rad
  double limit_clamp_margin = 0.02;  // rad, hard clamp offset from the stop
  // Default infinite; JointImpedanceMode fills these from the URDF (Task 4).
  JointVec q_lower = JointVec::Constant(-std::numeric_limits<double>::infinity());
  JointVec q_upper = JointVec::Constant( std::numeric_limits<double>::infinity());
};

struct IkResult {
  double pos_err   = 0.0;   // m,   after the final iteration
  double rot_err   = 0.0;   // rad, after the final iteration
  int    iters     = 0;
  bool   converged = false;
};

// Bounded damped-least-squares Gauss-Newton IK, warm-started from the caller's
// current q. Redundancy is resolved by a null-space posture bias plus limit
// avoidance, so the solution is a deterministic function of (target, seed) rather
// than a free DOF. RT-safe: fixed-size scratch, no heap allocation after ctor.
class DiffIkSolver {
 public:
  DiffIkSolver(Dynamics& dyn, DiffIkParams p);
  IkResult solve(const Pose& target, JointVec& q);   // refines q IN PLACE
  void set_params(const DiffIkParams& p) noexcept { p_ = p; }
  const DiffIkParams& params() const noexcept { return p_; }

 private:
  Dynamics& dyn_;
  DiffIkParams p_;
  Jacobian6 J_;   // preallocated RT scratch
};

}  // namespace kinova
```

- [ ] **Step 2: Write the failing tests** `tests/diff_ik_test.cpp`

```cpp
#include <gtest/gtest.h>
#include <cmath>
#include "kinova_lowlevel/diff_ik.h"
using namespace kinova;

namespace {
JointVec sample_q() {
  JointVec q; q << 0.1, 0.3, -0.2, 0.8, 0.5, -0.4, 0.2; return q;
}
// Solver tuned for tests: converge fully rather than take one teleop-sized step.
DiffIkParams converging_params() {
  DiffIkParams p;
  p.max_iters = 200;
  p.posture_gain = 0.0;
  p.limit_gain = 0.0;
  p.max_pos_err = 1.0;
  p.max_rot_err = 1.0;
  p.max_joint_step = 0.2;
  return p;
}
}  // namespace

TEST(DiffIk, ConvergesToReachablePoseFromPerturbedSeed) {
  Dynamics dyn(URDF_PATH);
  const JointVec q_true = sample_q();
  const Pose target = dyn.fk(q_true);

  DiffIkSolver ik(dyn, converging_params());
  JointVec q = q_true;
  q[1] += 0.15; q[3] -= 0.20; q[5] += 0.10;      // perturbed warm start

  IkResult r = ik.solve(target, q);
  EXPECT_TRUE(r.converged);
  // The POSE must match. q itself need not — the arm is redundant.
  Pose reached = dyn.fk(q);
  EXPECT_LT((reached.p - target.p).norm(), 1e-3);
  Vector6 e = pose_error(target, reached);
  EXPECT_LT(e.tail<3>().norm(), 1e-2);
}

TEST(DiffIk, WarmStartAtSolutionIsNoOp) {
  Dynamics dyn(URDF_PATH);
  const JointVec q_true = sample_q();
  DiffIkSolver ik(dyn, converging_params());
  JointVec q = q_true;
  IkResult r = ik.solve(dyn.fk(q_true), q);
  EXPECT_TRUE(r.converged);
  EXPECT_EQ(r.iters, 0);                          // returned before doing any work
  EXPECT_NEAR((q - q_true).norm(), 0.0, 1e-12);
}

TEST(DiffIk, RespectsHardJointLimits) {
  Dynamics dyn(URDF_PATH);
  DiffIkParams p = converging_params();
  dyn.joint_limits(p.q_lower, p.q_upper);
  p.limit_clamp_margin = 0.02;
  DiffIkSolver ik(dyn, p);

  // Drive at a target far outside the workspace so the solve pushes hard on the
  // limited joints (2/4/6) for many iterations.
  Pose target = dyn.fk(sample_q());
  target.p += Eigen::Vector3d(2.0, 2.0, -2.0);
  JointVec q = sample_q();
  ik.solve(target, q);

  for (int i = 0; i < kNumJoints; ++i) {
    EXPECT_TRUE(std::isfinite(q[i])) << "joint " << i;
    if (std::isfinite(p.q_lower[i]))
      EXPECT_GE(q[i], p.q_lower[i] + p.limit_clamp_margin - 1e-9) << "joint " << i;
    if (std::isfinite(p.q_upper[i]))
      EXPECT_LE(q[i], p.q_upper[i] - p.limit_clamp_margin + 1e-9) << "joint " << i;
  }
}

TEST(DiffIk, UnreachableTargetStaysBoundedAndFinite) {
  Dynamics dyn(URDF_PATH);
  DiffIkParams p;                                  // production-ish: 4 iters, clamps on
  dyn.joint_limits(p.q_lower, p.q_upper);
  DiffIkSolver ik(dyn, p);
  Pose target = dyn.fk(sample_q());
  target.p += Eigen::Vector3d(10.0, -10.0, 10.0);  // wildly unreachable

  JointVec q = sample_q();
  const JointVec q0 = q;
  ik.solve(target, q);
  EXPECT_FALSE(r_is_nan(q));
  // Per-iteration joint clamp bounds total motion by max_iters * max_joint_step.
  EXPECT_LE((q - q0).lpNorm<Eigen::Infinity>(),
            p.max_iters * p.max_joint_step + 1e-9);
}

TEST(DiffIk, PostureBiasMovesConfigWithoutBreakingTheTask) {
  Dynamics dyn(URDF_PATH);
  const JointVec q_true = sample_q();
  const Pose target = dyn.fk(q_true);

  DiffIkParams p = converging_params();
  p.posture_gain = 0.3;
  p.q_rest = q_true;
  p.q_rest[2] += 0.5;                              // pull the redundant DOF
  DiffIkSolver ik(dyn, p);

  JointVec q = q_true;
  IkResult r = ik.solve(target, q);
  EXPECT_GT((q - q_true).norm(), 1e-3);            // the posture bias actually moved it
  EXPECT_LT(r.pos_err, 1e-3);                      // ...without breaking the task
  EXPECT_LT(r.rot_err, 1e-2);
}

TEST(DiffIk, PostureErrorWrapsForContinuousJoints) {
  // Joint 1 (index 0) is continuous. q_rest=+3.0 and q=-3.0 are 0.28 rad apart the
  // short way round, not 6.0. Without wrapping the bias would drive the long way.
  Dynamics dyn(URDF_PATH);
  DiffIkParams p = converging_params();
  dyn.joint_limits(p.q_lower, p.q_upper);
  p.max_iters = 1;
  p.posture_gain = 1.0;
  p.q_rest = sample_q();
  p.q_rest[0] = 3.0;
  DiffIkSolver ik(dyn, p);

  JointVec q = sample_q();
  q[0] = -3.0;
  ik.solve(dyn.fk(q), q);
  EXPECT_LT(q[0], -3.0);        // wrapped: moves NEGATIVE, toward -pi and around
}
```

Replace the `r_is_nan(q)` placeholder with `q.hasNaN()` (Eigen provides it) — write it as `EXPECT_FALSE(q.hasNaN());`.

- [ ] **Step 3: Run to verify it fails** — `./local_tools/build_on_abra.sh abra`, expect compile failure (no `diff_ik.h`).

- [ ] **Step 4: Implement** `src/diff_ik.cpp`

```cpp
#include "kinova_lowlevel/diff_ik.h"
#include <algorithm>
#include <cmath>
#include <Eigen/Cholesky>
namespace kinova {
namespace {

// Push away from a hard stop: 0 in the interior, ramping linearly to ±1 at the
// stop. Continuous joints (infinite limits) contribute nothing.
double limit_gradient(double q, double lo, double hi, double margin) {
  if (margin <= 0.0) return 0.0;
  if (std::isfinite(lo) && q < lo + margin) return (lo + margin - q) / margin;
  if (std::isfinite(hi) && q > hi - margin) return (hi - margin - q) / margin;
  return 0.0;
}

// Posture error, wrapped to [-pi, pi] for continuous joints. Without this, a rest
// angle of +3.0 against a measured -3.0 reads as a 6.0 rad error and the bias
// drives the joint the long way round instead of the 0.28 rad short way.
double posture_error(double q_rest, double q, bool continuous) {
  const double d = q_rest - q;
  return continuous ? std::remainder(d, 2.0 * M_PI) : d;
}

}  // namespace

DiffIkSolver::DiffIkSolver(Dynamics& dyn, DiffIkParams p) : dyn_(dyn), p_(p) {
  J_.setZero();
}

IkResult DiffIkSolver::solve(const Pose& target, JointVec& q) {
  IkResult r;
  for (int k = 0; k <= p_.max_iters; ++k) {
    Vector6 e = pose_error(target, dyn_.fk(q));
    r.pos_err = e.head<3>().norm();
    r.rot_err = e.tail<3>().norm();
    r.iters = k;
    r.converged = (r.pos_err < p_.pos_tol && r.rot_err < p_.rot_tol);
    if (r.converged || k == p_.max_iters) return r;

    if (r.pos_err > p_.max_pos_err) e.head<3>() *= p_.max_pos_err / r.pos_err;
    if (r.rot_err > p_.max_rot_err) e.tail<3>() *= p_.max_rot_err / r.rot_err;

    dyn_.jacobian(q, J_);
    Eigen::Matrix<double, 6, 6> A = J_ * J_.transpose();
    A.diagonal().array() += p_.damping * p_.damping;
    // Fixed-size LDLT: no heap. Same pattern as the null-space projector in
    // cartesian_impedance_mode.cpp. Explicit type, not `auto` — an Eigen
    // decomposition bound by `auto` can dangle on its operand.
    const Eigen::LDLT<Eigen::Matrix<double, 6, 6>> A_ldlt(A);

    JointVec dq = J_.transpose() * A_ldlt.solve(e);

    // Secondary objectives, projected into null(J) so they cannot disturb the
    // task. The projector uses the DAMPED inverse, so it is approximate near
    // singularities — deliberate: robustness beats exactness here.
    JointVec q0;
    for (int i = 0; i < kNumJoints; ++i) {
      const bool continuous =
          !std::isfinite(p_.q_lower[i]) && !std::isfinite(p_.q_upper[i]);
      q0[i] = p_.posture_gain * posture_error(p_.q_rest[i], q[i], continuous) +
              p_.limit_gain *
                  limit_gradient(q[i], p_.q_lower[i], p_.q_upper[i], p_.limit_margin);
    }
    const Eigen::Matrix<double, 6, kNumJoints> AinvJ = A_ldlt.solve(J_);
    dq += q0 - J_.transpose() * (AinvJ * q0);      // (I - Jt A^-1 J) q0

    for (int i = 0; i < kNumJoints; ++i) {
      q[i] += std::clamp(dq[i], -p_.max_joint_step, p_.max_joint_step);
      const double lo = p_.q_lower[i] + p_.limit_clamp_margin;
      const double hi = p_.q_upper[i] - p_.limit_clamp_margin;
      if (std::isfinite(lo) && q[i] < lo) q[i] = lo;
      if (std::isfinite(hi) && q[i] > hi) q[i] = hi;
    }
  }
  return r;
}

}  // namespace kinova
```

- [ ] **Step 5: Register in CMake** — add `src/diff_ik.cpp` to `KINOVA_LIB_SOURCES` and `tests/diff_ik_test.cpp` to the `unit_tests` target.

- [ ] **Step 6: Run to verify it passes** — `./local_tools/build_on_abra.sh abra`, expect PASS.

- [ ] **Step 7: Commit**

```bash
git add include/kinova_lowlevel/diff_ik.h src/diff_ik.cpp tests/diff_ik_test.cpp CMakeLists.txt
git commit -m "feat(ik): RT-safe damped-least-squares diff IK with posture and limit avoidance"
```

---

### Task 4: `JointImpedanceMode`

**Files:**
- Create: `include/kinova_lowlevel/joint_impedance_mode.h`, `src/joint_impedance_mode.cpp`
- Test: `tests/joint_impedance_mode_test.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `DiffIkSolver` / `DiffIkParams` / `IkResult` (Task 3), `PoseTargetSink` (Task 2), `Dynamics::joint_limits` (Task 1).
- Produces:
  - `struct JointImpedanceParams` — `Kq`, `Dq`, `torque_limit` (all `JointVec`), `max_tracking_error`, `max_ref_speed`, `gain_ramp_s` (all `double`), `ik` (`DiffIkParams`).
  - `class JointImpedanceMode : public ControlMode, public PoseTargetSink` — ctor `(Dynamics&, JointImpedanceParams = {})`, plus `set_gains(const JointImpedanceParams&) noexcept`, `set_target(const Pose&) noexcept override`, `JointVec reference() const noexcept`, `IkResult last_ik() const noexcept`.

- [ ] **Step 1: Write the header** `include/kinova_lowlevel/joint_impedance_mode.h`

```cpp
#pragma once
#include <atomic>
#include "kinova_lowlevel/control_mode.h"
#include "kinova_lowlevel/diff_ik.h"
#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/pose_target_sink.h"
namespace kinova {

struct JointImpedanceParams {
  JointVec Kq = (JointVec() << 100, 100, 100, 100, 40, 40, 40).finished();  // N·m/rad
  JointVec Dq = (JointVec() <<  12,  12,  12,  12,  5,  5,  5).finished();  // N·m·s/rad
  // Per-joint ceiling. The URDF gives joints 5-7 an effort limit of 9 N·m; the
  // single scalar CartesianImpedanceParams uses would overrun the wrist by 4x
  // under stiff joint gains.
  JointVec torque_limit = (JointVec() << 39, 39, 39, 39, 9, 9, 9).finished();
  // Spring leash: caps |q_ref - q| per joint so spring torque saturates at
  // Kq*leash while gravity compensation still passes through in full. The total
  // torque clamp cannot do this — it eats the gravity term under load and the arm
  // sags.
  double max_tracking_error = 0.35;   // rad
  double max_ref_speed      = 1.0;    // rad/s cap on reference motion
  double gain_ramp_s        = 0.5;    // fade the spring in over this window on entry
  DiffIkParams ik{};
};

// Joint-space impedance driven by in-loop IK:
//   q_d  <- DiffIk(target, warm start q_d)          (all 7 joints commanded)
//   tau   = g(q) + ramp * ( Kq∘clamp(q_d-q, ±leash) - Dq∘qd )
// Unlike CartesianImpedanceMode this leaves no uncommanded DOF: the redundant
// joint is resolved inside the IK by posture bias + limit avoidance, so the arm
// cannot drift into an arbitrary configuration.
//
// Tradeoff: end-effector stiffness becomes J^-T Kq J^-1 — configuration dependent
// and not diagonal in the task frame. For teleop, predictable posture is worth
// more than an exactly shaped task-space ellipsoid.
class JointImpedanceMode : public ControlMode, public PoseTargetSink {
 public:
  JointImpedanceMode(Dynamics& dyn, JointImpedanceParams p = {});
  ActuatorModes required_modes() const override;
  void on_enter(const JointFeedback& fb) override;
  void compute(const JointFeedback& fb, double dt_s, JointCommand& out) override;
  void on_exit() override {}

  // Non-RT setters (call from one supervisor thread).
  void set_gains(const JointImpedanceParams& p) noexcept;
  void set_target(const Pose& x_d) noexcept override;

  // RT-thread-owned state, for tests and post-stop inspection. Not synchronized:
  // do not call these from another thread while the RT loop is running.
  JointVec reference() const noexcept { return q_d_; }
  IkResult last_ik() const noexcept { return last_ik_; }

 private:
  JointImpedanceParams params() const noexcept;   // RT-safe value snapshot
  // Fills any non-finite IK limit with the URDF value cached at construction, so
  // a caller-supplied tighter software limit survives but the default does not
  // leave the solver unbounded.
  void seed_limits(JointImpedanceParams& p) const noexcept;

  Dynamics& dyn_;
  DiffIkSolver ik_;
  JointVec q_lower_urdf_ = JointVec::Zero();   // cached in ctor: set_gains must not
  JointVec q_upper_urdf_ = JointVec::Zero();   // touch Dynamics off the RT thread

  JointImpedanceParams gains_[2];
  std::atomic<int> gains_active_{0};

  Pose entry_pose_;
  Pose ext_target_[2];
  std::atomic<int> ext_active_{0};
  std::atomic<bool> has_ext_target_{false};

  JointVec q_d_ = JointVec::Zero();    // integrated reference configuration
  IkResult last_ik_{};
  double ramp_elapsed_ = 0.0;

  JointVec g_ = JointVec::Zero();      // preallocated RT scratch
  JointVec tau_ = JointVec::Zero();
};

}  // namespace kinova
```

- [ ] **Step 2: Write the failing tests** `tests/joint_impedance_mode_test.cpp`

```cpp
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include "kinova_lowlevel/joint_impedance_mode.h"
using namespace kinova;

namespace {
JointVec sample_q() {
  JointVec q; q << 0.1, 0.3, -0.2, 0.8, 0.5, -0.4, 0.2; return q;
}
// Isolate the impedance law: no ramp, no IK motion, no reference rate limit.
JointImpedanceParams static_params() {
  JointImpedanceParams p;
  p.gain_ramp_s = 0.0;
  p.ik.max_iters = 0;          // freeze q_d at its seeded value
  p.max_ref_speed = 1e9;
  return p;
}
}  // namespace

TEST(JointImpedance, RequiresTorqueAndPassesThroughPosition) {
  Dynamics dyn(URDF_PATH);
  JointImpedanceMode m(dyn, static_params());
  JointFeedback fb; fb.q = sample_q(); fb.qd.setZero();
  for (auto x : m.required_modes()) EXPECT_EQ(x, ActuatorMode::kTorque);
  JointCommand c; m.on_enter(fb); m.compute(fb, 0.001, c);
  EXPECT_EQ(c.mode, ActuatorMode::kTorque);
  EXPECT_NEAR((c.position - fb.q).norm(), 0.0, 1e-12);
}

TEST(JointImpedance, AtReferenceZeroVelGivesGravityOnly) {
  Dynamics dyn(URDF_PATH);
  JointImpedanceMode m(dyn, static_params());
  JointFeedback fb; fb.q = sample_q(); fb.qd.setZero();
  m.on_enter(fb);                       // q_d := fb.q -> zero spring error
  JointCommand c; m.compute(fb, 0.001, c);
  JointVec g; dyn.gravity(fb.q, g);
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(c.torque[i], g[i], 1e-9);
}

TEST(JointImpedance, ReferenceSeededAtMeasuredConfigOnEnter) {
  Dynamics dyn(URDF_PATH);
  JointImpedanceMode m(dyn, static_params());
  JointFeedback fb; fb.q = sample_q(); fb.qd.setZero();
  m.on_enter(fb);
  EXPECT_NEAR((m.reference() - fb.q).norm(), 0.0, 1e-12);
}

TEST(JointImpedance, MatchesIndependentlyComputedLaw) {
  Dynamics dyn(URDF_PATH);
  JointImpedanceParams p = static_params();
  JointImpedanceMode m(dyn, p);
  JointFeedback enter; enter.q = sample_q(); enter.qd.setZero();
  m.on_enter(enter);                                  // q_d := sample_q()

  JointFeedback fb; fb.q = sample_q();
  fb.q[1] -= 0.10; fb.q[4] += 0.08;                   // displace from the reference
  fb.qd.setConstant(0.05);
  JointCommand c; m.compute(fb, 0.001, c);

  JointVec g; dyn.gravity(fb.q, g);
  JointVec expected;
  for (int i = 0; i < kNumJoints; ++i) {
    const double e = std::clamp(enter.q[i] - fb.q[i],
                                -p.max_tracking_error, p.max_tracking_error);
    expected[i] = g[i] + p.Kq[i] * e - p.Dq[i] * fb.qd[i];
    expected[i] = std::clamp(expected[i], -p.torque_limit[i], p.torque_limit[i]);
  }
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(c.torque[i], expected[i], 1e-9);
}

TEST(JointImpedance, PerJointTorqueClampHonorsWristLimit) {
  Dynamics dyn(URDF_PATH);
  JointImpedanceParams p = static_params();
  p.Kq.setConstant(5000.0);
  p.max_tracking_error = 10.0;                        // let the spring run away
  JointImpedanceMode m(dyn, p);
  JointFeedback enter; enter.q = sample_q(); enter.qd.setZero();
  m.on_enter(enter);
  JointFeedback fb = enter; fb.q.array() += 0.5;      // huge error on every joint
  JointCommand c; m.compute(fb, 0.001, c);
  for (int i = 0; i < kNumJoints; ++i)
    EXPECT_LE(std::abs(c.torque[i]), p.torque_limit[i] + 1e-9) << "joint " << i;
  EXPECT_LE(std::abs(c.torque[6]), 9.0 + 1e-9);       // wrist limit, not 39
}

TEST(JointImpedance, LeashCapsSpringButNotGravity) {
  Dynamics dyn(URDF_PATH);
  JointImpedanceParams p = static_params();
  p.max_tracking_error = 0.05;
  p.torque_limit.setConstant(1e6);                    // isolate the leash
  JointImpedanceMode m(dyn, p);
  JointFeedback enter; enter.q = sample_q(); enter.qd.setZero();
  m.on_enter(enter);
  JointFeedback fb = enter; fb.q[0] -= 1.0;           // way past the leash
  JointCommand c; m.compute(fb, 0.001, c);
  JointVec g; dyn.gravity(fb.q, g);
  // Spring saturates at Kq*leash; gravity is untouched (never scaled or clamped).
  EXPECT_NEAR(c.torque[0], g[0] + p.Kq[0] * p.max_tracking_error, 1e-9);
}

TEST(JointImpedance, RampScalesSpringButGravityStaysFull) {
  Dynamics dyn(URDF_PATH);
  JointImpedanceParams p = static_params();
  p.gain_ramp_s = 1.0;
  JointImpedanceMode m(dyn, p);
  JointFeedback enter; enter.q = sample_q(); enter.qd.setZero();
  m.on_enter(enter);
  JointFeedback fb = enter; fb.q[1] -= 0.10;
  JointVec g; dyn.gravity(fb.q, g);

  JointCommand c0; m.compute(fb, 0.25, c0);           // elapsed 0 -> ramp 0
  for (int i = 0; i < kNumJoints; ++i) EXPECT_NEAR(c0.torque[i], g[i], 1e-9);

  JointCommand c1; m.compute(fb, 0.25, c1);           // elapsed 0.25 -> ramp 0.25
  EXPECT_NEAR(c1.torque[1], g[1] + 0.25 * p.Kq[1] * 0.10, 1e-7);
}

TEST(JointImpedance, ReferenceSpeedLimitBoundsMotionOnTeleportedTarget) {
  Dynamics dyn(URDF_PATH);
  JointImpedanceParams p;
  p.gain_ramp_s = 0.0;
  p.max_ref_speed = 0.5;                              // rad/s
  JointImpedanceMode m(dyn, p);
  JointFeedback fb; fb.q = sample_q(); fb.qd.setZero();
  m.on_enter(fb);
  Pose far = dyn.fk(fb.q); far.p += Eigen::Vector3d(0.4, 0.3, -0.2);
  m.set_target(far);

  const JointVec before = m.reference();
  JointCommand c; m.compute(fb, 0.001, c);
  const double moved = (m.reference() - before).lpNorm<Eigen::Infinity>();
  EXPECT_GT(moved, 0.0);                              // it did move toward the target
  EXPECT_LE(moved, p.max_ref_speed * 0.001 + 1e-12);  // ...but no faster than allowed
}

TEST(JointImpedance, IkLimitsSeededFromUrdf) {
  Dynamics dyn(URDF_PATH);
  JointImpedanceMode m(dyn, JointImpedanceParams{});
  JointVec lo, hi; dyn.joint_limits(lo, hi);
  // The mode must not leave the solver unbounded just because the default params
  // carry infinite limits.
  EXPECT_NEAR(m.last_ik().pos_err, 0.0, 1e-12);       // fresh IkResult
  JointFeedback fb; fb.q = sample_q(); fb.qd.setZero();
  m.on_enter(fb);
  Pose far = dyn.fk(fb.q); far.p += Eigen::Vector3d(3.0, 3.0, 3.0);
  m.set_target(far);
  for (int k = 0; k < 5000; ++k) { JointCommand c; m.compute(fb, 0.001, c); }
  const JointVec q_ref = m.reference();
  for (int i = 0; i < kNumJoints; ++i) {
    if (std::isfinite(lo[i])) EXPECT_GE(q_ref[i], lo[i] - 1e-6) << "joint " << i;
    if (std::isfinite(hi[i])) EXPECT_LE(q_ref[i], hi[i] + 1e-6) << "joint " << i;
  }
}
```

- [ ] **Step 3: Run to verify it fails** — `./local_tools/build_on_abra.sh abra`, expect compile failure.

- [ ] **Step 4: Implement** `src/joint_impedance_mode.cpp`

```cpp
#include "kinova_lowlevel/joint_impedance_mode.h"
#include <algorithm>
#include <cmath>
namespace kinova {

JointImpedanceMode::JointImpedanceMode(Dynamics& dyn, JointImpedanceParams p)
    : dyn_(dyn), ik_(dyn, p.ik) {
  // Cache the URDF limits once. set_gains runs on a non-RT thread and must never
  // touch Dynamics — it is not thread-safe against the RT loop's fk/jacobian.
  dyn.joint_limits(q_lower_urdf_, q_upper_urdf_);
  seed_limits(p);
  ik_.set_params(p.ik);
  gains_[0] = p;
  gains_[1] = p;
}

void JointImpedanceMode::seed_limits(JointImpedanceParams& p) const noexcept {
  for (int i = 0; i < kNumJoints; ++i) {
    if (!std::isfinite(p.ik.q_lower[i])) p.ik.q_lower[i] = q_lower_urdf_[i];
    if (!std::isfinite(p.ik.q_upper[i])) p.ik.q_upper[i] = q_upper_urdf_[i];
  }
}

ActuatorModes JointImpedanceMode::required_modes() const {
  ActuatorModes modes; modes.fill(ActuatorMode::kTorque); return modes;
}

JointImpedanceParams JointImpedanceMode::params() const noexcept {
  return gains_[gains_active_.load(std::memory_order_acquire)];
}

void JointImpedanceMode::set_gains(const JointImpedanceParams& p) noexcept {
  const int next = 1 - gains_active_.load(std::memory_order_relaxed);
  gains_[next] = p;
  seed_limits(gains_[next]);
  gains_active_.store(next, std::memory_order_release);
}

void JointImpedanceMode::set_target(const Pose& x_d) noexcept {
  const int next = 1 - ext_active_.load(std::memory_order_relaxed);
  ext_target_[next] = x_d;
  ext_active_.store(next, std::memory_order_release);
  has_ext_target_.store(true, std::memory_order_release);
}

void JointImpedanceMode::on_enter(const JointFeedback& fb) {
  entry_pose_ = dyn_.fk(fb.q);
  // The reference starts exactly at the measured configuration, then integrates
  // OPEN-LOOP. Re-seeding from fb.q every cycle would collapse the spring to zero
  // error and degenerate this into rigid tracking, losing all compliance.
  q_d_ = fb.q;
  has_ext_target_.store(false, std::memory_order_release);
  ramp_elapsed_ = 0.0;
  last_ik_ = IkResult{};
}

void JointImpedanceMode::compute(const JointFeedback& fb, double dt_s,
                                 JointCommand& out) {
  const JointImpedanceParams p = params();   // one snapshot for the whole cycle
  const Pose target = has_ext_target_.load(std::memory_order_acquire)
                          ? ext_target_[ext_active_.load(std::memory_order_acquire)]
                          : entry_pose_;

  ik_.set_params(p.ik);                      // fixed-size copy, no alloc
  const JointVec q_prev = q_d_;
  last_ik_ = ik_.solve(target, q_d_);        // warm-started from last cycle

  // Bound reference speed so a teleported target ramps in instead of slamming.
  const double max_step = p.max_ref_speed * dt_s;
  for (int i = 0; i < kNumJoints; ++i)
    q_d_[i] = std::clamp(q_d_[i], q_prev[i] - max_step, q_prev[i] + max_step);

  dyn_.gravity(fb.q, g_);

  // Leash the SPRING only. Gravity is never scaled or leashed, so the arm cannot
  // sag when the spring saturates. Applied to the torque, not to q_d_ — pushing
  // the arm away must not corrupt the IK reference.
  for (int i = 0; i < kNumJoints; ++i) {
    const double e = std::clamp(q_d_[i] - fb.q[i], -p.max_tracking_error,
                                p.max_tracking_error);
    tau_[i] = p.Kq[i] * e - p.Dq[i] * fb.qd[i];
  }

  const double ramp = (p.gain_ramp_s <= 0.0)
                          ? 1.0
                          : std::min(1.0, ramp_elapsed_ / p.gain_ramp_s);
  tau_ = g_ + ramp * tau_;
  ramp_elapsed_ += dt_s;

  for (int i = 0; i < kNumJoints; ++i)
    tau_[i] = std::clamp(tau_[i], -p.torque_limit[i], p.torque_limit[i]);

  out.mode = ActuatorMode::kTorque;
  out.torque = tau_;
  out.position = fb.q;                       // passthrough for following-error hold
}

}  // namespace kinova
```

- [ ] **Step 5: Register in CMake** — add `src/joint_impedance_mode.cpp` to `KINOVA_LIB_SOURCES`, `tests/joint_impedance_mode_test.cpp` to `unit_tests`.

- [ ] **Step 6: Run to verify it passes** — `./local_tools/build_on_abra.sh abra`.

- [ ] **Step 7: Commit**

```bash
git add include/kinova_lowlevel/joint_impedance_mode.h src/joint_impedance_mode.cpp \
        tests/joint_impedance_mode_test.cpp CMakeLists.txt
git commit -m "feat(impedance): joint-space impedance mode with in-loop IK reference"
```

---

### Task 5: RT-safety coverage

**Files:**
- Modify: `tests/rt_safety_test.cpp`

**Interfaces:**
- Consumes: `JointImpedanceMode` (Task 4).
- Produces: nothing consumed downstream.

- [ ] **Step 1: Add the test** — copy the structure of the existing `RtSafety.ImpedanceModeNoMajorFaultsSteadyState` verbatim, substituting the mode. This is the check that proves the in-loop IK does not allocate.

```cpp
TEST(RtSafety, JointImpedanceModeNoMajorFaultsSteadyState) {
  JointFeedback init; init.q.setZero();
  SimTransport t(init);
  Dynamics dyn(URDF_PATH);
  JointImpedanceMode mode(dyn);                  // defaults: IK runs every cycle
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

Add `#include "kinova_lowlevel/joint_impedance_mode.h"` at the top.

- [ ] **Step 2: Run** — `./local_tools/build_on_abra.sh abra`, expect PASS.

- [ ] **Step 3: Commit**

```bash
git add tests/rt_safety_test.cpp
git commit -m "test(rt): assert JointImpedanceMode allocates nothing in steady state"
```

---

### Task 6: Teleop server `--joint-impedance`

**Files:**
- Modify: `apps/teleop_socket_server.cpp`
- Modify: `README.md`, `docs/guide/control-modes.md`

**Interfaces:**
- Consumes: `JointImpedanceMode` (Task 4), `PoseTargetSink` (Task 2).
- Produces: the `--joint-impedance` CLI surface.

- [ ] **Step 1: Add the flags.** Alongside `CartesianImpedanceParams gains;` add `JointImpedanceParams jgains;` and `bool joint_mode = false;`. Add a helper that parses either a scalar or 7 comma-separated values into a `JointVec` (the existing `--ns-qrest` parser handles only the 7-value form; generalize it into a local `parse_joint_vec(const std::string&, JointVec&)` returning `bool`, and reuse it for `--ns-qrest`, `--jkp`, `--jkd`, `--ik-qrest`).

New arms in the parse loop:

```cpp
else if (a == "--joint-impedance") joint_mode = true;
else if (a == "--jkp") { if (!parse_joint_vec(next("--jkp"), jgains.Kq)) fail("--jkp"); }
else if (a == "--jkd") { if (!parse_joint_vec(next("--jkd"), jgains.Dq)) fail("--jkd"); }
else if (a == "--leash") jgains.max_tracking_error = std::stod(next("--leash"));
else if (a == "--ref-speed") jgains.max_ref_speed = std::stod(next("--ref-speed"));
else if (a == "--ik-iters") jgains.ik.max_iters = std::stoi(next("--ik-iters"));
else if (a == "--ik-posture-gain") jgains.ik.posture_gain = std::stod(next("--ik-posture-gain"));
else if (a == "--ik-qrest") { if (!parse_joint_vec(next("--ik-qrest"), jgains.ik.q_rest)) fail("--ik-qrest"); }
```

- [ ] **Step 2: Construct the selected mode.** Replace the single `CartesianImpedanceMode mode(dyn, gains);` with both modes constructed conditionally via `std::unique_ptr`, plus the two pointers the rest of the app uses:

```cpp
  // Both modes implement PoseTargetSink, so the rx thread does not care which is
  // live. Only the selected one is constructed; the other stays null.
  std::unique_ptr<CartesianImpedanceMode> cart_mode;
  std::unique_ptr<JointImpedanceMode> joint_mode_impl;
  ControlMode* mode = nullptr;
  PoseTargetSink* sink = nullptr;
  if (joint_mode) {
    joint_mode_impl = std::make_unique<JointImpedanceMode>(dyn, jgains);
    mode = joint_mode_impl.get();
    sink = joint_mode_impl.get();
  } else {
    cart_mode = std::make_unique<CartesianImpedanceMode>(dyn, gains);
    mode = cart_mode.get();
    sink = cart_mode.get();
  }
  std::cout << "[teleop-srv] control mode: "
            << (joint_mode ? "joint-space impedance (IK in loop)"
                           : "cartesian impedance") << "\n";
```

Replace every `mode.set_target(...)` with `sink->set_target(...)` and `ex.request_mode(&mode)` with `ex.request_mode(mode)`.

- [ ] **Step 3: Handle SET_GAINS in joint mode.** In the `kSetGains` case, branch before building `CartesianImpedanceParams`:

```cpp
          if (joint_mode) {
            // The Cartesian Kx/Dx/null-space fields have no meaning here. Honor
            // what does transfer and say so ONCE — silently dropping operator
            // gains is an hour of confused hardware debugging.
            static std::once_flag warned;
            std::call_once(warned, [] {
              std::cerr << "[teleop-srv] SET_GAINS: joint-space mode ignores "
                           "Kx/Dx/nullspace_kp/nullspace_kd/pinv_damping/"
                           "nullspace_on; set joint gains with --jkp/--jkd. "
                           "Honoring torque_limit and gain_ramp_s.\n";
            });
            JointImpedanceParams jp = jgains;
            jp.torque_limit.setConstant(pkt.torque_limit);
            jp.gain_ramp_s = pkt.gain_ramp_s;
            joint_mode_impl->set_gains(jp);
            break;
          }
```

Add `#include <mutex>` (already present) and `#include <memory>` (already present).

- [ ] **Step 4: Update the file header comment** — it currently says the server bridges "the CartesianImpedanceMode seam". Say it bridges either mode and name the flag.

- [ ] **Step 5: Document** — add a `--joint-impedance` section to `docs/guide/control-modes.md` covering the control law, the redundancy-resolution rationale, the `J^-T Kq J^-1` stiffness tradeoff, and the tuning knobs. Add the invocation to `README.md` next to the existing teleop example.

- [ ] **Step 6: Build and run the whole suite** — `./local_tools/build_on_abra.sh abra`. Then sim smoke test on abra:

```bash
ssh abra 'cd ~/kinova-gen3-driver/build && timeout 5 ./teleop_socket_server --sim --joint-impedance --urdf ../models/gen3_7dof_2f85.urdf --port 9099; echo exit=$?'
```

Expect the mode banner, `listening; RT loop running`, and exit 124 (timeout kill), not a crash.

- [ ] **Step 7: Commit**

```bash
git add apps/teleop_socket_server.cpp README.md docs/guide/control-modes.md
git commit -m "feat(teleop): --joint-impedance selects joint-space mode; no protocol change"
```

---

### Task 7: Deploy to abra

- [ ] **Step 1:** `./local_tools/build_on_abra.sh abra` — full build + `ctest`, all green.
- [ ] **Step 2:** Rebuild the KORTEX (real-robot) configuration, since the demo runs against the arm:

```bash
ssh abra 'cd ~/kinova-gen3-driver/build_kortex && cmake --build . -j 2>&1 | tail -20'
```

If `build_kortex` is not configured for KORTEX, report that rather than assuming.
- [ ] **Step 3:** Report the exact command to run the demo, and the tuning knobs to reach for first on hardware.

## Self-Review

**Spec coverage:** Task 1 → spec §2. Task 2 → §4. Task 3 → §1. Task 4 → §3. Task 5 + tests in 3/4 → §6. Task 6 → §5. All spec sections have a task.

**Placeholder scan:** One found and fixed inline — `r_is_nan(q)` in Task 3's test is called out in the step text and replaced with `q.hasNaN()`.

**Type consistency:** `DiffIkParams`/`IkResult`/`DiffIkSolver::solve(const Pose&, JointVec&)` are used identically in Tasks 3 and 4. `JointImpedanceParams::torque_limit` is a `JointVec` everywhere (Task 4 header, tests, and the Task 6 `setConstant` call). `set_target` is `noexcept` on the base and both overrides. `joint_limits(JointVec&, JointVec&) const` matches between Tasks 1, 3 and 4.
