# Tilted Arm Mount (driver) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development
> (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let the driver build its dynamics from the rendered, mount-carrying arm model that
kinova-gen3-ros2 publishes, and stop any standalone app from silently using a level model.

**Architecture:** `Dynamics` gains a constructor from URDF XML and reduces any model it loads
to the joints on the chain from the root to the EE frame (everything else locked at q = 0),
so the 13-DOF articulated description is accepted and gives the same dynamics as today's
frozen 7-DOF file. No dynamics maths changes: Pinocchio applies gravity in the URDF root, so
a root of `arm_mount` puts gravity, FK, the Jacobian and everything downstream in `arm_mount`.
New tests pin that property. The standalone apps lose their default URDF.

**Tech Stack:** C++17, Pinocchio 3 (cmeel pip prefix), Eigen, GoogleTest, CMake.

**Spec:** `rammp-deployments: docs/superpowers/specs/2026-10-15-tilted-arm-mount-design.md`
(section "kinova-gen3-driver").

## Global Constraints

- Builds and tests only on Linux/aarch64 (the Jetson). Build/test command (CLAUDE.md):
  `cmake -S . -B build -DCMAKE_PREFIX_PATH=/usr/local/lib/python3.10/dist-packages/cmeel.prefix && cmake --build build -j && ctest --test-dir build --output-on-failure`.
- Where to run: `ssh abra` (dojo's Jetson; existing checkout at `~/atdev/kinova-gen3-driver`)
  or `ssh snorlax` (flatbot's). Both arms are powered off, which is fine: every step here is
  sim-only (`--sim`, SimTransport). Sync this worktree with
  `rsync -a --delete --exclude build ./ abra:~/atdev/kinova-gen3-driver-mount/` and build
  there; do not push the branch without asking.
- All tests live in the single `unit_tests` gtest binary; filter with
  `./build/unit_tests --gtest_filter='Dynamics*'`.
- The `nv == kNumJoints (7)` guard stays and applies after reduction.
- Locking is at `q = 0` (Pinocchio `neutral`), which equals the URDF's `type="fixed"` origin.
- The library's default `ee_frame` (`gen3_end_effector_link`) stays; a missing frame already
  throws. Apps get `--ee-frame`, default `arm_end_effector_link`.
- Apps: `--urdf` is required, no default. Usage error exit code 2 (the apps' existing
  convention for argument errors).
- Names the ROS 2 side relies on (do not change): `struct kinova::UrdfXml { std::string xml; };`,
  `Dynamics(const UrdfXml&, const std::string& ee_frame)`,
  `std::vector<std::string> Dynamics::joint_names() const`.
- Out of scope: IMU, `SetGravityVector`, ROS code, any change to gravity/FK/Jacobian maths.

## Review Focus

1. **EE frame on the gripper chain.** A frame beyond a gripper joint keeps that joint, so the
   reduced model has nv ≠ 7: it must throw the existing nv message, not load. (Task 1 test.)
2. **Malformed or empty XML** (an empty `/arm/robot_description`): must throw, not produce an
   empty model. (Task 1 test.)
3. **Continuous joints after reduction.** The arm has continuous joints packed as (cos, sin);
   reduction must keep `nq > nv` and the existing packing must still work. (Existing
   `ContinuousJointsInflateNq`, re-run on an XML-built model in Task 1.)
4. **A tilt that the model ignores.** If Pinocchio treated the root specially, a tilted root
   would silently give level gravity. (Task 2 tests.)
5. **An app run with no `--urdf`.** Must exit 2 with a message, before connecting to the arm.
   (Task 3 manual check.)

---

### Task 1: `Dynamics` from XML, reduced to the arm chain

**Files:**
- Modify: `include/kinova_lowlevel/dynamics.h`
- Modify: `src/dynamics.cpp`
- Test: `tests/dynamics_test.cpp`

**Interfaces:**
- Produces: `struct kinova::UrdfXml { std::string xml; };`;
  `Dynamics(const UrdfXml& urdf, const std::string& ee_frame = "gen3_end_effector_link")`;
  `std::vector<std::string> Dynamics::joint_names() const` (the 7 arm joints, chain order).
  The existing path constructor now also reduces.

- [ ] **Step 1: Write the failing tests** — append to `tests/dynamics_test.cpp`:

```cpp
#include <fstream>
#include <sstream>
#include <vector>

namespace {
std::string read_file(const std::string& path) {
  std::ifstream f(path);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}
void replace_once(std::string& s, const std::string& from, const std::string& to) {
  const auto pos = s.find(from);
  ASSERT_NE(pos, std::string::npos) << "fixture text not found: " << from;
  s.replace(pos, from.size(), to);
}
// The fixture with the gripper's left knuckle articulated (revolute), the way the
// rendered 13-DOF description has it. Same origin, so at q=0 it is the frozen model.
std::string articulated_fixture() {
  std::string xml = read_file(URDF_PATH);
  replace_once(xml, "<joint name=\"gen3_robotiq_85_left_knuckle_joint\" type=\"fixed\">",
               "<joint name=\"gen3_robotiq_85_left_knuckle_joint\" type=\"revolute\">"
               "<axis xyz=\"0 -1 0\"/>"
               "<limit lower=\"0\" upper=\"0.8\" effort=\"50\" velocity=\"2\"/>");
  return xml;
}
const std::vector<JointVec> kPoses = [] {
  std::vector<JointVec> v(3, JointVec::Zero());
  v[1] << 0.3, 0.6, -1.0, 1.2, 0.4, -0.7, 1.1;
  v[2] << -1.4, -0.9, 2.5, -2.0, 1.6, 0.9, -2.2;
  return v;
}();
}  // namespace

TEST(DynamicsXml, MatchesPathConstructor) {
  Dynamics from_path(URDF_PATH);
  Dynamics from_xml(UrdfXml{read_file(URDF_PATH)});
  for (const auto& q : kPoses) {
    JointVec a, b;
    from_path.gravity(q, a);
    from_xml.gravity(q, b);
    EXPECT_LT((a - b).cwiseAbs().maxCoeff(), 1e-12);
    EXPECT_LT((from_path.fk(q).p - from_xml.fk(q).p).norm(), 1e-12);
  }
  EXPECT_GT(from_xml.nq(), from_xml.nv());  // continuous packing survives the XML path
}

TEST(DynamicsXml, EmptyOrMalformedXmlThrows) {
  EXPECT_ANY_THROW(Dynamics(UrdfXml{""}));
  EXPECT_ANY_THROW(Dynamics(UrdfXml{"<robot name=\"x\"><link"}));
}

TEST(DynamicsReduce, ArticulatedGripperIsLockedToTheFrozenModel) {
  Dynamics frozen(URDF_PATH);
  Dynamics reduced(UrdfXml{articulated_fixture()});
  EXPECT_EQ(reduced.nv(), kNumJoints);
  for (const auto& q : kPoses) {
    JointVec a, b;
    frozen.gravity(q, a);
    reduced.gravity(q, b);
    EXPECT_LT((a - b).cwiseAbs().maxCoeff(), 1e-12);
    Jacobian6 Ja, Jb;
    frozen.jacobian(q, Ja);
    reduced.jacobian(q, Jb);
    EXPECT_LT((Ja - Jb).cwiseAbs().maxCoeff(), 1e-12);
    EXPECT_LT((frozen.fk(q).p - reduced.fk(q).p).norm(), 1e-12);
  }
}

TEST(DynamicsReduce, EeFrameBeyondAGripperJointKeepsItAndThrows) {
  // A frame past the articulated knuckle keeps that joint on the chain: nv = 8.
  EXPECT_THROW(Dynamics(UrdfXml{articulated_fixture()}, "gen3_robotiq_85_left_finger_link"),
               std::runtime_error);
}

TEST(DynamicsReduce, JointNamesAreTheArmChainInOrder) {
  Dynamics dyn(UrdfXml{articulated_fixture()});
  const std::vector<std::string> want = {"gen3_joint_1", "gen3_joint_2", "gen3_joint_3",
                                         "gen3_joint_4", "gen3_joint_5", "gen3_joint_6",
                                         "gen3_joint_7"};
  EXPECT_EQ(dyn.joint_names(), want);
}
```

- [ ] **Step 2: Run to verify they fail**

Run: `cmake --build build -j 2>&1 | tail -5`
Expected: compile error, `'UrdfXml' was not declared in this scope` (and `joint_names`).

- [ ] **Step 3: Implement** — `include/kinova_lowlevel/dynamics.h`: add `#include <vector>`,
and before `class Dynamics`:

```cpp
// A URDF given as its XML text (e.g. read from /arm/robot_description), as
// opposed to a path. A distinct type so the two constructors cannot be confused.
struct UrdfXml {
  std::string xml;
};
```

In the class, below the path constructor:

```cpp
  // Same model, from XML text. Both constructors REDUCE the model to the joints on
  // the chain from the root to ee_frame, locking every other joint at q = 0 -- the
  // same as type="fixed" at its URDF origin -- so an articulated gripper's mass
  // stays lumped at the wrist. nv must be 7 after reduction or construction throws.
  explicit Dynamics(const UrdfXml& urdf,
                    const std::string& ee_frame = "gen3_end_effector_link");
  // The arm's joint names in chain order (root to tip), as named in the URDF.
  std::vector<std::string> joint_names() const;
```

`src/dynamics.cpp`: add `#include <pinocchio/algorithm/model.hpp>` and `#include <vector>`.
Replace `struct Dynamics::Impl`'s constructor and add a reduction helper:

```cpp
struct Dynamics::Impl {
  pinocchio::Model model;
  pinocchio::Data data;
  Eigen::VectorXd qcfg;
  pinocchio::FrameIndex frame_id = 0;
  // `full` is the model as parsed; keep only the chain root -> ee_frame.
  Impl(const pinocchio::Model& full, const std::string& ee_frame) {
    // Footgun guard: a typo'd/missing EE frame must fail loudly at startup, never
    // silently control the wrong point.
    if (!full.existFrame(ee_frame))
      throw std::runtime_error("Dynamics: EE frame '" + ee_frame + "' not in URDF");
    const auto& chain = full.supports[full.frames[full.getFrameId(ee_frame)].parentJoint];
    std::vector<pinocchio::JointIndex> lock;
    for (pinocchio::JointIndex j = 1; j < pinocchio::JointIndex(full.njoints); ++j)
      if (std::find(chain.begin(), chain.end(), j) == chain.end()) lock.push_back(j);
    model = lock.empty() ? full
                         : pinocchio::buildReducedModel(full, lock, pinocchio::neutral(full));
    data = pinocchio::Data(model);
    qcfg = pinocchio::neutral(model);
    frame_id = model.getFrameId(ee_frame);
  }
  // pack() unchanged
```

(add `#include <algorithm>`). Replace the existing constructor and add the new one plus
`joint_names`:

```cpp
namespace {
pinocchio::Model parse_path(const std::string& path) {
  pinocchio::Model m;
  pinocchio::urdf::buildModel(path, m);
  return m;
}
pinocchio::Model parse_xml(const std::string& xml) {
  if (xml.empty()) throw std::runtime_error("Dynamics: URDF XML is empty");
  pinocchio::Model m;
  pinocchio::urdf::buildModelFromXML(xml, m);
  return m;
}
// Guard against a wrong/mismatched URDF silently corrupting the fixed-size
// JointVec. Hard throw (not assert) so it fires in Release too. See no-silent-footgun.
void check_nv(const pinocchio::Model& m) {
  if (m.nv != kNumJoints)
    throw std::runtime_error("Dynamics: URDF nv=" + std::to_string(m.nv) +
                             " != kNumJoints=" + std::to_string(kNumJoints) +
                             " after reducing to the EE chain (wrong URDF or EE frame)");
}
}  // namespace

Dynamics::Dynamics(const std::string& urdf_path, const std::string& ee_frame)
    : impl_(std::make_unique<Impl>(parse_path(urdf_path), ee_frame)) {
  check_nv(impl_->model);
}
Dynamics::Dynamics(const UrdfXml& urdf, const std::string& ee_frame)
    : impl_(std::make_unique<Impl>(parse_xml(urdf.xml), ee_frame)) {
  check_nv(impl_->model);
}
std::vector<std::string> Dynamics::joint_names() const {
  // names[0] is "universe"; the reduced model holds exactly the arm chain.
  return {impl_->model.names.begin() + 1, impl_->model.names.end()};
}
```

If the installed Pinocchio is 2.x (`python3 -c "import pinocchio; print(pinocchio.__version__)"`),
`Frame::parentJoint` is spelled `parent`; use whichever compiles, nothing else differs.

- [ ] **Step 4: Run the tests**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='Dynamics*'`
Expected: all `Dynamics*` tests PASS, including the pre-existing ones
(`UnknownFrameThrows` still throws: the frame check moved into `Impl`).

- [ ] **Step 5: Run the whole suite**

Run: `ctest --test-dir build --output-on-failure`
Expected: `100% tests passed` (every mode test builds `Dynamics(URDF_PATH)`; the frozen
fixture locks nothing, so their behaviour is unchanged).

- [ ] **Step 6: Commit**

```bash
git add include/kinova_lowlevel/dynamics.h src/dynamics.cpp tests/dynamics_test.cpp
git commit -m "feat(dynamics): build from URDF XML and reduce to the EE chain

Lets the driver load the 13-DOF articulated arm description that
kinova-gen3-ros2 publishes on /arm/robot_description: joints off the
root->EE chain are locked at q=0, identical to type=fixed.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

### Task 2: Pin that a tilted root is honoured

These are characterization tests: the behaviour already holds in Pinocchio; the tests make
it a guarantee, so the mount can never silently stop being applied.

**Files:**
- Test: `tests/dynamics_test.cpp`

**Interfaces:**
- Consumes: `UrdfXml`, `Dynamics(const UrdfXml&, ...)` from Task 1.

- [ ] **Step 1: Write the tests** — append:

```cpp
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace {
constexpr double kTilt = M_PI / 6.0;  // 30 deg, new_adapter_plate_v3
// The fixture with its root joint (world -> gen3_base_link) pitched +30 deg about Y:
// the flatbot / gen_1_5 mount. Root "world" plays the role of arm_mount.
std::string tilted_fixture() {
  std::string xml = read_file(URDF_PATH);
  replace_once(xml,
               "<joint name=\"gen3_base_joint\" type=\"fixed\">\n"
               "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>",
               "<joint name=\"gen3_base_joint\" type=\"fixed\">\n"
               "    <origin rpy=\"0 0.52359877559829887 0\" xyz=\"0 0 0\"/>");
  return xml;
}
Eigen::Matrix3d Ry(double a) {
  return Eigen::AngleAxisd(a, Eigen::Vector3d::UnitY()).toRotationMatrix();
}
}  // namespace

TEST(DynamicsMount, TiltedRootRotatesFkAndJacobianIntoTheRootFrame) {
  Dynamics flat(URDF_PATH), tilted(UrdfXml{tilted_fixture()});
  const Eigen::Matrix3d R = Ry(kTilt);
  for (const auto& q : kPoses) {
    const Pose a = flat.fk(q), b = tilted.fk(q);
    EXPECT_LT((R * a.p - b.p).norm(), 1e-12);
    EXPECT_LT((R * a.R.toRotationMatrix() - b.R.toRotationMatrix()).cwiseAbs().maxCoeff(),
              1e-12);
    Jacobian6 Ja, Jb;
    flat.jacobian(q, Ja);
    tilted.jacobian(q, Jb);
    EXPECT_LT((R * Ja.topRows<3>() - Jb.topRows<3>()).cwiseAbs().maxCoeff(), 1e-12);
    EXPECT_LT((R * Ja.bottomRows<3>() - Jb.bottomRows<3>()).cwiseAbs().maxCoeff(), 1e-12);
  }
}

TEST(DynamicsMount, TiltedRootGravityEqualsLevelModelWithGravityRotated) {
  // Ground truth built independently: the LEVEL model, with Pinocchio's gravity
  // vector expressed in the arm's base frame, R^T * (0, 0, -g).
  pinocchio::Model level;
  pinocchio::urdf::buildModel(URDF_PATH, level);
  level.gravity.linear(Ry(kTilt).transpose() * Eigen::Vector3d(0, 0, -9.81));
  pinocchio::Data data(level);
  Dynamics tilted(UrdfXml{tilted_fixture()});
  for (const auto& q : kPoses) {
    Eigen::VectorXd cfg = pinocchio::neutral(level);
    for (int i = 0; i < level.nv; ++i) {  // same (cos, sin) packing as Dynamics
      const auto jid = level.getJointId(level.names[i + 1]);
      const int k = level.idx_qs[jid];
      if (level.nqs[jid] == 2) {
        cfg[k] = std::cos(q[i]);
        cfg[k + 1] = std::sin(q[i]);
      } else {
        cfg[k] = q[i];
      }
    }
    const Eigen::VectorXd want = pinocchio::computeGeneralizedGravity(level, data, cfg);
    JointVec got;
    tilted.gravity(q, got);
    EXPECT_LT((want - got).cwiseAbs().maxCoeff(), 1e-9);
  }
}

TEST(DynamicsMount, TiltLoadsTheBaseYawJointThatLevelLeavesFree) {
  // Physical sanity: on a level arm joint 1's axis is vertical, so gravity never
  // loads it. Tilted 30 deg, it does whenever the arm's CoM is off that axis.
  Dynamics flat(URDF_PATH), tilted(UrdfXml{tilted_fixture()});
  JointVec q = JointVec::Zero(), tf, tt;
  q[1] = M_PI / 2.0;  // arm horizontal
  q[0] = M_PI / 2.0;  // pointing along +Y, sideways to the tilt
  flat.gravity(q, tf);
  tilted.gravity(q, tt);
  EXPECT_NEAR(tf[0], 0.0, 1e-9);
  EXPECT_GT(std::abs(tt[0]), 1.0);
}
```

- [ ] **Step 2: Run them**

Run: `cmake --build build -j && ./build/unit_tests --gtest_filter='DynamicsMount*'`
Expected: 3 tests PASS. If `TiltedRootGravity…` fails, stop: the root is not being
honoured and the whole design's driver half rests on it.

- [ ] **Step 3: Sanity-check the test can fail** — temporarily change `kTilt` in
`tilted_fixture`'s rpy string to `0 0 0`, rebuild, rerun: expected FAIL on the first two
tests. Revert.

- [ ] **Step 4: Commit**

```bash
git add tests/dynamics_test.cpp
git commit -m "test(dynamics): a tilted URDF root puts gravity, FK and J in the root frame

Pins the property the tilted-mount design rests on: the mount lives in the
model, so nothing downstream needs to know about it.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

### Task 3: Standalone apps require `--urdf`, take `--ee-frame`

**Files:**
- Modify (each: the `urdf` default, the arg parser, the `Dynamics` construction):
  `apps/benchmark_cartesian_impedance.cpp:50,76-77,129`,
  `apps/benchmark_grav_comp.cpp:99,103,141-144,182`,
  `apps/benchmark_joint_impedance.cpp:44,70-71,112`,
  `apps/benchmark_joint_velocity.cpp:49,73-74,117`,
  `apps/joint_position_check.cpp:133,165-166,228`,
  `apps/joint_torque_check.cpp:83,105-106,166`,
  `apps/stream_check.cpp:125,147-148,260`,
  `apps/teleop_socket_server.cpp:121,153-154,214-216`,
  `apps/trajectory_run.cpp:109,145-146,227`,
  `apps/velocity_hold_check.cpp` (find its `urdf` default and `Dynamics dyn(urdf)` at 199)
- Modify: any doc that shows an app invocation without `--urdf`
  (`grep -rn -E '(benchmark_|_check|trajectory_run|teleop_socket_server)' README.md docs CLAUDE.md`)
- Modify: `CHANGELOG.md` (Unreleased section)

**Interfaces:**
- Consumes: nothing new.

- [ ] **Step 1: Write the check that fails today**

Run: `./build/benchmark_joint_impedance --sim --duration 0.5; echo "exit=$?"`
Expected today: runs with the level default model and `exit=0` — the silent footgun.

- [ ] **Step 2: Change every app the same way.** In each file:

1. The default becomes empty, and an EE-frame variable is added next to it (in
   `benchmark_grav_comp.cpp` the variable exists; change its default):

```cpp
  std::string urdf;  // required: no default, a level model on a tilted arm is silent
  std::string ee_frame = "arm_end_effector_link";
```

2. In the argument loop, beside the existing `--urdf` branch (use the file's own accessor,
   `next("--ee-frame")` or `val()`):

```cpp
    else if (a == "--ee-frame")
      ee_frame = next("--ee-frame");
```

3. Immediately after the argument loop, before anything connects or builds:

```cpp
  if (urdf.empty()) {
    std::cerr << "--urdf <file> is required: the arm model for THIS robot, including its "
                 "mount (render one with kinova-gen3-ros2's render script, or use "
                 "models/gen3_7dof_2f85.urdf --ee-frame gen3_end_effector_link for a "
                 "level arm)\n";
    return 2;
  }
```

4. Every `Dynamics x(urdf)` becomes `Dynamics x(urdf, ee_frame)` (three in
   `teleop_socket_server.cpp`, two in `stream_check.cpp`).

5. Where the app prints `urdf=` in its startup line, add `<< " ee_frame=" << ee_frame`.

- [ ] **Step 3: Build and verify**

Run: `cmake --build build -j 2>&1 | grep -E 'error|warning: unused' ; ./build/benchmark_joint_impedance --sim --duration 0.5; echo "exit=$?"`
Expected: no errors; the usage message, `exit=2`.

Run: `./build/benchmark_joint_impedance --sim --duration 0.5 --urdf models/gen3_7dof_2f85.urdf --ee-frame gen3_end_effector_link; echo "exit=$?"`
Expected: runs as before, `exit=0`.

Run: `for a in benchmark_cartesian_impedance benchmark_grav_comp benchmark_joint_velocity joint_position_check joint_torque_check stream_check teleop_socket_server trajectory_run velocity_hold_check; do ./build/$a --sim >/dev/null 2>&1; echo "$a exit=$?"; done`
Expected: every line `exit=2`.

- [ ] **Step 4: Docs and changelog.** Add `--urdf models/gen3_7dof_2f85.urdf --ee-frame
gen3_end_effector_link` to every app invocation the grep in **Files** finds. Under
`CHANGELOG.md`'s Unreleased:

```markdown
### Changed
- **Breaking (apps):** every standalone app requires `--urdf`; there is no default model.
  A level default silently applied the wrong gravity on a tilted mount. Apps take
  `--ee-frame` (default `arm_end_effector_link`, the rendered model's name).

### Added
- `Dynamics` builds from URDF XML (`UrdfXml`) and reduces any model to the root→EE chain,
  locking other joints at zero, so the articulated description is accepted.
  `Dynamics::joint_names()` returns the arm's joints in chain order.
```

- [ ] **Step 5: Full suite and commit**

Run: `ctest --test-dir build --output-on-failure`
Expected: `100% tests passed`.

```bash
git add apps CHANGELOG.md README.md docs CLAUDE.md
git commit -m "feat(apps)!: require --urdf; add --ee-frame

A default level model on a tilted arm commands the wrong gravity with
nothing saying so. Running an app now means choosing a robot's model.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

## Self-review notes

- Spec coverage (driver section): XML constructor and reduction — Task 1; no maths change
  pinned — Task 2 (the spec's g(q) equivalence test is `TiltedRootGravityEquals…`); apps
  require `--urdf` — Task 3; `models/` kept as fixture — unchanged and used by every test;
  reduced-vs-frozen test — Task 1.
- The spec says the node defaults `--ee-frame` to `arm_end_effector_link`; that default lives
  in kinova-gen3-ros2 and in the apps here. The library default is deliberately unchanged
  (139 test call sites use it with the fixture, and a missing frame throws).
- Release: kinova-gen3-ros2's plan bumps its driver pin to the commit that lands Task 1.
