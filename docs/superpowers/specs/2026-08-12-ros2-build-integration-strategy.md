# ROS2 Build Integration Strategy

**Date:** 2026-08-12
**Status:** Decided (team, 2026-08-12) — **b1** (export + `find_package`) and the
**ROS2 layer lives in a separate repo**. See "Decisions" below.
**Context:** Phase 2 of the arm-driver interface layer adds a ROS2 frontend
(`Ros2Backend` + a custom action). This note is *only* about how ROS2 fits into
the build without compromising the driver core. It does not re-open the interface
design (see `2026-08-10-arm-driver-interface-design.md`).

## Decisions (2026-08-12)

1. **b1 — the core exports a CMake config; the ROS2 side `find_package`s it.** Not
   `add_subdirectory` (b2). The core becomes a first-class installable library.
2. **The ROS2 layer lives in its own repo** (`kinova_arm_ros2`), not in a `ros2/`
   directory of this repo. It vendors this core repo via a `.repos` file into its
   colcon workspace (the ros2_kortex pattern), so editing the core and rebuilding
   the whole workspace stays one motion.

The rest of this document reflects those decisions. The reasoning that led here is
preserved below for the record.

---

## The requirement

Two hard constraints that pull in opposite directions:

1. **The driver core must build and run with plain CMake, no ROS2, no colcon.**
   It runs under non-ROS middleware (ATOS), in CI, and as standalone benchmarks.
   A machine building the core for ATOS may not have ROS2 installed at all.
2. **The ROS2 frontend is naturally built with colcon/ament.** A custom action
   (`ExecuteJointTrajectory`) needs `rosidl` code generation, and the node wants
   the normal rclcpp / launch / param ecosystem.

So: the core cannot depend on ROS, but a real ROS2 package needs to sit on top of
it. The question is how to structure the repo and the build so both are true at
once — cleanly, not with `#ifdef` spaghetti.

## Why this is not just "add a flag"

The obvious first idea — one `CMakeLists.txt` with a `-DKINOVA_ENABLE_ROS2=ON`
flag that does `find_package(rclcpp)`, mirroring how we already gate KORTEX — is
the wrong tool here, for two concrete reasons:

- **`rosidl` interface generation only works inside an `ament_cmake` package.**
  Generating a custom `.action`/`.msg` uses `rosidl_generate_interfaces()`, the
  ament resource index, and `<member_of_group>rosidl_interface_packages</…>`.
  There is no supported way to do it in a plain-CMake project. The interface
  definition *forces* an ament package to exist.
- **It couples the core's build graph to ROS.** A ROS flag drags ROS discovery,
  codegen, and `ament_package()` conditionals into the core, and couples the
  core's release cadence to ROS. The core is meant to outlive any one middleware
  (that's the entire point of the ATOS requirement).

The KORTEX flag is fine because KORTEX is a plain prebuilt C++ SDK (headers +
`.so`) — `find_package` behind a flag is the right shape for that. ROS2 is a
different kind of dependency (a code-generation + package ecosystem), so it wants
a different shape.

## The mechanism that makes a clean split possible

The key fact that resolves the tension:

> A `package.xml` can declare `<export><build_type>cmake</build_type></export>`.
> colcon reads that and builds the package with **vanilla CMake** (not ament).
> Raw `cmake -S . -B build` **never reads `package.xml` at all** — it is inert to
> plain CMake.

So the **same** `CMakeLists.txt` builds two ways with no changes:

| Build path | Command | `package.xml` | ROS in the graph |
|---|---|---|---|
| ATOS / CI / benchmarks | `cmake -S . -B build && cmake --build build` | ignored | none |
| ROS2 frontend | `colcon build` | read → picks the plain-CMake build task | only in the wrapper packages |

This is defined by **REP-149** (the manifest spec; `<build_type>cmake</build_type>`
= "a non-catkin CMake project") and implemented by **colcon-ros**, which maps the
build-type string to a build task (`ros.cmake → CmakeBuildTask`,
`ros.ament_cmake → AmentCmakeBuildTask`). The core stays plain CMake; adding a
`package.xml` only teaches colcon *how* to build it when ROS is in play, and costs
the standalone build nothing.

## What the community actually does (evidence)

We checked real repositories rather than guessing.

**`ros2_kortex` (Kinova's own ROS2 driver) — a cautionary example, not a model.**
Its packages are all ament; there is no ROS-free core, so nothing in it is
reusable outside ROS. Its `kortex_api` vendor package is `ament_cmake` and only
installs headers — it does *not* export a linkable CMake target — so the consumer
`kortex_driver` has to fetch the KORTEX binary a **second time** and hand-wrap it
as a `STATIC IMPORTED` library. This is exactly the coupling we want to avoid.

**ORB-SLAM3 — shows the failure mode of *not* exporting a CMake config.**
ORB-SLAM3 is a pure-CMake library, but it ships no `ORB_SLAM3Config.cmake`. As a
result its ROS2 wrappers (e.g. `zang09/ORB_SLAM3_ROS2`) resort to a hand-written
`FindORB_SLAM3.cmake` with a **hardcoded install path** (`~/Install/ORB_SLAM/…`).
The lesson is the inverse: **if the core installs a proper `Config.cmake`, the
wrapper collapses to a clean `find_package(<Core> CONFIG REQUIRED)`** — no
Find-modules, no hardcoded paths.

**Consensus for a core that must work with *and* without ROS2:** a **pure-CMake
core library + a separate ament wrapper package that depends on it**, with custom
interfaces isolated in their own small ament package (because `rosidl` forces it).

## Recommended architecture

A three-package split across **two repos**. This repo (the core) stays plain
CMake and gains an exported config + a `package.xml`; a **separate** repo holds
the two ament packages.

```
kinova-gen3-driver/                 (THIS repo — plain CMake, no ROS)
  CMakeLists.txt          kinova_lowlevel core. Default build UNCHANGED.
                          KINOVA_ENABLE_KORTEX flag stays (KORTEX is a plain C++ SDK).
                          + install(TARGETS … EXPORT) + kinova_lowlevelConfig.cmake   (b1)
  include/ src/ tests/    Layer A ports (pure-virtual) + Layer C Supervisor live HERE.
                          They are ROS-free — that is the hexagonal boundary.
  package.xml   (NEW)     <build_type>cmake</build_type> → colcon can build the core
                          when it is vendored into the ROS2 workspace. Inert to raw
                          cmake, so ATOS/CI builds are unaffected.

kinova_arm_ros2/                     (SEPARATE repo — the ROS2 frontend)
  kinova_arm.repos        vcs-imports kinova-gen3-driver (the core) into the workspace.
  kinova_arm_interfaces/   ament_cmake — ExecuteJointTrajectory.action
                           + JointImpedanceGains.msg. rosidl codegen only.
  kinova_arm_ros2/         ament_cmake — Ros2Backend (Layer B) + bring-up node.
                           find_package(kinova_lowlevel CONFIG) + rclcpp
                           + rclcpp_action + kinova_arm_interfaces.
```

**How the separate repo consumes the core.** The ROS2 repo carries a `.repos`
file that `vcs import`s this core repo into `src/`. `colcon build` then builds the
core first (as a `cmake`-type package, because of its `package.xml`), installs its
`kinova_lowlevelConfig.cmake` into the workspace overlay, and the two ament
packages `find_package(kinova_lowlevel CONFIG REQUIRED)` against it. Editing the
core and rebuilding the workspace is one `colcon build`. (Alternative — installing
the core to a prefix and treating it as a prebuilt binary dependency — is better
only once the core is a stable released artifact; during active development,
vendoring via `.repos` is the right call.)

**How it maps onto the hexagonal design** (from the interface spec):

- **Layer A (ports)** and **Layer C (Supervisor)** are ROS-free → they live in the
  **core** (`kinova_lowlevel`). Nothing new links ROS.
- **Layer B (`Ros2Backend`)** is the *only* unit that includes rclcpp → it lives
  in **`kinova_arm_ros2`**, the ament wrapper.
- The bring-up node does the dependency injection: construct `Ros2Backend`,
  construct the `Supervisor` against its ports, wire them. Neither the core nor
  the supervisor ever names ROS.

This is the same discipline the codebase already applies to KORTEX and Pinocchio:
one unit owns each heavy dependency, and it never leaks into the core interfaces.

## How each build path works

**Standalone (ATOS / CI / benchmarks) — unchanged from today:**
```sh
cmake -S . -B build -DCMAKE_PREFIX_PATH=<pinocchio prefix>
cmake --build build -j && ctest --test-dir build
```
`package.xml` is ignored; no ROS on the machine required.

**ROS2 frontend (in the separate `kinova_arm_ros2` repo's workspace, Humble sourced):**
```sh
source /opt/ros/humble/setup.bash
mkdir -p ws/src && cd ws
vcs import src < ../kinova_arm_ros2/kinova_arm.repos   # pulls in the core repo
colcon build            # builds: kinova_lowlevel (cmake) → interfaces → kinova_arm_ros2
source install/setup.bash
ros2 run kinova_arm_ros2 <bringup_node> --ros-args ...
```
colcon builds the core first as a plain-CMake package, installs its
`Config.cmake` into the workspace overlay, then builds the two ament packages that
`find_package` it.

## Why b1 (chosen) over b2

Both options satisfy "core builds without colcon." They differ in coupling, and
the team chose **b1** — decisive once the ROS2 layer is a separate repo (b2's
`add_subdirectory(../..)` doesn't even apply cleanly across repos).

**b1 — Export + `find_package` (chosen).**
The core adds `install(TARGETS … EXPORT …)` + a generated
`kinova_lowlevelConfig.cmake`. colcon builds the core as its own cmake-type
package; `kinova_arm_ros2` does `find_package(kinova_lowlevel CONFIG REQUIRED)`
and links `kinova_lowlevel::kinova_lowlevel`.
- **Pros:** most decoupled; core built once; core becomes a first-class
  installable library (useful for ATOS too); the clean path ORB-SLAM3 lacked;
  works naturally across the repo boundary.
- **Cons:** ~20 lines of export/config boilerplate in the core `CMakeLists.txt`.

**b2 — `add_subdirectory` the core from the wrapper (rejected).**
Would have the wrapper build the core in-tree. No export boilerplate, but the
wrapper reaches into the core's source layout and rebuilds it inside the ROS
build — and with the ROS2 layer in a separate repo, this means a fragile relative
path or a nested checkout. Rejected.

## Alternatives considered and rejected

- **Single `CMakeLists` with a `KINOVA_ENABLE_ROS2` flag.** Rejected: `rosidl`
  won't run outside ament, and it couples the core to ROS (see "Why this is not
  just a flag").
- **One ament package with a ROS-free library target + ROS nodes.** Rejected: the
  "library" still lives inside an ament package, so ATOS/CI can't build it without
  ament on the path. This is essentially the ros2_kortex structure and why its
  core isn't reusable.

## What changes vs. today

**In this repo (the core):**
- The default build stays **byte-for-byte the same**. b1 adds an `install(EXPORT)`
  + config block, which does not affect the existing build/test commands.
- New: a `package.xml` at the repo root (`<build_type>cmake</build_type>`), inert
  to raw cmake.
- New (Phase 2 code): Layer A ports + Layer C Supervisor, both ROS-free, in the
  core library.
- CI keeps building and testing exactly as it does now (plain CMake).

**In the new `kinova_arm_ros2` repo:**
- `kinova_arm.repos` (vendors the core), plus `kinova_arm_interfaces` and
  `kinova_arm_ros2` ament packages. Its own CI runs a colcon build. This repo does
  not exist yet — standing it up is the first task of the ROS2 work.

## Resolved

1. **b1 vs b2 → b1** (export + `find_package`).
2. **In-repo vs separate ROS2 repo → separate repo** (`kinova_arm_ros2`), vendoring
   the core via `.repos`.
3. **Interfaces package naming** — `kinova_arm_interfaces` (open to
   `kinova_arm_msgs` if that matches an existing convention; cosmetic).

## References

- REP-149 — package manifest format, `<build_type>` (incl. `cmake`):
  https://github.com/ros-infrastructure/rep/blob/master/rep-0149.rst
- colcon-ros build-type → task mapping (`ros.cmake`, `ros.ament_cmake`):
  https://github.com/colcon/colcon-ros
- ament design article (plain CMake vs ament_cmake; `CMAKE_PREFIX_PATH` overlay):
  https://design.ros2.org/articles/ament.html
- ros2_kortex (vendor + hardware_interface structure):
  https://github.com/Kinovarobotics/ros2_kortex
- ORB_SLAM3_ROS2 wrapper (Find-module + hardcoded path anti-pattern):
  https://github.com/zang09/ORB_SLAM3_ROS2
- Interface design spec (the architecture this build serves):
  `docs/superpowers/specs/2026-08-10-arm-driver-interface-design.md`
