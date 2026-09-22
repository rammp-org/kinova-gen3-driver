# Changelog

All notable changes to this project are recorded here.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and
this project follows [semantic versioning](CONTRIBUTING.md#versioning) from 1.0.0
on. Downstream repos pin an exact tag, so **this file is how you decide whether
to move a pin** — a tag on its own does not tell you that.

Add entries under `Unreleased` as work merges into `dev`; the release PR renames
that heading to the new version and bumps `package.xml`.

## [Unreleased]

### Added

- `velocity_hold_check`, a temporary attended harness for the three hardware
  questions the velocity fix leaves open: hold, freeze on a dead stream, and
  tracking at speed against the leash ([procedure](docs/integration/velocity_hold_check.md)).

### Fixed

- `JointImpedanceMode` holds the measured joint configuration on entry until an
  explicit target arrives. Previously, entry-pose IK could move the redundant
  posture before the supervisor submitted its first command. Explicit Cartesian
  targets still run IK; joint targets, gains and limits are unchanged.
- `JointVelocityMode` **holds at a zero command.** The actuator's own velocity
  servo does not reject gravity at zero — joint 2 crept ~0.038 rad/s with zeros
  streamed, and Kinova's own low-level example never holds a loaded joint in
  VELOCITY mode ([#34]). The mode now integrates the limited velocity into a
  position reference at the RT rate and requires `kPosition` on every actuator.
  The reference is leashed to within 0.1 rad of the measured position (the
  windup guard for contact or a blocked joint), bounded joints stop at their
  URDF limits, and a stale stream freezes the reference at the measured
  position before latching. The public API is unchanged; two observable
  differences: `required_modes()` reports `kPosition`, and `JointCommand::
  velocity` is zero as in every position-commanding mode (`commanded()` still
  reads the limited velocity fed to the integrator). RT cost, measured A/B on the Jetson AGX Orin
  (`benchmark_joint_velocity --sim --rate 1000 --duration 5`, pinned to core 10
  while the live driver held core 11, so noisier than the isolated-core figures
  in the deep dive), `compute_ns` p50 / p99 / max: joint path 128 / 512 / 1472
  after vs 128 / 512 / 1952 before; twist path 2048 / 8192 / 18912 after vs
  2048 / 8192 / 19680 before. Zero major faults, zero dropped samples; the full
  suite including `RtSafety*` passes on aarch64. Verified in sim; the hardware
  hold check is the release gate.

## [1.1.0] — 2026-09-14

A minor release: one additive signature change, a changed default, and a fix for
trajectories through a continuous joint's ±π boundary.

### Added

- `TrajectoryExecutor` takes an optional continuous-joint mask. It defaults to
  all-false, which is exactly the previous behaviour; `Supervisor` derives the
  mask from the URDF, so no caller has to opt in ([#52]).

### Changed

- `SupervisorConfig::sampler_hz` defaults to 1000 (was 250). At 250 Hz each
  trajectory target was held ~4 ms and the 1 kHz position rate limiter turned a
  smooth plan into a staircase; on the arm, a looped cuRobo joint tour showed
  ~15% less velocity ripple and ~20% less jerk at 1 kHz. The sampler is still a
  separate non-RT thread and publishes action feedback every tick; evaluating the
  trajectory inside the RT loop is the intended follow-up ([#56]).
- `SimTransport` reports joint positions wrapped into (-π, π], as
  `KortexTransport` does, so CI runs in the representation the hardware reports
  ([#52]).

### Fixed

- The trajectory divergence guard no longer aborts a goal when a continuous joint
  crosses ±π. Feedback arrives wrapped while a planner emits unwrapped targets,
  so the raw difference read ~2π; every goal through the boundary ended
  `kPathToleranceViolated` with a final error of zero ([#52]).

### Known limitations

In addition to the 1.0.0 list:

- An exception from the Kortex API is not caught in `KortexTransport`, so it
  terminates the process instead of raising a fault. Seen on the arm as
  `WRONG_SERVOING_MODE` when the base left low-level servoing ([#59]).

## [1.0.0] — 2026-09-08

First tagged release. The driver has been in use for months; this release does
not add behaviour, it declares the interface stable and gives consumers something
to pin other than a moving branch.

### The promise

The public surface — everything under `include/kinova_lowlevel/`, plus SI/radians
internally and the RT contract — is now covered by semantic versioning. See
[Versioning](CONTRIBUTING.md#versioning) for what counts as a break, including
the rule that adding a *pure* virtual to an interface implemented downstream is a
MAJOR change.

### Added

- **Continuous integration.** Build, `ctest`, an allocation-freedom gate
  (`RtSafety*`: zero major page faults, zero dropped telemetry samples), an
  install-and-consume check of the exported CMake package, and a sim smoke run —
  on **both amd64 and arm64**, natively. Plus a **KORTEX build** on both
  architectures, so the real-arm transport is compiled and linked by a gate
  rather than only on a developer's Jetson. No CI job touches a robot.
- **Versioning machinery.** `kinova_lowlevelConfigVersion.cmake` is now installed
  (`SameMajorVersion`), so a consumer can write
  `find_package(kinova_lowlevel 1.0 CONFIG REQUIRED)` and have it mean something.
  Previously no version could be requested at all.
- **An MIT licence.** The repo was public with no LICENSE file and a
  `Proprietary` declaration in `package.xml`; nothing pinned to it carried a
  grant to use it.
- **A support matrix** in [the docs](docs/index.md#support-matrix): per control
  mode, whether it has run on hardware, whether its RT cost has been measured,
  and what is known to be wrong with it.
- `CONTRIBUTING.md`, `.clang-format`, and pre-commit hooks (file hygiene,
  `actionlint`, `shellcheck`, `clang-format`).

### Changed

- `package.xml` is the single source of truth for the version; CMake reads it
  rather than repeating it. Bumping a release means editing one file.
- The tree is formatted with clang-format (stock Google, 100 columns). Mechanical
  only — no behaviour change. The reformat commit is in
  `.git-blame-ignore-revs`; run
  `git config blame.ignoreRevsFile .git-blame-ignore-revs` so `git blame` reaches
  authors.
- `README.md` and the docs landing page describe the five control modes, the
  gripper and the interface tier, instead of the two modes they still advertised.

### Known limitations

Shipped deliberately and disclosed rather than hidden — the support matrix is the
full list, and none of these are API:

- The URDF masses do not match the true system, which weakens every gravity-model
  number downstream of it ([#5]).
- `JointImpedanceMode` — the most expensive mode — has never been timed, and its
  gains were tuned by feel ([#6]).
- `JointVelocityMode` **will not hold a zero command**: joint 2 creeps ~0.038
  rad/s under gravity. This is the actuator's own velocity servo, not this
  driver; do not park an arm there and stream zeros ([#34]).
- Gravity-comp hold droops and drifts on the arm rather than holding ([#40]).
- Trajectory goals are **positional** — `joint_names` is not consulted ([#16]) —
  and complete on elapsed time rather than arrival, so `goal_tolerance` and
  `goal_time_tolerance_s` are reserved and not enforced ([#17]).
- A KORTEX-enabled `install()` bakes the build machine's SDK path into the
  exported target, so it is not relocatable ([#50]).

[Unreleased]: https://github.com/rammp-org/kinova-gen3-driver/compare/v1.1.0...HEAD
[1.1.0]: https://github.com/rammp-org/kinova-gen3-driver/compare/v1.0.0...v1.1.0
[1.0.0]: https://github.com/rammp-org/kinova-gen3-driver/releases/tag/v1.0.0
[#5]: https://github.com/rammp-org/kinova-gen3-driver/issues/5
[#6]: https://github.com/rammp-org/kinova-gen3-driver/issues/6
[#16]: https://github.com/rammp-org/kinova-gen3-driver/issues/16
[#17]: https://github.com/rammp-org/kinova-gen3-driver/issues/17
[#34]: https://github.com/rammp-org/kinova-gen3-driver/issues/34
[#40]: https://github.com/rammp-org/kinova-gen3-driver/issues/40
[#50]: https://github.com/rammp-org/kinova-gen3-driver/issues/50
[#52]: https://github.com/rammp-org/kinova-gen3-driver/issues/52
[#56]: https://github.com/rammp-org/kinova-gen3-driver/issues/56
[#59]: https://github.com/rammp-org/kinova-gen3-driver/issues/59
