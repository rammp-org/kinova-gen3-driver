# Changelog

All notable changes to this project are recorded here.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and
this project follows [semantic versioning](CONTRIBUTING.md#versioning) from 1.0.0
on. Downstream repos pin an exact tag, so **this file is how you decide whether
to move a pin** — a tag on its own does not tell you that.

Add entries under `Unreleased` as work merges into `dev`; the release PR renames
that heading to the new version and bumps `package.xml`.

## [Unreleased]

## [2.0.0] — 2026-10-30

A major release. Six threads of work land together: one gains contract on every
impedance surface, with target feedforward and compliant velocity and twist
streams; unexpected-contact detection in the RT loop; a driver that survives arm
faults, disconnects and lost frames instead of exiting (`ArmLink`); RT and comms
hardening (bounded cyclic calls, an RT-priority UDP receive thread, mode timers
on wall time); tilted arm mounts, with the model read from URDF XML; and a
measured EE wrench on `ArmState`.

It is MAJOR because it breaks the public surface in several places: new pure
virtuals on `Transport`, a new `ArbitrationSink::grant` override, retyped gain
fields, new `HaltReason` and `StreamCloseCause` enumerators, and feedback torque
that changes sign. `kinova-gen3-ros2` needed an adoption commit for each. Work
in progress called this release v1.3.0; the plans and specs under
`docs/superpowers/` still use that name.

### Upgrading from 1.2.0

Each item below breaks a build or changes behaviour a caller can see. Read the
whole list before moving a pin.

**If you implement the driver's interfaces**

- `Transport` gains two pure virtuals, `release_low_level()` and
  `arm_in_fault()` (#96, #101). Implement both in every custom transport and
  test fake.
- `ArbitrationSink::grant` is overridden as
  `grant(const std::string&, const std::optional<Token>& = std::nullopt)`
  (#89, #90). Callers are unaffected.
- `JointTargetSink` implementers override `set_joint_target(const JointTarget&)`
  (`{q, qd, qdd}`). The position-only `set_target` remains as a non-virtual
  convenience (#75).

**If you construct or switch on driver types**

- `TrajectoryGoal` drops `has_gains` / `gains` (`JointImpedanceGainValues`) for
  `ImpedanceGains gains`; an unset `ImpedanceGains` means the session default.
  `GainsRequest.gains` is now `GainsRequest.spec` (#75).
- `HaltReason` gains `kArmUnavailable` (#101) and `kContact` (#109);
  `StreamCloseCause` gains `kContact` (#109). Exhaustive `switch`es need the new
  cases.
- `ExecStatus::promoted` is gone, and `TrajectoryExecutor::submit` can return
  the new `SubmitResult::kRejectedQueueWhileActive` (#106) and
  `SubmitResult::kRejectedBadTimestamps` (#107).
- Both `Dynamics` constructors require exactly seven 1-DOF revolute joints after
  reduction to the arm chain; a prismatic or multi-DOF joint is refused, not just
  a wrong count (#81).

**Behaviour you will see**

- **Feedback torque changes sign** (#73, #74). `JointFeedback::tau` and
  `ArmState::tau` are in the command convention, the torque the actuator
  applies (≈ `+g(q)` at a free hold), instead of KORTEX's raw reaction sign
  (≈ `−g(q)`). `KortexTransport` flips it where degrees become radians, and
  `SimTransport` echoes the commanded torque in torque mode. Anything that
  correlates `tau` with the model flips with it: gravity residuals, contact
  monitors, `/joint_states` effort downstream. The teleop protocol is now
  `kVersion = 2` because `FeedbackPacket.tau` changed meaning, so update both
  ends together. Recordings made with 1.2.0 have the old sign; `contact_replay
  --tau-sign -1` reads them.
- **`kQueue` behind an active goal is refused** (#84, #106). It settles
  `kInvalidGoal` ("queueing is disabled: …") before any mode switch, and the
  active goal runs on untouched. Queueing was unsafe as built: nothing checked
  that a queued goal starts where its predecessor ends, goals complete on time
  rather than on arrival (#17), and cancelling either goal cancelled both. Plan
  through all the waypoints in one goal instead. A `kQueue` goal sent to an idle
  arm runs as before, so callers that send `preemption = 0` one goal at a time
  are unaffected. `kQueue` stays in the enum and on the wire.
- **Badly stamped trajectories are refused** (#107): consecutive waypoints
  less than 10 ns apart (`kMinSegmentSpanSec`), which includes duplicate and
  out-of-order `time_from_start`, or any non-finite stamp. They are refused at
  `on_trajectory_goal` and again before the mode switch when the goal runs
  (settled `kInvalidGoal`). Duplicates used to step the reference at that
  instant, and a goal whose stamps were all zero commanded its first point and
  reported success without moving through the rest.
- **Gains are validated, never ignored or clamped** (#63, #64, #75).
  Non-finite values, out-of-range stiffness or damping ratio, a torque limit
  below the per-joint gravity floor, and an out-of-enum profile byte are refused
  with a reason. Gains on a surface where they cannot act (a position goal, a
  non-impedance stream) are refused, not ignored.
- **Higher gravity floors for custom gains** (#81): joint 1 (base yaw) rises from
  0.5 to 18.8 N·m and joint 3 from 14.5 to 14.6 N·m, so the floors cover the 30°
  tilted mount as well as level. A custom `torque_limit` below them is refused.
- **`KortexTransport` errors are reclassified** (#101). An arm's error reply is
  `ArmRejected` (was `KDetailedException`); a failed local `sendto` or an RPC
  timeout is a survivable `TransportTimeout` (the `sendto` failure was a fatal
  `runtime_error`); an actuator-count mismatch is `std::invalid_argument` (was
  `runtime_error`). `connect()` no longer clears faults silently, so a
  standalone app (benchmarks, checks) started on a faulted arm stops with
  `ArmRejected`: clear the fault from the web app first.
- `KortexTransport::set_servoing_low_level()` drops the retained gripper command,
  so re-entering low level leaves the gripper uncommanded, as at startup (#101).
- An arm address that is not IPv4 is a startup error (#101).

**Build, apps and scripts**

- CMake requires Pinocchio 3.0 or later (`Frame::parentJoint`) (#81).
- Every standalone app requires `--urdf`; there is no default model, because a
  level default silently applied the wrong gravity on a tilted mount. `--ee-frame`
  defaults to `arm_end_effector_link`, the rendered model's name (#81).
- `scripts/rt_grant_once.sh` and `scripts/rt_setup.sh` are gone. They are host
  setup for every RT process on the chair and now live in
  `rammp-org/rammp-deployments/scripts`, with the `rt-setup.service` unit;
  `docs/rt-tuning.md` stays here and points there (#51).

### Added

**Compliance: one gains contract** (#63, #64, #75)

- `ImpedanceGains` names the compliance on every impedance surface: a named
  profile (`soft` / `medium` / `stiff`, each a complete `JointImpedanceParams`),
  `custom` raw gains (kq / zeta / torque_limit over the session default), or
  nothing, which means the session default (initially `medium`). Trajectory
  goals, stream opens (`StreamOpenRequest::gains`, applied at open) and
  `on_set_gains` all take it; `on_set_gains` is now implemented and replaces the
  session default. Gains apply when a goal starts running and on every impedance
  command, mode switch or not, and never carry into the next command or session.
  A named profile overlays only its gain fields onto the session default, so
  deployment tuning (IK limits, `cmd_timeout_s`, ramp) survives every command.
  Gains on an open stream cannot change mid-session: close and re-open. The
  gravity floors behind validation are pinned from the worst-case URDF gravity
  (2F-85 model), and a test re-derives them so a model change fails loudly.
- **Target feedforward.** Joint-space targets carry `{q, qd, qdd}` (`JointTarget`).
  `JointImpedanceMode` damps the velocity error against the commanded rate (`qd`,
  clamped to `max_ref_speed`) and adds `M(q)·qdd`, removing the standing tracking
  lag `2ζ·qd·√(M/Kq)` on profiled trajectories. The executor scales the
  derivatives into wall time under the speed scale (`qd·s`, `qdd·s²`).
  Feedforward engages only when a target carries a profile; the teleop, pose and
  entry-hold paths and the staleness freeze are unchanged.
- **Compliant velocity and twist streams**: joint velocity × `kImpedance` and
  EE twist × `kImpedance`. The sampler integrates the command into a held
  reference (the 1.1.1 `JointVelocityMode` mechanism, factored out) and writes
  targets into `JointImpedanceMode`, which latches the newest one each 1 kHz
  cycle. Commands are capped to the URDF velocity ratings before integrating, and
  the feedforward is the rate the reference actually advanced at: the commanded
  rate in free motion, zero once the leash pins against a blocked arm.
- `JointImpedanceMode::params()` is public, for gain read-back.
- `benchmark_joint_impedance`, a sim benchmark of the joint impedance compute
  path (`--track` streams profiled sinusoid targets). The feedforward's cost is
  within the noise floor (x86 sim, mean ~1.5 µs).

**Unexpected-contact detection** (#109)

- In the RT loop: `MomentumObserver` estimates external joint torque (using the
  new `Dynamics::coriolis`), and `ContactMonitor` (`ContactDetection::kObserve`
  / `kEnforce`) thresholds it per joint, with debounce, and latches.
  `RtExecutor::attach_contact(monitor, on_contact)` steps the monitor after every
  exchange; on a trip it switches to `on_contact`, normally the new `FloatMode`,
  in the same cycle when the actuator modes match, and otherwise freezes and
  adopts it at the next boundary. `CycleSample` bit3 flags latched cycles. With
  no monitor attached nothing changes.
- **Contact stop in the Supervisor.** With a `ContactMonitor` in
  `SupervisorDeps::contact`, a latch halts with `HaltReason::kContact`: goals
  settle with the new `result_code::kContactStop = -11` ("halted: unexpected
  contact"), an open stream closes with `StreamCloseCause::kContact`, and while
  latched, goals, stream opens (`kContactStop`, "stopped on unexpected contact;
  call /clear_faults") and gripper setpoints are refused. The latch survives link
  outages. `Supervisor::clear_contact()` clears it and returns the arm to the
  position hold at the measured pose.
- Threshold profiles (`ContactProfile`: `kDefault = 0`, `kSensitive = 1`,
  `kLoose = 2`, `kCustom = 3`, the `SetContactThresholds.PROFILE_*` wire values,
  so an unset profile selects the default; `interface/contact.h`), set through
  the token-gated `CommandSink::on_set_contact_thresholds` and read through the
  ungated `on_query_contact`. Both have default bodies. A threshold that is
  non-finite, ≤ 0 or above the ceiling is refused, naming the joint and the
  limit; bad profiles in startup config throw from the `Supervisor` constructor.
- `contact_replay`, an offline tool that replays a recorded joint-state CSV
  through the `ContactMonitor` (observe mode) to tune thresholds between arm
  sessions, and `tools/contact/parity.py`, which checks its residual against the
  Python reference observer. The core is `replay()` in `contact_replay.h`. See
  `docs/guide/contact-detection.md`.
- `SimTransport::set_external_torque()` and `set_gravity_model()`, for driving
  the observer in tests.

**Surviving arm-side events** (#96, #59, #100; #101)

- `ArmLink` (`arm_link.h`) owns the link lifecycle (`kConnecting` / `kRunning` /
  `kFaulted`) around `RtExecutor::run()`. An unreachable arm is retried every
  second, forever, including an arm still booting (0 actuators, or actuators not
  yet registered). A fault, a rejected command, any unknown exception, or a comms
  loss while commanding hands the arm back to its base, is reported, and waits
  for `request_clear()`. After a power-cutting e-stop the arm comes back under its
  base, and the Kinova controller keeps it until the clear. Only an
  actuator-count mismatch still escapes.
- The `Supervisor` listens (`on_link_state`): goals halt with "halted: arm fault
  or connection lost" (`HaltReason::kArmUnavailable`), work is refused while the
  arm is down, and after recovery the arm holds where it is.
- With no direct route to the arm (the arm NIC without an address or a link),
  each connect attempt fails at once, naming the interface, instead of blocking
  ~127 s in TCP retries out the default route (`arm_route.h`).
- `ExecutorConfig::exit_on_fault` (default off; `ArmLink` sets it) makes `run()`
  throw the new `ArmFault` once feedback confirms a fault or that the base left
  low-level servoing. New `ArmRejected`, `JointFeedback::low_level` and
  `RtExecutor::config()`.

**RT and transport**

- `KortexTransport` takes optional `rx_priority` / `rx_cpu` (#97, #98, #99).
  `connect()` moves the SDK's UDP receive thread, which every `exchange()` waits
  on, to `SCHED_FIFO` at `rx_priority`, pinned to `rx_cpu` only if the priority
  took. The TCP receive and keepalive threads are left alone. It was a normal
  thread on any core, preempted ~10k times in 30 min under perception load. The
  apps pass `--rt-priority - 1` and `--cpu`. New `raise_thread()` in
  `rt_system`.

**Tilted arm mounts and the model** (#81)

- `Dynamics` builds from URDF XML (`UrdfXml`; the caller names `ee_frame`, there
  is no default) and reduces any model to the root→EE chain, locking other joints
  at zero, so the articulated description is accepted. `Dynamics::joint_names()`
  returns the arm's joints in chain order, and `Dynamics::mount_tilt_deg()` reads
  the mount tilt from the model. Every app prints it at startup: `0.0` level,
  `30.0` on flatbot / gen_1_5.

**Measured EE wrench** (#73, #74)

- `ArmState::ee_wrench`: the external wrench the environment applies to the tool,
  `[force; torque]` in N / N·m, `LOCAL_WORLD_ALIGNED`, estimated in the
  Supervisor's pump from the joint-torque residual, from the same frame, model
  and sample as `ee_twist`. Quasi-static, contact-detection grade; NaN while
  the arm reports a fault. Derivation and accuracy floor:
  `docs/superpowers/specs/2026-10-04-ee-wrench-design.md`. Since #119 it comes
  from the momentum observer when one runs (see Changed).

**Arbitration** (#89, #90)

- `ArbitrationSink::grant(owner_id, token)` adopts a caller's pre-generated key,
  so an orchestrator can share one key across several capabilities. Omitted, the
  driver mints one as before. An all-zero token is refused without disturbing the
  current owner.

### Changed

- **`ArmState::ee_wrench` comes from the momentum observer when one runs**
  (#112, #119). With a `ContactMonitor` attached, its observer settled and the RT
  loop stepping it, the pump maps the observer residual (from
  `ContactMonitor::status()`) instead of the quasi-static `g(q) − τ`. The
  observer models the arm's own inertia, so far less of the arm's own motion
  reads as force: on a position-mode tour at speed 0.6 with nobody touching, the
  force error at the flange fell from median 3.3 / p95 8.0 / max 13.7 N to
  1.9 / 5.4 / 6.2 N. It trails the quasi-static estimate by about 25 ms on hand
  taps (first order, time constant 1/K = 40 ms). With no monitor, during the 5/K
  settle after each run start and clear, while the RT loop is stopped, and on a
  torn status read, the pump publishes the quasi-static estimate as before. NaN
  under a fault either way. New `ContactStatus::settled` and
  `ContactMonitor::status_version()`.
- `KortexTransport` reports more faults and shuts down faster (#101). Base fault
  banks and `ARMSTATE_IN_FAULT` now set `JointFeedback::fault`.
  `safe_shutdown()`'s revert bounds each RPC at 500 ms and stops at the first
  unanswered one (a dead link took ~24 s); a refused one moves on. The standalone
  apps get the route check.
- CI builds against Pinocchio 3.9.0, the exact cmeel set the robot runs
  (`.github/actions/setup-cpp-deps/requirements.txt`, mirroring
  kinova-gen3-ros2's `docker/requirements.txt`), instead of 2.7.0 (#81).
- `docs/deep-dive/trajectory-interpolation.md` documents that cubic and quintic
  Hermite interpolation is not hull-preserving, that the path-tolerance guard
  cannot see the excursion, and that re-timing a plan without rescaling `qd` /
  `qdd` is unsafe, with measured excursions (#19, #107). No `submit()` check.

### Fixed

- **A single lost KORTEX UDP frame no longer kills the driver** (#95).
  `KortexTransport` bounds every cyclic call on the RT path (`exchange`, `send`,
  `receive`, and the pumps during a mode switch) at `cyclic_timeout_ms` (new
  sixth constructor argument, after `rx_priority` / `rx_cpu`; default
  `kDefaultCyclicTimeoutMs` = 20 ms, where KORTEX's default was 3000 ms) and
  reports expiry as the new `TransportTimeout`, detected by type with a
  `steady_clock` deadline, not by matching KORTEX's error message. `RtExecutor`
  survives a lost exchange: it keeps the last good feedback, re-sends the
  previous command, sets `CycleSample` bit2, and
  `TelemetrySink::console_line()` gains `comm_timeouts=`. `run()` throws
  `CommsLost` after `ExecutorConfig::max_consecutive_misses` in a row (default
  `kMaxConsecutiveTimeouts` = 3, i.e. 60 ms with no reply), after
  `max_window_misses` within `miss_window` exchanges (default 5 in 1000), or when
  a mode switch's pumps time out. Miss limits below 1 are rejected. Other
  exchange errors propagate as before.
- **Actuator-mode switches no longer command zero torque** (#92, #117).
  `KortexTransport::set_actuator_modes` switches one actuator at a time and keeps
  the cyclic channel alive with a frame after each switch. Those frames sent
  torque 0, which an actuator in TORQUE mode takes as its command: entering a
  torque mode, each actuator went limp from its own switch to the end of the
  switch; leaving one, the joints not yet switched went limp until their turn, as
  did joints a lost revert left in TORQUE on a reconnect. The frames now hold
  each joint's torque measured once, before the first switch, as the seed frame
  in `set_servoing_low_level()` already did. The actuator's torque loop tracks
  its own sensor, so echoing the reading should hold the torque the joint was
  applying (unverified on the arm). Actuators already in POSITION see no change.
  On a contact trip from a position-type mode the hold includes the contact
  torque, so through the switch to `FloatMode` the joints keep pushing toward
  the contact instead of dropping (see `RtExecutor` in `docs/reference/api.md`).
- **Mode timers count the time lost to missed exchanges** (#103, #105).
  `RtExecutor` passed `compute()` a fixed `1 / rate_hz`, so each ~20 ms miss went
  uncounted and a 100 ms `cmd_timeout_s` could stretch to ~180 ms before the miss
  budget declared comms lost. `compute()` now gets the wall time since the mode's
  previous `compute()`, capped at `max_consecutive_misses` × (the cyclic timeout
  + one period), 63 ms at the defaults; the first `compute()` in a mode still
  gets `1 / rate_hz`. `dt` therefore carries the loop's wake and round-trip
  jitter even without misses; every mode in the library sums it, none divides by
  it. The command watchdogs, gain ramps, `ik_fault_s`, the torque feedforward
  decay, the reference rate limiters and `JointVelocityMode`'s integration all
  follow wall time. After a miss the position and velocity modes' setpoint moves
  about 21 ms worth in one cycle, at most the cap's worth: ~10 mrad per miss at
  the position mode's default 0.5 rad/s, up to ~29 mrad in the velocity mode at a
  1.4 rad/s `max_qd`. Their leashes still bound it.
- **A KORTEX-enabled install is relocatable** (#50, #108), the known limit
  listed under 1.0.0. The exported `kinova_lowlevelTargets.cmake` listed
  `<KORTEX_HW_DIR>/lib/release/libKortexApiCpp.a` as an absolute path, so a
  consumer whose SDK sat elsewhere failed at link time. The export now names an
  imported target, `kinova_lowlevel::kortex_api`, which
  `find_package(kinova_lowlevel)` creates from the consumer's `-DKORTEX_HW_DIR`,
  falling back to the build-time path. A missing SDK stops `find_package` with an
  error naming the path it tried. Sim-only installs and consumers on the build
  machine need no change.
- A trajectory target on the far wrap branch of a continuous joint no longer
  marches the impedance reference a full turn; home's `j3 = π` sits exactly on
  the boundary (#75).

## [1.2.0] — 2026-10-02

### Added

- `TrajectoryGoal::speed_scale` and a runtime speed override on the `Supervisor`
  (`on_set_speed_override`): execute a planned trajectory slower by dilating the
  executor's clock. The path is unchanged; velocity scales with `s`, acceleration
  with `s²`. Effective scale is the slower of goal and override, clamped to
  `[kMinSpeedScale, 1.0]` (`kMinSpeedScale = 0.01`; zero would hang the goal's
  clock forever), and changes are slew-limited so the commanded velocity never
  steps mid-goal; a goal or override outside that interval, including
  non-finite, is refused outright rather than clamped. Raising the runtime
  override past the arm's current effective speed requires the live
  arbitration token and is refused under an e-stop latch; lowering it never
  does (#69).

## [1.1.1] — 2026-09-22

A patch release: `JointVelocityMode` holds at a zero command. No signature or
field under `include/` changes, and `kinova-gen3-ros2` needed no adoption
commit.

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
  suite including `RtSafety*` passes on aarch64. Verified on the arm
  2026-09-22 with `velocity_hold_check`: 0.1 mrad of joint-2 drift over 60 s of
  streamed zeros (was ~2.3 rad); a stream cut mid-jog stops within the 0.2 s
  watchdog window (12 mrad at 0.054 rad/s); wrist jogs at 0.1, 0.3, 0.6 and
  1.0 rad/s track at 100% with the leash dormant. The measured lead grows
  linearly at about 70 ms of servo lag (0.071 rad at 1.0 rad/s), so the margin
  to the 0.1 rad leash at the 1.22 rad/s URDF cap is thin and loaded joints are
  uncharacterised; making the leash a parameter is a 1.2.0 follow-up. The
  `ee_twist` path was re-validated by gamepad teleop through the ROS2 node:
  still at zero twist, stops on release, tracks the stick.

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

[Unreleased]: https://github.com/rammp-org/kinova-gen3-driver/compare/v2.0.0...HEAD
[2.0.0]: https://github.com/rammp-org/kinova-gen3-driver/compare/v1.2.0...v2.0.0
[1.2.0]: https://github.com/rammp-org/kinova-gen3-driver/compare/v1.1.1...v1.2.0
[1.1.1]: https://github.com/rammp-org/kinova-gen3-driver/compare/v1.1.0...v1.1.1
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
