# On-arm test session — v1.3.0 compliance stack (2026-10-05)

**Headline question: how do EE trajectories do under impedance?**
(GoToEEPose plans through cuRobo exactly as before; what's new is compliant
execution + feedforward + the gains contract on every surface.)

## 0. What's being tested (branch state, nothing released)

- driver `feat/63-compliant-everything` (base v1.2.0): gains contract (#63/#64),
  target feedforward, compliant velocity/twist pairs, benchmark_joint_impedance.
- rammp_arm_interfaces + kinova-gen3-ros2 `feat/compliant-everything`:
  GainsSpec messages, GoTo mode+gains, SetGains service, two new stream
  controllers. **Message changes = old and new nodes cannot talk; everything
  rebuilds together.**
- Rollback at any point: redeploy the released chain (driver v1.2.0 tag,
  ros2 v1.0.1+, images per rammp-deployments) — nothing on the arm persists.

## 0b. Workspace pinning (the chain is version-pinned; override it)

- `kinova_gen3.repos` pins driver **v1.2.0**, which does NOT compile against
  these branches (TrajectoryGoal retype). Point the workspace at the local
  driver branch via the Makefile's CORE_REF escape hatch:
  driver = kinova-gen3-driver `feat/63-compliant-everything`.
- Build rammp-interfaces-ros2 `feat/compliant-everything` IN THE WORKSPACE —
  the rammp-base image's compiled interfaces copy is 1.1.0 and lacks GainsSpec.
  (That branch is based on origin/main, not the stale local main.)
- kinova_arm_ros2 branch: `feat/compliant-everything` (off dev).
- colcon build + colcon test were NOT run anywhere yet (no ROS 2 on the dev
  laptop) — they are the FIRST gate tomorrow, before anything touches the arm.

## 1. Safety prechecks (before any torque)

- E-stop in hand; workspace clear; start from the home preset.
- `docker ps` on abra FIRST — the live driver container owns core 11. Never
  sync over /home/abra/kinova-gen3-driver; hil-sync to a distinct dir.
- NIC coalescing after any reboot: `ethtool -c eno1` → rx-usecs must be 8
  (chronic regression eats 80% of the 1 kHz budget; fix: rx-usecs 8 rx-frames 1).
- Driver unit suite + rt_safety_test ON THE JETSON before the arm moves:
  `ctest --test-dir build --output-on-failure` (aarch64 is the real platform;
  x86 green ≠ Jetson green).
- Jetson benchmark before/after percentiles for the feedforward delta:
  `./build/benchmark_joint_impedance --sim --rate 1000 --duration 10 --track`
  (record; x86 said "noise floor" — confirm on the RT platform).

## 2. Test matrix (escalating authority)

### A. Hold + defaults sanity (gravity floor + medium profile)
1. `execute_joint_trajectory` short move, `control_mode=1`, **gains empty**
   (PROFILE_SESSION_DEFAULT). Expect: medium gains, no droop at rest
   (watch for the known torque-mode droop signature — this is the spec §5
   "hold" criterion), clean completion.
2. Push the arm mid-hold: visible give, springs back. ("Give" criterion.)
3. Repeat with `profile=SOFT`, then `STIFF`: stiffness difference obvious by
   hand; leash tighter on stiff (0.25 vs 0.45 rad).

### B. Gains contract on the wire
4. Custom gains goal with torque_limit all-zero → **rejected at accept**
   (the #64 regression test, on the real stack).
5. Custom goal (kq=60, zeta=0.6, limits default) → runs; then a bare goal →
   back on medium (no-leak, verify via a second push test or SetGains echo).
6. `SetGains` service: set default=soft, bare impedance goal runs soft;
   `set_gains` with zero limits → refused with the floor message.

### C. EE trajectories under impedance (the headline)
7. GoToEEPose, position mode (baseline): record final error + path feel.
8. Same pose, `control_mode=IMPEDANCE`, default gains: compare tracking
   (feedforward should hold it near position-mode error at default speed;
   spec §5 "track" criterion — record the number, it becomes the documented
   bound), compliance on contact mid-move.
9. Same goal at `speed_scale 0.5` and `0.2`: tracking must not degrade
   disproportionately (scaled feedforward: qd·s, qdd·s²; the old bug shape
   would be lag exploding at low scale).
10. Longer cuRobo plan (across the workspace, through a wrist-heavy region):
    joints 5–7 have 9 N·m ceilings; watch last_damping/torque clipping in
    telemetry CSV.

### D. Compliant streams
11. `joint_velocity_impedance`: open, stream a slow constant qd, push the arm —
    it should yield and resume; stop streaming → deadline close → hold at
    measured q (close cause kDeadlineExpired).
12. `ee_twist_impedance`: slow Cartesian creep, same push test; verify no
    runaway when the hand blocks the arm (leash caps the reference).
13. Regression: `joint_velocity` (stiff) and `ee_twist` unchanged from v1.1.1
    behavior (hold/stale/track quick pass).

### E. Conformance + soak
14. ROS conformance suite from the repo (incl. new controllers, availability
    of pairs, cartesian_impedance still unavailable).
15. 10-min impedance hold soak: zero dropped samples, zero major faults, no
    thermal/overrun drift (telemetry CSV percentiles vs 1.2.0 numbers).

## 3. Tuning pass (spec §5 — the defaults ARE the deliverable)

If medium fails a criterion (droop / sluggish tracking / harsh contact):
retune `profile_params(kMedium)` in src/interface/gains.cpp, rebuild on the
Jetson, re-run A1–A3 + C8. The GainsFloor test re-validates floors after any
URDF/profile change. Record final numbers in the profile table comment and
CHANGELOG. Soft/stiff get a hand-feel pass only (they ship as starting points).

## 4. Exit criteria

- All of A, B, C green; D green or consciously deferred (velocity/twist
  compliant tiers are not on tomorrow's critical path for RAMMP).
- Tracking bound for impedance EE trajectories at default speed WRITTEN DOWN
  (goes into docs/guide/control-modes.md as the medium-profile promise).
- rt_safety + benchmark deltas on Jetson recorded in the PR.
- Anything red: file against the branch, decide fix-forward vs drop-from-1.3.
