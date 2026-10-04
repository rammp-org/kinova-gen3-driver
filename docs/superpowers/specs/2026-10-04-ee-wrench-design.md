# Measured EE Wrench — Design Spec (#73)

## Goal

Give `ArmState` a measured external wrench at the tool, so `/ee_state` downstream
can finally publish one. `EeState.msg` says it itself: *"There is deliberately no
wrench here. The driver has no force estimate of its own"* — this spec is that
estimate. The gripper-tier spec deferred this to "its own spec"; this is it.

## Out of scope

- **The ROS2 surface.** `rammp_arm_interfaces/EeState.msg` grows a
  `geometry_msgs/Wrench wrench` field and `kinova_arm_ros2` copies six numbers from
  `ArmState::ee_wrench` — both separate repos, sketched at the end.
- **A dynamic observer.** No momentum observer (Haddadin et al.), no Coriolis or
  inertial terms. The estimator below is quasi-static; upgrading it later changes
  no field semantics, only accuracy at speed.
- **Friction / torque-offset calibration.** The measured residual floor (below) is
  accepted, not modelled away.
- **Sign-normalising `JointFeedback::tau` at the Transport boundary.** KORTEX's
  reaction-signed torque arguably should be flipped where degrees become radians
  (see fact 2), but `ArmState::tau` ships that raw sign to consumers today —
  changing it is a breaking decision for its own change, not a rider on this one.

## Facts established (not assumed)

1. **KORTEX's `tool_external_wrench` is dead in `LOW_LEVEL_SERVOING`.** Kinova,
   on [kortex#52](https://github.com/Kinovarobotics/kortex/issues/52): low-level
   servoing bypasses the Control library, so every URDF-derived base value —
   the tool wrench included — stops being computed. No workaround exists; only
   per-actuator torque sensing stays live. This driver lives in low-level
   servoing, so the KORTEX field is not an option at all. (The gripper-tier spec's
   out-of-scope note blamed the tool-frame offset; the real blocker is one step
   earlier — there is no data.)

2. **KORTEX actuator torque feedback is reaction-signed.** At near-rest on the
   lab arm (abra run `20260918T180357Z-2a13240537/impedance.csv`), measured
   torque ≈ **−g(q)**: j4 measured −12.127 N·m vs model gravity +12.168; j2
   +9.187 vs −10.197. So in the command/model convention (where grav-comp
   commands `+g(q)` and the arm holds — proven on-arm), the externally applied
   joint torque is `τ_ext = g(q) + τ_raw`, which that same data shows ≈ 0.04 N·m
   on j4 at free hold. Note: `benchmark_grav_comp --dry-run` prints
   `fb.tau − g(q)` and calls it "small if the URDF matches" — on real hardware
   that residual is ≈ 2|g|. Flagged here; fixing that instrument is separate.

3. **The residual noise floor is ~1–2.5 N·m on proximal joints** (same run: j2
   gap 1.0 N·m, j3 2.5 N·m at near-rest — friction plus model mismatch, the same
   unresolved gap as the torque-mode droop investigation). Mapped through the arm
   geometry that is a few Newtons at the tool. This estimator is
   **contact-detection / monitoring grade, not force-control grade**, and the
   docs must say so.

## Design

One quasi-static estimator, computed in the Supervisor's pump (non-RT, 100 Hz)
from the same feedback sample and the same URDF model as `ee_pose`/`ee_twist`:

```
τ_ext = g(q) + τ_raw                      // external joint torque, command convention
F̂    = (J Jᵀ + λ²I)⁻¹ J τ_ext            // damped pseudoinverse of Jᵀ, λ = 1e-3
```

- **Statics check:** `τ_motor = g − Jᵀ F_ext` at rest and `τ_raw = −τ_motor`, so
  `g + τ_raw = Jᵀ F_ext` and `F̂ ≈ F_ext` exactly when the wrench is in range of
  `Jᵀ`; the damping bounds the estimate near singular poses instead of letting a
  tiny residual explode into a huge phantom force.
- **Sign convention: `F̂` is the wrench the ENVIRONMENT applies to the tool**,
  `[force; torque]` in N / N·m. Press down on the EE and `force.z` goes negative.
- **Frame: `LOCAL_WORLD_ALIGNED` at the tool** — same frame as `ee_twist`, same
  `fk`/`jacobian` doctrine: one model, one sample, all three fields agree by
  construction.
- **Placement:** the map `(J, τ_ext) → F̂` is Eigen-only math, so it lives in
  `cartesian.h` beside `pose_error` as a free function
  (`ee_wrench_from_residual`), unit-testable without threads. The pump supplies
  `g(q)` from `pump_dyn_` and reuses the `pump_J_` it already computes for
  `ee_twist`. The RT path is untouched — no new RT-safe surface, nothing for
  `rt_safety_test` to re-prove.
- `ArmState` grows `Vector6 ee_wrench` (zero-initialised, like `ee_twist`).

### Sim

`SimTransport` echoes its seeded feedback and never synthesises torque, so in sim
`τ_raw` is whatever the fixture set (default zero) and the published wrench is the
phantom image of `+g(q)` — meaningless, the same caveat class as the sim gripper's
modelled effort. Tests inject `τ_raw = −g(q) + Jᵀ F` directly; nothing downstream
may calibrate against the sim wrench.

### On-arm validation (gate before the field is trusted)

1. Free hold under grav comp: `‖force‖` within a few N, `‖torque‖` under ~1 N·m.
2. Push the EE down / pull up / push along base +x: `force` tracks −z / +z / +x
   in sign and roughly in magnitude.
3. Hang a known mass from the tool (1 kg ⇒ 9.81 N on −z) and check magnitude.

### Downstream sketch (separate repos)

- `rammp_arm_interfaces/EeState.msg`: replace the "deliberately no wrench"
  paragraph with a `geometry_msgs/Wrench wrench` field documented as above
  (environment-on-tool, world-aligned at the tool, quasi-static accuracy caveat).
- `kinova_gen3_ros2/ros2_backend.cpp`: copy `s.ee_wrench[0..5]` in the same
  pump-tick publish that already fills pose and twist.
