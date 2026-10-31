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

## Review reconciliation (2026-10-09, PR #74)

Decisions from the PR #74 review, now implemented — this section supersedes the
matching parts above:

- **Sign normalization is no longer deferred.** `KortexTransport` flips the
  reaction-signed feedback torque at the Transport boundary (the same place
  degrees become radians), so `JointFeedback::tau` / `ArmState::tau` carry ONE
  convention: the torque the actuator applies, ≈ `+g(q)` at free hold. The
  estimator residual is therefore `g(q) − τ`, the dry-run gravity check's
  "residual should be small" promise is true as printed, and no consumer needs
  the reaction-sign folklore. Breaking for anything that correlated raw `tau`
  with the model (CHANGELOG entry).
- **Damping λ: 1e-3 → 0.05.** The damped per-direction gain `σ/(σ²+λ²)` peaks
  at `1/(2λ)`; at 1e-3 that is a 500× cap, which lets the 1–2.5 N·m residual
  noise floor masquerade as hundreds-to-thousands of newtons near the candle
  pose — "bounded" but useless. λ = 0.05 caps amplification at 10× (noise floor
  ⇒ ≲25 N worst case) and biases working-pose estimates only a few percent.
  DiffIk's 1e-3 is NOT a precedent: DiffIk clamps its input before the solve,
  the estimator feeds raw noise in. `cartesian_test` pins the
  `‖F‖ ≤ ‖τ‖/(2λ)` bound and the ≤10% working-pose bias.
- **Fault ⇒ NaN.** Under `fb.fault` the pump publishes a NaN wrench instead of
  a confident number computed from possibly stale torque: runtime-checkable,
  matching the ROS "no measurement" convention downstream.
- **Sim is no longer meaningless.** `SimTransport` echoes the commanded torque
  in torque mode (its motor applies exactly what is commanded), so gravity
  residuals and the wrench read ≈0 under grav comp / impedance in sim. An idle
  (zero-torque) sim arm reads τ = 0 — a limp arm — which the estimator reports
  as the support wrench, faithfully.

## Addendum (2026-10-31, [#112](https://github.com/rammp-org/kinova-gen3-driver/issues/112)): the observer's wrench

This section supersedes "One quasi-static estimator" under Design wherever
contact detection runs. The wrench now comes from the momentum observer of the contact monitor (#109)
once it is trusted, and falls back to the quasi-static estimate otherwise.

- **Source.** With a `ContactMonitor` attached and `ContactStatus::settled` (the
  observer is past 5/K since its last re-init; not the same as `kArmed` or
  `kLatched`, since a run that restarts latched re-settles), the pump maps the observer residual: `F = ee_wrench_from_residual(J, r)`,
  `r` read from `ContactMonitor::status()` (the Seqlock the RT loop already
  stores each cycle; nothing new on the RT thread, no new lock). `J` is still
  the pump's Jacobian at `fb.q`, so the frame, model and λ above are unchanged.
- **Fallback.** No monitor (`contact_detection: off`), not settled (5/K = 200 ms
  after each `RtExecutor::run` start and each contact clear), a status not
  stored since the pump's previous iteration (`ContactMonitor::status_version()`
  unchanged: the RT loop is stopped, e.g. ArmLink in `kFaulted`, which still
  delivers feedback, so a frozen `r` would be published as live), or a torn
  status read: `ee_wrench_from_residual(J, g(q) − τ)`, exactly as before. Under a fault,
  NaN whichever the source. Neither `ArmState` nor `EeState` says which estimate
  a state carries: the conditions above decide it.
- **Sign.** Both are the external torque, positive as the environment applies
  it. The observer's dynamics are `M q̈ + C q̇ + g = τ + τ_ext`
  (`momentum_observer.h`), so at rest `r → g − τ`, the quasi-static residual.
  `ContactWrench.EstimatesAgreeOnAStaticPush` pins this on the sim.
- **Accuracy.** The quasi-static estimate drops `M q̈ + C q̇`, which reads as
  external force in motion; the observer models it. Force error at the flange,
  2026-10-30, position-mode tour at speed 0.6, nobody touching:

  | estimate | median | p95 | max |
  | --- | --- | --- | --- |
  | quasi-static | 3.3 N | 8.0 N | 13.7 N |
  | observer | 1.9 N | 5.4 N | 6.2 N |

  Model error is still there in both as a slow pose-dependent bias, and both
  read a held payload as external force (#111).
- **Lag.** The observer is a first-order filter at K = 25 rad/s (time constant
  1/K = 40 ms). On hand taps
  it trailed the quasi-static estimate by 23–25 ms (50 Hz data, ±10 ms). Fine for
  monitoring and thresholds at 50 Hz; a client doing fast force control would
  want the quasi-static estimate or a faster source.
