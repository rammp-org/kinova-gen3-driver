# Gravity Model Identification — Design Spec

**Date:** 2026-10-20
**Status:** Phases D, I, L and V run on dojo, 2026-10-20 (see **Results**). Phase E not
started. Landed as: `KortexTransport::set_torque_offset` (#91, folded into this PR).
**Scope:** A multi-phase campaign to find out what is wrong with the arm's
gravity model and correct it: diagnose, identify, land, validate. This spec
covers the whole campaign. Its first implementation pass is offline tooling
only (pose grid, fitting, URDF writer), proven on synthetic data.

## Results (2026-10-20, dojo)

This section supersedes the hypothesis ranking and the initial gate numbers further down,
which are kept as the record of what was expected before the data.

- **The cause was torque-sensor offsets, not masses.** From a 49-pose grid, each pose
  approached from both sides (98 holds), the best fit (family F1) gives
  `-0.225, 1.065, 1.906, 0.475, -0.079, 0.007, 0.221` N·m. A read-only candle measurement
  agrees within 0.15 N·m on every joint except j6. The production URDF's tip needs
  only −34 ± 3 g. The in-repo fixture's camera and 0.5 kg mount are wrong for dojo.
  Hypothesis 1 (tip mass) is refuted for production; hypothesis 2 (offsets) is confirmed.
- **The actuators' torque loop tracks the biased sensor**, so the correction belongs
  in the command: send `tau + offset`. This settles open decision 1, by way of
  option (ii).

  | 3 s gravity comp at damping 10, from Home | j3 drift | EE drift |
  |---|---|---|
  | no calibration | −11.35° | 81 mm |
  | `--torque-offset`, through #91's Transport | +0.01° | 28 mm |
  | j3 and j7 offsets only | −0.32° | 21 mm |

  At damping 0 (2 s) only j2 drifted (+6.8°). Every other joint was held by friction.
  j2's static offset overcorrects in short runs. Whether to keep j2 and j4 is a
  per-robot choice.
- **The sensors see joint friction:** returns to candle from ±10° differ by 1.0 N·m on
  j2. Static readings have a floor of about 0.4–0.6 N·m on j1–j3 that no gravity
  model explains: held-out RMS after offsets is 0.60 on j2 and 0.62 on j3. The
  static-gate limits below (0.5 N·m) are tighter than this floor and are replaced
  by it. Approaching every pose from both sides is mandatory.
- **The startup droop is not model error.** `set_actuator_modes` sends zero torque
  for about 160 ms while it switches the actuators one at a time (#92).
- **Collection incident:** the automated grid (`onarm/run_grid.sh`) drove the gripper
  into the table at its last approach waypoint and self-collided several times. Its
  height check covered only joint origins and the EE frame, and not the approach
  waypoints. No more automated grids until a real collision checker gates every
  target and path.
- **The new-arm procedure** is `docs/integration/torque_offset_calibration.md`.
- **2026-10-21, j7 full turn** (30° steps, both sides, j1–j6 fixed): no mass sits off
  the j7 axis (fit ≈ 1 g·m, flat j5 and j6). j7 reads `0.343 + 0.165·sin(3θ + φ)` N·m,
  with friction 0.04. The ripple is mechanical, in an old gearbox, and is accepted
  as is. The F1 j7 offset (0.221) sampled it at whatever angles the grid hit, so the
  full-turn mean replaces it. That shortfall explains the −0.6° j7 settle #94 saw in joint impedance.
- **Wrist payload fitter** (`gravity_id wrist`, for the coming wrist camera). Static
  j5–j7 data cannot separate a payload's mass from its distance along the tool axis.
  Fitting both leaves about 3 kg of uncertainty, and j1–j4 carry about 0.5 N·m of friction.
  So the mass comes from a scale, and j7 sweeps at two or more j6 angles fit m·c in the
  joint-7 frame, together with the j5–j7 offsets and j7 ripple harmonics 1–3. At one j6
  angle a 1-per-turn ripple is indistinguishable from an off-axis mass. Across j6 angles
  the mass's torque scales with the tilt of j7's axis and the ripple's does not. A
  design with column-scaled condition number > 100 is refused: one j6 angle, angles too
  close together, or angles mirrored about where the tool axis is horizontal, which are
  singular at any spacing. `gravity_id wrist --plan` runs that check before the arm moves. Every pose is approached from both sides on j6 and j7: j6 friction
  that differs between sweeps reads as m·c_z (±0.05 N·m moved a 150 g COM by about 140 mm).
  The result lands as a corrected URDF (body 7 via the Phase L writer).

## Why

Gravity compensation and Cartesian impedance both drift on the real Gen3.
Every mode that commands torque adds `g(q)` from `Dynamics`, and the EE wrench
estimator (#74) subtracts it. If `g(q)` is wrong, all of them are wrong in the
same way, and every stack above the driver inherits it. Tuning numbers by hand
until the arm stops drifting would hide the error rather than measure it. This
campaign measures it first.

## Goals

1. Separate the two causes of the 2026-08-28 drift that were never separated:
   (a) the gravity model is wrong, (b) `JointTorqueParams::damping = 0` leaves
   the arm neutrally buoyant, so any residual integrates into motion.
2. Localize model error: unmodeled tip payload, wrong distal or proximal link
   inertials, per-joint torque-sensor offsets, or a mount-tilt error.
3. Identify corrected parameters that stay physically consistent, and land them
   where the driver already reads its model (the URDF), with nothing added to
   the RT path.
4. A before/after validation gate with numbers.

## Non-goals

- **Dynamic identification** (inertia tensors, Coriolis, friction models).
  Phase E below is an optional follow-up with its own spec, if the static
  result leaves impedance tracking error that only dynamics explains.
- **Changing `JointTorqueParams::damping`'s default.** The campaign measures how
  much damping the corrected model needs. Changing the default is a separate,
  user-visible decision.
- **Friction compensation in any control mode.**
- **The EE wrench estimator's math.** It improves automatically when `g(q)`
  improves (see Coordination).

## Anti-goals (a technically working result that would still be a failure)

- A fit that drives the residual to zero by giving a link negative mass or a
  center of mass outside the arm. Such a fit has absorbed sensor offsets or
  friction into fictitious inertia. It would pass the static gate at the poses
  it was fit on and then fail at other poses.
- Any new code in `compute()`, the executor cycle, or `Dynamics::gravity()`.
- A correction that lives only on one machine (an untracked URDF on abra) and
  not in version control.
- Passing the gate on the same poses used for fitting. Validation uses
  held-out poses.

## Facts established (not assumed)

1. **Torque sign is normalized since #74.** `KortexTransport` flips KORTEX's
   reaction-signed torque at the Transport boundary
   (`src/kortex_transport.cpp:94`), so `JointFeedback::tau ≈ +g(q)` at free
   hold. The `benchmark_grav_comp --dry-run` residual (`fb.tau − g`) and its
   "should be small" text are therefore **correct today**. This changes the
   2026-09-18 evidence: that CSV predates the flip, so its raw values are
   compared after negating them.
2. **There are two different arm models, and they disagree by up to 4.7 N·m.**
   The in-repo fixture `models/gen3_7dof_2f85.urdf` and the production model
   the dojo arm actually runs (rendered by kinova-gen3-ros2 `:nightly` from
   `rammp-deployments/.../robots/dojo/mount.yaml`, root `arm_mount`, EE
   `arm_end_effector_link`, level) have identical link inertials for bodies
   1–6. They differ only at the tip:

   | Body 7 (everything past joint 7) | Fixture | Dojo production |
   |---|---|---|
   | bracelet + 2F-85 (0.364 + 0.921 kg) | yes | yes |
   | `wrist_mounted_camera_link` | 0.072 kg | absent |
   | `wrist_camera_mount_link` | **0.5 kg** (a round number, likely a placeholder) | absent |
   | custom fingertips (#79) | absent | absent |
   | **total** | **1.857 kg** | **1.285 kg** |

   Over 200 random poses the two models' `g(q)` differ by up to 4.7 N·m. Both
   recorded residual runs were scored against the **fixture**. Neither model is
   known to match what is physically on the wrist.
3. **Static residuals exist, scored here against both models.** The 2026-09-18
   `impedance.csv` (pre-#74, so measured torque negated) has 3 at-rest rows,
   all at one pose (q ≈ −13.6, 6.2, −135.4, −90.9, 65.3, −27.2, 151.6°; |q̇| <
   0.005 rad/s; the impedance spring was active):

   | Joint | Measured τ | g fixture | g dojo | Residual vs fixture | Residual vs dojo |
   |---|---|---|---|---|---|
   | j2 | −8.97 | −11.03 | −9.26 | **+2.14** | **+0.36** |
   | j3 | +3.51 | +1.06 | +0.86 | +2.43 | +2.63 |
   | j4 | +11.99 | +13.26 | +10.84 | −1.26 | +1.16 |
   | j5 | +1.27 | +1.49 | +1.15 | −0.22 | +0.12 |

   The 2026-10-09 read-only dry-run (fixture, a different "retract-ish" pose,
   q not recorded) gave j2 −11.8 vs −15.1 (+3.3) and j4 +8.7 vs +9.4 (−0.7).
   This is one or two poses' worth of evidence. It is enough to rank
   hypotheses, but not enough to fit anything.
4. **KORTEX `tool_external_wrench` is dead in low-level servoing**
   (kortex#52). It cannot be an independent reference.
5. **The stock 2F-85 inertials match Robotiq's description bit for bit**
   (0.920864 kg total). The custom fingertips (#79) are not modeled in either
   model.
6. **`Dynamics` reduces any loaded URDF to the root→EE chain**, locking the
   gripper joints at `q = 0` (open) and merging their mass into body 7. So the
   gravity model has exactly 7 rigid bodies, and anything rigidly attached past
   joint 7 (fingertips, camera, camera mount, bracelet) is one body. Static data
   cannot tell those parts apart, and gravity does not need to.
7. **The dojo arm is level** (`mount.yaml` `rpy_deg: [0, 0, 0]`). On a level
   mount, joint 1's axis is vertical and gravity never loads it. So body 1's
   parameters, and some first-moment directions of the others, are
   unidentifiable from static data. They stay at their URDF values (see the
   prior below). "F4 chosen" therefore means "the identifiable combinations
   changed", not "every link was corrected".

## Hypothesis ranking

Ranked by the evidence in hand. The diagnostic phase is designed to confirm or
kill each one.

0. **Cause (a) is real; cause (b) amplifies it.** A static residual of 1–3 N·m
   at rest cannot come from `damping = 0`. Damping only acts on velocity, and
   the arm was not moving. So drift at `damping = 0` is not the whole story:
   the arm is being commanded the wrong gravity. `damping = 0` is real too, but
   it turns that error into motion; it does not create it. The open question
   is which part of the model is wrong, ranked below.

1. **Wrong tip mass: the payload past joint 7.** This is the strongest
   suspect. The two models in use disagree by 0.57 kg at the tip (fact 2), and
   that difference alone moves `g` by up to 4.7 N·m. At the one logged pose,
   the lighter production model explains j2 (0.36 vs 2.14 N·m), and j4's truth
   falls between the two (+1.16 vs −1.26). Both readings say the true tip is
   heavier than 1.285 kg and lighter than 1.857 kg. That fits a camera and
   fingertips being physically present, with the 0.5 kg mount placeholder too
   heavy. The 2026-08-28 "end-effector droops" fits too, if that run's model
   under-predicted the tip. **D3 (weighing the parts) tests this with no model
   at all.**
2. **Per-joint sensor offset or stiction, at least on j3.** j3 reads 2.4–2.6
   N·m off under *both* models, at a pose where gravity loads j3 with only
   about 1 N·m. No tip mass explains this, because the two models agree on j3
   to within 0.2 N·m. Two things remain. One is a constant offset in j3's
   torque sensor. The other is stiction the sensor can see: that row was held
   by the impedance spring, so a joint resting against friction reads gravity
   ± friction. Whether gearbox stiction shows on the sensor depends on where
   the sensor sits in the actuator, which is unknown. **D2 (approach from both
   sides) separates offset from stiction. D1's offset columns separate offset
   from mass.**
3. **Wrong link inertials (bodies 1–6).** There is no specific evidence for
   this. The two models share these values from Kinova's description. Phase I
   tests it as family F3/F4 and keeps it only if it beats F2 on held-out
   poses.
4. **Mount tilt.** Ruled out for dojo: `mount.yaml` is level, and the residuals
   are a few N·m, not the tens a wrong 30° plate gives. A tilt of a few degrees
   would show as pose-dependent j1 torque. D1 checks for this for free.

## Phases

```
D  Diagnose      read-only pose grid, repeatability, hysteresis      (on-arm, Swapnil)
I  Identify      offline fit of nested model families, held-out set  (offline)
L  Land          corrected URDF, version-controlled at its source    (offline + review)
V  Validate      static gate + attended drift test, before/after     (on-arm, Swapnil)
E  (optional)    dynamic excitation, if V leaves dynamic error        (own spec)
```

### Phase D — Diagnose (read-only, on the arm)

The data vehicle is the existing `benchmark_grav_comp --dry-run`. It connects,
never enters low-level servoing, never commands torque, and prints `q`, measured
`τ`, and `g(q)` per joint at 2 Hz. Kinova's own position controller holds each
pose. That controller does not depend on our model, so the reading at rest is
the link-side load, whatever we believe about the mass model.

**Call (mine): parse the dry-run transcript rather than add a CSV flag.** It
runs on the binary already deployed on the Jetson, with no rebuild. Its
precision (0.01° and 0.01 N·m) is far finer than the residuals in question.
The parser is pinned by a test against the exact `printf` format. A `--log`
flag can replace it later if the format proves fragile.

The transcript also lets the fit check itself. Every table carries the dry-run's
own `g(q)`, so the fitter recomputes `g(q)` from the URDF it was given. It
refuses to run if the two differ by more than 0.02 N·m (wrong URDF), or if
measured torque anti-correlates with gravity (a pre-#74 binary).

Three measurements, in one attended session (see the runbook in the plan):

- **D1 — pose grid.** 40 fit poses plus 10 held-out validation poses,
  generated offline (below), each held ≥ 5 s. The home pose is repeated at
  the start, middle and end of the session to measure repeatability and
  thermal drift in the sensor offsets.
- **D2 — hysteresis.** At 3 poses, approach j2 and j4 from +5° and from −5°.
  Half the difference in held torque estimates the stiction the sensor sees.
  If it exceeds 0.3 N·m, every grid pose is collected from both directions and
  averaged, and the session is re-run.
- **D3 — independent mass check.** List what is physically mounted past
  joint 7 (camera, camera mount, custom fingertips, cables, anything else).
  Weigh every part that can come off (kitchen scale, ±5 g), and note roughly
  where each sits relative to the flange. This is model-free ground truth for
  hypothesis 1. On its own it may already settle which of the two models' tips
  is closer.

Constraints on the session: arm warmed up ~10 min, gripper **open** for every
pose (the model locks the fingers at `q = 0`), nothing touching the arm, the
live driver stack stopped. The arm runs under Kinova's high-level control only.

### Pose grid (offline)

Candidates are sampled uniformly within the URDF joint limits, with a 10°
margin and an operator-set j1 sector for workspace clearance. A candidate is
rejected unless every joint-frame origin and the EE sit above `z_min` along the
model's up direction (the opposite of `model.gravity`, so a tilted mount is
handled). From the survivors, **greedy D-optimal selection** picks the 40 fit
poses that maximize `log det(AᵀA)` of the scaled regressor of the largest model
family. The 10 validation poses are drawn at random from the rest, so they are
not cherry-picked to be easy. Poses are ordered nearest-neighbour from home to
cut travel. Output is a CSV with pose id, set (fit/val), and joint angles in
degrees, ready to type into the Kinova web app.

### Phase I — Identify (offline)

**Model.** Gravity torque is linear in each body's mass and first moment:

```
τ_meas(q) = Y(q) π + o + ε
```

`π` holds `(m, m·c_x, m·c_y, m·c_z)` for each of the 7 bodies of the reduced
model. These are the first four of Pinocchio's ten dynamic parameters. `Y(q)` is
those columns of `pinocchio.computeJointTorqueRegressor(q, 0, 0)`. `o` is a
per-joint constant sensor offset, and `ε` is noise. Inertia tensors do not
appear when the arm is at rest, so they cannot be identified here and are left
unchanged.

**Fit relative to the URDF, as a Bayesian ridge.** The fit solves for the
correction `Δπ, o` against the residual `r = τ_meas − g_urdf(q)`. Each
parameter has a prior centred on "no change": σ_m = max(0.2 kg, 30 % of the
body's mass), σ_mc = 0.1 m × σ_m, σ_o = 2 N·m, with measurement noise σ_τ =
0.1 N·m. This is ordinary least squares on an augmented system. Directions the
data pins down are set by the data. Directions it cannot see (the
unidentifiable combinations a 7-DOF serial arm always has) stay at the URDF
values instead of wandering. This is how the result stays physically plausible
without a constrained solver. The posterior covariance gives each parameter a
standard error.

**Hard physical checks after the fit, which fail loudly:** every body's
corrected mass > 0, and every corrected COM within 0.5 m of its joint frame.
If a fit violates them, that model family is wrong for the data. It is reported
as rejected, not clipped into compliance.

**Nested families: the smallest correction that reproduces the torques.** Each
family is fit on the 40 fit poses and scored on the 10 held-out poses:

| Family | Free parameters | What it can reproduce |
|---|---|---|
| F0 | none (URDF as-is) | baseline |
| F1 | 7 offsets | sensor offsets only |
| F2 | offsets + body 7 (4) | a wrong tip, *or anything a tip correction mimics* |
| F3 | offsets + bodies 5–7 (12) | errors anywhere from the elbow out |
| F4 | offsets + all 7 bodies (28) | everything static |

The chosen family is the **smallest one whose held-out RMS residual is within
10 % of the best family's**. The report prints every family's held-out residual
per joint, so the reader sees the whole comparison, not just the winner.

**What a family does not tell you: which physical body is wrong.** Static
gravity data cannot attribute mass to a particular link. This was found while
building the tooling and is pinned by its tests. A 15 % error in body 3 or
body 4 is reproduced to the noise floor by F3, which only changes bodies 5–7.
A 15 % error in body 5 is reproduced by F2 alone. The driver does not need the
attribution: gravity compensation only needs `g(q)` right, and any correction
that reproduces the torques on held-out poses gives that. Attribution matters
for fixing the *model source* correctly (which part to re-measure, which repo
to edit). For that the evidence is D3, the weighed parts, not the fit. If F2
wins and its fitted mass agrees with D3's weights, the tip explanation stands.
If they disagree, the correction still works for control, but the root cause
is elsewhere, and the report says so instead of naming a part.

**Sensor gain** (a measured/model ratio that differs from 1) is not a family in this first
pass. If F4's held-out residual still scales with `|g|` per joint, a per-joint
gain column is added in a follow-up. It is a column in the same linear system,
not a new method.

### Phase L — Land

**The corrected parameters land in the URDF and nowhere else.** `Dynamics`
already loads the URDF at startup and fails loudly on a malformed one, so it
needs no code change, gets no new file format, and adds nothing to the RT path.

The fitter writes a corrected URDF. For each body it changed, it edits the
`<inertial>` of the heaviest URDF link in that rigid group: it sets the new mass
and moves the COM `xyz`, and keeps the link's rotational inertia about its own
COM, so the tensor stays positive-definite. For gravity this is exact, because
only the rigid group's total `m` and `m·c` matter. A round-trip test reloads the
written URDF through the same chain reduction and checks that it reproduces the
fitted `g(q)` to 1e-9.

Where it lands, in order:

1. **The production model's source.** The real arm runs the rendered
   `robot.urdf` from kinova-gen3-ros2 (mount included), not `models/`. The edit
   goes into that description, in that repo. The fitter's report prints the
   exact per-link changes so they can be applied there by hand and reviewed.
2. **`models/gen3_7dof_2f85.urdf`** in this repo. Arm-general corrections
   (bodies 1–6) go here as well. Its tip (camera plus a 0.5 kg mount) must
   either be reconciled with production or be labeled as a different payload,
   so the two models stop disagreeing silently. Arm-specific payload (this
   arm's fingertips) belongs with that arm's config.
3. **Per-joint offsets are not a URDF quantity.** If F1 or a larger family
   finds offsets above 0.3 N·m, where they land is an open decision for
   Swapnil (below). They are not silently folded into link masses.

### Phase V — Validate (on the arm, attended)

**Static gate.** These are measured on the 10 held-out poses, or on a fresh
held-out grid on another day, which is better:

| Joints | Held-out RMS residual | Held-out max residual |
|---|---|---|
| j1–j4 | ≤ 0.5 N·m | ≤ 1.0 N·m |
| j5–j7 | ≤ 0.2 N·m | ≤ 0.4 N·m |

If D1's measured repeatability is worse, each threshold becomes 2× the
repeatability for that joint, and the report says so. In addition, j2's RMS
must drop by ≥ 50 % against the F0 baseline. If F0 already passes, the
campaign's answer is "the model is fine and the drift is cause (b)".

**Drift test.** This is the only phase that commands torque. Swapnil runs it
attended with a hand near the arm. `benchmark_grav_comp --duration 5
--damping 5.0` is run at 5 named mid-workspace poses, with the original URDF
and then the corrected one. Joint angles are read with dry-run before and after
each run, and the fitter computes per-joint Δq and EE displacement through FK.

Above stiction, a joint drifts at about residual/damping. So 0.5 N·m left on
j2 at `damping = 5` still means about 6°/s. What actually holds the arm still
at the static gate's residuals is stiction, whose size we do not know. The
static gate is therefore the model-accuracy number. The drift test measures
what a user feels, and it is judged relative to the original model:

- **Pass:** at every one of the 5 poses, the corrected model's EE displacement
  over 5 s is ≤ 25 % of the original model's, and ≤ 3 cm in absolute terms.
- **Separation of (a) and (b):** once the corrected model passes, repeat at
  `damping = 0`. The extra drift relative to `damping = 5` is cause (b). Its
  size feeds the separate damping-default decision.

`damping = 5` and not 2, because the original model's 2–3 N·m residual at 2
would drift at about 1 rad/s, which is too fast to run attended.

All gate thresholds are initial numbers chosen before data. D1's repeatability
may loosen them, and any change is written into this spec with the reason.

### Phase E — Dynamic excitation (optional, later)

Only if V passes statically but Cartesian impedance still shows tracking error
that grows with speed. It needs the joint-velocity/acceleration logging that
does not exist today, plus a friction model. It gets its own spec.

## Offline tooling (first implementation pass)

A uv Python project at `tools/gravity_id/`. It uses `pin` (pinocchio, pinned to
the same 3.9.0 the robot and CI run) and `numpy`, and is tested with `pytest`.
It is not part of the CMake build and touches no C++.

| Command | Input → output |
|---|---|
| `posegrid` | URDF, EE frame, j1 sector, z_min → `poses.csv` (fit/val) |
| `parse` | dry-run transcript(s) + `poses.csv` → `holds.csv` (median q, τ, std per held pose, matched to pose ids) |
| `fit` | URDF + `holds.csv` → report (per-family per-joint held-out residuals, parameters ± std, checks) + corrected URDF |

It loads models the same way `Dynamics` does: it reduces the URDF to the
root→EE chain with gripper joints locked at `q = 0` and packs continuous joints
as `(cos, sin)`. A test pins Python `g(q)` to the C++ dry-run's printed `grav`
values.

**Proven on synthetic data** before any arm data exists. Truth is the URDF plus
injected errors, with Gaussian noise σ = 0.05 N·m:

- tip payload +0.25 kg off-axis, plus offsets → F2 chosen; mass within
  0.02 kg, COM within 1 cm, offsets within 0.05 N·m;
- a proximal link (body 3) 15 % heavy → F2 cannot reach the noise floor,
  F3 or F4 is chosen, held-out RMS ≤ 2σ;
- a wrist link (body 5) 15 % heavy → F2 is chosen and reaches the noise floor.
  This is pinned so nobody reads "F2 chosen" as proof of a tip payload;
- no error → F0 or F1 chosen (no phantom payload);
- the corrected URDF round-trips to the fitted `g(q)`.

## Coordination with the EE wrench estimator (#74)

The estimator's residual is `τ_ext = g(q) − τ`. Its 1–3 N·m floor *is* this
campaign's static residual, so a passing V gate lowers the estimator's floor
directly, with no code change on its side. After L lands, the free-hold check in
its on-arm gate (`‖force‖` within a few N) is re-run as a second consumer-level
check. If offsets turn out significant and are corrected in Transport (an option
below), the estimator gets that correction for free as well.

## Baseline model

The baseline for every phase is **the production model**: the URDF rendered
for the arm being identified, with its EE frame (`arm_end_effector_link`). For
dojo that is the level render of `robots/dojo/mount.yaml`. Every D session, the
parity check, the fit and the corrected URDF use that file. The in-repo
fixture is used only by the tooling's own tests. Its tip disagrees with
production (fact 2). Reconciling the two is part of Phase L, once D3 says what
is physically on the wrist.

## Open decisions (Swapnil's)

1. **DECIDED (2026-10-20):** option (ii), at the Transport boundary
   (`KortexTransport::set_torque_offset`). The arm showed the inner torque loop does
   close on the biased sensor. Kept below as the record. **Where per-joint offsets
   land, if they matter (> 0.3 N·m).** Options:
   (i) Kinova's own per-actuator torque offset (ActuatorConfig, firmware-side,
   no driver change, but it changes the robot's stored state);
   (ii) a startup-loaded calibration at the Transport boundary, the same place
   the sign flip happens. Which one is right depends on something the data does
   not show: whether Kinova's inner torque loop closes on the biased sensor.
   This is decided once D/I report numbers.
2. **Which arms.** This spec assumes the dojo arm (abra). The flatbot arm needs
   its own D session with its own rendered URDF, because the payload differs
   per arm.
3. **Getting it into ROS 2:** a node parameter, with a per-robot value stored next to
   `mount.yaml`. This needs a driver release that includes #91.
4. **Whether to fix the in-repo fixture's tip** (camera and 0.5 kg mount), or label
   it as a different payload.

## Calls I made (please check)

- Transcript parsing instead of a new `--log` flag (no Jetson rebuild).
- The URDF as the only landing place for inertial corrections; no overlay
  format.
- Bayesian ridge toward the URDF instead of an LMI/SDP physically-consistent
  solver. With static data only `m` and `m·c` are in play, so mass > 0 and a
  COM bound are the whole consistency story.
- 40 fit and 10 held-out poses, with initial gate numbers; all can be revised
  after D1.
- Python under `tools/`, run with uv, kept out of CMake and CI for this pass.
