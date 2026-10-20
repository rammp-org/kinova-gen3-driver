# Gravity Model Identification — On-Arm Runbook

> **Attended.** Robot present, e-stop in reach. Phase D never commands torque:
> the arm stays under Kinova's own control and the driver only reads. Phase V's
> drift test commands torque and needs a hand near the arm.

This is the data-collection side of the gravity identification campaign
([design](../superpowers/specs/2026-10-20-gravity-model-identification-design.md),
issues [#5](https://github.com/rammp-org/kinova-gen3-driver/issues/5) and
[#40](https://github.com/rammp-org/kinova-gen3-driver/issues/40)). The offline
side is `tools/gravity_id` (see its README). Every `gravity_id` command below runs
from `tools/gravity_id` with `uv run python -m gravity_id`, on any machine. Every
`benchmark_grav_comp` command runs on the Jetson.

## Preconditions

1. **Nothing else drives the arm.** Stop the stack and any scratch driver
   container (`docker ps` on the Jetson). The dry-run needs the arm in Kinova's
   normal high-level mode, not low-level servoing.
2. **The production model.** Render `robot.urdf` for this robot
   ([how](../integration-runbook.md#mount-caveat)) and use it, with
   `--ee-frame arm_end_effector_link`, for **every** command below, on the
   Jetson and offline alike. `parse` refuses a transcript taken with a different
   model. Check the `[model] mount tilt` line: 0.0 on dojo.
3. **Gripper open** for the whole session. The model locks the fingers at
   `q = 0` (open), so a closed gripper is a different payload.
4. **Warm up** about 10 minutes (arm powered, holding a pose), so sensor offsets
   are not drifting with temperature during the session.
5. Nothing touching the arm or its cables beyond their normal routing.

Generate the pose list offline once, with a j1 sector that keeps the arm clear
of the bench and its surroundings:

```sh
uv run python -m gravity_id posegrid --urdf robot.urdf --ee-frame arm_end_effector_link \
    --out poses.csv --j1-range=-90,90 --z-min 0.10
```

`poses.csv` holds 50 poses (`fit` and `val`), ordered to cut travel, with
angles in degrees. Every joint origin and the EE sit at least `z_min` above
the base, at the poses themselves. That is a geometric floor, not a
collision check. It checks neither the path the Web App takes between poses
nor the gripper and camera beyond the EE frame. **Look at each pose before
sending it, run the Web App at low speed, and watch every move.**

## D1 — the pose grid (≈ 45 min)

Start one dry-run for the whole session and keep its output:

```sh
./build/benchmark_grav_comp --ip 192.168.1.10 --dry-run --urdf robot.urdf \
    --ee-frame arm_end_effector_link --duration 0 | tee d1.txt
```

Then, in the Kinova Web App (Arm → angular control):

1. Move to **home** and hold 10 s.
2. For each row of `poses.csv` in order: enter the seven angles, let the move
   finish, then **hold ≥ 6 s with nothing touching the arm**. The dry-run prints
   every 0.5 s. The parser keeps a hold once it sees ≥ 5 s at rest, and drops the
   first 2 s as settling. On dojo, Kinova's hold torque takes about 2 s to settle
   after arriving.
3. After pose 25, go **home** and hold 10 s. After the last pose, go **home**
   again and hold 10 s.
4. Ctrl-C the dry-run.

```sh
uv run python -m gravity_id parse --urdf robot.urdf --ee-frame arm_end_effector_link \
    --poses poses.csv --out holds.csv d1.txt
```

Expect `50 matched`. A missed pose just drops out. The `repeat x3` line for
home gives the **repeatability**: the per-joint torque spread across the session.
If any joint's spread exceeds half its gate limit (below), the static gate for
that joint is loosened to 2× the spread. Record that in the spec.

## D2 — hysteresis (≈ 10 min)

This checks whether the torque sensors see joint stiction. Pick three `fit`
poses with large gravity load on j2 and j4. At each one, with a fresh
transcript:

```sh
./build/benchmark_grav_comp --ip 192.168.1.10 --dry-run --urdf robot.urdf \
    --ee-frame arm_end_effector_link --duration 0 | tee d2_P07.txt
```

1. Go to the pose with j2 and j4 both **+5°**, then to the pose itself. Hold 5 s.
2. Go to the pose with j2 and j4 both **−5°**, then to the pose itself. Hold 5 s.

Run `parse` on each `d2_*.txt` alone. The `repeat x2` spread is twice the
stiction the sensor sees. **If any spread exceeds 0.6 N·m (0.3 N·m of
stiction), stop.** A single-direction grid would bias the fit. Re-run D1 with
every pose approached from both sides, and average the two.

## D3 — weigh the tip (≈ 10 min)

List everything physically mounted past joint 7: camera, camera mount, custom
fingertips, cable clips, anything else. Weigh every part that comes off
(kitchen scale, ±5 g), and note roughly where each sits relative to the tool
flange. The fit cannot tell which body is wrong. These weights are what decide
whether its tip correction is really the tip.

## Fit (offline)

```sh
uv run python -m gravity_id fit --urdf robot.urdf --ee-frame arm_end_effector_link \
    --holds holds.csv --out-dir fit/
```

Read `fit/report.md`. It contains every family's held-out residual per joint, the
chosen correction (with the tip correction expressed as a mass in the EE frame,
to compare with D3), the URDF edits, and the static gate. Attach `report.md`,
`d1.txt` and the D3 weights to the campaign's PR or issue.

## V — validation

**Static gate.** `report.md`'s gate section judges `corrected.urdf` alone.
Fitted offsets are not in the URDF, so they are left out, and the report warns
when any offset exceeds 0.3 N·m. The stronger check is a fresh grid on another
day, with **`corrected.urdf` as the model throughout**: `posegrid --seed 1`,
then D1's dry-run with `--urdf corrected.urdf` (`parse` refuses a mismatch),
then `parse` and `fit` with it too. Read the **F0** rows: F0 is the corrected
URDF as-is. On that day the "j2 drops ≥ 50 %" rule does not apply, because it
compares against the original model, which is not in play. The limits are:

| Joints | Held-out RMS | Held-out max |
|---|---|---|
| j1–j4 | ≤ 0.5 N·m | ≤ 1.0 N·m |
| j5–j7 | ≤ 0.2 N·m | ≤ 0.4 N·m |

j2's RMS must also drop by ≥ 50 % against the original model (`F0`).

**Drift test (commands torque).** Do this at 5 mid-workspace poses, first with
the original `robot.urdf` and then with the corrected one. At each pose:

```sh
# 1. arm at the pose (Web App), then a 2 s read-only snapshot
./build/benchmark_grav_comp --ip 192.168.1.10 --dry-run --urdf robot.urdf \
    --ee-frame arm_end_effector_link --duration 2 > pre.txt
# 2. gravity comp for 5 s, hand near the arm
./build/benchmark_grav_comp --ip 192.168.1.10 --urdf robot.urdf \
    --ee-frame arm_end_effector_link --duration 5 --damping 5.0
# 3. snapshot where it ended up
./build/benchmark_grav_comp --ip 192.168.1.10 --dry-run --urdf robot.urdf \
    --ee-frame arm_end_effector_link --duration 2 > post.txt
uv run python -m gravity_id drift --urdf robot.urdf --ee-frame arm_end_effector_link \
    --before pre.txt --after post.txt
```

The pass criterion is the same at every pose: with the corrected model, EE
displacement over 5 s is ≤ 25 % of the original model's and ≤ 3 cm.
`damping = 5` because the original model's 2–3 N·m residual would drift at
about 1 rad/s at `damping = 2`. Once the corrected model passes, repeat it at
`--damping 0`. The extra drift is the `damping = 0` share of the problem.
