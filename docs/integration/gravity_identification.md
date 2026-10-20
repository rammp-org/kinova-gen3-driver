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

## Automated session (Kinova high-level moves)

!!! danger "The automated grid is not collision-checked"
    On 2026-10-20, `run_grid.sh` drove dojo's gripper into the table at an
    approach waypoint, and self-collided several times. Nothing in these scripts
    knows where the table is, or checks self-collision: not the pose grid's height
    floor, not `move.py`, and not Kinova's planner or `ValidateWaypointList`.
    **Do not run the grid automatically** until a real collision checker (table plus
    self-collision, along every path) gates `move.py`. Until then, step through
    the poses with the Web App, and review every target, including the ±5°
    approach waypoints that `--plan` lists.

`tools/gravity_id/onarm/` holds the scripts used on dojo on 2026-10-20. They run on
the Jetson with host `python3` and `kortex_api`, outside the uv project. **Copy the
scripts, `robot.urdf` and `poses.csv` into one session directory.** Each script
reads its files from its own directory, and the logger writes there too.

- `dryrun.sh OUT [S]`: the read-only logger (the nightly image's
  `benchmark_grav_comp --dry-run`).
- `move.py`: Kinova's own `reach_joint_angles`, the same as the Web App.
  - It holds the arm in single-level servoing, refuses if anything has it in
    low-level servoing, and calls `Base.Stop` on Ctrl-C, timeout or abort,
    including a move Kinova rejects while planning it.
  - It sizes each move's duration from the joint's **real** travel. Limited
    joints never wrap, so a −130° → +130° move counts as 260°.
  - It refuses a first move of more than 30° from the current pose unless
    `--from-anywhere` is given.
  - `--bidirectional 5` approaches every pose from +5° and from −5° (see D2).
  - `--plan` lists every move with its travel, duration and mean speed, and
    moves nothing.
- `validate.py`: Kinova's `ValidateWaypointList` on a `move.py --plan`, with
  no motion. It joins the whole list into one spline that does not stop at each
  waypoint, so it can report limit overshoots that the stop-at-target moves never
  make. Check any flagged segment's endpoints.
- `run_grid.sh`: candle → first half of the grid → candle → second half →
  candle, both sides at every pose, logging to a timestamped `d1_*.txt`. It
  refuses to start if a logger is already running, stops the logger however it
  exits, and ends the run if the logger dies. **Not collision-checked; see the
  box above.**

`move.py` uses a *duration* constraint (1.5× the time at `--speed`). On dojo, a
*speed* constraint made Kinova reject a 104° move with `METHOD_FAILED`.
Prefix every `move.py`/`validate.py` call with
`PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python`: the Jetson's protobuf is too new
for `kortex_api` 2.6.

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

1. Go to the pose with j2 and j4 both **+5°**, then to the pose itself. Hold ≥ 6 s.
2. Go to the pose with j2 and j4 both **−5°**, then to the pose itself. Hold ≥ 6 s.

(The parser drops the first 2 s of a hold and keeps it only if at least 3 s remain,
so a 5 s hold can be dropped without warning.)

Run `parse` on each `d2_*.txt` alone. The `repeat x2` spread is twice the
stiction the sensor sees. **If any spread exceeds 0.6 N·m (0.3 N·m of
stiction), stop.** A single-direction grid would bias the fit. Re-run D1 with
every pose approached from both sides, and average the two. Approach by hand in the
Web App. `move.py --bidirectional 5` does the same automatically, but it also
nudges **every** joint ±5°, and it is not collision-checked (see the box above). On dojo (2026-10-20), candle returns from +10° and from −10° differed
by 1.0 N·m on j2 (0.5 N·m of friction), so dojo always needs both sides.

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
when any offset exceeds 0.3 N·m. Offsets go into the arm's `--torque-offset`
instead ([calibration](torque_offset_calibration.md)). They are valid only if
the session was logged with no offset set: with one set, the fit returns what
is left over, not the full offset. The stronger check is a fresh grid on another
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
