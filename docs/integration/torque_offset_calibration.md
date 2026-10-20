# Torque-Sensor Offset Calibration (new arm)

> **Attended.** Robot present, e-stop in reach. Steps 1–2 never command torque.
> Step 3 commands torque for 3 s.

Every Gen3 actuator's joint torque sensor reads with a constant offset. The
actuator's torque loop tracks its own sensor, so a joint asked for `τ` delivers
`τ − offset`, and gravity comp drifts. On dojo, j3's +1.9 N·m offset alone drifted
the elbow 11° in 3 s. `KortexTransport::set_torque_offset` compensates the
offsets ([API](../reference/api.md)), but the values belong to one particular arm
and have to be measured. Run this procedure:

- on every new arm;
- after any actuator is replaced, or its firmware is updated;
- whenever free-hold residuals (step 3) creep above about 0.6 N·m.

The scripts live in `tools/gravity_id/onarm/` (run them on the Jetson with host
`python3` and `PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python`). The offline tools
are in `tools/gravity_id` (run with `uv run python -m gravity_id`). The background is in the
[gravity identification runbook](gravity_identification.md).

## 0. Preconditions

- Nothing else drives the arm (`docker ps`; stop the stack).
- This robot's rendered `robot.urdf` and `--ee-frame arm_end_effector_link`
  ([how](../integration-runbook.md#mount-caveat)).
- Gripper open, arm warmed up for about 10 min, nothing touching it.

**Identify with no offset set.** `dryrun.sh` passes none. If the logger ran with
`--torque-offset`, the readings already have that calibration taken out, and the
tools return what is left over, not the full offset.

## 1. Candle, from both sides (about 5 min, read-only)

At candle (every joint at 0°) gravity loads no joint, so each sensor's reading **is**
its offset, with no model needed. The joint friction that the sensors see on the
Gen3 (±0.5 N·m on dojo's j2) means the reading depends on the approach direction,
so approach from +10° and from −10° and average the two. Start the logger, then
make the small moves with Kinova's own controller:

```sh
./dryrun.sh candle.txt 0 &          # read-only logger; Ctrl-C / docker kill it at the end
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
python3 move.py --angles=0,0,0,0,0,0,0     --speed 10 --hold 2
python3 move.py --angles=10,10,10,10,10,10,10 --speed 10 --hold 1
python3 move.py --angles=0,0,0,0,0,0,0     --speed 10 --hold 8
python3 move.py --angles=-10,-10,-10,-10,-10,-10,-10 --speed 10 --hold 1
python3 move.py --angles=0,0,0,0,0,0,0     --speed 10 --hold 8
docker kill --signal INT gid-dryrun
```

**Check the moves before sending them.** These are small motions around candle
and are safe on a clear bench. The first move to candle can come from anywhere,
so make that one with the Web App if the arm starts somewhere unusual. Then:

```sh
uv run python -m gravity_id candle --urdf robot.urdf --ee-frame arm_end_effector_link candle.txt
```

It prints the friction per joint (half the spread between the two approaches) and
a ready-to-paste `--torque-offset o1,...,o7`.

## 2. Cross-check with the pose grid (first calibration of an arm)

Candle measures each offset at one configuration. The full pose grid
([runbook](gravity_identification.md), D1 plus `fit`) measures them across the
workspace, and also checks the URDF masses. Family `F1` in the report gives the
grid's offsets. On dojo the two methods agreed within 0.15 N·m on every joint
except j6 (candle +0.38, grid +0.01). Where they disagree, prefer the grid value,
and confirm with step 3.

!!! danger "The automated grid is not collision-checked yet"
    On 2026-10-20 the automated grid (`run_grid.sh`) drove dojo's gripper into
    the table and self-collided several times. Its height check covered only joint
    origins and the EE frame, and never covered the approach waypoints. Until a
    real collision checker (table plus self-collision, along every path) gates
    `move.py`, step through the grid with the Web App, or review every target
    yourself.

## 3. Verify on the arm

Read-only first. With the calibration, the dry-run residual `τ − g(q)` should sit
within the friction band (≤ about 0.6 N·m on dojo) at any pose:

```sh
benchmark_grav_comp --ip 192.168.1.10 --dry-run --urdf robot.urdf \
    --ee-frame arm_end_effector_link --duration 2 --torque-offset o1,...,o7
```

Then the drift A/B, which commands torque: 3 s of gravity comp at damping 10, from
Kinova's Home, with and without the calibration. Use `torque.sh` in `onarm/`, or
`benchmark_grav_comp --duration 3 --damping 10 [--torque-offset ...]`. Take a
dry-run snapshot before and after each run, and compare them with
`gravity_id drift`.

| dojo, 2026-10-20 | j3 drift | EE drift |
|---|---|---|
| no calibration | −11.4° | 81 mm |
| all offsets | −0.4° | 31 mm |
| j3 + j7 only | −0.3° | 21 mm |

**Keep a joint's offset if it reduces that joint's drift.** On dojo, j2 and j4's
static offsets made their short-run drift slightly worse (j2 +1.1° → +3.2°): those
joints are friction-dominated. Hand-guided, though, the full set felt best. Record
which choice you made, and why.

## 4. Record

Store the values with the robot's own configuration, next to its `mount.yaml`, in
rammp-deployments. Include the date, the method (candle or grid), the actuator
firmware, and the per-joint keep/zero decisions. Until the ROS 2 node takes the
parameter, pass it to the standalone apps with `--torque-offset`.

dojo, 2026-10-20 (49-pose grid, family F1): `-0.225,1.065,1.906,0.475,-0.079,0.007,0.221`.
