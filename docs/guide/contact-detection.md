# Contact detection — tuning

The driver's `ContactMonitor` watches the momentum-observer residual `r` (the
estimated external joint torque, N·m) and trips when a joint stays over its
threshold `θ_i` for `debounce` consecutive cycles. This page is the procedure for
choosing `θ`, and the offline tool for doing it between arm sessions. The design
is in the [contact-detection spec](https://github.com/rammp-org/kinova-gen3-ros2/blob/main/docs/superpowers/specs/2026-10-29-contact-detection-design.md).

## Choosing thresholds

1. **Measure the motion floor.** With the monitor in `observe` mode and nobody
   touching the arm, run the demo motions at demo speed and record the peak `|r_i|`
   per joint. Set `θ_i(default) = max(m × peak_i, small absolute minimum)`, with
   `m` ≈ 3 to start.
2. **Pose check.** From logged `q` along the feeding approach, for contact points
   on the tool and forearm, find the smallest force (worst direction) that crosses
   some `θ_i`. That is the detector's real force threshold at that pose. If it
   exceeds the target anywhere, lower `θ` or change the approach. There is no tool
   for this yet.
3. **Blind spot.** A force along a line through the joint axes (near-extended arm,
   push along the arm) produces almost no joint torque. The pose check should
   report where the approach goes near such poses; the fix there is the approach.

## contact_replay

`contact_replay` runs a recorded CSV through the same `ContactMonitor` (observe
mode) the RT loop uses, so a threshold change can be tried without the arm. It is
built with the other apps (`cmake --build <dir> --target contact_replay`).

```
contact_replay --urdf arm.urdf --ee-frame arm_end_effector_link \
    --csv tour.csv --out r.csv \
    [--gain 25] [--debounce 5 | --debounce-ms 5] [--thresholds 16,16,12,12,4,4,4] \
    [--tau-sign -1]
```

`--ee-frame` is the end-effector link in the model you pass: the rendered
`robot_description` names it `arm_end_effector_link`, the repo's test URDF
(`models/gen3_7dof_2f85.urdf`) `gen3_end_effector_link`.

- **Input:** a CSV with a header naming `t, q1..q7, qd1..qd7, tau1..tau7` (extra
  columns are ignored). `tau` must be in command convention, where a still arm
  reads `tau = g(q)`. A raw core v1.2.0 recording has the opposite sign: pass
  `--tau-sign -1`.
  Every `t, q, qd, tau` field must be a finite number (no `nan`, no `inf`):
  anything else stops the replay with the row and the column. Rows further apart
  than `1/gain` are stepped as `1/gain`, as the RT monitor steps a gap.
- **Output:** `r.csv` with `t, r1..r7`.
- **Stdout:** per joint, RMS and peak `|r|` over the moving samples (max `|qd|` >
  0.02 rad/s, after the `5/gain` settle), the first would-be trip, and the total
  `would_trips`.

!!! warning "Debounce is counted in rows"
    The RT monitor debounces 5 cycles at 1 kHz, which is 5 ms. `--debounce N`
    counts recording rows, so on a 50 Hz recording it means 100 ms and
    `would_trips` is silently undercounted. Use `--debounce-ms 5`: it converts to
    rows from the median row spacing (at least 1 row, so 1 row on a 50 Hz
    recording). The tool prints the effective debounce in rows and ms, and warns
    when the median dt exceeds 2 ms, since a slow recording cannot reproduce the
    arm's timing exactly. `--debounce` and `--debounce-ms` are mutually exclusive.

### Parity with the Python reference

`tools/contact/parity.py` runs the Python reference observer over the same
recording and compares it with `contact_replay`'s output. It exits 1 if any joint
differs by more than 0.05 N·m after the first `5/gain` seconds.

```
uv run -q --no-project --with pin --with numpy python -I tools/contact/parity.py \
    --observer-py observer.py --csv tour.csv --urdf arm.urdf --cpp r.csv [--gain 25] [--tau-sign -1]
```

## On-arm bench steps

Each motion batch is confirmed with Swapnil before it runs.

Through the ROS node, `would_trips`, the active `θ` and the last trip record are
in the `Contact detection` task on `/diagnostics`. To record `r`, start the node
with `contact_debug_topic:=true`: it publishes `r` at 50 Hz on
`/contact_monitor/residual` (`sensor_msgs/JointState`, `effort` = `r` in N·m).
Record that topic with `/joint_states` and `/ee_state` for the steps below. See
the node's [interface doc](https://github.com/rammp-org/kinova-gen3-ros2/blob/dev/docs/interface.md#contact-stops).

1. `observe`, no motion, 60 s: the still floor per joint.
2. `observe`, the tour at speed 0.6, then 1.0, nobody touching: motion RMS and
   peak per joint, plus `would_trips`. Set `default` to about 3x the peak, rerun,
   and expect `would_trips == 0`. When replaying a 50 Hz recording, pass
   `--debounce-ms 5`.
3. `observe`, still arm, slow hand pushes on the forearm and the tool: record `r`
   against the `/ee_state` wrench, which is exact at rest. This maps `θ` to force.
4. `enforce`, slow move, hand push: trip, float, latch, `-11` on the goal,
   `/clear_faults`, hold. Then at demo speed.
5. After a trip: force build-up, the position-mode to float window (look for the
   #92 droop), and the RT cycle time with and without the monitor (`--rt-csv`).
   Also measure the observer transient at position↔impedance mode switches:
   the monitor steps the observer once over the `set_actuator_modes` handshake,
   capped at `1/K` (40 ms), so look at `r` across each switch.
6. `observe` with real payloads (spoon, container): `r` while carrying. Then
   `enforce` with the same payloads: trip and check float with a payload heavier
   than the model. The expected risk is a slow, damped droop: the spring gives at
   most `K_s·δ` ≈ 1 N·m, against about 3 N·m on j2 from a 0.5 kg container.
7. Pose check (step 2 above) from the logged `q`.

Write the results as a bench report in `docs/integration/`, and replay the
recorded CSVs through `contact_replay` to tune offline between sessions.
The first report, on how small a tap the residual resolves still and in motion,
is [contact_detection_bench.md](../integration/contact_detection_bench.md).
