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
    [--gain 25] [--debounce 5] [--thresholds 8,8,6,6,2,2,2] [--tau-sign -1]
```

- **Input:** a CSV with a header naming `t, q1..q7, qd1..qd7, tau1..tau7` (extra
  columns are ignored). `tau` must be in command convention, where a still arm
  reads `tau = g(q)`. A raw core v1.2.0 recording has the opposite sign: pass
  `--tau-sign -1`.
- **Output:** `r.csv` with `t, r1..r7`.
- **Stdout:** per joint, RMS and peak `|r|` over the moving samples (max `|qd|` >
  0.02 rad/s, after the `5/gain` settle), the first would-be trip, and the total
  `would_trips`.

The debounce counts rows, not milliseconds: at the 1 kHz RT rate `--debounce 5` is
5 ms, but a 50 Hz recording makes it 100 ms. Scale `--debounce` to the recording
rate, or expect fewer trips offline than on the arm.

### Parity with the Python reference

`tools/contact/parity.py` runs the Python reference observer over the same
recording and compares it with `contact_replay`'s output. It exits 1 if any joint
differs by more than 0.05 N·m after the first `5/gain` seconds.

```
uv run -q --no-project --with pin --with numpy python -I tools/contact/parity.py \
    --observer-py observer.py --csv tour.csv --urdf arm.urdf --cpp r.csv [--gain 25]
```

## On-arm bench steps

Each motion batch is confirmed with Swapnil before it runs.

1. `observe`, no motion, 60 s: the still floor per joint.
2. `observe`, the tour at speed 0.6, then 1.0, nobody touching: motion RMS and
   peak per joint, plus `would_trips`. Set `default` to about 3x the peak, rerun,
   and expect `would_trips == 0`.
3. `observe`, still arm, slow hand pushes on the forearm and the tool: record `r`
   against the `/ee_state` wrench, which is exact at rest. This maps `θ` to force.
4. `enforce`, slow move, hand push: trip, float, latch, `-11` on the goal,
   `/clear_faults`, hold. Then at demo speed.
5. After a trip: force build-up, the position-mode to float window (look for the
   #92 droop), and the RT cycle time with and without the monitor (`--rt-csv`).
6. `observe` with real payloads (spoon, container): `r` while carrying.
7. Pose check (step 2 above) from the logged `q`.

Write the results as a bench report in `docs/integration/`, and replay the
recorded CSVs through `contact_replay` to tune offline between sessions.
