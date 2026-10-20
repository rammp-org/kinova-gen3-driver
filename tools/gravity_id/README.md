# gravity_id

Offline gravity-model identification for the Gen3 driver. It turns a read-only
`benchmark_grav_comp --dry-run` session into a checked correction to the URDF.
Nothing here connects to the arm.

- Design: `docs/superpowers/specs/2026-10-20-gravity-model-identification-design.md`
- On-arm procedure: `docs/integration/gravity_identification.md`

```sh
cd tools/gravity_id
uv run pytest -q                                   # synthetic proofs, no arm
E="--urdf robot.urdf --ee-frame arm_end_effector_link"   # the URDF the dry-run used
uv run python -m gravity_id posegrid $E --out poses.csv --j1-range=-90,90
uv run python -m gravity_id parse    $E --poses poses.csv --out holds.csv session.txt
uv run python -m gravity_id fit      $E --holds holds.csv --out-dir fit/
uv run python -m gravity_id drift    $E --before pre.txt --after post.txt
uv run python -m gravity_id synth    $E --poses poses.csv --out fake.txt --payload 0.25,0,0,0.1
uv run python -m gravity_id candle   $E candle.txt     # two-sided candle -> --torque-offset
```

`fit` writes `report.md` (every model family's held-out residual, the chosen
correction, the static gate), `params.json`, and `corrected.urdf`.
