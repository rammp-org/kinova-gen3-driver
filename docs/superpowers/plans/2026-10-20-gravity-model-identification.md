# Gravity Model Identification Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development
> (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** Offline tooling that turns a read-only dry-run session on the arm into a localized,
physically checked gravity-model correction (a corrected URDF), plus the on-arm runbook that
produces the data and the gate that validates the result.

**Architecture:** A uv Python project at `tools/gravity_id/`, outside CMake. It loads the URDF
the way `Dynamics` does (reduced to the root→EE chain, gripper locked at `q = 0`, continuous
joints packed `(cos, sin)`), builds the static gravity regressor from Pinocchio, fits nested
model families as a Bayesian ridge toward the URDF, selects on held-out poses, and writes the
winner back into the URDF's `<inertial>` tags. The data source is the existing
`benchmark_grav_comp --dry-run` stdout. No C++ changes.

**Tech Stack:** Python ≥3.10, `pin` 3.9.0 (pinocchio, cmeel), numpy, pytest, uv.

**Spec:** `docs/superpowers/specs/2026-10-20-gravity-model-identification-design.md`

## Global Constraints

- No C++ change in this plan. Nothing touches `compute()`, the executor cycle or `Dynamics`.
- Pinocchio stack pinned exactly as `.github/actions/setup-cpp-deps/requirements.txt`:
  `pin==3.9.0`, `libpinocchio==3.9.0`, `cmeel-urdfdom==4.0.1` (an unpinned urdfdom resolves to
  6.x and dies on `liburdfdom_sensor.so.4.0`; reproduced while writing this plan).
- Run everything as `uv run --project tools/gravity_id <cmd>`; tests:
  `uv run --project tools/gravity_id pytest tools/gravity_id/tests -q`.
- SI units and radians internally; degrees only in `poses.csv` and in the transcript.
- Prior and noise values (spec, Phase I): σ_m = max(0.2 kg, 0.3·m_body), σ_mc = 0.1 m·σ_m,
  σ_o = 2.0 N·m, σ_τ = 0.1 N·m.
- Physical checks (spec): corrected mass > 0 and corrected COM within 0.5 m of the joint frame,
  else the family is **rejected**, never clipped.
- Selection (spec): smallest family whose held-out RMS ≤ 1.10 × best family's held-out RMS.
- Transcript self-checks (spec): |printed grav − tool g(q)| ≤ 0.02 N·m on every row, and
  measured τ must correlate positively with g, else refuse.
- Fit/val split: 40 fit + 10 val poses by default; `z_min` default 0.10 m; joint-limit margin 10°.
- Families: `F0` none, `F1` offsets, `F2` offsets+body 7, `F3` offsets+bodies 5–7,
  `F4` offsets+bodies 1–7 (bodies are 1-indexed by joint).
- Never connect to the arm from a test or from this tooling. Arm sessions follow
  `docs/integration/gravity_identification.md`, attended by Swapnil, motion confirmed per batch.

## Review Focus

1. **A transcript taken with a different URDF than the one passed to `fit`.** Expect a refusal
   naming the worst row, not a fit against the wrong model. → Task 3 test
   `test_parse_rejects_wrong_urdf`.
2. **A pre-#74 binary (reaction-signed τ).** Expect a refusal that says so. → Task 3 test
   `test_parse_rejects_reaction_signed`.
3. **Continuous-joint angles near ±180° in the transcript** (the same hold printed as 179.99 and
   −179.99). Expect one hold, not two. → Task 3 test `test_segment_wraps_continuous_joints`.
4. **Too few held poses for a family** (rank-deficient beyond what the prior regularizes, or no
   val poses). Expect a clear error, not a silent F0. → Task 4 test
   `test_fit_requires_val_poses`.
5. **A body whose only massive link is massless in the URDF** (`gen3_end_effector_link` has no
   `<inertial>`). Expect the writer to put the correction on the heaviest link of that rigid
   group, not create a negative-mass link. → Task 5 test `test_write_targets_heaviest_link`.

---

### Task 1: Project, model loader, regressor

**Files:**
- Create: `tools/gravity_id/pyproject.toml`, `tools/gravity_id/gravity_id/__init__.py`,
  `tools/gravity_id/gravity_id/model.py`, `tools/gravity_id/tests/conftest.py`,
  `tools/gravity_id/tests/test_model.py`

**Interfaces:**
- Produces:
  - `class ArmModel` with `ArmModel.load(urdf_path: str, ee_frame: str) -> ArmModel`;
    attributes `model` (reduced `pin.Model`), `joint_names: list[str]`, `lower/upper: np.ndarray(7)`
    (±inf for continuous), `continuous: np.ndarray[bool](7)`.
  - `ArmModel.gravity(q: np.ndarray) -> np.ndarray(7)`
  - `ArmModel.regressor(q) -> np.ndarray(7, 28)`: columns `[m, mcx, mcy, mcz]` per body 1..7, in
    each body's joint frame (Pinocchio dynamic-parameter order, first four of ten).
  - `ArmModel.params() -> np.ndarray(28)`: the URDF's own `[m, mc]` per body, so
    `regressor(q) @ params() == gravity(q)`.
  - `ArmModel.frame_heights(q) -> np.ndarray`: height of every joint origin and the EE along
    `-model.gravity` (the "up" direction), metres.
  - `ArmModel.ee_position(q) -> np.ndarray(3)`
  - `conftest.py` fixture `arm` = `ArmModel.load(<repo>/models/gen3_7dof_2f85.urdf,
    "gen3_end_effector_link")`.

- [ ] **Step 1: Write the failing tests** in `test_model.py`:
  - `test_chain_is_seven_revolute`: `len(arm.joint_names) == 7`, names `gen3_joint_1..7`.
  - `test_regressor_reproduces_gravity`: for 20 random q, `max|regressor(q) @ params() − gravity(q)| < 1e-10`.
  - `test_gravity_matches_dry_run_print`: at q = 0 the 2F-85 model's g(q) equals the C++
    `Dynamics::gravity` values pinned in `tests/dynamics_test.cpp` if present, else a
    hard-coded vector captured once from `benchmark_grav_comp --sim --dry-run` (record the
    source in a comment). Tolerance 0.01 N·m.
  - `test_body7_lumps_gripper`: `params()[24] ≈ 1.8569` kg (bracelet 0.364 + 2F-85 0.920864 +
    merged links, measured while writing this plan).
  - `test_continuous_packing`: `gravity(q)` equals `gravity(q + 2π e_k)` for each continuous k.
- [ ] **Step 2: Run** `uv run --project tools/gravity_id pytest tools/gravity_id/tests -q` → FAIL (import error).
- [ ] **Step 3: Implement `model.py`.** Reduction exactly as `src/dynamics.cpp`: keep
  `full.supports[full.frames[ee].parentJoint]`, lock the rest at `pin.neutral(full)`. Pack q per
  `nqs[j] == 2`. Raise `ValueError` if the EE frame is missing or the reduced chain is not seven
  1-DOF revolute joints (same posture as `check_arm_chain`). Regressor =
  `pin.computeJointTorqueRegressor(m, d, qcfg, 0, 0)` columns `10k + [0..3]`.
- [ ] **Step 4: Run tests** → PASS.
- [ ] **Step 5: Commit** `git add tools/gravity_id/pyproject.toml tools/gravity_id/gravity_id/__init__.py tools/gravity_id/gravity_id/model.py tools/gravity_id/tests/conftest.py tools/gravity_id/tests/test_model.py`
  — `feat(gravity_id): model loader and static regressor`.

### Task 2: Pose grid

**Files:** Create `tools/gravity_id/gravity_id/posegrid.py`, `tools/gravity_id/gravity_id/design.py`,
`tools/gravity_id/tests/test_posegrid.py`

**Interfaces:**
- Consumes: `ArmModel`, `ArmModel.regressor`, `ArmModel.frame_heights`.
- Produces: `make_grid(arm, n_fit=40, n_val=10, z_min=0.10, j1_range_deg=(-180, 180),
  margin_deg=10.0, n_candidates=3000, seed=0) -> list[Pose]`, where
  `Pose = dataclass(id: str, set: str, q: np.ndarray)` with ids `P00..`, set `"fit"|"val"`, and
  the list ordered nearest-neighbour from `q = 0`. `write_poses_csv(path, poses)` /
  `read_poses_csv(path) -> list[Pose]`; CSV columns `id,set,j1_deg..j7_deg`, angles wrapped to
  (−180, 180].

- [ ] **Step 1: Failing tests:** `test_grid_sizes_and_sets` (40 fit, 10 val, unique ids);
  `test_grid_respects_limits_and_height` (every q within limits−margin, `min(frame_heights) ≥
  z_min`); `test_grid_full_rank_for_f4` (scaled F4 design `[Y | I]` stacked over fit poses has
  numerical rank equal to the identifiable count from SVD of the same stack over 3000 random
  poses); `test_grid_beats_random` (`logdet` of selected fit set > median `logdet` of 20 random
  40-subsets of the candidates); `test_csv_roundtrip`.
- [ ] **Step 2: Run** → FAIL.
- [ ] **Step 3: Implement.** Greedy D-optimal on the column-scaled F4 design restricted to its
  identifiable subspace (project onto the top-r right singular vectors of the candidate stack,
  r from the SVD with relative tolerance 1e-6), adding a 1e-6 ridge so early `logdet`s are
  finite. Val poses: `rng.choice` from the unselected survivors.
- [ ] **Step 4: Run** → PASS.
- [ ] **Step 5: Commit** — `feat(gravity_id): D-optimal pose grid`.

### Task 3: Dry-run transcript parser

**Files:** Create `tools/gravity_id/gravity_id/transcript.py`, `tools/gravity_id/tests/test_transcript.py`,
`tools/gravity_id/tests/data/` (synthetic transcripts generated by the test helper, not checked-in arm data).

**Interfaces:**
- Consumes: `ArmModel.gravity`, `read_poses_csv`, `Pose`.
- Produces:
  - `parse_tables(text: str) -> list[Table]`, `Table = dataclass(q: ndarray(7) rad,
    tau: ndarray(7), grav: ndarray(7))`. Matches the exact `printf` of
    `apps/benchmark_grav_comp.cpp` dry-run: header `  j |   q[deg] | meas[Nm] | grav[Nm] |  resid[Nm]`
    then 7 rows `  %d | %8.2f | %8.2f | %8.2f | %+9.3f`. Ignores all other lines.
  - `format_table(q, tau, grav) -> str` — the inverse, used by tests and the synthetic generator.
  - `segment(tables, tol_deg=0.05, min_tables=6, settle=4) -> list[Hold]` (settle was 2 in the plan; dojo needed 4, see the Execution record),
    `Hold = dataclass(q, tau, tau_std, n, pose_id, set)`: consecutive tables whose joint angles differ by
    ≤ tol (wrapped difference) form a hold; drop the first `settle` tables; keep holds with
    ≥ `min_tables` remaining; q and τ are medians.
  - `check_against_model(tables, arm, tol=0.02)`: raises `ValueError` if any
    |grav − arm.gravity(q)| > tol (message names the table index, joint and both values), or if
    `corr(tau, grav)` over all loaded rows (|grav| > 1 N·m) is negative ("reaction-signed
    torque: binary predates #74").
  - `match_holds(holds, poses, tol_deg=1.0) -> tuple[list[Hold], list[Hold]]` (matched holds carry `pose_id` and `set`; unmatched returned second).
  - `write_holds_csv / read_holds_csv`.

- [ ] **Step 1: Failing tests:** `test_parse_roundtrip` (format → parse recovers to print
  precision); `test_parse_ignores_noise_lines` (the `[dry-run]`, `[model]` and `-> max|residual|`
  lines around tables); `test_segment_finds_holds` (3 holds of 10 tables separated by 4 moving
  tables → 3 holds); `test_segment_wraps_continuous_joints`; `test_parse_rejects_wrong_urdf`
  (grav column from a model with the gripper removed → `ValueError` matching `"grav"`);
  `test_parse_rejects_reaction_signed` (tau = −grav → `ValueError` matching `"#74"`);
  `test_match_holds_assigns_sets`.
- [ ] **Step 2: Run** → FAIL. **Step 3: Implement.** **Step 4: Run** → PASS.
- [ ] **Step 5: Commit** — `feat(gravity_id): dry-run transcript parser`.

### Task 4: Fit, families, selection

**Files:** Create `tools/gravity_id/gravity_id/fit.py`, `tools/gravity_id/gravity_id/synth.py`,
`tools/gravity_id/tests/test_fit.py`

**Interfaces:**
- Consumes: `ArmModel.regressor/params/gravity`, `Hold`.
- Produces:
  - `FAMILIES: dict[str, tuple[tuple[int, ...], bool]]` = name → (free bodies, offsets free):
    `F0 ((), False)`, `F1 ((), True)`, `F2 ((7,), True)`, `F3 ((5, 6, 7), True)`, `F4 ((1..7), True)`.
  - From `gravity_id/design.py` (shared with Task 2): `prior_std(arm) -> ndarray(28)`,
    `design(arm, q, bodies, offsets) -> ndarray(7, n)` (columns scaled by their prior std).
  - `fit_family(arm, q_fit, tau_fit, family) -> FamilyFit` with `dpi: ndarray(28)` (zeros
    outside the family), `offsets: ndarray(7)`, `std_dpi`, `std_offsets`, `rejected: str | None`.
  - `predict(arm, fam_fit, q) -> ndarray(7)` = `gravity(q) + regressor(q) @ dpi + offsets`.
  - `select(arm, holds) -> Selection` with `fits: dict[str, FamilyFit]`,
    `val_rms: dict[str, ndarray(7)]` (per joint), `val_max`, `chosen: str`. Raises `ValueError`
    if there are no `val` holds, or if the fit holds give fewer torque readings (7 per hold) than
    2× the largest family's unknowns (35 for F4, so ≥ 10 fit holds).
  - `synth.make_holds(arm, poses, dpi=None, offsets=None, noise=0.05, seed=0) -> list[Hold]`
    (truth = URDF + `regressor(q) @ dpi` + offsets + noise), with helpers
    `synth.point_mass(body, mass, com) -> dpi` and `synth.scale_body(arm, body, factor) -> dpi`.

- [ ] **Step 1: Failing tests** (all use a grid from `make_grid` with seed 0 and noise σ = 0.05):
  - `test_recovers_tip_payload`: truth = body 7 `+0.25 kg` at `(0.02, 0.0, 0.12)` m in the joint-7
    frame + offsets `[0.1, -0.4, 0.3, 0.2, -0.1, 0.05, 0.02]` → `chosen == "F2"`, |Δm − 0.25| ≤
    0.02, COM of the added mass within 0.01 m, offsets within 0.05.
  - `test_detects_proximal_link_error`: truth = body 3 mass ×1.15 (first moment scaled with it)
    → `chosen in {"F3", "F4"}`, every val-joint RMS of the chosen family ≤ 2σ, F2's val RMS on
    j2 or j4 > 2σ. (Implementation finding: static gravity cannot attribute mass to a body; F3
    reproduces a body-3 error to the noise floor. The spec records this.)
  - `test_tip_family_can_mimic_a_wrist_link`: truth = body 5 ×1.15 → `chosen == "F2"`, val RMS ≤ 2σ.
  - `test_no_error_no_phantom`: truth = URDF → `chosen in {"F0", "F1"}`.
  - `test_rejects_unphysical`: a family forced onto data from a −2 kg body-7 truth reports
    `rejected` containing `"mass"` and is never chosen.
  - `test_fit_requires_val_poses`.
- [ ] **Step 2: Run** → FAIL.
- [ ] **Step 3: Implement.** Residual `r = τ − gravity(q)` stacked over fit holds. Columns = the
  family's body columns (scaled by their σ) plus `I₇` per pose if offsets (scaled by σ_o).
  Augmented least squares `[A/σ_τ; I] x = [r/σ_τ; 0]` via `np.linalg.lstsq`; posterior std
  `sqrt(diag((AᵀA/σ_τ² + I)⁻¹))` rescaled. Physical checks per Global Constraints set `rejected`.
  Selection per Global Constraints over non-rejected families; RMS = sqrt(mean over joints
  and val holds).
- [ ] **Step 4: Run** → PASS.
- [ ] **Step 5: Commit** — `feat(gravity_id): nested-family Bayesian ridge fit with held-out selection`.

### Task 5: Corrected URDF writer

**Files:** Create `tools/gravity_id/gravity_id/urdf_write.py`, `tools/gravity_id/tests/test_urdf_write.py`

**Interfaces:**
- Consumes: `ArmModel`, `FamilyFit.dpi`.
- Produces: `write_corrected_urdf(urdf_in: str, ee_frame: str, dpi: ndarray(28), urdf_out: str)
  -> list[LinkChange]`, `LinkChange = dataclass(link, mass_old, mass_new, com_old, com_new)`.

- [ ] **Step 1: Failing tests:** `test_roundtrip_matches_fit` (write a nonzero Δπ on bodies 3 and
  7, reload with `ArmModel.load`, `max|gravity_new(q) − (gravity(q) + Y(q)Δπ)| < 1e-9` over 20
  random q); `test_write_targets_heaviest_link` (body 7's change lands on
  `gen3_robotiq_85_base_link`, the heaviest BODY frame whose `parentJoint` is joint 7, not the
  massless `gen3_end_effector_link`); `test_keeps_rotational_inertia` (the `<inertia>` element
  and the `rpy` of the edited link are byte-identical before/after); `test_negative_mass_refused`
  (Δm < −m_link → `ValueError`).
- [ ] **Step 2: Run** → FAIL.
- [ ] **Step 3: Implement.** Parse with `xml.etree.ElementTree`. For body j, pick the heaviest
  BODY frame with `parentJoint == j` in the reduced model; its `placement` is link→joint-j.
  New first moment in the joint frame `h' = m·(p + R c) + Δh`, `m' = m + Δm`, new COM in the link
  frame `c' = Rᵀ(h'/m' − p)`; write `<mass value>` and `<origin xyz>` only.
- [ ] **Step 4: Run** → PASS. **Step 5: Commit** — `feat(gravity_id): write corrections into URDF inertials`.

### Task 6: CLI, report, drift readout

**Files:** Create `tools/gravity_id/gravity_id/__main__.py`, `tools/gravity_id/gravity_id/report.py`,
`tools/gravity_id/tests/test_cli.py`, `tools/gravity_id/README.md`

**Interfaces:**
- Consumes: everything above.
- Produces the commands (all take `--urdf PATH --ee-frame NAME`):
  - `posegrid --out poses.csv [--n-fit 40 --n-val 10 --z-min 0.10 --j1-range LO,HI --seed 0]`
  - `parse --poses poses.csv --out holds.csv TRANSCRIPT...` (runs `check_against_model`; prints
    holds found, matched/unmatched)
  - `fit --holds holds.csv --out-dir DIR [--family FX]` → `DIR/report.md`,
    `DIR/params.json`, `DIR/corrected.urdf` (chosen or forced family)
  - `drift --before T1 --after T2` → per-joint Δq [deg] between the last table of T1 and the first
    of T2, and EE displacement [mm] via FK.
  - `synth --poses poses.csv --out transcript.txt [--payload M,X,Y,Z] [--offsets o1,..,o7]`
    writes a fake dry-run transcript (for rehearsing the pipeline end to end).

- [ ] **Step 1: Failing test** `test_end_to_end_synthetic`: in `tmp_path`, run `posegrid` →
  `synth --payload 0.25,0.02,0,0.12` → `parse` → `fit` through `subprocess` with
  `python -m gravity_id`; assert `report.md` contains `Chosen family: F2` and
  `corrected.urdf` loads with `ArmModel.load`. `test_drift_readout`: two synthetic transcripts
  0.5° apart on j2 → Δq j2 = 0.50 ± 0.01.
- [ ] **Step 2: Run** → FAIL. **Step 3: Implement.** The report has: the self-check results; a
  per-family table of val RMS and max per joint (N·m) with rejected families and their reason;
  the chosen family's parameters ± std, the tip payload expressed as mass + COM in the EE frame
  (for comparison with the D3 scale weight); the `LinkChange` list.
- [ ] **Step 4: Run all tests** → PASS. **Step 5: Commit** — `feat(gravity_id): CLI and report`.

### Task 7: Runbook and docs wiring

**Files:** Create `docs/integration/gravity_identification.md`; modify `mkdocs.yml` (add it
under `Operations`), `docs/integration-runbook.md` (one link), `CHANGELOG.md` (Unreleased).

- [ ] **Step 1:** Write the runbook: preconditions (live stack stopped, rendered `robot.urdf`,
  `[model] mount tilt` check, gripper open, 10 min warm-up); D1 (home → grid → home mid →
  grid → home end, ≥ 5 s per hold, transcript `tee`'d); D2 hysteresis protocol; D3 weighing;
  the offline commands; Phase V static gate and the attended drift test with its exact command
  (`benchmark_grav_comp --duration 5 --damping 5.0`) and pass numbers copied from the spec.
- [ ] **Step 2:** `mkdocs build --strict` (via `uvx --with mkdocs-material mkdocs build --strict`)
  → no warnings about the new page.
- [ ] **Step 3: Commit** — `docs: gravity identification runbook`.

---

### Campaign phases after this plan (not tasks here)

| Phase | Who | Output | Next artifact |
|---|---|---|---|
| D | Swapnil at the arm, runbook D1–D3 | transcripts, scale weights | `holds.csv` |
| I | offline, `fit` | `report.md`, `corrected.urdf` | report attached to the PR / issue |
| L | review, then a PR in kinova-gen3-ros2 (rendered model) and, if arm-general, `models/` here | URDF change | release notes |
| V | Swapnil, runbook V | gate table before/after | spec updated with the measured numbers |
| E | only if V leaves speed-dependent error | — | new spec |

The open decisions (offset landing, which arms) are revisited when the Phase I report exists.

## Execution record (2026-10-20)

Tasks 1–7 were implemented as planned. Beyond the plan, driven by the on-arm session:

- `parse` drops the first 2 s of each hold as settling. Kinova's hold torque takes
  about 2 s to settle after arrival.
- `posegrid --margin`. `gravity_id candle` turns a two-sided candle transcript into
  offsets.
- `tools/gravity_id/onarm/`: a read-only logger, a mover using Kinova high-level
  moves (`--bidirectional`), Kinova's `ValidateWaypointList`, and `run_grid.sh`.
  The grid run hit the table and self-collided; see the spec's Results. A
  collision checker is a prerequisite for using it again.
- The landing (Phase L) became `KortexTransport::set_torque_offset` (#91) rather
  than a URDF edit, because the offsets, not the masses, were wrong.
  `benchmark_grav_comp --torque-offset`, plus the procedure
  `docs/integration/torque_offset_calibration.md`.
