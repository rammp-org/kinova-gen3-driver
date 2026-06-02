# Gravity-Comp Static Check & Hold-Position Test

> Robot-in-the-loop procedures. **Run together, robot present, e-stop in reach.**
> The robot is OFF until then. Do these AFTER the prerequisites in
> [`../integration-runbook.md`](../integration-runbook.md) (aarch64 KORTEX SDK
> installed, real path built with `-DKINOVA_ENABLE_KORTEX=ON`).

These two tests validate that the model's torques match physics on the real arm.
Their accuracy is governed entirely by the URDF's inertial parameters —
**including any end-effector payload**.

> **Confirm the URDF first.** Check that `models/gen3_7dof.urdf` includes the
> actual tool/gripper inertia mounted on the arm. If the model omits or
> mis-states the end-effector payload, gravity-comp will be systematically wrong
> — the arm will **sag** (under-compensated) or **push up** (over-compensated),
> especially in distal joints. If a payload is attached and the URDF doesn't
> model it, fix the URDF before trusting these results.

---

## A. Static-pose gravity validation

**Goal:** compare `Dynamics.gravity(q)` against the robot's own measured joint
torques at several static poses. Agreement validates the URDF inertials
(link masses, CoM, inertia tensors) and the continuous-joint packing on real
hardware.

**Why static:** with the arm held still (`q̇ ≈ 0`, `q̈ ≈ 0`), the only joint
torque is gravity. So measured joint torque should equal `Dynamics.gravity(q)`
joint-for-joint, within friction/sensor tolerance.

### Procedure

1. Connect at low risk — no torque commanded yet. The simplest read-only path is
   to put the arm in a held pose and read feedback (the benchmark logs measured
   `tau` per cycle to its CSV).

2. Choose **several distinct static poses** that load different joints, e.g.:
   - all-zeros (reference / near-singular gravity loading),
   - shoulder out horizontal (`q[1] = π/2`) — maximal gravity load on the
     shoulder,
   - elbow bent with forearm horizontal,
   - a wrist-loaded pose (exercises the distal links + any payload),
   - one arbitrary "general" pose.

3. For each pose, with the arm **held stationary**:
   - Record the measured joint torques `tau_measured` (from feedback / CSV).
   - Record the joint angles `q`.
   - Compute the model gravity `tau_model = Dynamics.gravity(q)` for the same
     `q` (offline, or a small read-only harness — no torque output needed).

4. **Compare** `tau_model` vs `tau_measured` joint-by-joint.

### Interpreting results

- **Close agreement** (within joint friction / torque-sensor noise, typically a
  small N·m offset that grows with joint load): URDF inertials are good,
  including the payload.
- **Consistent offset that scales with pose / load:** a mass or CoM error,
  often the **end-effector payload** — revisit the URDF.
- **Sign error or gross mismatch on one joint:** suspect joint mapping or the
  continuous-joint (cos/sin) packing for that joint.

Repeat across the poses; a model that matches at every pose is trustworthy for
the live hold test.

---

## B. Hold-position test

**Goal:** run live gravity-comp and confirm the arm holds itself against gravity
(or drifts only slowly), then tune damping if needed.

### Procedure

1. Start conservative per the runbook (low `--scale`, conservative
   `--torque-limit`, short `--duration`), e-stop in reach:

   ```sh
   ./benchmark_grav_comp --ip 192.168.1.10 --urdf ../models/gen3_7dof.urdf \
     --rate 1000 --scale 0.5 --torque-limit 15 --duration 5 --csv /tmp/hold.csv
   ```

2. Observe the arm:
   - At `--scale 0.5` it should **sag gently and smoothly** (half gravity
     applied) — confirms sign and rough magnitude.
   - Step `--scale` up toward `1.0`. With a correct model the arm should
     **hold position** when released, drifting only slowly.

3. **Drift / instability tuning** with `--damping` (the `−b·q̇` term, default
   `b = 0`):
   - If the arm **oscillates or feels lively** (energy injected by sensor noise
     or slight over-compensation), add a small `--damping` (start small, e.g.
     `0.5`–`2.0` N·m·s/rad) to bleed off velocity and stabilize.
   - If it sags despite `--scale 1.0`, the model is under-compensating — that's a
     URDF/payload issue (Test A), **not** something damping fixes.

### What good looks like

- Near `--scale 1.0`: the arm holds where you leave it, with only slow drift.
- No oscillation, no runaway, `faults = 0`, `overruns = 0`.
- A modest `--damping` smooths residual liveliness without making the arm feel
  "sticky."

> Abort criteria and the full safety posture are in
> [`../integration-runbook.md`](../integration-runbook.md). Never run unattended.
