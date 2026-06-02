# Integration Runbook — Robot-in-the-Loop Bring-Up

> ## ⚠️ SAFETY POSTURE — READ FIRST
>
> **The robot is OFF and STAYS OFF until we run this together, in person.**
> **Never run any step that touches hardware unattended.** This document is the
> sequence to follow *together* once the Kinova is powered on **and** the
> aarch64 KORTEX SDK is installed. It is preparation only — nothing here has
> been executed against hardware.

This is the first time the driver drives a real Gen3. Treat every step as
load-bearing. An **e-stop must be within reach** for the entire session.

---

## 0. Prerequisites (do these before powering the robot)

1. **Install an aarch64 build of the KORTEX C++ SDK.** The vendored lib on
   `abra` (`~/rtos_testing/cpp/kortex_hardware/lib/.../libKortexApiCpp.a`) is
   **x86-64 only** and will NOT link into a Jetson binary. Until an aarch64
   KORTEX lib exists on the box, the real path cannot be built. (Symptom of the
   wrong arch: `ld: ... Relocations in generic ELF (EM: 62) ... file in wrong
   format`.)

2. **Build with the real path enabled**, pointing at the aarch64 SDK:

   ```sh
   ssh abra 'cd ~/kinova-gen3-driver/build && cmake .. \
     -DCMAKE_PREFIX_PATH=/usr/local/lib/python3.10/dist-packages/cmeel.prefix \
     -DKINOVA_ENABLE_KORTEX=ON \
     -DKORTEX_HW_DIR=/path/to/aarch64/kortex_hardware \
     && cmake --build . -j'
   ```

   Confirm `benchmark_grav_comp` **links cleanly** (no x86/aarch64 mismatch).

3. **Confirm the URDF matches the physical arm**, including any tool/gripper.
   See [`integration/grav_comp_static_check.md`](integration/grav_comp_static_check.md)
   — if `models/gen3_7dof.urdf` is missing the real end-effector payload
   inertia, gravity-comp will visibly **sag or push**. Verify before trusting
   any torque output.

---

## 1. Power-on checklist (together, at the robot)

- [ ] **E-stop physically within reach** and tested before energizing.
- [ ] Clear workspace — no people/objects in the arm's reach envelope.
- [ ] Robot mounted/clamped securely; cables strain-relieved.
- [ ] Power on the Gen3; wait for it to finish its own boot/self-check.
- [ ] Confirm the arm is in a safe, supported starting pose.

## 2. Confirm connectivity & credentials

- [ ] **Robot IP** — prototype default is `192.168.1.10`. Confirm the actual IP
      before connecting (`ping` from `abra`).
- [ ] **Credentials** — default `admin`/`admin` unless changed. Confirm.
- [ ] Confirm `abra` is on the robot's subnet.

## 3. Grant RT privileges

The RT loop wants `SCHED_FIFO` + `mlockall`. Without privileges it silently
falls back to `SCHED_OTHER` (the introspection report will show `policy: OTHER`,
which is fine for a first cautious run but is NOT a real RT measurement). Grant
privileges one of two ways:

- Run under `sudo`, **or**
- Grant the binary the capabilities once:

  ```sh
  sudo setcap cap_sys_nice,cap_ipc_lock+ep ./benchmark_grav_comp
  ```

Verify in the final introspection report that `policy: FIFO` and
`page faults: majflt+=0` (proves memory was locked).

## 4. FIRST live run — conservative

Start gentle. **Low scale, conservative torque limit, short duration**, and
watch the console the entire time:

```sh
./benchmark_grav_comp \
  --ip 192.168.1.10 \
  --urdf ../models/gen3_7dof.urdf \
  --rate 1000 \
  --scale 0.5 \
  --torque-limit 15 \
  --duration 5 \
  --csv /tmp/grav_first.csv
```

- `--scale 0.5` applies only half of computed gravity torque — the arm will
  partially sag, which is **expected and safe**, and confirms the sign/magnitude
  of the torque mapping before we trust full compensation.
- `--torque-limit 15` (N·m) is well below the 39 N·m default — caps any
  surprise.
- Keep a hand on the e-stop. Be ready to SIGINT (Ctrl-C) — the app reverts to
  POSITION + `SINGLE_LEVEL_SERVOING` on shutdown.

If the first run is clean, step up cautiously: raise `--scale` toward 1.0, then
raise `--torque-limit`, increasing `--duration` only once timing is stable.

## 5. What good output looks like

- **Loop rate** holds at ~1000 Hz; `overruns = 0` (or only the first warm-up
  cycle).
- **`faults = 0`** throughout.
- **`dropped = 0`** telemetry samples.
- Arm behavior matches the commanded scale: at `--scale 0.5` it sags gently and
  smoothly; near `--scale 1.0` it holds position (see the static check doc).
- Final report: `policy: FIFO`, `majflt+=0`, low involuntary context switches.

## 6. ABORT criteria — hit the e-stop / Ctrl-C immediately if

- **Overruns rising** cycle-over-cycle (missed 1 kHz deadlines can trip the
  robot's watchdog and fault the arm).
- **Any reported fault** (`faults > 0`).
- **Unexpected motion** — jump, oscillation, runaway, or a direction that
  doesn't match gravity.
- Buzzing/grinding, or the arm fighting itself.

After any abort: power-cycle/clear faults before the next attempt and review the
CSV.

## 7. Hardware-facing uncertainties to verify on this first run

These are correct-by-construction in sim but **unvalidated on hardware**:

- **Fault-flag source.** `KortexTransport` reports `fb.fault = true` when any
  actuator's `fault_bank_a` or `fault_bank_b` is non-zero. Confirm this is the
  right fault signal on the real arm (and that a tripped fault actually surfaces
  here).
- **Non-blocking send/receive path.** The split `send()`/`receive()`
  (`RefreshCommand` / `RefreshFeedback`) path is **untested on hardware** — the
  benchmark uses the blocking `exchange()`. Don't rely on the async path live
  until exercised.
- **Velocity / current modes ported by analogy.** Only torque (gravity-comp) is
  exercised. Velocity and current actuator-mode mappings were ported by analogy
  to the torque path and are **not validated**.
- **`kCurrent` reuses the torque field.** `JointCommand` has no dedicated
  current field yet, so `kCurrent` reuses `cmd.torque` as the current setpoint
  [A]. Do not use `kCurrent` on hardware without re-checking this mapping.

---

> **Reminder: the robot is off until we do this together. Never run
> unattended.**
