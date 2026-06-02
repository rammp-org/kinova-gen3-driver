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

1. **CONFIRM THE ROBOT'S FIRMWARE VERSION FIRST.** Kinova couples the Kortex API
   version to robot firmware — pick the API that matches the firmware, not simply
   the newest. Read it at power-on via the Kinova web app (robot IP in a browser)
   or `GetDeviceInformation`. The choice:
   - **firmware 2.x → use Kortex C++ API 2.8.0 aarch64** (the default below).
   - **firmware 3.x → 2.8.0 will likely refuse/misbehave.** The public 3.x C++
     bundle (`API/3.4.0/jetson.zip`) does **not** build — its `ClientService.h`
     includes `CoreBenchmarker.h`, which Kinova does not ship in the package
     (verified 2026-06-02). You'd need a *complete* 3.x C++ SDK from Kinova, or
     match the firmware's own 2.x API. Decide at bring-up.

   For low-level cyclic control (our use case) the API surface is stable across
   2.3–2.8, so we are not missing control features by being on 2.x.

2. **aarch64 KORTEX C++ SDK — INSTALLED & VERIFIED (2026-06-02): Kortex 2.8.0.**
   The lib vendored in `~/rtos_testing/cpp/kortex_hardware/` is **x86-64 only**,
   and 2.3.0 ships no aarch64 build at all. We use **2.8.0 aarch64** (newest
   *buildable* C++ version; built + linked clean against our code), installed to
   a user dir (no sudo):

   ```sh
   ssh abra '
     DEST=~/kortex_api_2.8.0_aarch64
     mkdir -p "$DEST"
     curl -sL "https://artifactory.kinovaapps.com/artifactory/generic-public/kortex/API/2.8.0/linux_aarch64_gcc_7.4.zip" -o /tmp/k.zip
     unzip -q -o /tmp/k.zip -d "$DEST"   # -> $DEST/include/{client,client_stubs,common,google,messages} + $DEST/lib/release/libKortexApiCpp.a
   '
   ```

   The layout matches our `KORTEX_HW_DIR` exactly; the lib is confirmed `AArch64`.
   (Symptom of pointing at the x86-64 lib again: `ld: ... Relocations in generic
   ELF (EM: 62) ... file in wrong format`.)

3. **Build with the real path enabled** (verified to link cleanly on abra):

   ```sh
   ssh abra 'cd ~/kinova-gen3-driver && rm -rf build_kortex && mkdir build_kortex && cd build_kortex && cmake .. \
     -DCMAKE_PREFIX_PATH=/usr/local/lib/python3.10/dist-packages/cmeel.prefix \
     -DKINOVA_ENABLE_KORTEX=ON \
     -DKORTEX_HW_DIR=$HOME/kortex_api_2.8.0_aarch64 \
     && cmake --build . -j'
   ```

   `benchmark_grav_comp` links against the aarch64 lib and runs the `--sim` path;
   the real `--ip` path is now compiled in and ready (untested on hardware — that
   is this session).

4. **Confirm the URDF matches the physical arm**, including any tool/gripper.
   See [`integration/grav_comp_static_check.md`](integration/grav_comp_static_check.md)
   — if the URDF is missing the real end-effector payload inertia, gravity-comp
   will visibly **sag** (under-comp) or push. Verify before trusting torque output.
   - **Bare arm:** `models/gen3_7dof.urdf`.
   - **With a Robotiq 2F-85 (the lab arm, confirmed 2026-06-02):** use
     `models/gen3_7dof_2f85.urdf`. It is Kinova's `kortex_description/gen3_2f85`
     with the 6 gripper finger joints converted to **`fixed`**, so the gripper is
     a rigid payload (+1.36 kg incl. coupler) and Pinocchio still reports `nv=7`
     (Dynamics requires exactly 7). **Lesson learned:** running the bare-arm URDF
     with the gripper mounted under-compensated by ~1.4 kg at the tip and the arm
     sagged on the first live torque run — always match the URDF to the mounted
     hardware, and verify read-only (Phase 2 / `--dry-run`) before torque.

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
