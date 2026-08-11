# Joint-Space Position Control — Hardware Check

> Robot-in-the-loop procedure. **Run together, robot present, e-stop in reach.**
> Do this AFTER the prerequisites in
> [`../integration-runbook.md`](../integration-runbook.md) (aarch64 KORTEX SDK
> installed, real path built with `-DKINOVA_ENABLE_KORTEX=ON`).

> **First hardware run: 2026-08-11 — passed.** The `--sequence` visual check ran
> correctly on the arm: the settle phase produced no motion, each joint moved
> alone in the commanded direction, and the arm returned home.
>
> **No tuning was performed.** Every parameter is still at its shipped default
> (`max_ref_speed` 0.5 rad/s, `max_following_error` 0.35 rad, URDF position
> limits). Nothing was measured or characterised — see
> [issue #6](https://github.com/rammp-org/kinova-gen3-driver/issues/6). "It moved
> correctly" is not "it is tuned".

**Position mode has no compliance.** Unlike the impedance modes there is no
spring to absorb a mistake: the actuator's own servo chases whatever is
commanded at full authority. It will not yield to contact — it will push
through until the actuator faults. Every step below is ordered so the cheapest
failure comes first.

Everything here runs through `joint_position_check`. Steps 1–2 are the ones that
matter most; if step 2 is clean, the rest is characterisation.

---

## The open question this exists to answer

`KortexTransport::write_command` sends `rad_to_deg(cmd.position)` **unwrapped**
(`src/kortex_transport.cpp:117`), while `fill_feedback` wraps every measured
angle to (−π, π] (`src/kortex_transport.cpp:83`). So a continuous joint sitting
past half a turn is read back as a **negative** degree value, and that is what we
would send.

This has never mattered before. In torque mode the position field is a
passthrough of the **raw** KORTEX feedback value (`kortex_transport.cpp:124`),
which bypasses our wrap entirely. **`kPosition` is the first code path that ever
sends our own wrapped angle back to the arm.** KORTEX reports continuous-joint
positions in [0, 360); whether it accepts a negative setpoint in POSITION mode
is unverified and cannot be settled in sim.

If it does not, the failure mode is a joint interpreting `−170°` as something
else and taking off. That is why step 2 commands **zero motion**.

---

## Joint map (0-indexed, as the CLI takes them)

| CLI | Kinova name | Limits (rad) |
|---|---|---|
| `j0` | joint 1 | continuous |
| `j1` | joint 2 | ±2.41 |
| `j2` | joint 3 | continuous ← *the one that spun under impedance* |
| `j3` | joint 4 | ±2.66 |
| `j4` | joint 5 | continuous |
| `j5` | joint 6 | ±2.23 |
| `j6` | joint 7 | continuous |

---

## Prerequisites

1. `rt_setup.sh` has been re-run since the last reboot (it does not persist).
2. Built with KORTEX:
   ```sh
   cmake .. -DKINOVA_ENABLE_KORTEX=ON -DKORTEX_HW_DIR=/path/to/kortex_hardware \
            -DCMAKE_PREFIX_PATH=/usr/local/lib/python3.10/dist-packages/cmeel.prefix
   cmake --build . -j
   ```
3. Nothing else is talking to the arm — stop `teleop_socket_server` first.

## Global abort criteria

Hit the e-stop and stop the run if **any** of these occur:

- Motion during a step that requests none.
- A joint that starts turning and does not stop, or accelerates.
- Any joint moving in a direction the step did not ask for.
- `faults=` non-zero in the console line.
- Sustained `overruns=`, or a growing `dropped=`.
- Any noise you have not heard from this arm before.

---

## Step 1 — Read-only survey. **Nothing moves.**

Never enters low-level servoing, never commands anything.

```sh
./joint_position_check --ip 192.168.1.10 --dry-run --duration 20
```

**Do this first:** with the arm still under its own control, use the pendant or
web app to park it so that **at least one continuous joint reads a negative
degree value**. That deliberately forces the open question instead of hoping to
stumble into it later.

**Look for:**
- The `NOTE: n joint(s) would be commanded as a NEGATIVE degree value` line.
  Write down which joints. Those are the ones to watch in step 2.
- Measured angles that match the pendant's display. A mismatch here means a joint
  mapping or unit problem and everything downstream is meaningless — stop.

---

## Step 2 — Hold at the entry configuration. **The decisive test.**

No target is given, so the mode commands exactly where the arm already is. A
correct implementation produces **no motion at all**.

```sh
./joint_position_check --ip 192.168.1.10 --duration 5
```

This is the sharpest possible form of the format question: there is no commanded
motion to confuse the picture, so if KORTEX misreads a negative setpoint the arm
jumps on the **first commanded cycle** and nothing else can be blamed.

**Expected:** arm perfectly still. `faults=0`, `overruns=0`, `majflt+=0`, all
residuals ≈ 0.

**ABORT on any motion whatsoever.** If it jumps, the negative-degree hypothesis
is confirmed — record which joints moved and how far, and stop. The fix is in the
transport (map to [0, 360) before sending), not in the mode, and it needs its own
test before anything else here is run.

Repeat step 2 with the arm parked in **two or three** different configurations,
including at least one where a continuous joint is negative and one where it is
not. Holding still in every one of them is what clears the question.

---

## Step 3 — Scripted visual check

This is the confidence pass: one joint at a time, each returning to start, then
home. Wrist-first, because the distal joints are the lightest and least able to
hurt anything if a direction is wrong.

**Read the plan before running it.** `--dry-run` prints exactly what will happen,
computed from the arm's current configuration, and commands nothing:

```sh
./joint_position_check --ip 192.168.1.10 --sequence --dry-run
```

```
[dry-run] sequence plan (8 waypoints, joints j4..j6 at 0.2 rad/s):
   1. [  3.0s] settle — NOTHING should move here
   2. [  4.7s] j6 +0.200 rad (+11.5 deg) — ONLY this joint should move
   3. [  6.4s] j6 back to start
   ...
   8. [ 16.2s] HOME — compare against where the arm started
```

Waypoints are clamped to the URDF limits in the plan as well as in the mode, so
what is printed is what will actually happen — not what was asked for.

Then run it:

```sh
./joint_position_check --ip 192.168.1.10 --sequence
```

**What to watch for** (the app prints this too):

- **Step 1: the arm does not move at all.** Any twitch here is a bug.
- **Each move: only the named joint turns**, in the direction printed.
- **Each return: that joint goes back**, and the others never moved.
- **At the end: the arm is visibly where it started**, and every residual in the
  report reads `~0.0000`. That is the reference being exact.
- **Throughout: motion is smooth and rate-limited**, never a snap.

Sim baseline on abra: all 7 residuals `+0.0000`, `majflt+=0`, `overruns=0`.

Once the wrist passes, widen to the whole arm — **only with the workspace clear**:

```sh
./joint_position_check --ip 192.168.1.10 --sequence --from-joint 1
```

`--from-joint 0` includes the base joint. There is no reason to need it for this
check, and it swings the most mass.

### Single-joint variant

For isolating one joint, or checking sign symmetry:

```sh
./joint_position_check --ip 192.168.1.10 --joint 5 --delta 0.2 --speed 0.2 --duration 8
./joint_position_check --ip 192.168.1.10 --joint 5 --delta -0.2 --speed 0.2 --duration 8
```

Expect a smooth ~1 s move of about 11° each way, then hold. Final residual ≈ 0.

---

## Step 4 — Rate limit, and the hardware speed ceiling

Two claims to check.

**(a) The commanded speed is respected.** Same distance, half the speed, twice
the time:

```sh
./joint_position_check --ip 192.168.1.10 --joint 5 --delta 0.5 --speed 0.1 --duration 12
```
Should take ≈ 5 s of travel (the app prints its own estimate). Time it.

**(b) A caller cannot exceed the URDF limit.** Ask for something absurd:

```sh
./joint_position_check --ip 192.168.1.10 --joint 5 --delta 0.5 --speed 5.0 --duration 8
```
The `effective speed after URDF clamp:` line must report the **URDF** value
(≈1.22 rad/s at the wrist), not 5.0. Travel time must not drop below what that
implies. If the line prints 5.0, the clamp is not working — stop and report.

---

## Step 5 — Continuous joint across the wrap

This is the failure that already happened once on this arm, in impedance mode:
reference and measurement both live in (−π, π], so a target on the far side of
the wrap reads as a ~2π error and the joint walks most of a full turn.

Park a continuous joint (`j2` is the historically interesting one; `j6` is the
safest since it carries nothing) close to ±π using the pendant — say −175°.
Confirm with `--dry-run`. Then command a small step across the wrap:

```sh
./joint_position_check --ip 192.168.1.10 --joint 6 --delta 0.2 --speed 0.2 --duration 10
```

**Expected:** a few degrees of travel the short way round, crossing ±180°.

**ABORT if the joint begins a long rotation.** That is the wrap bug, and the
short-way logic in `JointPositionMode::compute` is not doing its job on hardware
even though it does in the unit test.

---

## Step 6 — Joint-limit clamp

Ask for a target a full radian past the stop on a bounded joint:

```sh
./joint_position_check --ip 192.168.1.10 --joint 5 --delta 3.0 --speed 0.2 --duration 20
```

**Expected:** the arm travels to the ±2.23 limit and **parks there**, no stall
current, no fault. The end-of-run report is the evidence — `target` shows the
raw request, `final_ref` shows the clamped value, and `residual` shows the
difference the clamp absorbed.

Start from near zero so the travel is long enough to observe, and keep the run
short enough that you can abort.

---

## Step 7 — Following-error leash *(optional, do last)*

The leash caps how far the reference may lead the **measured** position, so a
blocked arm cannot let the reference march away and then snap when it frees.

The unit tests cover the binding behaviour exactly (`FollowingErrorLeashStops
ARunawayReference`). On hardware the useful check is the opposite one: confirm
it does **not** bind during normal motion — in steps 3–6 the reported residuals
should all be far below the 0.35 rad leash. If a residual sits pinned at 0.35
during a free move, the arm is not tracking and something else is wrong.

**Deliberately obstructing the arm to watch the leash bite is not recommended.**
Position mode has full authority and will push through. If you do it anyway:
soft obstacle only (foam block, never a hand), `--speed 0.05`, e-stop already in
your hand, and expect the actuator to fault before anything interesting happens.

---

## Step 8 — Compute timing

Read off the end-of-run report from any of the steps above.

Position mode runs **no dynamics at all** — no RNEA, no CRBA, no IK — so it is
the cheapest control path in the driver. Sim baseline on abra:
`compute p50=128ns p99=256ns`, `majflt+=0`, `overruns=0`.

**On hardware `cycle_ns` is dominated by the KORTEX round-trip, not by compute.**
That is the expected shape; a `compute_ns` anywhere near `comm_ns` would be the
surprise.

**Acceptance:** `overruns=0`, `majflt+=0`, `dropped=0` over a ≥60 s run. Use
`--csv /tmp/jpos.csv` and keep the file.

> `NanoHistogram::percentile` returns the lower bound of a log2 bucket, so the
> printed p99 reads low (3000 ns shows as 2048). Use the CSV for anything
> conclusive.

---

## What this does NOT cover

Both belong to [issue #6](https://github.com/rammp-org/kinova-gen3-driver/issues/6)
and neither is runnable today:

- **`JointImpedanceMode` compute timing.** `teleop_socket_server` creates a
  `SampleRing` but never drains it (`apps/teleop_socket_server.cpp:520-522`), so
  there is no timing report from the joint-impedance path at all. The incoming
  transport layer owns the RT loop and can consume the ring — measure it there
  rather than instrumenting a server that is about to be replaced.
- **Impedance controller performance** — step response, tracking error vs
  commanded velocity, whether `Dq = 2ζ√(Kq·M_ii(q))` delivers the intended ζ
  across the workspace. Needs a target generator driving known joint-space steps
  and sweeps, which does not exist yet.

Also unresolved by design: mode-switch cost. `RtExecutor` calls
`set_actuator_modes` inline on the RT thread at a mode change
(`src/rt_executor.cpp:103`), which on KORTEX is 7 × `SetControlMode` plus 17
`pump()` round-trips. Nothing today switches modes mid-run, so the cost only
shows up as a one-off at startup. It becomes measurable — and worth measuring —
once the transport layer switches between torque and position live.

---

## Recording results

Paste the end-of-run report block and the `NOTE:` line from step 1 into
[issue #6](https://github.com/rammp-org/kinova-gen3-driver/issues/6), and attach
the CSV. The negative-degree outcome from step 2 is the single most useful thing
to write down — it either closes the question or opens a transport bug.
