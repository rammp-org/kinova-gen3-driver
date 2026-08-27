# Streaming check — the tier, on the arm

!!! warning "Temporary"
    This procedure and the `stream_check` app it drives are **bring-up
    scaffolding**. Both get deleted once the streaming tier has been validated on
    the arm. Do not build tooling on top of them.

**Attended procedure.**

## Why this exists

The unit tests prove the session state machine, the write handoff (raced
directly, and clean under ThreadSanitizer) and the watchdog — in simulation.
What they **cannot** prove is what the arm does when a stream goes stale or a
session closes, because `SimTransport` is a static echo: it never moves, so
*"the reference froze at measured q"* and *"the reference froze at zero"* are
indistinguishable there.

That distinction is the entire safety story of the tier, and only hardware can
settle it.

## Safety posture — it depends on the mode you pick

These are not alike, and the app prints which one you chose before it starts:

| `--mode` | posture |
|---|---|
| `position` | **No compliance.** The actuator servo chases the command at full authority. Nothing absorbs a mistake. |
| `impedance` | Compliant — the arm yields to contact. **Start here.** |
| `torque` | Compliant, but no spring pulls it back: a sustained feedforward keeps accelerating the joint. |

Defaults are one wrist joint and 0.05 rad of travel. The app refuses a `--delta`
above 0.2 rad, a `--tau` above half the target joint's clamp, and a `--timeout`
of zero or less — an unbounded stream has no safe-stop, and the driver refuses it
at open anyway.

## The four phases

The run is scripted, and **the second phase is the test, not a pause**:

1. **STREAM** — setpoints at `--cmd-rate`, walking one joint by `--delta`.
2. **STALE** — the app *stops pushing* for `--stale-s`. The mode's own watchdog
   should make the output safe at 1 kHz.
3. **RESUME** — pushing again, confirming the freeze releases on a fresh command.
4. **CLOSE** — the session closes; the teardown latches hold-at-measured-q.

## Procedure

**1. Read-only.** Connects, reads, commands nothing, never enters low-level
servoing:

```sh
./build/stream_check --ip 192.168.1.10 --dry-run
```

**2. Joint positions into impedance mode** — the compliant path, start here:

```sh
./build/stream_check --ip 192.168.1.10 --kind joint-position --mode impedance
```

**3. Then the other supported pairs**, once you trust the first:

```sh
./build/stream_check --ip 192.168.1.10 --kind pose         --mode impedance
./build/stream_check --ip 192.168.1.10 --kind joint-torque --mode torque --tau 1.0
./build/stream_check --ip 192.168.1.10 --kind joint-position --mode position   # no compliance
```

The velocity kinds are refused — they need `JointVelocityMode`, which does not
exist yet. The app rejects those pairs before it ever connects.

## What to check

The app prints a 20 Hz trace with each sample labelled by phase, then a checklist:

- **STREAM** — the joint moves toward the commanded value, smoothly and
  rate-limited, never a snap.
- **STALE** — motion **stops** within about `--timeout` of the last setpoint.
  Position and impedance freeze the reference at measured q; torque ramps its
  feedforward out and settles to gravity-compensation hold. *Drift here is the
  finding.*
- **RESUME** — motion picks up again, so the freeze released rather than latching
  permanently.
- **CLOSE** — the arm holds where it is and does **not** slew back toward the last
  streamed setpoint. A slew here means the teardown hold failed, which is the bug
  the `close_stream` hold latch exists to prevent.
- **Throughout** — no goal-settle lines appear. The app prints one loudly if a
  goal settles during a stream, which would mean the mutual exclusion leaked.

## Known residual

A microsecond-wide interleaving remains where post-e-stop motion is bounded to
one sampler period (~2 mrad). It is documented in the streaming guide; this
procedure does not exercise it, and you would not see 2 mrad by eye.

## Note on `--sim`

`--sim` exercises the plumbing only: the arm never moves, the trace is flat, and
the STALE phase proves nothing about hardware. Use it to confirm the app parses
and runs, nothing more.
