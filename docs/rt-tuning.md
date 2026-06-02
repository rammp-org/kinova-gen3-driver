# Real-Time Tuning for a Steady Loop (Jetson AGX Orin / `abra`)

A PREEMPT_RT kernel is necessary but **not sufficient** for a steady 1 kHz
control loop. This document is the checklist of settings that determine
loop-timing stability, their state on `abra` as of 2026-06-02, why each matters,
and exactly how to set them. The goal: low and *bounded* wake jitter, no missed
deadlines (which can trip the Gen3 watchdog and fault the arm).

## Current state on `abra` (2026-06-02)

`abra` = NVIDIA Jetson AGX Orin (tegra234, 12 cores), kernel `5.15.185-rt-tegra`.

| Setting | State | Want | Status |
|---|---|---|---|
| PREEMPT_RT kernel | `5.15-rt`, `/sys/kernel/realtime=1` | RT kernel | ✅ |
| Power model | MAXN (mode 0, all cores, no cap) | MAXN | ✅ |
| CPU governor | `schedutil` (cpu0 floating 0.7–2.2 GHz) | `performance` | ❌ |
| Clocks locked | `jetson_clocks` not applied | locked at max | ❌ |
| Core isolation | none (`isolcpus`/`nohz_full`/`rcu_nocbs` absent) | isolate ≥1 core | ❌ |
| Deep CPU idle | `c7` enabled, **5000 µs** exit latency | disabled on RT cores | ❌ |
| RT throttling | `sched_rt_runtime_us = 950000` (95%) | `-1` (with isolation) | ⚠️ |
| timer_migration | `1` | `0` | ⚠️ |
| Transparent huge pages | (verify) | `never` | ⚠️ |
| cyclictest (`rt-tests`) | not installed | installed | ❌ |

**Verdict: capable kernel + max power, but NOT yet tuned for steady timing.**
The governor, deep C-state, missing isolation, and unlocked clocks are the items
that will show up as tail-latency / jitter. Apply the steps below before
trusting any timing numbers.

---

## Privilege model — run the driver without `sudo`

The driver must run as a normal user (no `sudo` at run time). What needs
privilege, and how each is granted *once* so the driver itself does the work:

| RT action | Where it happens | Privilege (granted once) |
|---|---|---|
| Core affinity (`--cpu`) | in `rt_system` | none |
| `mlockall` + lock memory | in `rt_system` | `memlock` rlimit |
| `SCHED_FIFO` priority | in `rt_system` | `rtprio` rlimit |
| Pin C-states (`/dev/cpu_dma_latency`) | in `rt_system` | udev rule on the device |

All four are **driver-local** (done in `rt_system::enable_rt()`); they only need
the user to *have permission*. Grant it once with:

```sh
sudo ./scripts/rt_grant_once.sh        # creates 'realtime' group, rtprio/memlock
                                        # limits, and the cpu_dma_latency udev rule
# log out + back in (group membership), then run the driver with NO sudo.
```

This is preferred over `setcap cap_sys_nice,cap_ipc_lock+ep` because the grant is
on the *user/group*, so it **survives every rebuild** — `setcap` is on the binary
inode and is wiped each time you recompile (painful in the build-on-abra loop).

`enable_rt()` is best-effort: if the grant isn't in place it logs a note and runs
at `SCHED_OTHER` (the driver still *runs* without sudo, just without hard-RT
guarantees). Confirm success in the report: `policy: FIFO` and
`cpu_dma_latency: pinned@0us`.

The settings below (governor, clocks, isolation, throttling) are **not**
driver-local — they are CPU-/system-global and need root or a boot edit. They
improve the tail (especially under load) but aren't required for the driver to
run.

## A. Runtime tunings (scripted, non-persistent)

Run the provided script with `sudo`. It sets the governor, locks clocks,
disables deep idle, removes RT throttling, pins timers, disables THP, and nudges
IRQs off the RT core. **These reset on reboot.**

```sh
sudo ./scripts/rt_setup.sh 11        # 11 = the core you'll pin the loop to
```

Verify afterward:

```sh
cat /sys/devices/system/cpu/cpu11/cpufreq/scaling_governor   # performance
cat /proc/sys/kernel/sched_rt_runtime_us                     # -1
cat /sys/devices/system/cpu/cpu11/cpuidle/state*/disable      # 0 (WFI) then 1 (c7)
```

Why each (the script documents inline too):
- **`performance` governor + `jetson_clocks`** — a floating governor wakes the
  core at a low frequency and ramps up, adding variable latency to the first
  work after each sleep. Pinning at max removes that ramp.
- **Disable deep idle (`c7`, 5 ms exit)** — between cycles the loop sleeps; if
  the core drops into a deep C-state, the wake can be delayed by milliseconds.
  Keep only `WFI` (~1 µs).
- **`sched_rt_runtime_us = -1`** — the default throttles RT tasks to 95% of a
  core; our sleep-spin pacing busy-waits and can be throttled. Safe **only**
  with core isolation (otherwise a runaway RT thread can wedge the core).
- **`timer_migration = 0`, THP `never`** — remove timer bounce and khugepaged
  compaction stalls.

## B. Boot-time tunings (kernel cmdline — needs a reboot)

Core **isolation** is the single biggest win for tail latency and must be set on
the kernel command line. On the Jetson this is edited in
`/boot/extlinux/extlinux.conf` (not GRUB). Append to the `APPEND` line of the
primary boot entry:

```
isolcpus=11 nohz_full=11 rcu_nocbs=11
```

- `isolcpus=11` — the scheduler won't place other tasks on core 11.
- `nohz_full=11` — stop the periodic scheduler tick on core 11 when one task
  runs (removes ~per-ms tick jitter).
- `rcu_nocbs=11` — offload RCU callbacks off core 11.

Then reboot and confirm:

```sh
cat /proc/cmdline | grep -o 'isolcpus=[^ ]*'      # isolcpus=11
cat /sys/devices/system/cpu/cpu11/topology/...    # core present
```

Pick the **highest-numbered** core (11) so it's least likely to host boot/IRQ
defaults. Isolate only what you need (1 core for a single RT loop) — isolating
cores removes them from the general scheduler pool.

> ⚠️ Editing `extlinux.conf` wrong can make the Jetson unbootable. Keep the
> original `APPEND` line as a second `LABEL` entry so you can fall back. Do this
> step with physical/console access, not over a flaky SSH link.

## C. Per-run (the binary) — no sudo

After the one-time grant (privilege-model section above), run pinned to the
isolated core with a high RT priority — **as a normal user, no sudo**:

```sh
./benchmark_grav_comp --sim --urdf ../models/gen3_7dof.urdf \
  --rate 1000 --cpu 11 --rt-priority 80 --pacing sleepspin --duration 60
```

The driver sets `SCHED_FIFO`, `mlockall`, affinity, and pins `/dev/cpu_dma_latency`
to 0 µs itself (the last one suppresses deep C-states for our process — the same
trick cyclictest uses — so we don't depend on the system-wide cpuidle disable in
§A). Confirm in the final `introspect()` report:
- `policy: FIFO`, the expected `priority`/`cpu`
- `majflt+=0` (proves `mlockall` locked memory)
- `cpu_dma_latency: pinned@0us` (deep idle suppressed for our process)
- low **involuntary context-switch** count — a non-zero count means something
  still preempted the RT thread; revisit isolation (§B).

## D. Validate — measure, don't assume

Install the standard RT latency probe and get a number:

```sh
sudo apt install rt-tests
sudo cyclictest -m -S -p 90 -i 1000 -d 0 -D 60     # 1 kHz, 60 s, all cores
# or target the isolated core:
sudo cyclictest -m -t 1 -a 11 -p 90 -i 1000 -D 60
```

Read the **Max** latency. Rough expectations on a well-tuned Orin:
- Untuned (current state): max can spike into the **hundreds of µs–ms**.
- After A+B+C: max should be **tens of µs**, ideally < ~50 µs.

`cyclictest` validates the platform independent of our code. Then cross-check
with the driver's own `--pacing sleepspin` vs `--pacing nanosleep` cycle-time
histograms over a 60 s run — they should agree on the order of magnitude.

## E. Persistence (optional)

Section A resets on reboot. To make it stick, install a oneshot systemd unit
that runs `rt_setup.sh` at boot (after `jetson_clocks`’ own service), e.g.
`/etc/systemd/system/rt-setup.service`:

```ini
[Unit]
Description=RT tunings for kinova-gen3-driver
After=nvpmodel.service jetson_clocks.service

[Service]
Type=oneshot
ExecStart=/home/abra/kinova-gen3-driver/scripts/rt_setup.sh 11

[Install]
WantedBy=multi-user.target
```

`sudo systemctl enable rt-setup.service`. (Section B is already persistent — it's
in the bootloader config.)

---

## Quick order of operations

1. `sudo ./scripts/rt_grant_once.sh` — one-time: lets the driver use FIFO / mlock
   / cpu_dma_latency **without sudo** (survives rebuilds). Log out + back in.
2. `sudo ./scripts/rt_setup.sh 11` — system-global runtime tunings (governor,
   clocks, C-states, throttling). Re-run after reboot, or enable the systemd unit.
3. Edit `extlinux.conf` → `isolcpus=11 nohz_full=11 rcu_nocbs=11` → reboot.
4. `sudo cyclictest -m -a 11 -p 90 -i 1000 -D 1h` → record max-latency baseline.
5. Run the driver (NO sudo) `--cpu 11 --rt-priority 80`; confirm `policy: FIFO`,
   `cpu_dma_latency: pinned@0us`, `majflt+=0`, zero overruns, low involuntary
   context switches.

Steps 2–4 are system/boot-global (governor, isolation) and improve the tail,
especially under load. Step 1 is the only one required for the driver to get
real RT, and it's a one-time grant — no per-run sudo.
