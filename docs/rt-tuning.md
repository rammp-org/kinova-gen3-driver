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

## C. Per-run (the binary)

The RT loop needs scheduling privileges and should be pinned to the isolated
core. Grant capabilities once (preferred over running the whole app as root):

```sh
sudo setcap cap_sys_nice,cap_ipc_lock+ep ./benchmark_grav_comp
```

Then run pinned to the isolated core with a high RT priority:

```sh
./benchmark_grav_comp --sim --urdf ../models/gen3_7dof.urdf \
  --rate 1000 --cpu 11 --rt-priority 80 --pacing sleepspin --duration 60
```

Confirm in the final `introspect()` report: `policy: FIFO`, the expected
`priority`/`cpu`, and `majflt+=0` (proves `mlockall` locked memory). A non-zero
**involuntary context-switch** count on the RT thread means something still
preempted it — revisit isolation.

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

1. `sudo ./scripts/rt_setup.sh 11` (runtime tunings)
2. Edit `extlinux.conf` → `isolcpus=11 nohz_full=11 rcu_nocbs=11` → reboot, then
   re-run step 1 (or enable the systemd unit).
3. `sudo setcap cap_sys_nice,cap_ipc_lock+ep ./benchmark_grav_comp`
4. `sudo cyclictest ... -a 11` → record max latency baseline.
5. Run the driver `--cpu 11 --rt-priority 80`; confirm `policy: FIFO`,
   `majflt+=0`, zero overruns, low involuntary context switches.
