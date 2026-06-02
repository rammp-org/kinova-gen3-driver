# RT Validation — Steps & Results (Jetson AGX Orin / `abra`)

Record of the real-time tuning applied to `abra` and the measured loop-timing
results. Date: 2026-06-02. Platform: NVIDIA Jetson AGX Orin (tegra234, 12 cores),
kernel `5.15.185-rt-tegra` (PREEMPT_RT). See [`rt-tuning.md`](rt-tuning.md) for
the full rationale of each setting; this doc is the as-run procedure + numbers.

## What was applied

1. **One-time no-sudo grant** — `sudo ./scripts/rt_grant_once.sh`
   (`realtime` group + `rtprio`/`memlock` limits + `/dev/cpu_dma_latency` udev
   rule). Persists across reboots; lets the driver use FIFO / mlock / C-state pin
   as a normal user.
2. **Runtime tunings** — `sudo ./scripts/rt_setup.sh 11` (governor → performance,
   `jetson_clocks`, deep C-states disabled, RT throttling off, THP off).
   **Non-persistent — reset on reboot** (re-run, or install the systemd unit in
   `rt-tuning.md §E`).
3. **Core isolation** — bootloader (`/boot/extlinux/extlinux.conf`): a new `rt`
   `LABEL` (the `primary` `APPEND` + `isolcpus=11 nohz_full=11 rcu_nocbs=11`),
   `DEFAULT rt`, `primary` kept as the fallback. Persists (bootloader). Confirmed
   after reboot: `/proc/cmdline` has all three, `/sys/devices/system/cpu/isolated
   = 11`.

The driver itself does the per-process RT setup in `rt_system::enable_rt()`:
`SCHED_FIFO`, `mlockall`, `sched_setaffinity(--cpu)`, and pinning
`/dev/cpu_dma_latency` to 0 µs (suppresses deep idle for our process).

## cyclictest results — the progression

`cyclictest` measures pure OS wake-up latency (µs) at the 1 kHz interval. The
worst-case (**Max**) is the number that matters.

| Stage | Command | Load (1-min) | Min | Avg | Max |
|---|---|---|---|---|---|
| Untuned, ~idle | `cyclictest -m -a 11 -p90 -i1000 -D1m` | 2.4 | 1 | 4 | **48 µs** |
| Untuned, heavy load | `cyclictest -m -a 11 -p90 -i1000 -D1m` | 16.6 | 2 | 7 | **34 µs** |
| Untuned (post-reboot, tunings wiped), load | same | 15.0 | 2 | 6 | **35 µs** |
| **Tuned + isolated, heavy load** | `taskset -c 11 cyclictest -m -t1 -p90 -i1000 -D1m` | **19.6** | **1** | **1** | **7 µs** |

**Takeaways:**
- Tuning + isolation cut worst-case wake latency from **~48 µs → 7 µs**, and avg
  from 4 → 1 µs.
- The decisive property: the **7 µs held under loadavg ~19.6** (heavily
  oversubscribed). On an isolated core, system load can't perturb the loop —
  untuned runs were both higher and load-sensitive.
- Why a *busy* untuned core (34 µs) looked better than an *idle* untuned one
  (48 µs): a busy core stays hot and out of deep idle, accidentally masking the
  governor/C-state weaknesses that the tuning fixes deliberately. Our loop sits
  on an *idle* isolated core between cycles, so the tuning is what protects it.

## Driver results (`benchmark_grav_comp`, sim transport, no sudo)

Per-cycle timing from the driver's own telemetry, pinned to core 11, FIFO @ 80,
`cpu_dma_latency: pinned@0us`. Sim transport so `comm ≈ 0`; this isolates wake
jitter + compute.

| Cycle time | Un-isolated (core not fenced) | **Isolated + tuned** |
|---|---|---|
| p50 | 1.0 µs | **0.5 µs** |
| p99 | 2.0 µs | **1.0 µs** |
| p99.9 | 4.1 µs | **1.0 µs** |
| max | 52.7 µs | **25 µs** (one-time Pinocchio warm-up) |
| major faults (run delta) | — | **0** |

- p99.9 dropped 4 µs → 1 µs and the **recurring tail vanished**; the 25 µs max is
  the single first-cycle Pinocchio warm-up (`compute max ≈ 22 µs`), not a
  recurring spike. Steady-state major page faults: **0** (mlockall working).
- `nivcsw` (involuntary context switches) reads ~111 over a run, but that counter
  is **cumulative for the whole process incl. the unpinned startup** (model load,
  seeding) — it is not the steady-state rate. The clean p99.9 and `majflt+=0` are
  the evidence the steady loop is undisturbed. Residual switches on an isolated
  core come from per-CPU kernel threads (`migration`@99, `kworker`/`ksoftirqd`),
  the `nohz_full` ~1 Hz residual tick, and IPIs — which isolation does not remove.

## Gotcha: cyclictest on an isolated core

`isolcpus=11` removes core 11 from the **default** affinity mask, so a normal
process shows `Cpus_allowed_list: 0-10` (this is the fence working — nothing lands
on 11 unless it explicitly asks). Consequences:
- `cyclictest -a 11` (V2.20) intersects `-a 11` with the inherited `0-10` mask,
  gets the empty set, and fails: `WARN: Couldn't setaffinity ... Invalid argument`
  / `FATAL: No allowable cpus to run on`.
- **Fix:** launch under `taskset`, which sets the mask explicitly:
  `sudo taskset -c 11 cyclictest -m -t1 -p 90 -i 1000 -D 1m`.
- The **driver is unaffected** — `--cpu 11` calls `sched_setaffinity` explicitly,
  which is permitted (the cgroup cpuset includes 11).

## Conclusion & what remains

The platform is **validated for a 1 kHz hard-RT loop**: 7 µs worst-case wake
latency under load, driver p99.9 = 1 µs, zero steady-state major faults — all
**without runtime sudo**. Remaining items, for the live-robot session:

1. **NIC IRQ.** The one jitter source not exercised here is the robot's UDP
   interrupt. When the Gen3 is connected, pin its NIC IRQ to a housekeeping core
   (not 11) so it doesn't land on the RT core. This is the last real jitter knob.
2. **Persistence.** Governor/C-state tunings (`rt_setup.sh`) reset on reboot —
   enable the systemd unit (`rt-tuning.md §E`). Isolation already persists.
3. **Real numbers.** These are sim-transport (`comm ≈ 0`). The live `comm`
   round-trip adds to cycle time but is robot/network latency, not loop jitter.
