# Real-Time Tuning for a Steady Loop (Jetson AGX Orin)

A PREEMPT_RT kernel is necessary but **not sufficient** for a steady 1 kHz
control loop. This is a generic guide to the platform settings that determine
loop-timing stability — what each one is, why it matters, and how to set it on a
Jetson AGX Orin (tegra234, 12 cores; developed against kernel `5.15-rt-tegra`).
The goal: low and *bounded* wake jitter and no missed deadlines (a missed 1 kHz
deadline can trip the Gen3 watchdog and fault the arm).

Throughout, the examples pin the RT loop to **core 11** — pick the
highest-numbered core on your machine and substitute it everywhere.

## What to audit before trusting timing numbers

A capable RT kernel with max power is still not enough. On a stock Jetson the
items below default to settings that show up as tail latency / jitter; audit and
fix each one before recording any numbers.

| Setting | Stock / untuned default | Want | Set in |
|---|---|---|---|
| PREEMPT_RT kernel | (must be flashed) | `/sys/kernel/realtime == 1` | kernel |
| Power model | varies | MAXN (all cores, no cap) | §A |
| CPU governor | `schedutil` (floating freq) | `performance` | §A |
| Clocks locked | not applied | locked at max (`jetson_clocks`) | §A |
| Core isolation | none | isolate ≥1 core | §B (boot) |
| Deep CPU idle | `c7` enabled (~5 ms exit) | disabled on RT cores | §A |
| RT throttling | `sched_rt_runtime_us = 950000` (95%) | `-1` (with isolation) | §A |
| `timer_migration` | `1` | `0` | §A |
| Transparent huge pages | varies | `never` | §A |
| `cyclictest` (`rt-tests`) | not installed | installed | §D |

Verify the kernel and a setting or two first:

```sh
cat /sys/kernel/realtime                                    # 1
uname -r                                                    # ...-rt-tegra
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor   # schedutil (until tuned)
```

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
inode and is wiped each time you recompile.

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
- **Disable deep idle (`c7`, ~5 ms exit)** — between cycles the loop sleeps; if
  the core drops into a deep C-state, the wake can be delayed by milliseconds.
  Keep only `WFI` (~1 µs).
- **`sched_rt_runtime_us = -1`** — the default throttles RT tasks to 95% of a
  core; sleep-spin pacing busy-waits and can be throttled. Safe **only** with
  core isolation (otherwise a runaway RT thread can wedge the core).
- **`timer_migration = 0`, THP `never`** — remove timer bounce and khugepaged
  compaction stalls.

## A2. NIC interrupt coalescing on the arm link (the largest single win)

**This one cost 80% of the cycle budget for weeks without being noticed, and no
metric the driver reports about itself shows it.** If you are chasing loop
timing on a new machine, check this before anything else.

The Jetson's `nvethernet` driver defaults `rx-usecs` to **512**. That timer is
armed by the arriving packet, so every reply from the arm sits in the NIC for a
fixed ~512 µs before the kernel sees it. At 1 kHz that is most of the period
spent waiting on a timer — not on the network, and not on the arm.

Measured on abra (`eno1`, 1 Gb direct link, 7.2M cycles before / 156k after):

| | `rx-usecs 512` (default) | `rx-usecs 8` |
| --- | --- | --- |
| comm p50 | 799 µs | **282 µs** |
| comm p99 | 979 µs | **594 µs** |
| comm p99.9 | 1242 µs | **658 µs** |
| comm max | 14795 µs | **1103 µs** |
| overruns | 0.364% | **0.001%** |

At the default, p99 sat 21 µs under a 1000 µs budget — about 2% margin — so any
small perturbation produced visible stutter. Note what did *not* change: wake
jitter p99 stayed at 0.1 µs and compute p99 at 0.5 µs throughout. The loss
happens before the kernel is involved, which is exactly why in-process
telemetry cannot see it. The `TelemetrySink` summary line now warns when comm
p50 exceeds 40% of the period, so it is at least loud from now on.

### Diagnose it in 30 seconds, without the driver

Compare a slow ping (each packet arms a fresh timer) against a fast one (the
timer is already armed, so packets ride an existing one):

```sh
ping -c 8   -i 0.3   192.168.1.10      # slow: 0.642 ms min  <- the penalty
ping -c 200 -i 0.002 192.168.1.10      # fast: 0.114 ms min  <- the true RTT
```

A direct 1 Gb link one hop away should be ~0.1 ms. A ~0.5 ms gap between those
two numbers **is** the coalescing delay, and it matches `rx-usecs` almost to
the microsecond. This needs no driver, no arm motion, and no root.

### Applying it

`ethtool -C` is **refused while the interface is up** — the driver logs
`Coalesce parameters can be changed only if interface is down`. Two commands
are needed, and they do different jobs:

```sh
sudo nmcli device disconnect eno1   # stop NM (autoconnect=yes) racing you back up
sudo ip link set eno1 down          # actually clear IFF_UP -- the flag the driver tests
sudo ethtool -C eno1 rx-usecs 8 rx-frames 1
sudo nmcli device connect eno1      # hand it back to NM, which owns the IP config
```

Gotchas, each of which cost a round trip to discover:

- **`nmcli device disconnect` alone is not enough.** It tears down the
  connection and IP config but leaves the interface `UP`, so the change is
  still refused. You need both commands, in that order.
- **`ip link set eno1 up` alone is not enough either.** The link comes up with
  no IPv4 address and traffic for the arm silently leaks out over the default
  route (wifi), which looks like 100% packet loss to the arm. Reconnect through
  NM.
- **Stop the driver first.** It holds an open KORTEX session over that link.
- **8 is the floor on this hardware.** 0, 1, 2 and 4 are refused; `rx-frames`
  accepts 1.
- **Do not bounce a link your ssh session arrives over.** You will be cut off
  mid-change with the link down. `ip route get <your client ip>` tells you.

`scripts/rt_setup.sh` does all of this (step 9), idempotently — it reads the
current value, skips the bounce entirely when already correct, and refuses to
bounce if the driver is running or if your ssh session would be cut.

### It is chronic, not a regression

Worth stating plainly, because it is easy to misread a large improvement as
having found a *new* fault. Checked against 27 archived HIL runs in
`/home/abra/kinova-hil-runs/*/timing.csv` from 2026-09-17/18: **every one** shows
comm p50 of 770–944 µs with the same ~512 µs signature above its own floor. The
penalty has been present in all recorded history. Fixing it removes chronic
fragility; it does not explain any particular stutter you are investigating.

### Multiple devices on the arm link

Coalescing hurts most at *low* packet rates, which is exactly the arm at ~1
kpps. Measured 1070 interrupts/s for 1007 packets/s — 1:1 — so the 512 µs timer
was batching nothing: only one packet ever arrived inside a window. It was pure
latency with no throughput benefit, and the change costs nothing.

Under heavier traffic (cameras sharing the link) NAPI switches to polling and
batches regardless of the coalescing config, so per-packet interrupt cost does
not scale linearly. The real concern there is different: camera packets share
the arm's receive path and can queue ahead of its replies, and `NET_RX` softirq
work already lands on the RT core. Prefer giving bulk devices their own NIC; if
they must share, steer RPS explicitly away from the RT core and re-measure.

## B. Boot-time tunings (kernel cmdline — needs a reboot)

Core **isolation** is the single biggest win for tail latency and must be set on
the kernel command line. On the Jetson this is edited in
`/boot/extlinux/extlinux.conf` (not GRUB). Append to the `APPEND` line of the
primary boot entry (substitute your chosen core for `11`):

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
```

Pick the **highest-numbered** core so it's least likely to host boot/IRQ
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
sudo cyclictest -m -S -p 90 -i 1000 -d 0 -D 60     # all cores
# Target the ISOLATED core — must use taskset, NOT -a (see gotcha below):
sudo taskset -c 11 cyclictest -m -t1 -p 90 -i 1000 -D 60
```

> **Gotcha — isolated core + `cyclictest -a`:** `isolcpus=11` removes core 11 from
> the *default* affinity mask (a normal process shows `Cpus_allowed_list: 0-10` —
> this is the fence working). `cyclictest -a 11` (V2.20) intersects `-a 11` with
> that inherited `0-10` mask, gets nothing, and dies with `WARN: Couldn't
> setaffinity ... Invalid argument` / `FATAL: No allowable cpus to run on`.
> **Use `taskset -c 11 cyclictest -t1 …`** instead — it sets the mask explicitly.
> The driver is unaffected: `--cpu 11` calls `sched_setaffinity` explicitly.

Read the **Max** latency — that's the worst-case wake delay, the number that
matters for a hard 1 kHz deadline. As a sense of the achievable improvement on a
tuned Jetson AGX Orin: an untuned box can show a worst case in the tens of µs
that climbs under load, while tuning + isolating a core brings it into the
single-digit µs range *and holds there under heavy load* — because nothing else
can schedule onto the isolated core. Record your own baseline and keep it with
your test notes.

`cyclictest` validates the platform independent of our code. Then cross-check
with the driver's own `--pacing sleepspin` vs `--pacing nanosleep` cycle-time
histograms over a 60 s run — they should agree on the order of magnitude.

## E. Persistence (optional)

Section A resets on reboot. To make it stick, install a oneshot systemd unit
that runs `rt_setup.sh` at boot (after `jetson_clocks`’ own service), e.g.
`/etc/systemd/system/rt-setup.service` (point `ExecStart` at your checkout):

```ini
[Unit]
Description=RT tunings for kinova-gen3-driver
After=nvpmodel.service jetson_clocks.service

[Service]
Type=oneshot
ExecStart=/home/<user>/kinova-gen3-driver/scripts/rt_setup.sh 11

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
3. `sudo ethtool -c eno1 | grep rx-usecs` — if it is not 8, see **A2**. This is
   the largest single win and the easiest to miss; `rt_setup.sh` handles it.
4. Edit `extlinux.conf` → `isolcpus=11 nohz_full=11 rcu_nocbs=11` → reboot.
5. `sudo taskset -c 11 cyclictest -m -t1 -p 90 -i 1000 -D 1h` → record
   max-latency baseline.
6. Run the driver (NO sudo) `--cpu 11 --rt-priority 80`; confirm `policy: FIFO`,
   `cpu_dma_latency: pinned@0us`, `majflt+=0`, zero overruns, low involuntary
   context switches.

Steps 2–5 are system/boot-global (governor, isolation) and improve the tail,
especially under load. Step 1 is the only one required for the driver to get
real RT, and it's a one-time grant — no per-run sudo.
