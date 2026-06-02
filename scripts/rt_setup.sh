#!/usr/bin/env bash
# Apply runtime real-time tunings for a steady 1 kHz control loop on the
# Jetson AGX Orin (abra). RUN WITH sudo. These are NON-PERSISTENT (reset on
# reboot) — see docs/rt-tuning.md for the boot-time settings (isolcpus etc.)
# and for making these persistent.
#
#   sudo ./scripts/rt_setup.sh [RT_CORE]
#
# RT_CORE (optional, default 11): the core you intend to pin the RT loop to
# (matches `benchmark_grav_comp --cpu <RT_CORE>`). Deep idle is disabled on all
# cores; the governor is set to performance on all cores.
set -euo pipefail

RT_CORE="${1:-11}"

if [[ "${EUID}" -ne 0 ]]; then
  echo "ERROR: run with sudo (changes system tunables)." >&2
  exit 1
fi

log() { printf '[rt-setup] %s\n' "$*"; }

# 1. Power model: MAXN (all cores online, no power cap).
if command -v nvpmodel >/dev/null 2>&1; then
  nvpmodel -m 0 >/dev/null 2>&1 || true
  log "nvpmodel -> MAXN (mode 0)"
fi

# 2. Lock clocks to max (CPU/GPU/EMC) — removes frequency-scaling jitter.
if command -v jetson_clocks >/dev/null 2>&1; then
  jetson_clocks
  log "jetson_clocks applied (clocks locked to max)"
fi

# 3. CPU governor -> performance on every core (pins frequency at max so the
#    RT core never pays a ramp-up latency on wake).
for g in /sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_governor; do
  echo performance > "$g" 2>/dev/null || true
done
log "cpufreq governor -> performance (all cores)"

# 4. Disable deep CPU idle states everywhere (keep only WFI ~1us). The 'c7'
#    state on Orin has ~5 ms exit latency — catastrophic for a 1 kHz loop that
#    sleeps between cycles. (Trades idle power for determinism.)
disabled=0
for st in /sys/devices/system/cpu/cpu[0-9]*/cpuidle/state*/; do
  name="$(cat "${st}name" 2>/dev/null || echo '?')"
  lat="$(cat "${st}latency" 2>/dev/null || echo 0)"
  if [[ "$name" != "WFI" && "$lat" -gt 100 ]]; then
    echo 1 > "${st}disable" 2>/dev/null && disabled=$((disabled+1)) || true
  fi
done
log "disabled ${disabled} deep cpuidle state(s) (kept WFI)"

# 5. Remove RT bandwidth throttling. Default caps RT tasks at 95% of CPU; a
#    busy SCHED_FIFO loop (sleep-spin pacing) can be throttled. Safe BECAUSE the
#    RT loop runs on an isolated core (see boot-time isolcpus in docs).
echo -1 > /proc/sys/kernel/sched_rt_runtime_us
log "sched_rt_runtime_us -> -1 (RT throttling disabled)"

# 6. Pin timers (don't migrate them around cores).
echo 0 > /proc/sys/kernel/timer_migration 2>/dev/null || true
log "timer_migration -> 0"

# 7. Transparent huge pages -> never (khugepaged compaction causes latency spikes).
if [[ -w /sys/kernel/mm/transparent_hugepage/enabled ]]; then
  echo never > /sys/kernel/mm/transparent_hugepage/enabled
  log "transparent_hugepage -> never"
fi

# 8. Best-effort: move IRQ affinity off the RT core (only effective for IRQs
#    whose affinity is writable; isolated-core IRQ steering is the boot-time job).
moved=0
for irq in /proc/irq/[0-9]*; do
  if [[ -w "${irq}/smp_affinity_list" ]]; then
    # Steer to core 0 if currently allowed on the RT core. Ignore failures
    # (managed IRQs reject writes).
    cur="$(cat "${irq}/smp_affinity_list" 2>/dev/null || echo '')"
    if [[ ",$cur," == *",$RT_CORE,"* || "$cur" == *"-"* ]]; then
      echo 0 > "${irq}/smp_affinity_list" 2>/dev/null && moved=$((moved+1)) || true
    fi
  fi
done
log "nudged ${moved} IRQ affinities away from core ${RT_CORE} (best-effort)"

log "done. Verify with:  cat /proc/sys/kernel/sched_rt_runtime_us ; cat /sys/devices/system/cpu/cpu${RT_CORE}/cpufreq/scaling_governor"
log "Measure jitter with: sudo cyclictest -m -S -p 90 -i 1000 -D 60   (apt install rt-tests)"
log "NOTE: these reset on reboot, and core isolation (isolcpus) is a SEPARATE boot-time step — see docs/rt-tuning.md"
