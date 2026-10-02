#!/usr/bin/env bash
# Apply runtime real-time tunings for a steady 1 kHz control loop on a
# Jetson AGX Orin. RUN WITH sudo. These are NON-PERSISTENT (reset on
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
    # shellcheck disable=SC2015  # the || branch is `true`; this is 'try, ignore failure'
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
      # shellcheck disable=SC2015  # the || branch is `true`; this is 'try, ignore failure'
      echo 0 > "${irq}/smp_affinity_list" 2>/dev/null && moved=$((moved+1)) || true
    fi
  fi
done
log "nudged ${moved} IRQ affinities away from core ${RT_CORE} (best-effort)"

# 9. NIC interrupt coalescing on the arm link.
#    MEASURED 2026-10-01: the nvethernet default rx-usecs=512 holds every reply
#    from the arm in the NIC for a fixed ~512 us before the kernel sees it. At
#    1 kHz that is 80% of the cycle budget spent waiting on a timer:
#
#      rx-usecs 512 -> comm p50 799us, p99.9 1242us, max 14795us, overruns 0.364%
#      rx-usecs 8   -> comm p50 282us, p99.9  658us, max  1103us, overruns 0.001%
#
#    It costs nothing: at ~1 packet/ms only one packet ever arrives inside a
#    512 us window, so the timer batched nothing (measured 1070 interrupts/s for
#    1007 packets/s, i.e. 1:1 either way). Pure latency, no throughput benefit.
#
#    Diagnose without the driver: `ping -c8 -i0.3` (fresh timer each packet) vs
#    `ping -c200 -i0.002` (timer already armed). 0.64ms vs 0.11ms = coalescing.
#
#    nvethernet REFUSES the change while the interface is up ("Coalesce
#    parameters can be changed only if interface is down"), so this bounces the
#    link -- but only when the value is actually wrong, making it idempotent and
#    safe to re-run.
ARM_IFACE="${ARM_IFACE:-eno1}"
ARM_RX_USECS="${ARM_RX_USECS:-8}"      # 8 is the lowest this hardware accepts (0/1/2/4 refused)
ARM_RX_FRAMES="${ARM_RX_FRAMES:-1}"

coalesce_now() { ethtool -c "$1" 2>/dev/null | awk -v k="^$2:" '$0 ~ k {print $2}'; }

if ! command -v ethtool >/dev/null 2>&1; then
  log "SKIP coalescing: ethtool not installed"
elif [[ ! -e "/sys/class/net/${ARM_IFACE}" ]]; then
  log "SKIP coalescing: no interface ${ARM_IFACE}"
else
  cur_u="$(coalesce_now "$ARM_IFACE" rx-usecs)"
  cur_f="$(coalesce_now "$ARM_IFACE" rx-frames)"
  if [[ "$cur_u" == "$ARM_RX_USECS" && "$cur_f" == "$ARM_RX_FRAMES" ]]; then
    log "coalescing already rx-usecs=${cur_u} rx-frames=${cur_f} on ${ARM_IFACE} (no bounce needed)"
  else
    # Never cut the link this session came in over: the bounce would kill it
    # mid-change and the reconnect would never run.
    via=""
    if [[ -n "${SSH_CONNECTION:-}" ]]; then
      client="$(printf '%s' "$SSH_CONNECTION" | awk '{print $1}')"
      via="$(ip route get "$client" 2>/dev/null | sed -n 's/.*dev \([^ ]*\).*/\1/p' | head -1)"
    fi
    if [[ "$via" == "$ARM_IFACE" ]]; then
      log "SKIP coalescing: this ssh session returns over ${ARM_IFACE}; bouncing it would cut you off."
      log "  reconnect over another interface, or run: sudo ethtool -C ${ARM_IFACE} rx-usecs ${ARM_RX_USECS} rx-frames ${ARM_RX_FRAMES} (link down)"
    elif pgrep -f 'kinova_gen3_no[d]e' >/dev/null 2>&1; then
      log "SKIP coalescing: the arm driver is running and holds a session over ${ARM_IFACE}."
      log "  stop it, then re-run this script (currently rx-usecs=${cur_u})"
    else
      log "coalescing on ${ARM_IFACE}: rx-usecs ${cur_u}->${ARM_RX_USECS}, rx-frames ${cur_f}->${ARM_RX_FRAMES} (bouncing the link)"
      # NetworkManager (autoconnect) will race a bare `ip link down`, and
      # `nmcli device disconnect` alone leaves IFF_UP set -- the flag the driver
      # tests. Both are needed, in this order.
      command -v nmcli >/dev/null 2>&1 && { nmcli device disconnect "$ARM_IFACE" >/dev/null 2>&1 || true; }
      ip link set "$ARM_IFACE" down 2>/dev/null || true
      for _ in $(seq 1 20); do
        ip link show "$ARM_IFACE" | head -1 | grep -q "state DOWN" && break
        sleep 0.5
      done
      if ip link show "$ARM_IFACE" | head -1 | grep -q "state DOWN"; then
        ethtool -C "$ARM_IFACE" rx-usecs "$ARM_RX_USECS" 2>/dev/null || log "  WARN: rx-usecs=${ARM_RX_USECS} refused"
        ethtool -C "$ARM_IFACE" rx-frames "$ARM_RX_FRAMES" 2>/dev/null || log "  WARN: rx-frames=${ARM_RX_FRAMES} refused"
      else
        log "  WARN: ${ARM_IFACE} never reached state DOWN; the driver will refuse the change"
      fi
      # Hand the device back to NM, which owns the IP config. Bringing the link
      # up with `ip link set up` alone leaves it addressless and arm traffic
      # silently leaks out over the default route.
      if command -v nmcli >/dev/null 2>&1; then
        nmcli device connect "$ARM_IFACE" >/dev/null 2>&1 || true
      else
        ip link set "$ARM_IFACE" up 2>/dev/null || true
      fi
      for _ in $(seq 1 30); do
        ip -o -4 addr show dev "$ARM_IFACE" 2>/dev/null | grep -q inet && break
        sleep 0.5
      done
      if ip -o -4 addr show dev "$ARM_IFACE" 2>/dev/null | grep -q inet; then
        log "  coalescing now rx-usecs=$(coalesce_now "$ARM_IFACE" rx-usecs) rx-frames=$(coalesce_now "$ARM_IFACE" rx-frames); ${ARM_IFACE} reconfigured"
      else
        log "  WARN: ${ARM_IFACE} has NO IPv4 address. Fix: nmcli device connect ${ARM_IFACE}"
      fi
    fi
  fi
fi

log "done. Verify with:  cat /proc/sys/kernel/sched_rt_runtime_us ; cat /sys/devices/system/cpu/cpu${RT_CORE}/cpufreq/scaling_governor"
log "Measure jitter with: sudo cyclictest -m -S -p 90 -i 1000 -D 60   (apt install rt-tests)"
log "NOTE: these reset on reboot, and core isolation (isolcpus) is a SEPARATE boot-time step — see docs/rt-tuning.md"
