#!/usr/bin/env bash
# ONE-TIME setup so the driver can use SCHED_FIFO, mlockall, and the
# /dev/cpu_dma_latency C-state pin AS A NORMAL USER — no `sudo` at run time, and
# (unlike `setcap`) it survives every rebuild because the grant is on the
# user/group, not the binary inode.
#
#   sudo ./scripts/rt_grant_once.sh [USER]
#
# USER defaults to the invoking sudo user. Creates a 'realtime' group, adds the
# user, grants rtprio/memlock limits, and a udev rule for /dev/cpu_dma_latency.
# Log out and back in afterward for the group membership to take effect.
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "ERROR: run with sudo (writes /etc/security/limits.d and /etc/udev)." >&2
  exit 1
fi
TARGET_USER="${1:-${SUDO_USER:-}}"
if [[ -z "${TARGET_USER}" ]]; then
  echo "ERROR: could not determine target user; pass it explicitly: sudo $0 <user>" >&2
  exit 1
fi
log() { printf '[rt-grant] %s\n' "$*"; }

# 1. 'realtime' group + membership.
getent group realtime >/dev/null || groupadd realtime
usermod -aG realtime "${TARGET_USER}"
log "user '${TARGET_USER}' added to group 'realtime'"

# 2. rtprio + memlock limits for the group (enables SCHED_FIFO + mlockall, no caps).
cat > /etc/security/limits.d/99-kinova-rt.conf <<'EOF'
# kinova-gen3-driver: let the realtime group use RT scheduling + locked memory.
@realtime   -   rtprio      99
@realtime   -   memlock     unlimited
EOF
log "wrote /etc/security/limits.d/99-kinova-rt.conf (rtprio 99, memlock unlimited)"

# 3. udev rule so the realtime group can open /dev/cpu_dma_latency (C-state pin).
cat > /etc/udev/rules.d/99-cpu-dma-latency.rules <<'EOF'
# kinova-gen3-driver: allow the realtime group to set CPU PM-QoS (pin C-states).
KERNEL=="cpu_dma_latency", GROUP="realtime", MODE="0660"
EOF
udevadm control --reload-rules 2>/dev/null || true
udevadm trigger /dev/cpu_dma_latency 2>/dev/null || true
# Apply to the already-created node immediately too (udev trigger can be flaky for it).
[[ -e /dev/cpu_dma_latency ]] && { chgrp realtime /dev/cpu_dma_latency && chmod 0660 /dev/cpu_dma_latency; }
log "wrote udev rule + applied to /dev/cpu_dma_latency"

log "DONE. Log out and back in (or reboot) so '${TARGET_USER}' picks up the realtime group."
log "Then run the driver with NO sudo; confirm 'policy: FIFO' and 'cpu_dma_latency: pinned@0us' in the report."
