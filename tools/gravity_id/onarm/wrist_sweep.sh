#!/usr/bin/env bash
# Wrist payload sweep: read-only logger + Kinova high-level moves. j1-j5 stay at the
# given pose. At each j6 angle, j7 steps -165..165 deg by 30, and every step is held
# twice: approached from (j6+5, j7+10) and from (j6-5, j7-10), so j6 and j7 friction
# cancel. j7 stays inside +-175 deg and alternate sweeps run backwards, so a wrist
# cable never winds more than half a turn. NOT COLLISION-CHECKED: moving j6 swings the
# gripper. Check before running, and watch every move.
# Usage: ./wrist_sweep.sh J1,J2,J3,J4,J5 J6 J6 [J6 ...]   (deg; J6 whole degrees, distinct,
# >= 20 apart). Holds are 8 s: the logger prints ~2 tables/s and the fit needs 6 after settling.
set -uo pipefail
cd "$(dirname "$0")" || exit 1
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
[ $# -ge 3 ] || { echo "usage: $0 J1,J2,J3,J4,J5 J6 J6 [J6 ...]"; exit 2; }
P="$1"
shift
[ "$(printf '%s\n' "$@" | sort -u | wc -l)" -eq $# ] || { echo "j6 angles must be distinct"; exit 2; }
M="python3 move.py --speed 10"
OUT="wrist_$(date -u +%Y%m%dT%H%M%SZ).txt"

if docker ps -a --format '{{.Names}}' | grep -qx gid-dryrun; then
  echo "a gid-dryrun logger already exists (docker ps -a); stop it first"
  exit 1
fi
trap 'docker kill --signal INT gid-dryrun > /dev/null 2>&1' EXIT
logger_alive() {
  docker ps --format '{{.Names}}' | grep -qx gid-dryrun || { echo "logger died"; return 1; }
}
# j7 from $2 to $3 at j6 = $1, in <= 90 deg steps along the direct path (never across
# +-180 deg, where Kinova would take the short way round and wind the cable).
j7_to() {
  local j6=$1 at=$2 to=$3 step
  while [ "$at" -ne "$to" ]; do
    step=$((to - at))
    [ $step -gt 90 ] && step=90
    [ $step -lt -90 ] && step=-90
    at=$((at + step))
    $M --hold 0 --from-anywhere --angles="$P,$j6,$at" || return
  done
}

(nohup ./dryrun.sh "$OUT" 0 > /dev/null 2>&1 &)
sleep 5
logger_alive || { echo DONE 1; exit 1; }
echo "logging to $OUT"
# Only the first move keeps move.py's 30 deg first-move guard (the arm must start near
# the sweep pose with j7 at 90); every later move starts where the previous one ended.
$M --hold 0 --angles="$P,$1,90" || { echo DONE 1; exit 1; }
rc=0 j7=90 dir=1
for j6 in "$@"; do
  if [ $dir = 1 ]; then ths=$(seq -165 30 165); else ths=$(seq 165 -30 -165); fi
  for th in $ths; do
    j7_to "$j6" "$j7" "$th" || { rc=$?; break 2; }
    j7=$th
    for s in 1 -1; do
      { $M --hold 0 --from-anywhere --angles="$P,$((j6 + 5 * s)),$((th + 10 * s))" \
        && $M --hold 8 --from-anywhere --angles="$P,$j6,$th"; } || { rc=$?; break 3; }
    done
  done
  logger_alive || { rc=1; break; }
  echo "J6_${j6}_OK"
  dir=$((-dir))
done
if [ $rc = 0 ]; then
  { j7_to "$j6" "$j7" 90 && $M --hold 0 --from-anywhere --angles="$P,$1,90"; } || rc=$?
fi
echo DONE $rc
exit $rc
