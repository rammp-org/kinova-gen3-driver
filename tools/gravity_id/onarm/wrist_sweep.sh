#!/usr/bin/env bash
# Wrist payload sweep: read-only logger + Kinova high-level moves. j1-j5 stay at the
# given pose; at each j6 angle, j7 turns a full turn in 30 deg steps, approaching each
# step from +10 and -10 deg. NOT COLLISION-CHECKED: moving j6 swings the gripper, and
# j7's full turn winds any wrist cable. Check both before running, and watch every move.
# Usage: ./wrist_sweep.sh J1,J2,J3,J4,J5 J6 [J6 ...]   (deg; two or more J6 angles)
set -uo pipefail
cd "$(dirname "$0")" || exit 1
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
[ $# -ge 3 ] || { echo "usage: $0 J1,J2,J3,J4,J5 J6 J6 [J6 ...]"; exit 2; }
P="$1"
shift
M="python3 move.py --speed 10 --hold 6"
OUT="wrist_$(date -u +%Y%m%dT%H%M%SZ).txt"

if docker ps -a --format '{{.Names}}' | grep -qx gid-dryrun; then
  echo "a gid-dryrun logger already exists (docker ps -a); stop it first"
  exit 1
fi
trap 'docker kill --signal INT gid-dryrun > /dev/null 2>&1' EXIT
logger_alive() {
  docker ps --format '{{.Names}}' | grep -qx gid-dryrun || { echo "logger died"; return 1; }
}

(nohup ./dryrun.sh "$OUT" 0 > /dev/null 2>&1 &)
sleep 5
logger_alive || { echo DONE 1; exit 1; }
echo "logging to $OUT"
# Only the first move keeps move.py's 30 deg first-move guard (the arm must start near
# the sweep pose); every later move starts where the previous one ended.
$M --angles="$P,$1,90" || { echo DONE 1; exit 1; }
M="$M --from-anywhere"
rc=0
for j6 in "$@"; do
  for th in 90 120 150 180 210 240 270 300 330 0 30 60; do
    for j7 in $((th + 10)) "$th" $((th - 10)) "$th"; do
      $M --angles="$P,$j6,$j7" || { rc=$?; break 3; }
    done
  done
  logger_alive || { rc=1; break; }
  echo "J6_${j6}_OK"
done
[ $rc = 0 ] && $M --angles="$P,$1,90"
echo DONE $rc
exit $rc
