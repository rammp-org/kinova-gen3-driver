#!/usr/bin/env bash
# Batch: read-only logger + Kinova high-level moves through the grid, both-sided approaches.
# NOT COLLISION-CHECKED: on 2026-10-20 this drove dojo's gripper into the table and
# self-collided (docs/integration/gravity_identification.md). Review every target first.
set -uo pipefail
cd "$(dirname "$0")" || exit 1
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
M="python3 move.py --speed 15 --hold 6 --bidirectional 5"
OUT="d1_$(date -u +%Y%m%dT%H%M%SZ).txt"

if docker ps -a --format '{{.Names}}' | grep -qx gid-dryrun; then
  echo "a gid-dryrun logger already exists (docker ps -a); stop it first"
  exit 1
fi
# Whatever ends this script (done, abort, Ctrl-C), the logger stops with it.
trap 'docker kill --signal INT gid-dryrun > /dev/null 2>&1' EXIT
logger_alive() {
  docker ps --format '{{.Names}}' | grep -qx gid-dryrun || { echo "logger died"; return 1; }
}

(nohup ./dryrun.sh "$OUT" 0 > /dev/null 2>&1 &)
sleep 5
logger_alive || { echo DONE 1; exit 1; }
echo "logging to $OUT"
rc=0
# Only the FIRST move starts from an unknown pose, so only it keeps move.py's 30 deg
# first-move guard; every later call starts where the previous one ended.
K="$M --from-anywhere"
$M --angles=0,0,0,0,0,0,0 && echo CANDLE_START_OK && logger_alive \
  && $K --poses poses.csv --start P00 --stop P24 && logger_alive \
  && $K --angles=0,0,0,0,0,0,0 && echo CANDLE_MID_OK \
  && $K --poses poses.csv --start P25 --stop P49 && logger_alive \
  && $K --angles=0,0,0,0,0,0,0 && logger_alive && echo CANDLE_END_OK || rc=$?
echo DONE $rc
exit $rc
