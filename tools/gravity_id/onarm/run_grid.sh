#!/usr/bin/env bash
# Batch 2: read-only logger + Kinova high-level moves through the grid, both-sided approaches.
set -uo pipefail
cd "$(dirname "$0")" || exit 1
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
M="python3 move.py --speed 15 --hold 6 --bidirectional 5"
(nohup ./dryrun.sh d1.txt 0 > /dev/null 2>&1 &)
sleep 5
docker ps --format '{{.Names}}' | grep -q gid-dryrun || { echo "logger did not start"; echo DONE 1; exit 1; }
rc=0
$M --angles=0,0,0,0,0,0,0 && echo CANDLE_START_OK \
  && $M --poses poses.csv --start P00 --stop P24 \
  && $M --angles=0,0,0,0,0,0,0 && echo CANDLE_MID_OK \
  && $M --poses poses.csv --start P25 --stop P49 \
  && $M --angles=0,0,0,0,0,0,0 && echo CANDLE_END_OK || rc=$?
docker kill --signal INT gid-dryrun > /dev/null
echo DONE $rc
