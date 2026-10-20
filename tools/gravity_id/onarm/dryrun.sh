#!/usr/bin/env bash
# READ-ONLY gravity dry-run against the dojo arm: connects, reads feedback, never
# enters low-level servoing, never commands torque. Usage: ./dryrun.sh OUT.txt [DURATION_S]
# DURATION_S 0 = until Ctrl-C.
set -euo pipefail
cd "$(dirname "$0")"
docker run --rm -i --name gid-dryrun --network host -v "$PWD":/gid ghcr.io/rammp-org/kinova-gen3-ros2:nightly \
  /module_ws/build/kinova_lowlevel/benchmark_grav_comp --ip 192.168.1.10 --dry-run \
  --urdf /gid/robot.urdf --ee-frame arm_end_effector_link --duration "${2:-0}" | tee "$1"
