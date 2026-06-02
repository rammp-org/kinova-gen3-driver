#!/usr/bin/env bash
# Sync + configure + build + test on the Jetson host (default: abra).
#
# Reused by every task. It bakes in the verified CMAKE_PREFIX_PATH for
# Pinocchio discovery on abra. Pass extra cmake args after the remote name,
# e.g.  ./scripts/build_on_abra.sh abra -DCMAKE_BUILD_TYPE=Debug
set -euo pipefail
REMOTE=${1:-abra}
shift || true
EXTRA_CMAKE_ARGS="$*"

# Verified working prefix: the cmeel.prefix where pip-installed Pinocchio lives.
# pinocchioConfig.cmake is at <prefix>/lib/cmake/pinocchio/pinocchioConfig.cmake
PINOCCHIO_PREFIX="/usr/local/lib/python3.10/dist-packages/cmeel.prefix"

"$(dirname "$0")/sync_to_abra.sh" "$REMOTE"

ssh -o BatchMode=yes -o ConnectTimeout=10 "$REMOTE" "
  set -euo pipefail
  mkdir -p ~/kinova-gen3-driver/build
  cd ~/kinova-gen3-driver/build
  cmake .. -DCMAKE_PREFIX_PATH=${PINOCCHIO_PREFIX} ${EXTRA_CMAKE_ARGS}
  cmake --build . -j
  ctest --output-on-failure
"
