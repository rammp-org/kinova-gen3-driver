#!/usr/bin/env bash
# Sync this repo to the Jetson build host (default: abra).
#
# This project CANNOT build on macOS (RT APIs + KORTEX SDK are Linux/aarch64
# only). The dev loop is: edit on Mac -> sync_to_abra.sh -> build+ctest on abra.
#
# Working CMAKE_PREFIX_PATH for Pinocchio discovery on abra (record for later
# tasks; baked into scripts/build_on_abra.sh):
#   /usr/local/lib/python3.10/dist-packages/cmeel.prefix
set -euo pipefail
REMOTE=${1:-abra}
# NOTE: exclude ALL build dirs ('build', 'build_kortex', ...) — otherwise
# --delete wipes the remote-only KORTEX build dir on every sync.
rsync -az --delete --exclude 'build*' --exclude .git \
  "$(dirname "$0")/../" "$REMOTE:~/kinova-gen3-driver/"
echo "synced to $REMOTE:~/kinova-gen3-driver"
