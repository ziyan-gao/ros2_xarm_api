#!/usr/bin/env bash

set -euo pipefail

# The unified stack is the normal operating mode. The other modes are kept for
# commissioning and rollback; they must never be run alongside one another.
case "${START_MODE:-unified}" in
  unified)
    exec /workspace/start_unified.sh
    ;;
  moveit)
    exec /workspace/start_moveit_phase1.sh
    ;;
  legacy)
    exec /workspace/start_all.sh
    ;;
  *)
    echo "Unknown START_MODE='${START_MODE}'. Expected: unified, moveit, or legacy." >&2
    exit 2
    ;;
esac
