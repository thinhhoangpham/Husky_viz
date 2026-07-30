#!/usr/bin/env bash
# PHASE 3 — EXPLOITATION. Runs an existing attack_*.py UNCHANGED against the
# remote master (spec §1, §3). Usage: ./attack.sh <name> [script args...]
set -euo pipefail
source /opt/ros/noetic/setup.bash

NAME="${1:-}"; shift || true
case "${NAME}" in
  cmd_vel|compass|odom|param) ;;
  *)
    echo "usage: ./attack.sh <cmd_vel|compass|odom|param> [args...]" >&2
    exit 2 ;;
esac

SCRIPT="/repo/attack_${NAME}.py"
if [[ ! -f "${SCRIPT}" ]]; then
  echo "[attack] ${SCRIPT} not found — is the repo mounted at /repo?" >&2
  exit 1
fi

echo "[attack] firing ${SCRIPT} against ${ROS_MASTER_URI} (ROS_IP=${ROS_IP})"
exec python3 "${SCRIPT}" "$@"
