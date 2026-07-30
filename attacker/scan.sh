#!/usr/bin/env bash
# PHASE 1 — REACHABILITY. Pure network scan; no ROS calls (spec §4).
# Sweeps the docker0 subnet for an open ROS master port (11311).
set -euo pipefail

: "${ROBOT_HOST_IP:?set ROBOT_HOST_IP to the docker0 gateway IP (see README Phase 0)}"

# Scan the /24 the gateway lives on, e.g. 172.17.0.1 -> 172.17.0.0/24.
SUBNET="$(echo "${ROBOT_HOST_IP}" | awk -F. '{print $1"."$2"."$3".0/24"}')"

echo "[scan] nmap -p 11311 ${SUBNET}"
OUT="$(nmap -p 11311 --open "${SUBNET}")"
echo "${OUT}"

if echo "${OUT}" | grep -q "11311/tcp open"; then
  echo "[scan] OK: found an open ROS master on 11311."
  echo "[scan] Next: run ./enum.sh to read the graph (Phase 2)."
  exit 0
else
  echo "[scan] FAIL: no host answered on 11311." >&2
  echo "[scan] Fix Phase 0 on the host: export ROS_IP + ROS_MASTER_URI to the" >&2
  echo "[scan] docker0 gateway IP before roslaunch, and check the host firewall." >&2
  exit 1
fi
