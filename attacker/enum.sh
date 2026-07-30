#!/usr/bin/env bash
# PHASE 2 — ENUMERATION. Read-only ROS graph read; the master volunteers this
# to any caller (spec §4). Run only after scan.sh (Phase 1) passes.
set -uo pipefail
source /opt/ros/noetic/setup.bash

echo "[enum] ROS_MASTER_URI=${ROS_MASTER_URI:-<unset>}  ROS_IP=${ROS_IP:-<unset>}"

# Short timeout so the ROS_IP gotcha surfaces as a clear message, not a hang.
run() {
  echo "=== $* ==="
  if ! timeout 15 "$@"; then
    echo "[enum] '$*' failed/timed out." >&2
    echo "[enum] nmap passed but ROS calls hang => the ROS_IP callback gotcha" >&2
    echo "[enum] (spec §2): the HOST master must export ROS_IP=<docker0 gw IP>" >&2
    echo "[enum] before roslaunch, else it advertises 127.0.0.1 to this peer." >&2
    return 1
  fi
}

run rosnode list
run rostopic list
run rosparam list
echo "[enum] OK: graph read complete. Next: ./attack.sh <name> (Phase 3)."
