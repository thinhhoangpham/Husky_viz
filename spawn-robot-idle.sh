#!/usr/bin/env bash
# Spawn the STOCK Husky + mapless move_base into an ALREADY-RUNNING world,
# then idle. ROBOT SIDE of the operator demo. The world must already be up
# (./load-park-stock-husky.sh in another terminal). Ctrl-C tears it all down.
set -euo pipefail
source /opt/ros/noetic/setup.bash
cd "$(dirname "$0")"
exec python3 ./spawn_robot_idle.py
