#!/usr/bin/env bash
# Derive this container's own IP and advertise it as ROS_IP so the remote
# master hands peers a reachable callback address (the ROS_IP gotcha).
set -euo pipefail
source /opt/ros/noetic/setup.bash

CONTAINER_IP="$(ip -4 -o addr show scope global | awk '{print $4}' | cut -d/ -f1 | head -n1)"
export ROS_IP="${CONTAINER_IP}"

echo "[operator] ROS_IP=${ROS_IP}"
echo "[operator] ROS_MASTER_URI=${ROS_MASTER_URI:-<unset>}"

exec "$@"
