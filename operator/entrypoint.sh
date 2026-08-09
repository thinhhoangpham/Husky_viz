#!/usr/bin/env bash
# Derive this container's own IP and advertise it as ROS_IP so the remote
# master hands peers a reachable callback address (the ROS_IP gotcha).
set -euo pipefail
source /opt/ros/noetic/setup.bash

CONTAINER_IP="$(ip -4 -o addr show scope global | awk '{print $4}' | cut -d/ -f1 | head -n1)"
export ROS_IP="${CONTAINER_IP}"

# `docker compose exec` shells are neither login nor interactive, so they
# source none of .bashrc/.profile//etc/profile.d automatically -- they only
# inherit ROS_MASTER_URI (set at the compose level) and NOT this entrypoint's
# ROS_IP export (this process's own env, not the container's). Write it to a
# file and point BASH_ENV (set in docker-compose.yml) at it so every `exec
# bash -c ...` sources ROS + this container's ROS_IP automatically.
cat > /etc/ros_env.sh <<EOF
source /opt/ros/noetic/setup.bash
export ROS_IP="${CONTAINER_IP}"
EOF

echo "[operator] ROS_IP=${ROS_IP}"
echo "[operator] ROS_MASTER_URI=${ROS_MASTER_URI:-<unset>}"

if [ "${OPERATOR_RVIZ:-1}" = "1" ]; then
  export DISPLAY=:1
  Xvfb :1 -screen 0 1280x720x24 &
  sleep 1
  fluxbox &
  x11vnc -display :1 -forever -nopw -quiet -bg
  websockify --web=/usr/share/novnc 6080 localhost:5900 &
  rosrun rviz rviz -d /repo/operator/operator.rviz &
  echo "[operator] RViz at http://localhost:6080/vnc.html"
fi

exec "$@"
