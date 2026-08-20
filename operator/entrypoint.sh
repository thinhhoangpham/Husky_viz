#!/usr/bin/env bash
# Derive this container's own IP and advertise it as ROS_IP so the remote
# master hands peers a reachable callback address (the ROS_IP gotcha).
set -euo pipefail
source /opt/ros/noetic/setup.bash
export DISABLE_ROS1_EOL_WARNINGS=1

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
export DISABLE_ROS1_EOL_WARNINGS=1
# RViz's RobotModel resolves the URDF's package://husky_description/... mesh
# URIs through ROS_PACKAGE_PATH. That package is NOT installed in this image --
# it lives in the bind-mounted repo -- so without this the meshes fail to load
# and the RobotModel display goes RED with the robot invisible. Appended AFTER
# the setup.bash source, which sets ROS_PACKAGE_PATH itself and would otherwise
# overwrite this.
export ROS_PACKAGE_PATH="/repo/natural_environments_ros_opt:\${ROS_PACKAGE_PATH}"
EOF

echo "[operator] ROS_IP=${ROS_IP}"
echo "[operator] ROS_MASTER_URI=${ROS_MASTER_URI:-<unset>}"

if [ "${OPERATOR_RVIZ:-1}" = "1" ]; then
  export DISPLAY=:1
  # RViz is launched from THIS shell, so it needs the mesh package path in this
  # process's own environment too -- /etc/ros_env.sh only covers `exec` shells.
  export ROS_PACKAGE_PATH="/repo/natural_environments_ros_opt:${ROS_PACKAGE_PATH}"
  Xvfb :1 -screen 0 1280x720x24 &
  sleep 1

  # fluxbox has no wmctrl/xdotool in this image, so force RViz maximized via
  # fluxbox's own apps-config rule instead. Must be written before `fluxbox &`
  # so fluxbox loads it on startup.
  mkdir -p /root/.fluxbox
  cat > /root/.fluxbox/apps <<'EOF'
[app] (name=rviz)
  [Maximized] {yes}
  [Dimensions] {100% 100%}
[end]
[app] (class=rviz)
  [Maximized] {yes}
  [Dimensions] {100% 100%}
[end]
EOF

  fluxbox &
  x11vnc -display :1 -forever -nopw -quiet -bg
  websockify --web=/usr/share/novnc 6080 localhost:5900 &
  rosrun rviz rviz -d /repo/operator/operator.rviz &
  echo "[operator] RViz at http://localhost:6080/vnc.html"

  # DTM terrain/water layers. Started HERE, with RViz, so they come up with the
  # container every time -- they were previously hand-started after each restart
  # and silently lost on the next one, leaving the DTM displays subscribed to a
  # dead topic (terrain simply absent, with no error shown).
  #
  # DTM_WORLD picks which map to publish (default lake; set DTM_WORLD=park for
  # the park). Empty DTM_WORLD disables both layers.
  #
  # DATUM: TRUE WORLD ELEVATION. The DTM is published unshifted (offset 0), so
  # terrain renders at its real height -- 3.5-5.9 m in the lake world. The robot
  # now renders there too: commit 07cd63c fused absolute z (lidar ground-plane
  # fit + DTM prior), so base_link sits ON the terrain at ~4.2 m instead of at 0.
  # scripts/relay_costmap_z.py lifts the global costmap to the DTM minimum for
  # the same reason. Robot, terrain and costmap therefore share ONE datum, and
  # what RViz draws is what is physically true.
  #
  # HISTORY -- this used to be the other way round, deliberately. Before 07cd63c
  # z was UNOBSERVED: base_link was pinned at 0 and the 2D costmaps are always
  # published at 0, so the terrain was pulled DOWN to meet them, by
  #     -(sensor_height + terrain_elevation_at_spawn)
  # (the lidar sits ~0.83 m up, so the ground it SEES lands near z=-0.83, not 0
  # -- aligning the DTM minimum to 0 floated it ~0.6-0.8 m above the lidar's own
  # ground returns). That was correct while the robot was the thing stuck at
  # zero. Now that the robot carries true altitude, dragging the terrain down
  # would leave the ROBOT floating ~4 m above it -- the same bug inverted. The
  # newer decision wins, so the shift is gone.
  #
  # Override with DTM_Z_OFFSET=<metres> to shift both layers (e.g. back to the
  # old nav-datum behaviour) -- the water layer always takes the terrain's
  # offset EXPLICITLY, never its own --z-align, which would flatten it onto the
  # terrain floor and destroy the water/terrain height relationship (i.e. the
  # shoreline).
  DTM_WORLD="${DTM_WORLD:-lake}"
  if [ -n "${DTM_WORLD}" ] && [ -f "/repo/maps/${DTM_WORLD}_dtm.npy" ]; then
    # 0 = true world elevation (see the DATUM note above). No computed shift.
    DTM_OFF="${DTM_Z_OFFSET:-0}"
    python3 /repo/scripts/publish_dtm_cloud.py --dtm "/repo/maps/${DTM_WORLD}_dtm.npy" \
      --topic /dtm_cloud --z-offset "${DTM_OFF}" > /tmp/dtm_terrain.log 2>&1 &
    if [ -f "/repo/maps/${DTM_WORLD}_water.npy" ]; then
      # SAME offset as the terrain -- never its own --z-align, which would
      # flatten the water onto the terrain floor and erase the shoreline.
      python3 /repo/scripts/publish_dtm_cloud.py --dtm "/repo/maps/${DTM_WORLD}_water.npy" \
        --topic /water_cloud --colormap water --z-offset "${DTM_OFF}" \
        > /tmp/dtm_water.log 2>&1 &
    fi
    echo "[operator] DTM layers publishing for world '${DTM_WORLD}' (/dtm_cloud, /water_cloud)"
  fi
fi

exec "$@"
