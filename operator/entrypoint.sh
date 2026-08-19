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
  # The water layer MUST use the SAME z offset as the terrain, not its own
  # --z-align: aligning it independently flattens it onto the terrain floor and
  # destroys the water/terrain height relationship (i.e. the shoreline).
  # Drawn with --z-align min so the terrain's lowest point sits at z=0, sharing
  # the datum used by the robot (z unobserved, settles near 0 -- see
  # localization.yaml) and by the 2D costmaps (always drawn at z=0). All three
  # then agree. Publishing at TRUE elevation instead put the terrain at 3.5-5.9 m
  # while the costmaps stayed at 0, ~4 m apart.
  # The water layer takes the terrain's offset EXPLICITLY, never its own
  # --z-align: aligning it independently flattens it onto the terrain floor and
  # destroys the water/terrain height relationship (i.e. the shoreline).
  # DTM_Z_OFFSET aligns the map to the frame the robot actually renders in.
  # base_link is pinned at z=0 (z is unobserved -- see localization.yaml) and the
  # lidar sits ~0.83 m above it, so the ground the robot SEES lands near z=-0.83,
  # NOT 0. Aligning the DTM's minimum to 0 therefore floated the map ~0.6-0.8 m
  # above the lidar's own ground returns. Instead offset the DTM by
  #     -(sensor_height + terrain_elevation_at_spawn)
  # so the terrain under the robot coincides with the ground the lidar reports.
  # Override with DTM_Z_OFFSET=<metres> if the robot starts elsewhere.
  DTM_WORLD="${DTM_WORLD:-lake}"
  if [ -n "${DTM_WORLD}" ] && [ -f "/repo/maps/${DTM_WORLD}_dtm.npy" ]; then
    DTM_OFF="${DTM_Z_OFFSET:-}"
    if [ -z "${DTM_OFF}" ]; then
      DTM_OFF=$(python3 - <<'PY' 2>/dev/null
import numpy as np, yaml, os
w = os.environ.get("DTM_WORLD", "lake")
z = np.load("/repo/maps/%s_dtm.npy" % w)
m = yaml.safe_load(open("/repo/maps/%s_dtm.yaml" % w))
# spawn x,y per world (the pose load-park-world.sh spawns the Husky at)
spawn = {"lake": (-47.07, -15.04), "park": (45.64, 0.02)}.get(w)
res, ox, oy = m["resolution"], m["origin_x"], m["origin_y"]
c = int((spawn[0] - ox) / res); r = int((spawn[1] - oy) / res)
here = z[r, c]
if not np.isfinite(here):
    here = np.nanmin(z)
SENSOR_H = 0.826          # base_link -> os0_lidar, from the URDF
print("%.4f" % (-(SENSOR_H + float(here))))
PY
)
    fi
    python3 /repo/scripts/publish_dtm_cloud.py --dtm "/repo/maps/${DTM_WORLD}_dtm.npy" \
      --topic /dtm_cloud --z-offset "${DTM_OFF}" > /tmp/dtm_terrain.log 2>&1 &
    if [ -f "/repo/maps/${DTM_WORLD}_water.npy" ]; then
      # SAME offset as the terrain -- never its own --z-align, which would
      # flatten the water onto the terrain floor and erase the shoreline.
      python3 /repo/scripts/publish_dtm_cloud.py --dtm "/repo/maps/${DTM_WORLD}_water.npy" \
        --topic /water_cloud --colormap water --z-offset "${DTM_OFF}" \
        > /tmp/dtm_water.log 2>&1 &
    fi
    if [ -f "/repo/maps/${DTM_WORLD}_slope.npy" ]; then
      # SAME offset as the terrain -- never its own --z-align, which would
      # flatten the slope layer onto the terrain floor and detach it from the
      # relief it is meant to sit on.
      # --grid-meta points geometry at the DTM's yaml: the slope .npy is
      # computed straight off the DTM array (same shape/origin/resolution),
      # but its own sibling yaml describes the resampled, map_server-style
      # PGM grid used for costmaps -- a different geometry entirely.
      python3 /repo/scripts/publish_dtm_cloud.py --dtm "/repo/maps/${DTM_WORLD}_slope.npy" \
        --grid-meta "/repo/maps/${DTM_WORLD}_dtm.yaml" \
        --topic /slope_cloud --colormap slope --z-offset "${DTM_OFF}" \
        > /tmp/dtm_slope.log 2>&1 &
    fi
    echo "[operator] DTM layers publishing for world '${DTM_WORLD}' (/dtm_cloud, /water_cloud, /slope_cloud)"
  fi
fi

exec "$@"
