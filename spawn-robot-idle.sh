#!/usr/bin/env bash
# Spawn the STOCK Husky + mapless move_base into an ALREADY-RUNNING world,
# then idle. ROBOT SIDE of the operator demo. The world must already be up
# (./load-park-stock-husky.sh in another terminal). Ctrl-C tears it all down.
#
# SPAWN-OR-RESET: this script NEVER kills/respawns nodes on a re-run.
#   * No husky up yet  -> normal FIRST SPAWN (exec spawn_robot_idle.py, idles).
#   * A husky IS up     -> RESET it in place (teleport + zero-vel + EKF re-sync
#     via reset-robot.py) and return to the prompt. Nothing is killed, nothing
#     is deleted/respawned, so the shared ROS master never gets ghost nodes.
# Re-run it any time to snap the robot back to the init pose.
set -euo pipefail
source /opt/ros/noetic/setup.bash
# Overlay workspace: provides the Ouster Gazebo plugin
# (libgazebo_ros_ouster_laser.so) AND overlays husky_description so the repo's
# os1_64.dae lidar mesh resolves ahead of the /opt/ros copy. Sourced AFTER the
# ros setup so the overlay takes precedence on ROS_PACKAGE_PATH.
source "$HOME/husky_overlay_ws/devel/setup.bash"
# The catkin env hooks do NOT populate GAZEBO_PLUGIN_PATH for this workspace, and
# the built plugin lives in devel/lib -- add it explicitly so Gazebo can find the
# Ouster plugin .so at spawn time.
export GAZEBO_PLUGIN_PATH="$HOME/husky_overlay_ws/devel/lib${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
cd "$(dirname "$0")"

# Bring up the FULL DATASET ROBOT exactly as the dataset itself does. The dataset's
# own env (park-env.sh / load-park-world.sh) sets exactly these two vars, and the
# dataset husky.urdf.xacro (resolved from the overlay) consumes them:
#   * HUSKY_SENSOR_ARCH=true   -> builds the sensor arch / mounting frame on top of
#                                 the robot (the frame the sensors mount to).
#   * HUSKY_URDF_EXTRAS=...sensor_description.urdf -> the dataset's own sensor
#                                 suite: OS1-64 GPU lidar (-> /os0_cloud_node/points)
#                                 + GPS (-> /navsat/fix, datum 49.9/8.9/0); stereo
#                                 camera left disabled in that file for perf.
# Together with the dataset dual-EKF control.launch (husky_control also resolves to
# the overlay dataset package), this is the whole dataset robot: dataset model +
# dataset sensor arch + dataset sensors + dataset dual-EKF/GPS localization.
export HUSKY_SENSOR_ARCH=true
export HUSKY_URDF_EXTRAS="$HOME/husky_overlay_ws/src/husky_description/urdf/sensor_description.urdf"

# NOTE: HUSKY_CONFIG_EXTRAS is intentionally NOT exported here. husky_control now
# resolves to the DATASET package (~/husky_overlay_ws/src/husky_control, symlinked
# from natural_environments_ros_opt), whose control.launch is the standard
# robot_localization DUAL-EKF + navsat_transform stack:
#   * ekf_localization      (localization.yaml)      -> odom->base_link
#   * ekf_localization_map  (localization_map.yaml)  -> map->odom, fuses GPS (odom1)
#   * navsat_transform_node (navsat_transform.yaml)  -> /navsat/fix -> /odometry/gps
# The GPS-anchored map-frame pose is /odometry/filtered_map; that is the drift-free
# pose to navigate on. We run the dataset dual-EKF AS SHIPPED first and test; the
# earlier single-EKF compass override (config/localization_compass.yaml) is left in
# the repo but no longer loaded.

# --- SPAWN-OR-RESET decision ------------------------------------------------
# NO node is ever killed here. We ask Gazebo whether a robot named 'husky' is
# already in the world and branch:
#   * husky present -> RESET in place (reset-robot.py) and return to the prompt.
#   * husky absent  -> FIRST SPAWN (spawn_robot_idle.py, which idles on spin()).
#
# WHY the old pkill block is GONE: it SIGKILL'd individual node names
# (ekf_localization, robot_state_publisher, twist_mux, ...). SIGKILL tears a node
# down while it still holds its XML-RPC connection to rosmaster, so the node
# never deregisters and leaves a GHOST entry. The shared master is thereby
# corrupted, the next node to register hits `write error (Connection refused)`,
# and the only recovery was a full world restart. Reset-in-place avoids all of
# that: it only SETS pose (sanctioned by CLAUDE.md) and re-syncs the EKF, leaving
# control.launch / the EKF / the controllers running untouched.

# The master must be up for ANY of the below to work; the world (and thus the
# master) is started by ./load-park-stock-husky.sh. Probe it FIRST so we can give
# a clear "start the world first" error instead of a confusing service timeout.
echo "[spawn] checking whether a robot is already up ..."
if ! rosservice list >/dev/null 2>&1; then
  echo "" >&2
  echo "Error: the ROS master is not reachable, so nothing can be spawned or" >&2
  echo "reset. The park WORLD (which starts the master) must be up first." >&2
  echo "Start it in another terminal:" >&2
  echo "  ./load-park-stock-husky.sh" >&2
  exit 1
fi

# Robust 'is a husky up?' signal: Gazebo lists model 'husky' in its world
# properties. This is the single most reliable indicator a robot is present
# (same service used by send_mapless_goal.py / load-park-world.sh). The world
# model is 'parque', so matching the exact token 'husky' cannot false-match it.
# If the service is momentarily unavailable the grep simply fails -> we fall
# through to the SPAWN path, which is the safe default (spawn_robot_idle.py's own
# delete_existing_robot handles a stale model if one somehow exists).
# The CLI renders the model_names array as e.g. `model_names: [parque, husky]`.
# Normalise brackets/commas/spaces to newlines so each model name is its own
# line, then exact-match the token 'husky' (grep -x) - no substring false-match.
if rosservice call /gazebo/get_world_properties 2>/dev/null \
   | tr ' ,[]' '\n\n\n\n' | grep -qx 'husky'; then
  echo "[spawn] robot already up -> RESET to init pose (no respawn, no kill)."
  # Fire-and-forget one-shot: reset-robot.py teleports + zeroes velocity +
  # re-syncs the EKF, then EXITS (returns to the prompt). It must NOT fall
  # through to a second spawn_robot_idle.py, so we exec it and stop here.
  exec python3 ./reset-robot.py
fi

echo "[spawn] no robot detected -> FIRST SPAWN (idle until Ctrl-C)."
exec python3 ./spawn_robot_idle.py
