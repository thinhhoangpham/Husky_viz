#!/usr/bin/env bash
# park-env-opt.sh — environment for the OPTIMIZED park simulation.
#
# Identical to park-env.sh except that it points at the "_opt" working trees:
#     /workspace/models_opt                    (instead of /workspace/models)
#     /workspace/natural_environments_ros_opt  (instead of /workspace/natural_environments_ros)
#
# The _opt tree's park.world replaces the 2.1M-triangle terreno_parque COLLADA
# terrain with a native Gazebo <heightmap> driven by
# models_opt/terreno_parque/terreno_parque_heightmap.png. The originals are left
# untouched, so sourcing park-env.sh instead gives you the unmodified world.
#
# SOURCE this inside the container, do not execute it:
#     source /workspace/park-env-opt.sh
#
# See park_world_notes.md for the known-harmless Gazebo mesh error about the
# Untitled2 model.

# ROS first — sourcing setup.bash resets ROS_PACKAGE_PATH, so it must come
# before the prepends below.
source /opt/ros/noetic/setup.bash

# Package overlay. rospack crawls this recursively, picking up
# natural_environments, husky_*, geonav_transform and ouster_*.
# Prepended (not appended) so this tree's husky_description -- which carries
# the Ouster/stereo sensor arch used by the bag -- shadows the container's
# stock ros-noetic-husky-* debs.
export ROS_PACKAGE_PATH=/workspace/natural_environments_ros_opt:$ROS_PACKAGE_PATH

# World meshes and textures: the 97 model dirs extracted from models.zip.
# This is also where Gazebo resolves the heightmap PNG referenced by park.world
# as model://terreno_parque/terreno_parque_heightmap.png.
export GAZEBO_MODEL_PATH=/workspace/models_opt:$GAZEBO_MODEL_PATH

# The Ouster lidar plugin (libgazebo_ros_ouster_laser.so) is NOT shipped as a
# deb -- it is built from source in the container at /root/catkin_ws. Without
# this path Gazebo silently loads the robot with no laser sensor at all and
# /os0_cloud_node/points never appears.
#
# NOTE: /root/catkin_ws lives inside the container, so it is DESTROYED by
# `docker compose down`. After a container recreate the plugin must be rebuilt:
#   source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && catkin_make
export GAZEBO_PLUGIN_PATH=/root/catkin_ws/devel/lib:$GAZEBO_PLUGIN_PATH

# Husky sensor configuration (natural_environments_ros_opt/readme.txt:28-29).
export HUSKY_SENSOR_ARCH=true
export HUSKY_URDF_EXTRAS=/workspace/natural_environments_ros_opt/husky/husky_description/urdf/sensor_description.urdf

# Xvfb display served by the entrypoint's VNC stack.
export DISPLAY=:1

echo "park-env-opt: ROS_PACKAGE_PATH -> ${ROS_PACKAGE_PATH%%:*}"
echo "park-env-opt: GAZEBO_MODEL_PATH -> ${GAZEBO_MODEL_PATH%%:*}"
echo "park-env-opt: ready. Next: roslaunch natural_environments create_park.launch"
