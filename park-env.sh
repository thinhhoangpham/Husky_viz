#!/usr/bin/env bash
# park-env.sh — environment for the natural_environments park simulation.
#
# SOURCE this inside the container, do not execute it:
#     source /workspace/park-env.sh
#
# Sets up the NEGS-UGV "natural environments" packages and the park world's
# model assets so that park.world (the world park_1.bag was recorded in)
# loads correctly. See park_world_notes.md for the known-harmless Gazebo
# mesh error about the Untitled2 model.

# ROS first — sourcing setup.bash resets ROS_PACKAGE_PATH, so it must come
# before the prepends below.
source /opt/ros/noetic/setup.bash

# Package overlay. rospack crawls this recursively, picking up
# natural_environments, husky_*, geonav_transform and ouster_*.
# Prepended (not appended) so this tree's husky_description -- which carries
# the Ouster/stereo sensor arch used by the bag -- shadows the container's
# stock ros-noetic-husky-* debs.
export ROS_PACKAGE_PATH=/workspace/natural_environments_ros:$ROS_PACKAGE_PATH

# World meshes and textures: the 97 model dirs extracted from models.zip.
export GAZEBO_MODEL_PATH=/workspace/models:$GAZEBO_MODEL_PATH

# The Ouster lidar plugin (libgazebo_ros_ouster_laser.so) is NOT shipped as a
# deb -- it is built from source in the container at /root/catkin_ws. Without
# this path Gazebo silently loads the robot with no laser sensor at all and
# /os0_cloud_node/points never appears.
#
# NOTE: /root/catkin_ws lives inside the container, so it is DESTROYED by
# `docker compose down`. After a container recreate the plugin must be rebuilt:
#   source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && catkin_make
export GAZEBO_PLUGIN_PATH=/root/catkin_ws/devel/lib:$GAZEBO_PLUGIN_PATH

# Husky sensor configuration (natural_environments_ros/readme.txt:28-29).
export HUSKY_SENSOR_ARCH=true
export HUSKY_URDF_EXTRAS=/workspace/natural_environments_ros/husky/husky_description/urdf/sensor_description.urdf

# Xvfb display served by the entrypoint's VNC stack.
export DISPLAY=:1

echo "park-env: ROS_PACKAGE_PATH -> ${ROS_PACKAGE_PATH%%:*}"
echo "park-env: GAZEBO_MODEL_PATH -> ${GAZEBO_MODEL_PATH%%:*}"
echo "park-env: ready. Next: roslaunch natural_environments create_park.launch"
