#!/usr/bin/env python3
"""Spawn a flat colored disc marker on the ground in Gazebo at a goal location.

Visualization ONLY. Uses the /gazebo/spawn_sdf_model and /gazebo/delete_model
services directly (rospy), so it works from a container WITHOUT gazebo_ros
installed -- it is just a service call to the master. This does NOT read pose or
any ground truth (project rule); it only PLACES a decoration at a coordinate the
caller already knows.

Used by:
  - operator/operate.py  -> GREEN disc at the operator's real goal
  - attack_goal.py        -> RED disc at the attacker's injected fake goal
"""
import math

import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel, DeleteModel

# The robot's odom frame originates at its SPAWN pose in the world (there is no
# tf between odom and world). These MUST match send_mapless_goal.py's SPAWN_*.
# Only used by the legacy frame="odom" path.
SPAWN_X = 38.26
SPAWN_Y = 1.25
SPAWN_YAW = -3.1281

# The park ground surface sits at z~3.0 (measured), NOT z=0.
GROUND_Z = 3.0

# Height at which to FLOAT the marker disc above the ground.
#
# WHY float it: the robot's Ouster lidar obstacle layer has a height gate of
# min_obstacle_height 0.15 .. max_obstacle_height 1.2 m above base_link (robot
# base ~z 3.12). A flat disc sitting on the ground (z~3.1) falls INSIDE that
# gate, so the lidar scans the MARKER ITSELF as a lethal obstacle sitting on the
# goal -- this walled off the goal and stopped the robot ~4.8 m short (verified
# live). Raising the disc so its bottom is well above ~1.2 m over the ground puts
# it above max_obstacle_height, so the height-gated obstacle layer ignores it,
# while it stays visible floating above the goal. Tune here if the gate changes.
MARKER_Z_ABOVE_GROUND = 2.0


def _disc_sdf(rgb):
    """A flat, static, collision-free disc lying on the ground.
    rgb is an 'R G B' string, e.g. '0 1 0' green, '1 0 0' red."""
    r, g, b = rgb.split()
    return """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="disc">
    <static>true</static>
    <link name="link">
      <visual name="v">
        <geometry><cylinder><radius>1.5</radius><length>0.1</length></cylinder></geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
          <emissive>{r} {g} {b} 1</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>""".format(r=r, g=g, b=b)


def place_goal_marker(name, x, y, rgb, timeout=5.0, frame="map"):
    """Delete any prior marker of this name, then spawn a flat disc of colour
    `rgb` at (x, y), FLOATING at z = GROUND_Z + MARKER_Z_ABOVE_GROUND so the
    lidar's height-gated obstacle layer does not scan it (see MARKER_Z_ABOVE_GROUND).
    Best-effort: logs and returns on failure, never raises -- a missing marker
    must not break driving or the attack.

    frame:
      "map"  (default) -- (x, y) are WORLD/map coords directly. For this
             GPS-anchored sim the map frame ~= world within ~0.1 m, so the disc
             is placed at (x, y) with no rotation. Use this for GPS/map goals.
      "odom" -- (x, y) are in the robot's ODOM frame; convert to world via the
             SPAWN_* pose (legacy path, for the old odom-frame senders).
    """
    try:
        rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=timeout)
    except rospy.ROSException:
        rospy.logwarn("goal_marker: /gazebo/spawn_sdf_model unavailable; "
                      "skipping marker '%s'.", name)
        return
    try:
        # Delete-first so re-runs don't collide on the model name.
        try:
            delete = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
            delete(name)
        except rospy.ServiceException:
            pass  # "model does not exist" is the normal first-run case.

        if frame == "odom":
            # LEGACY: goal (x, y) is in the robot's ODOM frame, but the marker
            # spawns in the Gazebo WORLD frame, and there is NO tf between them.
            # odom's origin is the robot's SPAWN pose in the world, so convert:
            #   world = R(spawn_yaw) . (x, y) + (spawn_x, spawn_y)
            c = math.cos(SPAWN_YAW)
            s = math.sin(SPAWN_YAW)
            wx = c * x - s * y + SPAWN_X
            wy = s * x + c * y + SPAWN_Y
        else:
            # frame == "map": (x, y) are already world/map coords (map ~= world
            # for this GPS-anchored sim). No rotation.
            wx, wy = x, y

        pose = Pose()
        pose.position.x = wx
        pose.position.y = wy
        # FLOAT the disc above the lidar height gate so it is not scanned as an
        # obstacle (see MARKER_Z_ABOVE_GROUND). No <collision> is defined either.
        pose.position.z = GROUND_Z + MARKER_Z_ABOVE_GROUND
        pose.orientation.w = 1.0
        spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        spawn(model_name=name, model_xml=_disc_sdf(rgb),
              robot_namespace="", initial_pose=pose, reference_frame="world")
        rospy.loginfo("goal_marker: placed '%s' (%s) frame=%s in(%.2f, %.2f) "
                      "= world(%.2f, %.2f, z=%.2f).", name, rgb, frame, x, y,
                      wx, wy, pose.position.z)
    except rospy.ServiceException as exc:
        rospy.logwarn("goal_marker: failed to place '%s': %s", name, exc)
