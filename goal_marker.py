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
import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel, DeleteModel


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
        <geometry><cylinder><radius>0.6</radius><length>0.05</length></cylinder></geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
          <emissive>{r} {g} {b} 1</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>""".format(r=r, g=g, b=b)


def place_goal_marker(name, x, y, rgb, timeout=5.0):
    """Delete any prior marker of this name, then spawn a flat disc of colour
    `rgb` at (x, y) on the ground. Best-effort: logs and returns on failure,
    never raises -- a missing marker must not break driving or the attack.
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

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.05
        pose.orientation.w = 1.0
        spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        spawn(model_name=name, model_xml=_disc_sdf(rgb),
              robot_namespace="", initial_pose=pose, reference_frame="world")
        rospy.loginfo("goal_marker: placed '%s' (%s) at (%.2f, %.2f).",
                      name, rgb, x, y)
    except rospy.ServiceException as exc:
        rospy.logwarn("goal_marker: failed to place '%s': %s", name, exc)
