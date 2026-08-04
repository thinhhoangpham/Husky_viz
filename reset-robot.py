#!/usr/bin/env python3
"""reset-robot.py - teleport the ALREADY-RUNNING park Husky back to its spawn
pose WITHOUT killing or respawning any node.

WHY THIS EXISTS
---------------
The old "reset" flow killed control.launch and re-spawned the robot. Against a
SHARED ROS master that leaves ghost nodes (a dead ekf_localization / controller
spawner half-registered), which corrupts subsequent runs. This script instead
leaves control.launch / the EKF / the controllers running untouched and does the
minimum to put the robot back on its mark:

  1. TELEPORT the "husky" model via /gazebo/set_model_state (SET pose only).
  2. ZERO the controller target with a single zero Twist on /cmd_vel.
  3. RE-SYNC the EKF via robot_localization's /set_pose, because after a teleport
     /odometry/filtered still believes the robot is at the OLD pose.

GROUND-TRUTH RULE (project CLAUDE.md)
-------------------------------------
This node only ever SETS pose. It never READS a Gazebo pose as data: no
/gazebo/get_model_state, no /gazebo/model_states subscription, no gazebo_msgs
pose import beyond the SET request types. /gazebo/set_model_state for
"repositioning between test runs" is explicitly sanctioned; using it as a
navigation/verification source is not, and this script does neither.

Run: ./reset-robot.py   (no args required)
"""

import sys

import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from robot_localization.srv import SetPose
from tf.transformations import quaternion_from_euler

# Canonical spawn pose. Source: send_mapless_goal.py:62-65 (identical values in
# drive-straight.py:95-98). x,y = first bag waypoint; yaw = WP1->WP2 heading, i.e.
# straight down the trail. z=3.3 spawns a few cm above the settled terrain so the
# robot drops and settles, exactly as the spawners do.
SPAWN_X = 38.26
SPAWN_Y = 1.25
SPAWN_Z = 3.3
SPAWN_YAW = -3.1281  # radians

# gazebo_msgs/ModelState model_name. Source: ROBOT_MODEL_NAME in
# send_mapless_goal.py / drive-straight.py.
ROBOT_MODEL_NAME = "husky"

# twist_mux `external` slot - the same topic drive-straight.py drives. Publishing
# a zero here zeros the controller's velocity TARGET (set_model_state only zeros
# the model's instantaneous twist in Gazebo, not the command the controller is
# still tracking).
CMD_VEL_TOPIC = "/cmd_vel"

# robot_localization EKF services. The node is named `ekf_localization`
# (Husky-main/husky_control/launch/control.launch:35) and advertises `set_pose`
# of type robot_localization/SetPose (public_interface.yaml). It resolves to the
# global /set_pose - the exact name auto_drive_waypoints.py:64 uses successfully.
SET_MODEL_STATE_SRV = "/gazebo/set_model_state"
SET_POSE_SRV = "/set_pose"

# Bounded waits, in the spirit of drive-straight.py's DELETE_SERVICE_TIMEOUT_S:
# if a service is not up within this window it is not coming, and we must not
# hang the reset forever.
SET_MODEL_STATE_TIMEOUT_S = 10.0
SET_POSE_TIMEOUT_S = 10.0

# EKF-frame reset z. The EKF (world_frame: odom, per CLAUDE.md) operates in the
# ground-plane; the 3.3 Gazebo drop height is not a pose in that 2D frame, so we
# hand the filter z=0.
EKF_RESET_Z = 0.0
# Small, confident diagonal covariance so the filter ACCEPTS the reset pose
# (auto_drive_waypoints.py:269 uses the same idea; 1e-3 per the task spec).
EKF_RESET_COVARIANCE = 1e-3


def _spawn_quaternion():
    """Quaternion for yaw=SPAWN_YAW, roll=pitch=0. Same helper family as the
    spawners' tf.transformations usage."""
    return quaternion_from_euler(0.0, 0.0, SPAWN_YAW)


def teleport_model():
    """SET the 'husky' model back to the spawn pose with zero twist via
    /gazebo/set_model_state. Returns True on success. A missing service or a
    ServiceException is fatal to the RESET (nothing moved), logged at error, and
    the caller returns non-zero. This is the one step that must work."""
    rospy.loginfo("Teleport: waiting for %s (up to %.0fs) ...",
                  SET_MODEL_STATE_SRV, SET_MODEL_STATE_TIMEOUT_S)
    try:
        rospy.wait_for_service(SET_MODEL_STATE_SRV,
                               timeout=SET_MODEL_STATE_TIMEOUT_S)
    except rospy.ROSException as exc:
        rospy.logerr("%s unavailable within %.0fs (%s) - is the sim running? "
                     "Cannot reset.", SET_MODEL_STATE_SRV,
                     SET_MODEL_STATE_TIMEOUT_S, exc)
        return False

    qx, qy, qz, qw = _spawn_quaternion()

    state = ModelState()
    state.model_name = ROBOT_MODEL_NAME
    state.pose.position.x = SPAWN_X
    state.pose.position.y = SPAWN_Y
    state.pose.position.z = SPAWN_Z
    state.pose.orientation.x = qx
    state.pose.orientation.y = qy
    state.pose.orientation.z = qz
    state.pose.orientation.w = qw
    # twist left at its ModelState default (all zeros) - stop any motion.
    state.reference_frame = "world"

    try:
        set_model_state = rospy.ServiceProxy(SET_MODEL_STATE_SRV, SetModelState)
        resp = set_model_state(model_state=state)
    except rospy.ServiceException as exc:
        rospy.logerr("Teleport call to %s failed (%s) - robot NOT reset.",
                     SET_MODEL_STATE_SRV, exc)
        return False

    if not resp.success:
        rospy.logerr("Teleport rejected by Gazebo: %s (is the model named '%s'?)",
                     resp.status_message, ROBOT_MODEL_NAME)
        return False

    rospy.loginfo("Teleported '%s' to x=%.2f y=%.2f z=%.2f yaw=%.4f (world), "
                  "twist zeroed.", ROBOT_MODEL_NAME, SPAWN_X, SPAWN_Y, SPAWN_Z,
                  SPAWN_YAW)
    return True


def zero_cmd_vel():
    """Publish a single zero Twist on /cmd_vel so the controller's velocity
    TARGET is zero, not just the model's instantaneous twist. Best-effort: a
    failed publish here does not undo the teleport, so it is logged at warn and
    never fatal. Latch so a late-connecting twist_mux still receives it."""
    try:
        pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1, latch=True)
        # Give the publisher a moment to register with subscribers; without this
        # the first publish can be dropped before twist_mux connects.
        rospy.sleep(0.5)
        pub.publish(Twist())  # all-zero
        rospy.loginfo("Published zero Twist to %s (controller target zeroed).",
                      CMD_VEL_TOPIC)
    except Exception as exc:  # noqa: BLE001 -- log, never hide
        rospy.logwarn("Could not publish zero Twist to %s (%s) - teleport still "
                      "stands; the controller will idle out on its own.",
                      CMD_VEL_TOPIC, exc)


def reset_ekf():
    """Re-sync the odom-frame EKF to the spawn pose via robot_localization's
    /set_pose. NON-FATAL but LOUD: after a teleport /odometry/filtered still
    reports the OLD pose, so anything reading it will be wrong until the EKF is
    told; but the teleport itself already succeeded, so a missing/failing EKF
    reset must not fail the whole reset. Returns True on success. Uses z=0 (2D
    odom ground plane) even though the Gazebo drop height is 3.3."""
    rospy.loginfo("EKF re-sync: waiting for %s (up to %.0fs) ...",
                  SET_POSE_SRV, SET_POSE_TIMEOUT_S)
    try:
        rospy.wait_for_service(SET_POSE_SRV, timeout=SET_POSE_TIMEOUT_S)
    except rospy.ROSException as exc:
        rospy.logwarn("%s unavailable within %.0fs (%s) - EKF NOT re-synced. "
                      "The teleport stands, but /odometry/filtered will keep "
                      "reporting the OLD pose until the filter is reset (is the "
                      "EKF running with a differently-namespaced set_pose?).",
                      SET_POSE_SRV, SET_POSE_TIMEOUT_S, exc)
        return False

    qx, qy, qz, qw = _spawn_quaternion()

    msg = PoseWithCovarianceStamped()
    msg.header.stamp = rospy.Time.now()
    # EKF world_frame is 'odom' (CLAUDE.md). Reset pose is expressed in it.
    msg.header.frame_id = "odom"
    msg.pose.pose.position.x = SPAWN_X
    msg.pose.pose.position.y = SPAWN_Y
    msg.pose.pose.position.z = EKF_RESET_Z
    msg.pose.pose.orientation.x = qx
    msg.pose.pose.orientation.y = qy
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw
    for i in range(6):
        msg.pose.covariance[i * 6 + i] = EKF_RESET_COVARIANCE

    try:
        set_pose = rospy.ServiceProxy(SET_POSE_SRV, SetPose)
        set_pose(msg)  # SetPose request field is `pose` (PoseWithCovarianceStamped)
    except rospy.ServiceException as exc:
        rospy.logwarn("EKF %s call failed (%s) - teleport stands, but "
                      "/odometry/filtered will keep reporting the OLD pose.",
                      SET_POSE_SRV, exc)
        return False

    rospy.loginfo("EKF re-synced via %s to x=%.2f y=%.2f z=%.2f yaw=%.4f "
                  "(frame=odom).", SET_POSE_SRV, SPAWN_X, SPAWN_Y, EKF_RESET_Z,
                  SPAWN_YAW)
    return True


def main():
    rospy.init_node("reset_robot", anonymous=True)

    rospy.loginfo("reset-robot: teleporting '%s' back to spawn WITHOUT killing "
                  "any node.", ROBOT_MODEL_NAME)

    if not teleport_model():
        rospy.logerr("RESET FAILED: the teleport did not happen, so the robot "
                     "was NOT moved.")
        return 1

    # Both are best-effort refinements of a teleport that already succeeded.
    zero_cmd_vel()
    ekf_ok = reset_ekf()

    if ekf_ok:
        rospy.loginfo("RESET COMPLETE: robot teleported, velocity zeroed, EKF "
                      "re-synced.")
    else:
        rospy.logwarn("RESET COMPLETE (teleport + zero-vel done) but the EKF was "
                      "NOT re-synced - /odometry/filtered may still report the "
                      "old pose. See the warning above.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        sys.exit(0)
