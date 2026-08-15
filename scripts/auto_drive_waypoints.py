#!/usr/bin/env python3
"""Autonomous waypoint driver for the Gazebo Husky (ROS 1 Noetic, Python 3).

The "legitimate autonomous driver" for a ROS security-research project.

Node identity (FIXED — a later phase attacks these, so do not rename):
    node name : husky_auto_drive
    param     : ~linear_speed  (default 0.5 m/s)
    input     : /gazebo/model_states (gazebo_msgs/ModelStates) -> ground-truth world pose
    output    : /kb_teleop/cmd_vel   (geometry_msgs/Twist) -> velocity commands

Behaviour: drives to 5 waypoints in order using a turn-in-place-then-go-straight
controller. The 5 waypoints are given directly in Gazebo WORLD coordinates and
lie on the park walkway centerline; they are driven in order from the spawn end
of the walkway to the far end. Steering uses the ground-truth world pose from
/gazebo/model_states. The EKF odom estimate (/odometry/filtered) is deliberately
NOT used for steering: the odom frame is offset and rotated from the Gazebo world
frame by an amount that cannot be predicted from the spawn pose, and it drifts
further whenever the wheels slip. Using the world pose puts the waypoints, the
Gazebo markers and the robot position in one frame, with no conversion and no drift.

Looping: after the robot reaches the final waypoint (WP5) it RESPAWNS to the
start and drives the route again, forever, until shutdown / Ctrl-C. A respawn
does two coupled things that MUST agree:
    1. Teleport the Gazebo model back to its spawn pose (world origin, identity
       orientation, zero twist) via /gazebo/set_model_state.
    2. Reset the fused odometry (/odometry/filtered) back to the origin via
       robot_localization's /set_pose service, so the driver's odom-frame goals
       line up with the physical corridor again.
The EKF here fuses only VELOCITIES from wheel odometry (odom0 pose fields are off)
and IMU orientation differentially, so a /set_pose reset to the origin is not
overridden by the continuing wheel odometry -- it settles at (0,0) and stays.
The respawn waits for the ground-truth world pose to actually read near the spawn
pose and hold there before starting the next lap.

Run inside the container:
    source /opt/ros/noetic/setup.bash && python3 /workspace/auto_drive_waypoints.py
"""

import math

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import PoseWithCovarianceStamped
from robot_localization.srv import SetPose

# --------------------------------------------------------------------------
# TUNABLES — edit these alone to change the driving behaviour.
# --------------------------------------------------------------------------
DEFAULT_LINEAR_SPEED = 0.5     # m/s, forward speed (overridable via ~linear_speed)
ARRIVAL_TOLERANCE = 0.7        # m, advance to next waypoint when within this
HEADING_TOLERANCE = 0.10       # rad, stop turning-in-place once aligned below this
ANGULAR_GAIN = 1.5             # proportional gain on heading error (1/s)
MAX_ANGULAR_SPEED = 1.0        # rad/s, cap on turn rate
CONTROL_RATE_HZ = 20.0         # control-loop frequency

# --------------------------------------------------------------------------
# RESPAWN / LOOP TUNABLES.
# --------------------------------------------------------------------------
GAZEBO_MODEL_NAME = "husky"                       # Gazebo model to teleport
SET_MODEL_STATE_SRV = "/gazebo/set_model_state"   # gazebo_msgs/SetModelState
SET_POSE_SRV = "/set_pose"                         # robot_localization/SetPose
ODOM_FRAME = "odom"                                # world_frame of the EKF

# Spawn pose the model is teleported back to, in the world frame. This is the
# bag's recorded start pose (park_1.bag, first /navsat/fix sample) and MUST match
# add_husky_park_1.launch -- the odom-frame waypoints are computed from it.
SPAWN_X = 45.64
SPAWN_Y = 0.02
SPAWN_Z = 3.3
SPAWN_YAW = 2.6132

# After a respawn, wait for the ground-truth world pose to read within this of
# the spawn pose and HOLD there before starting the next lap.
RESET_SETTLE_TOLERANCE = 0.5   # m, "near spawn pose" radius for both x and y
RESET_STABLE_SECONDS = 1.5     # s, how long it must stay near the spawn pose
RESET_SETTLE_TIMEOUT = 15.0    # s, give up (and log) if it never settles

# The 5 waypoints in Gazebo WORLD coordinates (metres), in visiting order.
# Derived from the bag's /navigation/objetive_gps waypoints:
#   (49.9003439105, 8.89998265669)  (49.9002436091, 8.89998465971)
#   (49.9000103866, 8.90003336643)  (49.8998565290, 8.90004639993)
#   (49.8997232844, 8.90004804940)
# mapped to world by a least-squares fit of /navsat/fix against
# /gazebo/model_states (lat -> world X, lon -> world -Y; residuals < 5 mm).
# All 5 verified to lie on the park walkway.
WORLD_WAYPOINTS = [
    (38.26, 1.25),
    (27.11, 1.10),
    (1.16, -2.40),
    (-15.95, -3.33),
    (-30.77, -3.45),
]


def yaw_from_quaternion(q):
    """Extract the yaw (rotation about z) from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def quaternion_from_yaw(yaw):
    """Return (x, y, z, w) for a rotation of `yaw` about the z axis."""
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


class WaypointDriver(object):
    def __init__(self):
        self.linear_speed = rospy.get_param("~linear_speed", DEFAULT_LINEAR_SPEED)

        # Fixed WORLD-frame waypoints supplied by restart-drive.sh as rosparam
        # ~world_waypoints (a list of [x, y] pairs in Gazebo world coordinates,
        # on the park walkway centerline). When present these are used directly
        # as absolute goals; when absent the driver falls back to WORLD_WAYPOINTS.
        self.param_waypoints = rospy.get_param("~world_waypoints", None)

        # Latest ground-truth world pose from /gazebo/model_states.
        self.have_odom = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Goals are (re)placed each lap, anchored at the robot's current pose.
        self.goals = None          # list of (x, y) in the Gazebo world frame
        self.goal_index = 0
        self.lap = 1               # current lap number (1-based)
        # `finished` means the current lap reached WP5 and a respawn is pending.
        self.finished = False

        self.cmd_pub = rospy.Publisher("/kb_teleop/cmd_vel", Twist, queue_size=1)
        self.state_sub = rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self.model_states_callback,
            queue_size=1)

        # Services used to respawn the robot to the start each lap.
        rospy.loginfo("husky_auto_drive: waiting for respawn services ...")
        rospy.wait_for_service(SET_MODEL_STATE_SRV)
        rospy.wait_for_service(SET_POSE_SRV)
        self.set_model_state = rospy.ServiceProxy(SET_MODEL_STATE_SRV, SetModelState)
        self.set_pose = rospy.ServiceProxy(SET_POSE_SRV, SetPose)

        rospy.loginfo("husky_auto_drive: linear_speed = %.3f m/s", self.linear_speed)
        rospy.loginfo("husky_auto_drive: waiting for first /gazebo/model_states ...")

    def model_states_callback(self, msg):
        """Track the robot's ground-truth world pose from /gazebo/model_states."""
        try:
            i = msg.name.index(GAZEBO_MODEL_NAME)
        except ValueError:
            return  # model not spawned yet
        pose = msg.pose[i]
        self.x = pose.position.x
        self.y = pose.position.y
        self.yaw = yaw_from_quaternion(pose.orientation)

        if not self.have_odom:
            self.have_odom = True
            self._anchor_goals()

    def _anchor_goals(self):
        """Set the goals to the fixed WORLD-frame waypoints.

        No anchoring or rotation: the goals, the robot pose and the Gazebo
        markers are all in the same world frame, so the waypoints are used
        exactly as given.
        """
        source = self.param_waypoints if self.param_waypoints else WORLD_WAYPOINTS
        self.goals = [(float(p[0]), float(p[1])) for p in source]
        rospy.loginfo("husky_auto_drive: %d world waypoints (from %s); "
                      "robot at world (%.2f, %.2f, yaw %.2f)",
                      len(self.goals),
                      "~world_waypoints" if self.param_waypoints else "WORLD_WAYPOINTS",
                      self.x, self.y, self.yaw)
        for i, (gx, gy) in enumerate(self.goals):
            rospy.loginfo("  WP%d -> world (%.2f, %.2f)", i + 1, gx, gy)

    def _publish(self, linear, angular):
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)

    def step(self):
        """One control iteration."""
        if not self.have_odom or self.goals is None:
            return
        if self.finished:
            self._publish(0.0, 0.0)
            return

        gx, gy = self.goals[self.goal_index]
        dx = gx - self.x
        dy = gy - self.y
        distance = math.hypot(dx, dy)

        # Arrived at current waypoint?
        if distance <= ARRIVAL_TOLERANCE:
            rospy.loginfo("husky_auto_drive: lap %d reached WP%d (dist %.2f m)",
                          self.lap, self.goal_index + 1, distance)
            self.goal_index += 1
            if self.goal_index >= len(self.goals):
                # Final waypoint of this lap reached; a respawn is now pending.
                self.finished = True
                self._publish(0.0, 0.0)
                rospy.loginfo("husky_auto_drive: lap %d complete "
                              "(all waypoints reached)", self.lap)
                return
            self._publish(0.0, 0.0)
            return

        # Heading error toward the goal.
        desired_yaw = math.atan2(dy, dx)
        heading_error = normalize_angle(desired_yaw - self.yaw)

        if abs(heading_error) > HEADING_TOLERANCE:
            # Turn in place to face the goal.
            angular = ANGULAR_GAIN * heading_error
            angular = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, angular))
            self._publish(0.0, angular)
        else:
            # Aligned: drive straight, with a small angular correction to hold heading.
            angular = ANGULAR_GAIN * heading_error
            angular = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, angular))
            self._publish(self.linear_speed, angular)

        rospy.loginfo_throttle(
            1.0,
            "husky_auto_drive: heading WP%d  dist=%.2f m  hdg_err=%.2f rad" % (
                self.goal_index + 1, distance, heading_error))

    def _teleport_model_to_spawn(self):
        """Teleport the Gazebo model back to its spawn pose with zero twist."""
        state = ModelState()
        state.model_name = GAZEBO_MODEL_NAME
        state.reference_frame = "world"
        state.pose.position.x = SPAWN_X
        state.pose.position.y = SPAWN_Y
        state.pose.position.z = SPAWN_Z
        qx, qy, qz, qw = quaternion_from_yaw(SPAWN_YAW)
        state.pose.orientation.x = qx
        state.pose.orientation.y = qy
        state.pose.orientation.z = qz
        state.pose.orientation.w = qw
        # Twist defaults to all zeros -> the model is placed at rest.
        resp = self.set_model_state(state)
        if not resp.success:
            rospy.logwarn("husky_auto_drive: set_model_state failed: %s",
                          resp.status_message)
        return resp.success

    def _reset_fused_odometry(self):
        """Reset /odometry/filtered to the origin via robot_localization/set_pose."""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = ODOM_FRAME
        # Position and orientation default to the origin / identity quaternion...
        msg.pose.pose.orientation.w = 1.0
        # ...with a small, confident covariance on the diagonal.
        for i in range(6):
            msg.pose.covariance[i * 6 + i] = 1e-6
        self.set_pose(msg)

    def _wait_for_odom_settled(self):
        """Block until the world pose reads near the spawn pose and holds there.

        Returns True if it settled within RESET_SETTLE_TIMEOUT, else False.
        """
        rate = rospy.Rate(CONTROL_RATE_HZ)
        deadline = rospy.Time.now() + rospy.Duration(RESET_SETTLE_TIMEOUT)
        stable_since = None
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            # Keep commanding zero velocity while we wait.
            self._publish(0.0, 0.0)
            near = (abs(self.x - SPAWN_X) <= RESET_SETTLE_TOLERANCE and
                    abs(self.y - SPAWN_Y) <= RESET_SETTLE_TOLERANCE)
            if near:
                if stable_since is None:
                    stable_since = rospy.Time.now()
                elif (rospy.Time.now() - stable_since).to_sec() >= RESET_STABLE_SECONDS:
                    rospy.loginfo(
                        "husky_auto_drive: pose settled at the spawn pose "
                        "(x=%.2f, y=%.2f)", self.x, self.y)
                    return True
            else:
                stable_since = None  # left the spawn-pose band; restart the timer
            rate.sleep()
        return False

    def _respawn_to_start(self):
        """Teleport the model AND reset odometry to the start, then re-anchor.

        The two must agree: the model is placed at the spawn pose and the fused
        odometry is reset to the origin, then we confirm the world pose actually
        reads near the spawn pose and holds before starting the next lap.
        """
        rospy.loginfo("husky_auto_drive: lap %d complete -- respawning to start",
                      self.lap)
        # Stop the robot before moving it.
        self._publish(0.0, 0.0)
        self._teleport_model_to_spawn()
        self._reset_fused_odometry()

        settled = self._wait_for_odom_settled()
        if not settled:
            rospy.logwarn(
                "husky_auto_drive: pose did NOT settle near the spawn pose within "
                "%.0fs (x=%.2f, y=%.2f); re-anchoring anyway",
                RESET_SETTLE_TIMEOUT, self.x, self.y)

        # Re-anchor the path at the fresh start and begin the next lap.
        self.lap += 1
        self.goal_index = 0
        self.finished = False
        self._anchor_goals()
        rospy.loginfo("husky_auto_drive: starting lap %d", self.lap)

    def run(self):
        rate = rospy.Rate(CONTROL_RATE_HZ)
        while not rospy.is_shutdown():
            if self.finished:
                # Reached WP5: respawn to the start and loop again.
                self._respawn_to_start()
                continue
            self.step()
            rate.sleep()


def main():
    rospy.init_node("husky_auto_drive")
    driver = WaypointDriver()
    try:
        driver.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Best-effort stop on exit.
        try:
            driver._publish(0.0, 0.0)
        except Exception:
            pass


if __name__ == "__main__":
    main()
