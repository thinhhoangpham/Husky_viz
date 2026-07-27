#!/usr/bin/env python3
"""Feed the park route's WORLD-frame waypoints to move_base, one at a time.

move_base plans in a metric frame (odom); the route's waypoints are given in the
Gazebo WORLD frame (x, y metres). This node is the only glue needed to let the
stock navigation stack drive the route:

    for each waypoint:
        WORLD (x,y) -> odom goal   (rigid 2D transform, RE-DERIVED per goal)
        send to move_base, wait for it to arrive, then send the next

NATIVE PORT (Python 3 / Noetic)
-------------------------------
This is adapted from the Melodic/Python2 reference gps_goal_sender.py. Two things
changed for the native setup:

  1. NO GPS / lat-lon. The reference consumed geodetic waypoints off
     /navigation/objetive_gps and did lat/lon -> UTM -> odom (geonav_transform +
     the python `utm` module). Neither the topic nor those packages exist here,
     and our waypoints are already WORLD-frame metres -- so those imports are
     dropped entirely and the waypoints are hardcoded (overridable via the
     rosparam ~world_waypoints, exactly like the old auto_drive_waypoints.py).

  2. WORLD -> odom transform. Instead of anchoring on the robot's first GPS fix,
     we anchor on the robot's CURRENT /odometry/filtered pose paired with its
     CURRENT world pose. This is the same rigid-transform math the known-correct
     native driver used (auto_drive_waypoints.py.gps-ekf, _world_to_odom): a
     single rotation between the world and odom frames plus a translation. But
     the transform is RE-DERIVED IMMEDIATELY BEFORE EVERY GOAL, not solved once
     at startup and held fixed. See _current_transform() / run() below.

WHY THE TRANSFORM IS RE-DERIVED PER GOAL (this was the bug)
-----------------------------------------------------------
An earlier version solved the transform once at startup and latched it forever.
The startup solve was arithmetically CORRECT (verified against a hand
calculation to within centimetres) and move_base dutifully reported "reached
waypoint 1/2" -- because in the ODOM frame it genuinely did arrive. The robot
still ended up 13.5 m off the intended world track, and the error GREW WITH
DISTANCE and always in the same direction: the signature of an accumulating
ROTATION error, not a constant offset.

The cause is EKF yaw drift. /ekf_localization runs with `world_frame: odom` and
`imu0_differential: true`, fusing wheel odometry plus RELATIVE imu yaw only --
NOTHING in that configuration observes ABSOLUTE heading. On a skid-steer Husky,
turning scrubs the wheels sideways, so wheel odometry systematically
mis-reports rotation. odom_yaw therefore drifts away from true world yaw as the
robot drives, and a theta frozen at t=0 rotates every later goal by the whole
accumulated drift. Measured divergence on a real run:

    world x=33.96  intended y ~ -0.7   actual y =  -3.43   ( 2.7 m off)
    world x=27.17  intended y ~ -1.0   actual y =  -8.69   ( 7.7 m off)
    world x=20.59  intended y ~ -1.4   actual y = -14.92   (13.5 m off)

Re-deriving theta AND the position anchor from sensors sampled at the same
instant, immediately before each goal, cancels the drift accumulated so far on
every leg instead of compounding it. It is drift-free by construction because
both endpoints of the transform come from the same moment in time.

  The goal-sending loop (actionlib SimpleActionClient, one goal at a time,
  wait_for_result, "reached waypoint i", continue on non-success), the sim-clock
  guard (rospy.Time.now()==0) and the move_base server-connect retry loop are all
  carried over verbatim from the reference.

Heading source: the WORLD->odom ROTATION is solved LIVE from two headings sampled
at the SAME anchor instant -- the WORLD heading from /compass/data (sensor_msgs/Imu,
whose yaw exactly matches ground-truth world yaw on this machine, offset 0.0 deg)
and the odom heading from /odometry/filtered. theta = normalize(odom_yaw -
compass_yaw). We do NOT hardcode the spawn heading: the robot spawns at z=3.3,
drops onto the terrain, and its true settled heading is off from the launch-file
SPAWN_YAW by tens of degrees (measured ~51 deg), so a hardcoded reference rotated
every goal by that error and drove the robot off at an angle into a lightpost.
/imu/data is mounted rotated 90 deg and is NOT used.

Position source: the POSITION anchor is the robot's CURRENT world position, read
from the Gazebo ground-truth service /gazebo/get_model_state (model 'husky',
relative to 'world') once per goal. SIMULATION ONLY -- this is a sim-only
waypoint driver, and drive-park-nav.sh already uses the same service. On real
hardware this anchor would come from GPS instead (the reference implementation
this was ported from anchored on a GPS fix). The known spawn position
(SPAWN_X, SPAWN_Y) is now only a startup-logging/fallback reference: it is valid
at t=0 only, which is exactly why it cannot anchor later goals.

Obstacle avoidance is INERT until /os0_cloud_node/points publishes: the costmaps
subscribe to that PointCloud2, which is dead on this machine (Ouster plugin
missing), so move_base plans on an EMPTY costmap. That is accepted for now -- the
robot still drives the route; avoidance switches on automatically when the lidar
returns.
"""
import math
import time

import rospy
import actionlib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from gazebo_msgs.srv import GetModelState


# The 5 waypoints in Gazebo WORLD coordinates (metres), in visiting order. These
# match auto_drive_waypoints.py / drive-park.sh so goals and markers coincide.
# Overridable at runtime via the rosparam ~world_waypoints.
WORLD_WAYPOINTS = [
    (38.26, 1.25),
    (27.11, 1.10),
    (1.16, -2.40),
    (-15.95, -3.33),
    (-30.77, -3.45),
]

# The robot's world spawn POSITION, from add_husky_park_1.launch. NO LONGER used
# to anchor the transform: it is only valid at t=0, and anchoring later goals on
# it is precisely what let EKF drift accumulate. Kept as documentation of the
# nominal spawn point. The live world anchor now comes from Gazebo ground truth
# per goal, and the heading from /compass/data (never hardcoded -- the robot
# drops onto the terrain and settles tens of degrees off any launch-file yaw).
SPAWN_X = 45.6396
SPAWN_Y = 0.0208


def _norm(a):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class WorldGoalSender(object):
    def __init__(self):
        rospy.init_node('gps_goal_sender')

        self.odom = None          # latest odom pose (x, y, yaw)
        self.compass_yaw = None   # latest WORLD heading from /compass/data (rad)
        # READINESS gate, NOT a frozen transform: set once BOTH a compass and an
        # odom message have arrived (see _try_calibrate). The WORLD->odom
        # transform itself is re-derived per goal in _current_transform().
        self.calibrated = False
        self._theta0 = None       # startup theta, kept only as a drift baseline
        # Last successfully derived transform, used as the fallback when the
        # Gazebo ground-truth service is unavailable for a given goal.
        # (theta, cos, sin, odom_anchor_xy, world_anchor_xy)
        self._last_tf = None

        # Waypoints: rosparam override, else the hardcoded list.
        param_wps = rospy.get_param('~world_waypoints', None)
        source = param_wps if param_wps else WORLD_WAYPOINTS
        self.waypoints = [(float(p[0]), float(p[1])) for p in source]
        rospy.loginfo('gps_goal_sender: %d world waypoints (from %s)',
                      len(self.waypoints),
                      '~world_waypoints' if param_wps else 'WORLD_WAYPOINTS')

        rospy.Subscriber('/odometry/filtered', Odometry, self.on_odom,
                         queue_size=1)
        rospy.Subscriber('/compass/data', Imu, self.on_compass, queue_size=1)

        # Wait for the SIM clock before any Duration-based timeout. On startup
        # rospy.Time.now() is 0; when /clock arrives it leaps to the sim's value
        # (e.g. 7281), which instantly expires any deadline computed from 0 --
        # wait_for_server then "times out" in 0.1 s. Use wall-clock sleeps here.
        t0 = time.time()
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() == 0.0:
            if time.time() - t0 > 60:
                rospy.logerr('sim clock never started')
                return
            time.sleep(0.2)

        # SIMULATION ONLY. Ground-truth world pose, used once per goal as the
        # POSITION anchor of the freshly re-derived WORLD->odom transform.
        # drive-park-nav.sh already relies on this same service. On real
        # hardware this anchor would come from GPS instead.
        self._get_model_state = rospy.ServiceProxy('/gazebo/get_model_state',
                                                   GetModelState)

        rospy.loginfo('waiting for move_base ...')
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        connected = False
        for _ in range(60):
            if rospy.is_shutdown():
                return
            if self.client.wait_for_server(rospy.Duration(2)):
                connected = True
                break
            time.sleep(1.0)
        if not connected:
            rospy.logerr('move_base action server never appeared')
            return
        rospy.loginfo('move_base connected')

        rospy.Timer(rospy.Duration(1.0), self.tick, oneshot=False)
        rospy.spin()

    def on_odom(self, msg):
        p = msg.pose.pose
        q = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        yaw = tf.transformations.euler_from_quaternion(q)[2]
        self.odom = (p.position.x, p.position.y, yaw)
        self._try_calibrate()

    def on_compass(self, msg):
        """Track the live WORLD heading from /compass/data (sensor_msgs/Imu).

        Its yaw matches ground-truth world yaw on this machine (offset 0.0 deg),
        so it is the trustworthy reference for the WORLD->odom rotation.
        """
        q = [msg.orientation.x, msg.orientation.y,
             msg.orientation.z, msg.orientation.w]
        self.compass_yaw = tf.transformations.euler_from_quaternion(q)[2]
        self._try_calibrate()

    def _try_calibrate(self):
        """READINESS gate: mark the node ready once BOTH a compass and an odom
        message are in hand. This no longer freezes a transform -- it only says
        "both sensor streams are alive, it is safe to start sending goals".
        Sending goals before the sensors arrive would be worse than the drift.

        It also records the startup theta ONCE, purely as a baseline so the
        per-goal logs can print how far the EKF has drifted since t=0.
        """
        if self.calibrated:
            return
        if self.odom is None or self.compass_yaw is None:
            return
        odom_x, odom_y, odom_yaw = self.odom
        compass_yaw = self.compass_yaw
        self._theta0 = _norm(odom_yaw - compass_yaw)
        self.calibrated = True
        rospy.loginfo('sensors ready; startup WORLD->odom baseline: '
                      'odom0=(%.2f, %.2f) '
                      'compass_yaw=%.3f rad (%.1f deg) odom_yaw=%.3f rad (%.1f deg) '
                      'theta0=%.3f rad (%.1f deg) '
                      '[baseline only -- transform is re-derived per goal]',
                      odom_x, odom_y,
                      compass_yaw, math.degrees(compass_yaw),
                      odom_yaw, math.degrees(odom_yaw),
                      self._theta0, math.degrees(self._theta0))

    def _current_transform(self):
        """Re-derive the rigid WORLD -> odom transform from the CURRENT state.

        Everything below is sampled at (essentially) the same instant, which is
        what makes the result drift-free -- see the module docstring for why a
        transform latched at startup put the robot 13.5 m off the route.

            theta      = normalize(odom_yaw_now - compass_yaw_now)
            odom_goal  = odom_now + R(theta) * (world_wp - world_now)

        i.e. the waypoint is expressed RELATIVE TO WHERE THE ROBOT IS NOW in
        world, then rotated into odom. Any yaw/position error the EKF has
        accumulated so far is absorbed into the anchor and cancels.

        world_now comes from Gazebo ground truth (SIMULATION ONLY; on real
        hardware this would be a GPS fix). Returns the tuple
        (theta, cos, sin, odom_now_xy, world_now_xy), or None if the current
        state cannot be sampled -- the caller then falls back to the previous
        transform rather than crashing.
        """
        if self.odom is None or self.compass_yaw is None:
            rospy.logwarn('no odom/compass sample available for re-derivation')
            return None

        odom_x, odom_y, odom_yaw = self.odom
        compass_yaw = self.compass_yaw

        # SIMULATION ONLY: ground-truth world pose as the position anchor.
        try:
            rospy.wait_for_service('/gazebo/get_model_state', timeout=2.0)
            res = self._get_model_state('husky', 'world')
        except Exception as exc:            # service down, timeout, or transport error
            rospy.logwarn('/gazebo/get_model_state unavailable (%s)', exc)
            return None
        if not res.success:
            rospy.logwarn('/gazebo/get_model_state failed: %s', res.status_message)
            return None

        world_x = res.pose.position.x
        world_y = res.pose.position.y

        # Cross-check: ground-truth world yaw should match /compass/data closely
        # (measured agreement 0.01 deg). A large gap means the compass is not
        # the absolute reference we believe it to be -- worth shouting about.
        q = [res.pose.orientation.x, res.pose.orientation.y,
             res.pose.orientation.z, res.pose.orientation.w]
        gt_yaw = tf.transformations.euler_from_quaternion(q)[2]
        yaw_gap = _norm(gt_yaw - compass_yaw)
        if abs(yaw_gap) > math.radians(5.0):
            rospy.logwarn('compass vs ground-truth world yaw disagree by %.1f deg',
                          math.degrees(yaw_gap))

        theta = _norm(odom_yaw - compass_yaw)
        return (theta, math.cos(theta), math.sin(theta),
                (odom_x, odom_y), (world_x, world_y))

    @staticmethod
    def _world_to_odom(tf_tuple, wx, wy):
        """WORLD (x, y) -> odom (x, y) using a transform from _current_transform.

            odom = odom_now + R(theta) * (world_wp - world_now)
        """
        _theta, cos_t, sin_t, (ox, oy), (awx, awy) = tf_tuple
        dx = wx - awx
        dy = wy - awy
        return (ox + cos_t * dx - sin_t * dy,
                oy + sin_t * dx + cos_t * dy)

    def tick(self, _evt):
        if getattr(self, '_started', False):
            return
        if not self.calibrated or not self.waypoints:
            return
        self._started = True
        self.run()

    def run(self):
        n = len(self.waypoints)

        for i in range(1, n + 1):
            if rospy.is_shutdown():
                return

            # RE-DERIVE the WORLD->odom transform from the CURRENT sensor state,
            # immediately before building this goal. This is the whole point of
            # the fix: whatever yaw the EKF has drifted by over the legs driven
            # so far is absorbed into this fresh anchor instead of compounding.
            tf_now = self._current_transform()
            if tf_now is None:
                if self._last_tf is None:
                    rospy.logerr('waypoint %d/%d: no transform available at all '
                                 '(and no previous one to fall back on) -- '
                                 'skipping', i, n)
                    continue
                rospy.logwarn('waypoint %d/%d: re-derivation failed, reusing the '
                              'previous transform (drift will NOT be cancelled '
                              'on this leg)', i, n)
                tf_now = self._last_tf
            else:
                self._last_tf = tf_now

            theta, _cos_t, _sin_t, odom_now, world_now = tf_now

            # Convert THIS waypoint (and, for the heading, the neighbouring one)
            # with the SAME freshly derived transform, so the orientation stays
            # consistent with the position it is attached to.
            x, y = self._world_to_odom(tf_now, *self.waypoints[i - 1])

            # Face the following waypoint, so the robot arrives already pointing
            # down the next leg. The last goal keeps the previous heading.
            if i < n:
                nx, ny = self._world_to_odom(tf_now, *self.waypoints[i])
                yaw = math.atan2(ny - y, nx - x)
            elif i > 1:
                px, py = self._world_to_odom(tf_now, *self.waypoints[i - 2])
                yaw = math.atan2(y - py, x - px)
            else:
                yaw = 0.0

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'odom'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y
            goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)

            # Per-goal diagnostic: the freshly computed theta, the world/odom
            # anchors it was built from, and how far theta has moved since
            # startup. That last number IS the accumulated EKF yaw drift being
            # cancelled on this leg -- if it grows monotonically, the drift is
            # real and this re-derivation is earning its keep.
            drift = _norm(theta - self._theta0) if self._theta0 is not None else 0.0
            rospy.loginfo('waypoint %d/%d -> odom (%.2f, %.2f) | '
                          'world_wp (%.2f, %.2f) world_now (%.2f, %.2f) '
                          'odom_now (%.2f, %.2f) theta=%.3f rad (%.1f deg) '
                          'drift vs startup=%.3f rad (%.1f deg)',
                          i, n, x, y,
                          self.waypoints[i - 1][0], self.waypoints[i - 1][1],
                          world_now[0], world_now[1],
                          odom_now[0], odom_now[1],
                          theta, math.degrees(theta),
                          drift, math.degrees(drift))
            self.client.send_goal(goal)
            self.client.wait_for_result(rospy.Duration(300))
            state = self.client.get_state()
            if state == 3:
                rospy.loginfo('  reached waypoint %d', i)
            else:
                rospy.logwarn('  waypoint %d ended with state %d (continuing)',
                              i, state)

        rospy.loginfo('route complete')


if __name__ == '__main__':
    try:
        WorldGoalSender()
    except rospy.ROSInterruptException:
        pass
