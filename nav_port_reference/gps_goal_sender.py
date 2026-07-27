#!/usr/bin/env python
"""Feed the dataset's GPS waypoints to move_base, one at a time.

move_base plans in a metric frame (odom); the dataset's waypoints are geodetic
(lat/lon on /navigation/objetive_gps). This is the only glue needed to let the
stock navigation stack drive the dataset's route:

    for each waypoint:
        lat/lon -> UTM -> offset from the robot's own first fix -> odom goal
        send to move_base, wait for it to arrive, then send the next

Using the robot's first GPS fix as the origin sidesteps having to know the
GPS->odom rotation: both the goal and the robot are expressed relative to the
same starting point, and the EKF's odom frame starts there too.

Replaces fixer_husky.py. The waypoints, and therefore the route, are identical --
only the thing deciding HOW to get between them changes.

Python 2 -- Melodic.
"""
import math
import time

import rospy
import actionlib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from geonav_transform.geonav_conversions import LLtoUTM


class GpsGoalSender(object):
    def __init__(self):
        rospy.init_node('gps_goal_sender')

        self.origin = None        # UTM easting/northing of the first fix
        self.yaw0 = None          # ENU heading at that first fix
        self.odom0 = None         # odom-frame XY at that first fix
        self.odom_yaw0 = None     # odom-frame heading at that first fix
        self.odom = None          # latest odom pose
        self.waypoints = []
        self.started = False

        rospy.Subscriber('/navsat/fix', NavSatFix, self.on_fix, queue_size=1)
        rospy.Subscriber('/compass/data', Imu, self.on_imu, queue_size=1)
        rospy.Subscriber('/odometry/filtered', Odometry, self.on_odom, queue_size=1)
        rospy.Subscriber('/navigation/objetive_gps', Float64MultiArray,
                         self.on_waypoints, queue_size=1)

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
        self.odom = (p.position.x, p.position.y,
                     tf.transformations.euler_from_quaternion(q)[2])

    def on_imu(self, msg):
        if self.yaw0 is None and self.origin is not None and self.odom is not None:
            q = [msg.orientation.x, msg.orientation.y,
                 msg.orientation.z, msg.orientation.w]
            self.yaw0 = tf.transformations.euler_from_quaternion(q)[2]
            # Anchor the odom frame at this same instant. odom does NOT start at
            # (0,0) -- it accumulates across runs -- so goals must be expressed
            # relative to where the robot actually was in odom at the first fix.
            self.odom0 = (self.odom[0], self.odom[1])
            self.odom_yaw0 = self.odom[2]
            rospy.loginfo('anchored: odom=(%.2f, %.2f) yaw_odom=%.3f yaw_enu=%.3f',
                          self.odom0[0], self.odom0[1], self.odom_yaw0, self.yaw0)

    def on_fix(self, msg):
        if self.origin is None:
            utm = LLtoUTM(msg.latitude, msg.longitude)
            self.origin = (utm[0], utm[1])
            rospy.loginfo('origin fix captured')

    def on_waypoints(self, msg):
        if self.waypoints:
            return
        d = msg.data
        pts = []
        for i in range(0, len(d) - 1, 2):
            p = (d[i], d[i + 1])
            if not pts or p != pts[-1]:     # the list contains duplicates
                pts.append(p)
        self.waypoints = pts
        rospy.loginfo('got %d unique waypoints', len(pts))

    def to_odom(self, lat, lon):
        """GPS -> odom-frame XY.

        Two corrections matter: the ENU offset must be rotated by the angle
        between the ENU and odom frames (their headings differed at the anchor
        instant), and then added to the robot's odom position at that instant --
        odom is not zeroed at start-up.
        """
        utm = LLtoUTM(lat, lon)
        de = utm[0] - self.origin[0]        # metres east
        dn = utm[1] - self.origin[1]        # metres north
        rot = self.odom_yaw0 - self.yaw0    # ENU -> odom rotation
        c, s = math.cos(rot), math.sin(rot)
        return (self.odom0[0] + c * de - s * dn,
                self.odom0[1] + s * de + c * dn)

    def tick(self, _evt):
        if self.started:
            return
        if (self.origin is None or self.yaw0 is None or self.odom0 is None
                or not self.waypoints):
            return
        self.started = True
        self.run()

    def run(self):
        # Pre-convert so each goal can face the NEXT one.
        pts = [self.to_odom(lat, lon) for (lat, lon) in self.waypoints]

        for i, (x, y) in enumerate(pts, 1):
            if rospy.is_shutdown():
                return
            # Face the following waypoint, so the robot arrives already pointing
            # down the next leg. The last goal keeps the previous heading.
            if i < len(pts):
                yaw = math.atan2(pts[i][1] - y, pts[i][0] - x)
            elif i > 1:
                yaw = math.atan2(y - pts[i - 2][1], x - pts[i - 2][0])
            else:
                yaw = 0.0

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'odom'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y
            goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)

            rospy.loginfo('waypoint %d/%d -> odom (%.2f, %.2f)',
                          i, len(self.waypoints), x, y)
            self.client.send_goal(goal)
            self.client.wait_for_result(rospy.Duration(300))
            state = self.client.get_state()
            if state == 3:
                rospy.loginfo('  reached waypoint %d', i)
            else:
                rospy.logwarn('  waypoint %d ended with state %d (continuing)', i, state)

        rospy.loginfo('route complete')


if __name__ == '__main__':
    try:
        GpsGoalSender()
    except rospy.ROSInterruptException:
        pass
