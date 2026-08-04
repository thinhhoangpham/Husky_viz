#!/usr/bin/env python3
"""Remote ROS operator: send ONE legitimate GPS goal (lat/lon) to the running
GPS-anchored move_base, watch telemetry, and write a normal-baseline CSV.

The operator now sends a GPS goal (a lat/lon waypoint like the dataset's
/navigation/objetive_gps points). The lat/lon is converted to a map-frame point
via the running navsat_transform /fromLL service (falling back to local WGS84
geodesy if /fromLL is unavailable), and sent as a move_base goal in the "map"
frame -- matching launch/move_base_gps.launch, whose costmaps live in "map".
This is NOT the attacker and there is NO hijack: just one legitimate goal,
monitored to completion in the map frame.

Runs in the operator container (own IP, docker0). Assumes the robot is ALREADY
spawned and move_base is up (host-side spawn-robot-idle.sh). Does NO robot
bring-up — it is a pure remote peer.

CSV columns are the union of every BASELINE signal the repo's attack CSVs log,
on a shared elapsed_time clock, so each attack's plot can overlay its series.
Attack-injected columns (fake_yaw_deg, value_written, d_*) have no baseline and
stay in the attack CSVs.
"""
import argparse
import csv
import math
import os
import sys
import threading

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# goal_marker.py lives at the repo root; when run by path, sys.path[0] is this
# script's dir (operator/), so add the repo root so the import resolves.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from goal_marker import place_goal_marker

# Map-frame pose so the yaw and distance-to-goal math are in the SAME frame as
# the map-frame goal we send. (The old /odometry/filtered is the odom frame.)
ODOM_TOPIC = "/odometry/filtered_map"
GOAL_FRAME = "map"
PLANNER_CMD_TOPIC = "/cmd_vel"                              # move_base output
CTRL_CMD_TOPIC = "/husky_velocity_controller/cmd_vel"      # controller input

# GPS -> map fallback datum + WGS84 geodesy (mirrors send_gps_goal.py /
# drive_to_point_gps.py). Datum from gps.urdf.xacro (referenceLatitude 49.9,
# referenceLongitude 8.9, referenceHeading 0). Only used if /fromLL is absent.
REF_LAT = 49.9
REF_LON = 8.9
EQUATORIAL_RADIUS = 6378137.0
FLATTENING = 1.0 / 298.257223563
E2 = 2.0 * FLATTENING - FLATTENING ** 2
_SIN2_REF_LAT = math.sin(math.radians(REF_LAT)) ** 2
RADIUS_NORTH = EQUATORIAL_RADIUS * (1.0 - E2) / (1.0 - E2 * _SIN2_REF_LAT) ** 1.5
RADIUS_EAST = (EQUATORIAL_RADIUS / math.sqrt(1.0 - E2 * _SIN2_REF_LAT)
               * math.cos(math.radians(REF_LAT)))
DEG_LAT_PER_METRE = math.degrees(1.0 / RADIUS_NORTH)
DEG_LON_PER_METRE = math.degrees(1.0 / RADIUS_EAST)
FROMLL_WAIT_S = 3.0   # short wait for /fromLL before falling back to geodesy
STATUS_TEXT = {
    GoalStatus.PENDING: "PENDING", GoalStatus.ACTIVE: "ACTIVE",
    GoalStatus.SUCCEEDED: "SUCCEEDED", GoalStatus.ABORTED: "ABORTED",
    GoalStatus.REJECTED: "REJECTED", GoalStatus.PREEMPTED: "PREEMPTED",
    GoalStatus.LOST: "LOST",
}
CSV_HEADER = ["elapsed_time", "fused_x", "fused_y", "fused_yaw", "fused_yaw_deg",
              "planner_linear_x", "planner_angular_z",
              "ctrl_linear_x", "ctrl_angular_z", "ref_x", "ref_y"]


def yaw_of(odom):
    q = odom.pose.pose.orientation
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def fix_to_world_fallback(latitude, longitude):
    """Local WGS84 conversion of a fix to (map_x, map_y). world +x NORTH,
    +y WEST (hence the minus on y). Mirrors send_gps_goal.py."""
    map_x = (latitude - REF_LAT) / DEG_LAT_PER_METRE
    map_y = -(longitude - REF_LON) / DEG_LON_PER_METRE
    return map_x, map_y


def latlon_to_map(latitude, longitude, altitude=0.0):
    """Convert (lat, lon, alt) -> (map_x, map_y) via the running navsat_transform
    /fromLL service (authoritative -- uses the live datum). Fall back to local
    WGS84 geodesy if /fromLL is not available. Returns (map_x, map_y, path)."""
    try:
        from robot_localization.srv import FromLL, FromLLRequest
        from geographic_msgs.msg import GeoPoint
    except ImportError:
        rospy.logwarn("robot_localization/geographic_msgs not available; using "
                      "local WGS84 geodesy fallback (datum %.4f/%.4f).",
                      REF_LAT, REF_LON)
        x, y = fix_to_world_fallback(latitude, longitude)
        return x, y, "geodesy-fallback"

    try:
        rospy.wait_for_service("/fromLL", timeout=FROMLL_WAIT_S)
    except rospy.ROSException:
        rospy.logwarn("/fromLL not available within %.1fs; using local WGS84 "
                      "geodesy fallback (datum %.4f/%.4f).",
                      FROMLL_WAIT_S, REF_LAT, REF_LON)
        x, y = fix_to_world_fallback(latitude, longitude)
        return x, y, "geodesy-fallback"

    try:
        from_ll = rospy.ServiceProxy("/fromLL", FromLL)
        req = FromLLRequest()
        req.ll_point = GeoPoint(latitude=latitude, longitude=longitude,
                                altitude=altitude)
        resp = from_ll(req)
        rospy.loginfo("/fromLL: (lat=%.7f, lon=%.7f, alt=%.2f) -> map=(%.3f, %.3f)",
                      latitude, longitude, altitude,
                      resp.map_point.x, resp.map_point.y)
        return resp.map_point.x, resp.map_point.y, "fromLL"
    except rospy.ServiceException as exc:
        rospy.logwarn("/fromLL call failed (%s); using local WGS84 geodesy "
                      "fallback.", exc)
        x, y = fix_to_world_fallback(latitude, longitude)
        return x, y, "geodesy-fallback"


class Operator(object):
    def __init__(self, args):
        self.args = args
        # Resolved map-frame goal, filled in run() once (lat,lon) is converted.
        self._goal_x = 0.0
        self._goal_y = 0.0
        self._lock = threading.Lock()
        self._odom = None          # latest Odometry
        self._planner_cmd = (0.0, 0.0)  # (linear.x, angular.z) from /cmd_vel
        self._ctrl_cmd = (0.0, 0.0)     # from controller cmd_vel
        rospy.Subscriber(ODOM_TOPIC, Odometry, self._on_odom, queue_size=1)
        rospy.Subscriber(PLANNER_CMD_TOPIC, Twist, self._on_planner, queue_size=1)
        rospy.Subscriber(CTRL_CMD_TOPIC, Twist, self._on_ctrl, queue_size=1)
        self._csv_file = open(args.csv, "w", newline="")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(CSV_HEADER)
        self._csv_file.flush()

    def _on_odom(self, msg):
        with self._lock:
            self._odom = msg

    def _on_planner(self, msg):
        with self._lock:
            self._planner_cmd = (msg.linear.x, msg.angular.z)

    def _on_ctrl(self, msg):
        with self._lock:
            self._ctrl_cmd = (msg.linear.x, msg.angular.z)

    def _write_row(self, elapsed):
        with self._lock:
            odom = self._odom
            plx, paz = self._planner_cmd
            clx, caz = self._ctrl_cmd
        if odom is None:
            return None
        px = odom.pose.pose.position.x
        py = odom.pose.pose.position.y
        yaw = yaw_of(odom)
        self._csv.writerow(
            ["%.3f" % elapsed, "%.4f" % px, "%.4f" % py,
             "%.4f" % yaw, "%.4f" % math.degrees(yaw),
             "%.4f" % plx, "%.4f" % paz, "%.4f" % clx, "%.4f" % caz,
             "%.4f" % self._goal_x, "%.4f" % self._goal_y])
        self._csv_file.flush()
        return (px, py, yaw)

    def run(self):
        client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server ...")
        if not client.wait_for_server(rospy.Duration(60.0)):
            rospy.logerr("move_base action server not available.")
            return 1

        # Convert the operator's GPS waypoint (lat, lon) to a map-frame point.
        gx, gy, path = latlon_to_map(self.args.lat, self.args.lon, self.args.alt)
        self._goal_x, self._goal_y = gx, gy
        rospy.loginfo("Operator GPS goal (lat=%.7f, lon=%.7f) -> map=(%.3f, %.3f) "
                      "via %s", self.args.lat, self.args.lon, gx, gy, path)

        # Heading toward the goal from the current map-frame pose, so the robot
        # faces its target on arrival. Pose is read in the SAME frame as the goal.
        start = rospy.wait_for_message(ODOM_TOPIC, Odometry, timeout=30.0)
        sp = start.pose.pose.position
        gyaw = math.atan2(gy - sp.y, gx - sp.x)
        gq = quaternion_from_euler(0.0, 0.0, gyaw)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = GOAL_FRAME
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = gx
        goal.target_pose.pose.position.y = gy
        goal.target_pose.pose.orientation.x = gq[0]
        goal.target_pose.pose.orientation.y = gq[1]
        goal.target_pose.pose.orientation.z = gq[2]
        goal.target_pose.pose.orientation.w = gq[3]

        rospy.loginfo("Sending goal (frame=%s): x=%.3f y=%.3f", GOAL_FRAME, gx, gy)
        # Visual: GREEN disc FLOATING above the real goal (best-effort). frame="map"
        # so (gx, gy) are used as world coords directly (no odom rotation), and the
        # fixed marker floats above the lidar height gate so it is not scanned.
        place_goal_marker("goal_marker_real", gx, gy, "0 1 0", frame="map")
        start_t = rospy.Time.now()
        client.send_goal(goal)

        rate = rospy.Rate(1.0)
        deadline = start_t + rospy.Duration(self.args.timeout)
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_t).to_sec()
            pose = self._write_row(elapsed)
            state = client.get_state()
            if pose is not None:
                dist = math.hypot(self._goal_x - pose[0],
                                  self._goal_y - pose[1])
                rospy.loginfo("state=%s pos=(%.2f, %.2f) dist_to_goal=%.2f m",
                              STATUS_TEXT.get(state, state), pose[0], pose[1], dist)
            else:
                rospy.loginfo("state=%s (no odom yet)", STATUS_TEXT.get(state, state))
            if state in (GoalStatus.SUCCEEDED, GoalStatus.ABORTED,
                         GoalStatus.REJECTED, GoalStatus.PREEMPTED, GoalStatus.LOST):
                rospy.loginfo("Final move_base state: %s", STATUS_TEXT.get(state, state))
                return 0 if state == GoalStatus.SUCCEEDED else 2
            if rospy.Time.now() > deadline:
                rospy.logwarn("Timed out after %ss; last state=%s",
                              self.args.timeout, STATUS_TEXT.get(state, state))
                return 3
            rate.sleep()
        return 0

    def shutdown(self):
        if self._csv_file and not self._csv_file.closed:
            self._csv_file.flush()
            self._csv_file.close()
            rospy.loginfo("CSV saved to %s", self.args.csv)


def main():
    p = argparse.ArgumentParser(
        description="Remote operator: send one GPS (lat/lon) move_base goal.")
    p.add_argument("--lat", type=float, required=True,
                   help="operator GPS goal latitude (decimal degrees)")
    p.add_argument("--lon", type=float, required=True,
                   help="operator GPS goal longitude (decimal degrees)")
    p.add_argument("--alt", type=float, default=0.0,
                   help="goal altitude (m) for /fromLL; default 0.0")
    p.add_argument("--csv", default="operator_run.csv",
                   help="baseline CSV output path (default operator_run.csv)")
    p.add_argument("--timeout", type=float, default=180.0,
                   help="give up after this many seconds (default 180)")
    args = p.parse_args()

    rospy.init_node("operator", anonymous=True)
    op = Operator(args)
    try:
        return op.run()
    finally:
        op.shutdown()


if __name__ == "__main__":
    sys.exit(main())
