#!/usr/bin/env python3
"""Send ONE GPS (or map-frame) goal to the ALREADY-RUNNING GPS-anchored move_base
and monitor progress until it succeeds or aborts.

This is the goal SENDER for launch/move_base_gps.launch. It assumes the robot,
the dual-EKF + navsat_transform stack, and move_base are ALL already running --
it starts nothing and stops nothing. It only converts a target to the `map`
frame, sends it as a MoveBaseAction goal, and logs distance-to-goal each second.

TARGET (pick one on the command line):
  --lat / --lon          a GPS waypoint (decimal degrees). Converted to a
                         map-frame point -- see "GPS -> map" below.
  --map-x / --map-y      a map-frame point directly, for testing without GPS.
  (neither)              default: a point DEFAULT_AHEAD_M metres straight ahead
                         of the robot's current /odometry/filtered_map pose.

GPS -> map conversion (two paths, tried in this order):
  1. /fromLL service (robot_localization/FromLL), advertised by the running
     navsat_transform_node. It converts lat/lon/alt -> a map-frame point using
     the SAME datum the live stack loaded, so it is authoritative. Preferred.
  2. FALLBACK, only if /fromLL is not available within FROMLL_WAIT_S: the local
     WGS84 geodesy from drive_to_point_gps.py (datum 49.9 / 8.9, world +x NORTH,
     world +y WEST). Identical math, reused here. The log states which path ran.

Goal orientation faces from the robot's current /odometry/filtered_map pose
toward the target, so the robot arrives pointing roughly the way it travelled.

Usage:
    ./send_gps_goal.py --lat 49.9001 --lon 8.8999
    ./send_gps_goal.py --map-x 40.0 --map-y 2.0
    ./send_gps_goal.py                      # a few metres straight ahead
"""
import argparse
import math
import sys

import rospy
import actionlib
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from actionlib_msgs.msg import GoalStatus

# ---------------------------------------------------------------------------
# GPS -> map (world) metres FALLBACK conversion.
#
# This is the SAME derivation as drive_to_point_gps.py -- datum from the GPS
# plugin's declared parameters in husky_description/urdf/gps.urdf.xacro
# (referenceLatitude 49.9, referenceLongitude 8.9, referenceHeading 0) plus the
# WGS84 ellipsoid radii of curvature at the reference latitude. Nothing here is
# measured from the simulator. IF THAT XACRO'S REFERENCE VALUES CHANGE, REF_LAT /
# REF_LON HERE MUST BE UPDATED TO MATCH -- nothing detects a mismatch at runtime.
# ---------------------------------------------------------------------------
REF_LAT = 49.9  # deg, <referenceLatitude>  in gps.urdf.xacro
REF_LON = 8.9   # deg, <referenceLongitude> in gps.urdf.xacro

EQUATORIAL_RADIUS = 6378137.0
FLATTENING = 1.0 / 298.257223563
E2 = 2.0 * FLATTENING - FLATTENING ** 2

_SIN2_REF_LAT = math.sin(math.radians(REF_LAT)) ** 2
RADIUS_NORTH = EQUATORIAL_RADIUS * (1.0 - E2) / (1.0 - E2 * _SIN2_REF_LAT) ** 1.5
RADIUS_EAST = (EQUATORIAL_RADIUS / math.sqrt(1.0 - E2 * _SIN2_REF_LAT)
               * math.cos(math.radians(REF_LAT)))

DEG_LAT_PER_METRE = math.degrees(1.0 / RADIUS_NORTH)  # deg latitude  per metre NORTH
DEG_LON_PER_METRE = math.degrees(1.0 / RADIUS_EAST)   # deg longitude per metre EAST

ODOM_TOPIC = "/odometry/filtered_map"
GOAL_FRAME = "map"
DEFAULT_AHEAD_M = 5.0     # default target distance straight ahead when no target given
FROMLL_WAIT_S = 3.0       # short wait for /fromLL before falling back to geodesy
ODOM_WAIT_S = 30.0        # wait for the first map-frame pose sample
MOVE_BASE_WAIT_S = 60.0   # wait for the move_base action server
GOAL_TIMEOUT_S = 300.0    # give up monitoring after this long

STATUS_TEXT = {
    GoalStatus.PENDING: "PENDING", GoalStatus.ACTIVE: "ACTIVE",
    GoalStatus.PREEMPTED: "PREEMPTED", GoalStatus.SUCCEEDED: "SUCCEEDED",
    GoalStatus.ABORTED: "ABORTED", GoalStatus.REJECTED: "REJECTED",
    GoalStatus.LOST: "LOST",
}


def fix_to_world_fallback(latitude, longitude):
    """Local WGS84 conversion of a geodetic fix to (map_x, map_y) metres.
    Mirrors drive_to_point_gps.py.fix_to_world. world +x NORTH, world +y WEST
    (hence the minus sign on y, from referenceHeading 0)."""
    map_x = (latitude - REF_LAT) / DEG_LAT_PER_METRE
    map_y = -(longitude - REF_LON) / DEG_LON_PER_METRE
    return map_x, map_y


def latlon_to_map(latitude, longitude, altitude=0.0):
    """Convert (lat, lon, alt) -> (map_x, map_y). Try the running
    navsat_transform /fromLL service first (authoritative -- uses the live
    datum); fall back to the local geodesy if it is not available. Returns
    (map_x, map_y, path_used)."""
    from robot_localization.srv import FromLL, FromLLRequest
    from geographic_msgs.msg import GeoPoint

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


def parse_args():
    p = argparse.ArgumentParser(
        description="Send one GPS/map goal to the running GPS-anchored move_base.")
    p.add_argument("--lat", type=float, help="target latitude (deg)")
    p.add_argument("--lon", type=float, help="target longitude (deg)")
    p.add_argument("--alt", type=float, default=0.0,
                   help="target altitude (m) for /fromLL; default 0.0")
    p.add_argument("--map-x", type=float, help="target map-frame x (m)")
    p.add_argument("--map-y", type=float, help="target map-frame y (m)")
    p.add_argument("--ahead", type=float, default=DEFAULT_AHEAD_M,
                   help="default target distance straight ahead (m) when no "
                        "target is given; default %(default)s")
    args = p.parse_args()
    if (args.lat is None) != (args.lon is None):
        p.error("--lat and --lon must be given together")
    if (args.map_x is None) != (args.map_y is None):
        p.error("--map-x and --map-y must be given together")
    if args.lat is not None and args.map_x is not None:
        p.error("give EITHER --lat/--lon OR --map-x/--map-y, not both")
    return args


def read_current_pose():
    """Read one /odometry/filtered_map sample. Returns (x, y, yaw)."""
    from tf.transformations import euler_from_quaternion
    rospy.loginfo("Waiting for %s ...", ODOM_TOPIC)
    msg = rospy.wait_for_message(ODOM_TOPIC, Odometry, timeout=ODOM_WAIT_S)
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
    rospy.loginfo("Current map pose: x=%.3f y=%.3f yaw=%.4f rad (frame=%s)",
                  p.x, p.y, yaw, msg.header.frame_id)
    return p.x, p.y, yaw


def resolve_target(args, cur_x, cur_y, cur_yaw):
    """Resolve the command-line target to (map_x, map_y). Returns
    (map_x, map_y, description)."""
    if args.lat is not None:
        x, y, path = latlon_to_map(args.lat, args.lon, args.alt)
        return x, y, "GPS (%.7f, %.7f) via %s" % (args.lat, args.lon, path)
    if args.map_x is not None:
        return args.map_x, args.map_y, "map point (direct)"
    # Default: straight ahead of the current pose.
    x = cur_x + args.ahead * math.cos(cur_yaw)
    y = cur_y + args.ahead * math.sin(cur_yaw)
    return x, y, "%.1f m straight ahead of current pose" % args.ahead


def run():
    args = parse_args()

    cur_x, cur_y, cur_yaw = read_current_pose()
    gx, gy, desc = resolve_target(args, cur_x, cur_y, cur_yaw)

    # Face from the current pose toward the goal.
    goal_yaw = math.atan2(gy - cur_y, gx - cur_x)
    gq = quaternion_from_euler(0.0, 0.0, goal_yaw)

    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server (up to %.0fs) ...",
                  MOVE_BASE_WAIT_S)
    if not client.wait_for_server(rospy.Duration(MOVE_BASE_WAIT_S)):
        rospy.logerr("move_base action server not available -- is "
                     "move_base_gps.launch running?")
        return 1

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = GOAL_FRAME
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position = Point(x=gx, y=gy, z=0.0)
    goal.target_pose.pose.orientation.x = gq[0]
    goal.target_pose.pose.orientation.y = gq[1]
    goal.target_pose.pose.orientation.z = gq[2]
    goal.target_pose.pose.orientation.w = gq[3]

    rospy.loginfo("Sending goal (frame=%s): x=%.3f y=%.3f yaw=%.4f  [%s]",
                  GOAL_FRAME, gx, gy, goal_yaw, desc)
    client.send_goal(goal)

    rate = rospy.Rate(1.0)
    deadline = rospy.Time.now() + rospy.Duration(GOAL_TIMEOUT_S)
    while not rospy.is_shutdown():
        state = client.get_state()
        try:
            cur = rospy.wait_for_message(ODOM_TOPIC, Odometry, timeout=2.0)
            cp = cur.pose.pose.position
            dist = math.hypot(gx - cp.x, gy - cp.y)
            rospy.loginfo("state=%s  pos=(%.2f, %.2f)  dist_to_goal=%.2f m",
                          STATUS_TEXT.get(state, state), cp.x, cp.y, dist)
        except rospy.ROSException:
            rospy.loginfo("state=%s (no %s sample)",
                          STATUS_TEXT.get(state, state), ODOM_TOPIC)

        if state in (GoalStatus.SUCCEEDED, GoalStatus.ABORTED,
                     GoalStatus.REJECTED, GoalStatus.PREEMPTED, GoalStatus.LOST):
            rospy.loginfo("Final move_base state: %s", STATUS_TEXT.get(state, state))
            return 0 if state == GoalStatus.SUCCEEDED else 2
        if rospy.Time.now() > deadline:
            rospy.logwarn("Timed out after %.0fs; last state=%s",
                          GOAL_TIMEOUT_S, STATUS_TEXT.get(state, state))
            client.cancel_goal()
            return 3
        rate.sleep()
    return 0


def main():
    rospy.init_node("send_gps_goal", anonymous=True)
    try:
        return run()
    except rospy.ROSInterruptException:
        rospy.logwarn("interrupted")
        return 130
    except KeyboardInterrupt:
        rospy.logwarn("SIGINT received")
        return 130


if __name__ == "__main__":
    sys.exit(main())
