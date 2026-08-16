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
import time

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PointStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseGoal
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Bool, String
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from gcs_state import GcsState
from gcs_csv import CSV_HEADER as GCS_CSV_HEADER, build_row
from gcs_intervene import Intervene
from gcs_commands import parse_command
from objects import load_objects, resolve as resolve_object  # operator/ is on sys.path[0]
from waypoint_queue import WaypointQueue

# goal_marker.py lives at the repo root; when run by path, sys.path[0] is this
# script's dir (operator/), so add the repo root so the import resolves.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from goal_marker import place_goal_marker

# Map-frame pose so the yaw and distance-to-goal math are in the SAME frame as
# the map-frame goal we send. (The old /odometry/filtered is the odom frame.)
ODOM_TOPIC = "/odometry/filtered_map"
GOAL_FRAME = "map"
# Named-objects table for `goal <name>`. Park by default so existing runs are
# unchanged; override per world without editing this file, either way:
#   ROS param : _objects_path:=/repo/maps/lake_objects.yaml
#   env var   : OBJECTS_PATH=/repo/maps/lake_objects.yaml   (works in the
#               container, where the REPL is exec'd rather than roslaunched)
# Resolved lazily in _objects_path() -- NOT at import, because rospy params are
# not available until init_node has run.
_DEFAULT_OBJECTS_PATH = os.path.join(
    os.path.dirname(__file__), "..", "maps", "park_objects.yaml")


def _objects_path():
    """Path to the named-objects YAML: ROS param, then env, then park default."""
    try:
        p = rospy.get_param("~objects_path", None)
    except Exception:
        p = None
    return p or os.environ.get("OBJECTS_PATH") or _DEFAULT_OBJECTS_PATH
# Route waypoint coordination with the localizer (ruling F3):
#   localizer -> operator: /landmark_arrival_confirmed (Bool) -- True when the
#     localizer PERCEIVES the expected distinctive region at the active waypoint.
#     This Bool is the ONLY thing that advances the route; move_base SUCCEEDED
#     and the (spoofable) fused pose never advance it.
#   operator -> localizer: /operator/active_waypoint (PointStamped, map frame)
#     and /operator/expected_anchors (String, comma-separated anchor names) tell
#     the localizer which waypoint is live and what to expect there.
ARRIVAL_CONFIRMED_TOPIC = "/landmark_arrival_confirmed"
ACTIVE_WAYPOINT_TOPIC = "/operator/active_waypoint"
EXPECTED_ANCHORS_TOPIC = "/operator/expected_anchors"
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

# Mirror of landmark_loc/abs_fix_selector.SOURCES (kept in sync by hand; the
# selector owns the authoritative copy). Operator sends the INPUT TOPIC NAME to
# the topic_tools/MuxSelect service.
ABS_FIX_SOURCES = {"gps": "/odometry/gps_fix",
                   "landmark": "/odometry/landmark_fix"}
ABS_FIX_SOURCES_INV = {v: k for k, v in ABS_FIX_SOURCES.items()}
SET_ABS_FIX_MODE_SRV = "/set_abs_fix_mode"
STATUS_TEXT = {
    GoalStatus.PENDING: "PENDING", GoalStatus.ACTIVE: "ACTIVE",
    GoalStatus.SUCCEEDED: "SUCCEEDED", GoalStatus.ABORTED: "ABORTED",
    GoalStatus.REJECTED: "REJECTED", GoalStatus.PREEMPTED: "PREEMPTED",
    GoalStatus.LOST: "LOST",
}

def snap_to_free(data, info, wx, wy, max_radius_m=5.0, free_cost=0,
                  clearance_m=0.6, lethal_cost=90):
    """Pure grid math: return (x,y) of the nearest costmap cell to world point
    (wx,wy) that is both genuinely free (cost in [0, free_cost], unknown -1
    excluded) AND has clearance -- every cell within clearance_m of it is
    below lethal_cost. Searches outward in square rings up to max_radius_m so
    the first accepted candidate is the closest one. `data` is the flat
    costmap.data sequence, `info` is the costmap.info (needs .resolution,
    .width, .height, .origin.position.x/y). clearance_m should cover the
    robot's half-footprint plus margin (default 0.6, vs. a 0.5 inflation
    radius) so the snapped goal is actually reachable by the planner, not
    just "less lethal" than the original point. Returns (wx,wy) unchanged if
    no cell satisfies both conditions within max_radius_m.
    """
    res = info.resolution
    ox, oy = info.origin.position.x, info.origin.position.y

    def cost(cx, cy):
        if 0 <= cx < info.width and 0 <= cy < info.height:
            return data[cy * info.width + cx]
        return 100

    def has_clearance(cx, cy):
        rad_cells = math.ceil(clearance_m / res)
        for dc in range(-rad_cells, rad_cells + 1):
            for dr in range(-rad_cells, rad_cells + 1):
                if cost(cx + dc, cy + dr) >= lethal_cost:
                    return False
        return True

    c0 = int((wx - ox) / res)
    r0 = int((wy - oy) / res)
    max_r = int(max_radius_m / res)
    for rad in range(0, max_r + 1):
        for dc in range(-rad, rad + 1):
            for dr in range(-rad, rad + 1):
                if max(abs(dc), abs(dr)) != rad:  # only the ring edge
                    continue
                cx, cy = c0 + dc, r0 + dr
                c = cost(cx, cy)
                if 0 <= c <= free_cost and has_clearance(cx, cy):
                    sx = ox + (cx + 0.5) * res
                    sy = oy + (cy + 0.5) * res
                    return sx, sy
    return wx, wy


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
        self._costmap = None       # latest global costmap (OccupancyGrid)
        self._planner_cmd = (0.0, 0.0)  # (linear.x, angular.z) from /cmd_vel
        self._ctrl_cmd = (0.0, 0.0)     # from controller cmd_vel
        self._abs_fix_mode = None       # latest /abs_fix_mode string (latched)
        rospy.Subscriber(ODOM_TOPIC, Odometry, self._on_odom, queue_size=1)
        rospy.Subscriber(PLANNER_CMD_TOPIC, Twist, self._on_planner, queue_size=1)
        rospy.Subscriber(CTRL_CMD_TOPIC, Twist, self._on_ctrl, queue_size=1)
        rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid,
                          self._on_costmap, queue_size=1)
        rospy.Subscriber("/abs_fix_mode", String, self._on_abs_fix_mode,
                          queue_size=1)

        self.state = GcsState()
        self._active_goal = None      # (x,y) from /move_base/goal
        self._last_odom_wall = None   # for heartbeat
        self._stop = threading.Event()
        rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, self._on_active_goal, queue_size=1)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self._on_status, queue_size=1)
        self._teleop_pub = rospy.Publisher("joy_teleop/cmd_vel", Twist, queue_size=1)
        self._estop_pub = rospy.Publisher("e_stop", Bool, queue_size=1, latch=True)
        self._intervene = Intervene(self._teleop_pub, self._estop_pub, Twist, Bool)

        # Multi-waypoint route state. _route is None unless a `route` command is
        # active; _route_objects caches the name->(x,y) table for the run.
        self._route = None            # WaypointQueue or None
        self._route_objects = None    # dict name -> (x, y)
        self._active_waypoint_pub = rospy.Publisher(
            ACTIVE_WAYPOINT_TOPIC, PointStamped, queue_size=1, latch=True)
        self._expected_anchors_pub = rospy.Publisher(
            EXPECTED_ANCHORS_TOPIC, String, queue_size=1, latch=True)
        rospy.Subscriber(ARRIVAL_CONFIRMED_TOPIC, Bool,
                         self._on_arrival_confirmed, queue_size=1)

        self._csv_file = open(args.csv, "w", newline="")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(GCS_CSV_HEADER)
        self._csv_file.flush()

    def _on_odom(self, msg):
        with self._lock:
            self._odom = msg
            self._last_odom_wall = time.time()

    def _on_costmap(self, msg):
        with self._lock:
            self._costmap = msg

    def _snap_to_free(self, wx, wy, max_radius_m=5.0, free_cost=0,
                       clearance_m=0.6, lethal_cost=90):
        """Snap (wx,wy) to the nearest free cell (with footprint clearance)
        in the latest global costmap. Returns the input unchanged if no
        costmap yet or nothing suitable found within max_radius_m; caller
        still sends the goal in that case."""
        with self._lock:
            cm = self._costmap
        if cm is None:
            rospy.logwarn("_snap_to_free: no costmap received yet; sending "
                          "goal unsnapped (%.2f, %.2f)", wx, wy)
            return wx, wy
        return snap_to_free(cm.data, cm.info, wx, wy, max_radius_m=max_radius_m,
                             free_cost=free_cost, clearance_m=clearance_m,
                             lethal_cost=lethal_cost)

    def _on_planner(self, msg):
        with self._lock:
            self._planner_cmd = (msg.linear.x, msg.angular.z)

    def _on_ctrl(self, msg):
        with self._lock:
            self._ctrl_cmd = (msg.linear.x, msg.angular.z)

    def _on_abs_fix_mode(self, msg):
        with self._lock:
            self._abs_fix_mode = msg.data

    def _on_active_goal(self, msg):
        p = msg.goal.target_pose.pose.position
        with self._lock:
            self._active_goal = (p.x, p.y)
            self.state.active_goal = (p.x, p.y)

    def _on_status(self, msg):
        # last status in the array is the current goal's
        if msg.status_list:
            self.state.nav_status = STATUS_TEXT.get(
                msg.status_list[-1].status, str(msg.status_list[-1].status))

    def _heartbeat_age(self):
        if self._last_odom_wall is None:
            return float("nan")
        return time.time() - self._last_odom_wall

    def _write_row(self, elapsed):
        if self._stop.is_set():
            return None
        with self._lock:
            odom = self._odom
            plx, paz = self._planner_cmd
            clx, caz = self._ctrl_cmd
            active_goal = self._active_goal
        if odom is None:
            pose = (None, None, None)
        else:
            px = odom.pose.pose.position.x
            py = odom.pose.pose.position.y
            yaw = yaw_of(odom)
            pose = (px, py, yaw)
        row = build_row(
            elapsed, pose, (plx, paz), (clx, caz),
            self.state.sent_goal, active_goal,
            self.state.nav_status, self._heartbeat_age(), self.state.mode)
        self._csv.writerow(row)
        self._csv_file.flush()
        return pose if pose[0] is not None else None

    def run(self, initial_goal=None):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server ...")
        self.client.wait_for_server(rospy.Duration(60.0))
        self._start_wall = time.time()
        # background telemetry/CSV writer (~2 Hz)
        self._writer = threading.Thread(target=self._telemetry_loop)
        self._writer.daemon = True
        self._writer.start()
        if initial_goal is not None:
            self._do_goal(initial_goal[0], initial_goal[1])
        self._print_help()
        while not rospy.is_shutdown():
            try:
                line = input("operator> ")
            except (EOFError, KeyboardInterrupt):
                break
            cmd, args = parse_command(line)
            if cmd == "quit":
                break
            try:
                self._dispatch(cmd, args)
            except Exception as e:
                rospy.logwarn("command failed: %s", e)
        return 0

    def _telemetry_loop(self):
        rate = rospy.Rate(2.0)
        while not rospy.is_shutdown() and not self._stop.is_set():
            self._write_row((time.time() - self._start_wall))
            try:
                rate.sleep()
            except rospy.exceptions.ROSInterruptException:
                break

    def _dispatch(self, cmd, args):
        if cmd == "noop":
            return
        if cmd == "goal":
            self._do_goal(args[0], args[1])
        elif cmd == "goal_xy":
            self._do_goal_xy(args[0], args[1])
        elif cmd == "goal_name":
            try:
                objects = load_objects(_objects_path())
                gx, gy = resolve_object(args[0], objects)
            except (KeyError, IOError) as exc:
                rospy.logwarn("named goal failed: %s", exc)
                return
            rospy.loginfo("named goal '%s' -> map (%.2f, %.2f)", args[0], gx, gy)
            sx, sy = self._snap_to_free(gx, gy)
            if (sx, sy) != (gx, gy):
                rospy.loginfo("named goal '%s': snapped (%.2f,%.2f)->(%.2f,%.2f) "
                              "to nearest free cell", args[0], gx, gy, sx, sy)
            self._do_goal_xy(sx, sy)
        elif cmd == "route":
            self._do_route(args)
        elif cmd == "cancel":
            self._route = None; self._route_objects = None
            self.client.cancel_all_goals(); self.state.set_mode("AUTO")
            rospy.loginfo("CANCELLED goal")
        elif cmd == "teleop":
            self.state.set_mode("MANUAL"); self._teleop_repl()
        elif cmd == "stop":
            self._route = None; self._route_objects = None
            self._intervene.stop(); self.client.cancel_all_goals()
            self.state.set_mode("STOPPED")
            rospy.loginfo("STOP (zero velocity + cancel active goal)")
        elif cmd == "estop":
            self._intervene.engage_estop(); self.state.engage_estop()
            rospy.logwarn("E-STOP ENGAGED")
        elif cmd == "release":
            self._intervene.release_estop(); self.state.release_estop()
            rospy.loginfo("E-STOP RELEASED")
        elif cmd == "auto":
            self.state.set_mode("AUTO"); rospy.loginfo("AUTO mode")
        elif cmd == "status":
            self._print_status()
        elif cmd == "mode":
            self._do_mode(args[0])
        elif cmd in ("help", "unknown", "error"):
            self._print_help() if cmd == "help" else rospy.logwarn(" ".join(args) or "?")

    def _do_goal(self, lat, lon):
        gx, gy, path = latlon_to_map(lat, lon)
        self._goal_x, self._goal_y = gx, gy
        self.state.sent_goal = (gx, gy)
        with self._lock:
            cached_odom = self._odom
        if cached_odom is not None:
            sp = cached_odom.pose.pose.position
        else:
            # No odom cached yet (e.g. right at startup) -- short bounded wait
            # instead of the REPL-freezing 30s blocking wait_for_message.
            try:
                start = rospy.wait_for_message(ODOM_TOPIC, Odometry, timeout=2.0)
                sp = start.pose.pose.position
            except rospy.ROSException:
                rospy.logwarn("_do_goal: no odom available yet; sending goal "
                              "with heading 0.0 (unable to face the target).")
                sp = None
        gyaw = math.atan2(gy - sp.y, gx - sp.x) if sp is not None else 0.0
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
        place_goal_marker("goal_marker_real", gx, gy, "0 1 0", frame="map")
        self.client.send_goal(goal)
        self.state.set_mode("AUTO")
        rospy.loginfo("SENT goal map=(%.2f, %.2f) via %s", gx, gy, path)

    def _do_goal_xy(self, gx, gy):
        """Send a goal already in map-frame metres (no lat/lon conversion).
        Identical to _do_goal from `self._goal_x = ...` onward; only the
        lat/lon -> map step is skipped because (gx, gy) is already map-frame."""
        self._goal_x, self._goal_y = gx, gy
        self.state.sent_goal = (gx, gy)
        with self._lock:
            cached_odom = self._odom
        if cached_odom is not None:
            sp = cached_odom.pose.pose.position
        else:
            # No odom cached yet (e.g. right at startup) -- short bounded wait
            # instead of the REPL-freezing 30s blocking wait_for_message.
            try:
                start = rospy.wait_for_message(ODOM_TOPIC, Odometry, timeout=2.0)
                sp = start.pose.pose.position
            except rospy.ROSException:
                rospy.logwarn("_do_goal_xy: no odom available yet; sending goal "
                              "with heading 0.0 (unable to face the target).")
                sp = None
        gyaw = math.atan2(gy - sp.y, gx - sp.x) if sp is not None else 0.0
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
        place_goal_marker("goal_marker_real", gx, gy, "0 1 0", frame="map")
        self.client.send_goal(goal)
        self.state.set_mode("AUTO")
        rospy.loginfo("SENT map goal (%.2f, %.2f)", gx, gy)

    def _do_route(self, names):
        """Start a multi-waypoint route. Resolve every name up front (so a typo
        fails the whole route before we move), build a WaypointQueue, then drive
        the first waypoint. The route advances ONLY when the localizer publishes
        True on /landmark_arrival_confirmed (see _on_arrival_confirmed) -- never
        on move_base status."""
        try:
            objects = load_objects(_objects_path())
        except IOError as exc:
            rospy.logwarn("route failed to load objects: %s", exc)
            return
        # Validate all names before committing, using the same resolver as `goal`.
        try:
            for name in names:
                resolve_object(name, objects)
        except KeyError as exc:
            rospy.logwarn("route aborted: %s", exc)
            return
        self._route_objects = objects
        self._route = WaypointQueue(names)
        rospy.loginfo("ROUTE started: %s", " -> ".join(names))
        self._send_current_waypoint()

    def _send_current_waypoint(self):
        """Drive the route's current waypoint: send its (snapped) map goal and
        publish active_waypoint + expected_anchors so the localizer knows what to
        look for. No-op (with a completion log) once the route is done."""
        if self._route is None:
            return
        name = self._route.current()
        if name is None:
            rospy.loginfo("ROUTE complete: all waypoints perception-confirmed.")
            self._route = None
            self._route_objects = None
            return
        gx, gy = resolve_object(name, self._route_objects)
        sx, sy = self._snap_to_free(gx, gy)
        if (sx, sy) != (gx, gy):
            rospy.loginfo("route waypoint '%s': snapped (%.2f,%.2f)->(%.2f,%.2f)",
                          name, gx, gy, sx, sy)
        # Tell the localizer which waypoint is live and what to expect there.
        # Expected-anchor choice: the waypoint's OWN name is the expected
        # distinctive region. Waypoints are named landmarks in park_objects.yaml,
        # so "arriving at pole_A" means "perceive the region around pole_A". If a
        # waypoint later needs a different/extra expected anchor set, this is the
        # single place to change it (publish a different comma-separated String).
        wp = PointStamped()
        wp.header.frame_id = GOAL_FRAME
        wp.header.stamp = rospy.Time.now()
        wp.point.x, wp.point.y = sx, sy
        self._active_waypoint_pub.publish(wp)
        self._expected_anchors_pub.publish(String(data=name))
        rospy.loginfo("route waypoint '%s': goal (%.2f, %.2f), expecting anchor "
                      "'%s'", name, sx, sy, name)
        self._do_goal_xy(sx, sy)

    def _on_arrival_confirmed(self, msg):
        """Localizer perception-confirmed arrival. This is the ONLY trigger that
        advances the route -- move_base SUCCEEDED and the fused pose are ignored
        by design (the fused pose is spoofable). Only True advances; a False
        message is a no-op per WaypointQueue policy."""
        if self._route is None:
            return
        if not msg.data:
            return
        name = self._route.current()
        self._route.on_arrival(confirmed=True)
        rospy.loginfo("route: arrival CONFIRMED at '%s' by localizer perception.",
                      name)
        self._send_current_waypoint()

    def _do_mode(self, name):
        from topic_tools.srv import MuxSelect
        topic = ABS_FIX_SOURCES.get(name)
        if topic is None:
            rospy.logwarn("mode: unknown source '%s'", name)
            return
        try:
            rospy.wait_for_service(SET_ABS_FIX_MODE_SRV, timeout=3.0)
        except rospy.ROSException:
            rospy.logwarn("mode: %s not available (is abs_fix_selector "
                          "running?)", SET_ABS_FIX_MODE_SRV)
            return
        try:
            select = rospy.ServiceProxy(SET_ABS_FIX_MODE_SRV, MuxSelect)
            resp = select(topic)
        except rospy.ServiceException as exc:
            rospy.logwarn("mode: service call failed: %s", exc)
            return
        if not resp.prev_topic:
            rospy.logwarn("mode: selector rejected '%s'", name)
            return
        rospy.loginfo("abs_fix source now '%s' (was '%s')", name,
                      ABS_FIX_SOURCES_INV.get(resp.prev_topic, resp.prev_topic))

    def _teleop_repl(self):
        """Raw-key teleop: i/,=fwd/back, j/l=turn, k=stop, x/Esc=exit.
        Falls back to a no-op if stdin is not a real TTY (e.g. piped input)."""
        try:
            import termios
            import tty
        except ImportError:
            rospy.logwarn("teleop: termios/tty unavailable on this platform; skipping.")
            return
        fd = sys.stdin.fileno()
        try:
            old_settings = termios.tcgetattr(fd)
        except termios.error:
            rospy.logwarn("teleop: stdin is not a TTY; skipping.")
            return
        rospy.loginfo("TELEOP mode: i/,=fwd/back j/l=turn k=stop x/Esc=exit")
        # twist_mux's joy_teleop slot has a 0.5s input timeout, so a single
        # publish per keystroke only moves the robot in short 0.5s pulses.
        # Keep republishing the last-commanded twist at ~10Hz from a small
        # daemon thread until the next key changes it (or stop/exit zeroes
        # it), so holding a direction drives smoothly and continuously.
        teleop_stop = threading.Event()
        desired = [0.0, 0.0]  # [linear.x, angular.z], mutated under teleop_lock
        teleop_lock = threading.Lock()

        def _repeat_publish():
            rate = rospy.Rate(10.0)
            while not teleop_stop.is_set() and not rospy.is_shutdown():
                with teleop_lock:
                    linx, angz = desired
                self._intervene.drive(linx, angz)
                try:
                    rate.sleep()
                except rospy.exceptions.ROSInterruptException:
                    break

        repeater = threading.Thread(target=_repeat_publish)
        repeater.daemon = True
        repeater.start()
        try:
            tty.setraw(fd)
            while not rospy.is_shutdown():
                ch = sys.stdin.read(1)
                if ch in ("x", "\x1b"):
                    break
                elif ch == "i":
                    with teleop_lock:
                        desired[0], desired[1] = 0.4, 0.0
                elif ch == ",":
                    with teleop_lock:
                        desired[0], desired[1] = -0.4, 0.0
                elif ch == "j":
                    with teleop_lock:
                        desired[0], desired[1] = 0.0, 0.8
                elif ch == "l":
                    with teleop_lock:
                        desired[0], desired[1] = 0.0, -0.8
                elif ch == "k":
                    with teleop_lock:
                        desired[0], desired[1] = 0.0, 0.0
        finally:
            teleop_stop.set()
            repeater.join(timeout=1.0)
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            self._intervene.stop()
            self.state.set_mode("AUTO")

    def _print_status(self):
        sx_sy = self.state.sent_goal
        ax_ay = self.state.active_goal
        with self._lock:
            odom = self._odom
        dist = "nan"
        if odom is not None and self.state.sent_goal is not None:
            px = odom.pose.pose.position.x
            py = odom.pose.pose.position.y
            dist = "%.2f" % math.hypot(self.state.sent_goal[0] - px,
                                       self.state.sent_goal[1] - py)
        age = self._heartbeat_age()
        age_str = "n/a" if math.isnan(age) else "%.1fs" % age
        with self._lock:
            abs_mode = self._abs_fix_mode
        abs_mode_str = abs_mode if abs_mode is not None else "n/a"
        print("state=%s | sent=%s | active=%s | dist=%s | mode=%s | estop=%s | "
              "link_age=%s | abs_fix=%s" % (
            self.state.nav_status, sx_sy, ax_ay, dist, self.state.mode,
            self.state.estop_engaged, age_str, abs_mode_str))

    def _print_help(self):
        print(
            "Commands:\n"
            "  goal <lat> <lon>  send a GPS goal (map-frame, via /fromLL)\n"
            "  goal xy <x> <y>   send a map-frame goal (metres) directly\n"
            "  goal <name>       send a goal by name (%s)\n" % os.path.basename(_objects_path()) +
            "  route <name> ...  drive named waypoints in order; advances to the\n"
            "                    next ONLY when the localizer perception-confirms\n"
            "                    arrival (/landmark_arrival_confirmed), never on\n"
            "                    move_base status\n"
            "  mode <gps|landmark>  switch the absolute-position source live\n"
            "  cancel            cancel the active move_base goal\n"
            "  teleop            enter raw-key teleop (i/,/j/l/k, x or Esc to exit)\n"
            "  stop              zero velocity, mode=STOPPED\n"
            "  estop             engage e-stop (latched)\n"
            "  release           release e-stop, mode=AUTO\n"
            "  auto              return to AUTO mode\n"
            "  status            print one-line state snapshot\n"
            "  help              show this message\n"
            "  quit              exit the operator"
        )

    def shutdown(self):
        self._stop.set()
        writer = getattr(self, "_writer", None)
        if writer is not None and writer.is_alive():
            writer.join(timeout=2.0)
        if self._csv_file and not self._csv_file.closed:
            self._csv_file.flush()
            self._csv_file.close()
            rospy.loginfo("CSV saved to %s", self.args.csv)


def main():
    p = argparse.ArgumentParser(description="Interactive GCS operator.")
    p.add_argument("--lat", type=float, default=None)
    p.add_argument("--lon", type=float, default=None)
    p.add_argument("--csv", default="operator_run.csv")
    p.add_argument("--timeout", type=float, default=180.0)
    args = p.parse_args()
    rospy.init_node("operator", anonymous=True)
    op = Operator(args)
    rospy.on_shutdown(op.shutdown)
    initial = (args.lat, args.lon) if (args.lat is not None and args.lon is not None) else None
    try:
        return op.run(initial_goal=initial)
    finally:
        op.shutdown()


if __name__ == "__main__":
    sys.exit(main())
