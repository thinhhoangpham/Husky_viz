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
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from gcs_state import GcsState
from gcs_csv import CSV_HEADER as GCS_CSV_HEADER, build_row
from gcs_intervene import Intervene
from gcs_commands import parse_command

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

        self.state = GcsState()
        self._active_goal = None      # (x,y) from /move_base/goal
        self._last_odom_wall = None   # for heartbeat
        self._stop = threading.Event()
        rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, self._on_active_goal, queue_size=1)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self._on_status, queue_size=1)
        self._teleop_pub = rospy.Publisher("joy_teleop/cmd_vel", Twist, queue_size=1)
        self._estop_pub = rospy.Publisher("e_stop", Bool, queue_size=1, latch=True)
        self._intervene = Intervene(self._teleop_pub, self._estop_pub, Twist, Bool)

        self._csv_file = open(args.csv, "w", newline="")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(GCS_CSV_HEADER)
        self._csv_file.flush()

    def _on_odom(self, msg):
        with self._lock:
            self._odom = msg
            self._last_odom_wall = time.time()

    def _on_planner(self, msg):
        with self._lock:
            self._planner_cmd = (msg.linear.x, msg.angular.z)

    def _on_ctrl(self, msg):
        with self._lock:
            self._ctrl_cmd = (msg.linear.x, msg.angular.z)

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
            self._dispatch(cmd, args)
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
        elif cmd == "cancel":
            self.client.cancel_all_goals(); self.state.set_mode("AUTO")
            rospy.loginfo("CANCELLED goal")
        elif cmd == "teleop":
            self.state.set_mode("MANUAL"); self._teleop_repl()
        elif cmd == "stop":
            self._intervene.stop(); self.state.set_mode("STOPPED")
            rospy.loginfo("STOP (zero velocity)")
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
        elif cmd in ("help", "unknown", "error"):
            self._print_help() if cmd == "help" else rospy.logwarn(" ".join(args) or "?")

    def _do_goal(self, lat, lon):
        gx, gy, path = latlon_to_map(lat, lon)
        self._goal_x, self._goal_y = gx, gy
        self.state.sent_goal = (gx, gy)
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
        place_goal_marker("goal_marker_real", gx, gy, "0 1 0", frame="map")
        self.client.send_goal(goal)
        self.state.set_mode("AUTO")
        rospy.loginfo("SENT goal map=(%.2f, %.2f) via %s", gx, gy, path)

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
        try:
            tty.setraw(fd)
            while not rospy.is_shutdown():
                ch = sys.stdin.read(1)
                if ch in ("x", "\x1b"):
                    break
                elif ch == "i":
                    self._intervene.drive(0.4, 0.0)
                elif ch == ",":
                    self._intervene.drive(-0.4, 0.0)
                elif ch == "j":
                    self._intervene.drive(0.0, 0.8)
                elif ch == "l":
                    self._intervene.drive(0.0, -0.8)
                elif ch == "k":
                    self._intervene.stop()
        finally:
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
        print("state=%s | sent=%s | active=%s | dist=%s | mode=%s | estop=%s | link_age=%.1fs" % (
            self.state.nav_status, sx_sy, ax_ay, dist, self.state.mode,
            self.state.estop_engaged, self._heartbeat_age()))

    def _print_help(self):
        print(
            "Commands:\n"
            "  goal <lat> <lon>  send a GPS goal (map-frame, via /fromLL)\n"
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
