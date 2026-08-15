#!/usr/bin/env python3
"""Planner-free autonomous driver modelled on the security report's
`/husky_auto_drive` node -- and the VICTIM of the Parameter Server Manipulation
attack (report §4, see attack_param.py).

  *** SIMULATION-ONLY SECURITY DEMONSTRATION. No real robot is involved. ***

WHAT THIS IS
------------
The report's §4 attack targets a node called `/husky_auto_drive` that owns a
private cruise-speed parameter `~linear_speed` and re-reads it every control
tick to decide how fast to drive. No path planner is involved (unlike
send_mapless_goal.py, which drives via move_base). This script IS that victim:
it does the same STOCK robot bring-up as send_mapless_goal.py, then runs a plain
~10 Hz "hold heading, drive at the param speed" loop instead of sending a
move_base goal. Only one driver can own the /cmd_vel twist_mux 'external' slot at
a time, so run this INSTEAD OF send_mapless_goal.py, never alongside it.

STOCK DEAD-RECKONING TOPOLOGY -- NO GPS, NO COMPASS
---------------------------------------------------
The report's victim robot is the STOCK Husky: wheel encoders + IMU fused by an
odom-frame EKF (/ekf_localization, world_frame: odom), with NO absolute position
or heading source. So this driver's ONLY pose/heading input is
/odometry/filtered (nav_msgs/Odometry, the stock EKF output), read exactly the
way send_mapless_goal.py's yaw_of() reads it. It deliberately does NOT use the
GPS/compass approach of drive_to_point_gps.py: that script steers off /navsat/fix
and /compass/data, sensors the report's robot does not have, so copying it here
would model the wrong robot. We subscribe to NO absolute sensor and read NO
Gazebo ground truth (hard project rule).

Because the EKF drifts in the odom frame, this driver does NOT try to reach an
absolute waypoint -- it simply holds the heading captured ONCE at loop start and
cruises at the param speed. That is enough to make the §4 attack observable: the
whole demonstration is about what the polled speed does to the wheels, not about
navigation accuracy.

WHY IT RE-READS THE PARAM EVERY TICK
------------------------------------
Each control tick this loop calls rospy.get_param("~linear_speed", ...) fresh and
caches NOTHING between ticks. That live re-read is the entire point: it is what
makes the parameter-server attack land. When attack_param.py writes a new value
to /husky_auto_drive/linear_speed, the very next tick here picks it up and the
commanded wheel speed changes -- reverse (-5.0), then over-speed (100.0, clamped
by the controller/physics), then stop (0.0). A driver that read the param once at
startup would be immune, and there would be nothing to demonstrate.

STEERING SIGN CONVENTION (CLAUDE.md)
------------------------------------
Positive angular.z is CCW / left. heading_error > 0 means the target heading is
to the left, so angular.z is positive -- no negation anywhere. We only hold the
initial heading, so heading_error is (initial_yaw - current_yaw), wrapped to
[-pi, pi].

We do NOT clamp the sign of the polled linear speed: a negative param must drive
the robot backward so the report's -5.0 step is actually visible on the wheels.

USAGE
-----
    ./husky_auto_drive.py [--rate 10] [--linear-speed 0.5] [--k-heading 1.5]
                          [--duration 0] [--csv]

Run alongside the attack (three terminals):
    (1) ./load-park-stock-husky.sh          # park WORLD only
    (2) ./husky_auto_drive.py               # this victim driver
    (3) ./attack_param.py                   # the §4 param attack

This script was NOT run live end-to-end here (the sim may be down, and running is
the operator's call). Topic/type wiring is stated to MATCH THE DESIGN:
nav_msgs/Odometry in on /odometry/filtered, geometry_msgs/Twist out on /cmd_vel
(twist_mux priority-1 'external' slot).
"""
import argparse
import csv
import math
import os
import signal
import subprocess
import sys
import time

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import DeleteModel
from tf.transformations import euler_from_quaternion

ODOM_TOPIC = "/odometry/filtered"
CMD_VEL_TOPIC = "/cmd_vel"  # twist_mux priority-1 'external' slot

# ---------------------------------------------------------------------------
# Report §4 reference value, kept as a named constant so the banner can echo it
# and a reader can see the sane cruise default the attack later overwrites.
# ---------------------------------------------------------------------------
REPORT_CRUISE_SPEED = 0.5   # m/s -- sane forward cruise, set as ~linear_speed
                            # at startup so the param EXISTS before any attack.

# On-path spawn: identical to send_mapless_goal.py (first bag waypoint, facing
# straight down the trail). See that file for the full derivation. z above the
# settled terrain so the robot drops and settles.
SPAWN_X = 38.26
SPAWN_Y = 1.25
SPAWN_Z = 3.3
SPAWN_YAW = -3.1281

# Bring-up wait bounds -- same rationale as send_mapless_goal.py.
ROBOT_DESCRIPTION_TIMEOUT_S = 60.0
CONTROLLER_TIMEOUT_S = 120.0

# Pre-spawn cleanup bounds -- same rationale as send_mapless_goal.py.
DELETE_SERVICE_TIMEOUT_S = 10.0
DELETE_SETTLE_S = 1.5
ROBOT_MODEL_NAME = "husky"


def yaw_of(odom):
    q = odom.pose.pose.orientation
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def wrap_to_pi(angle):
    """Wrap an angle to [-pi, pi] so a heading error never grows past half a
    turn (e.g. an error of +6.0 rad is really -0.28 rad)."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _stop_proc_group(proc, label):
    """Tear down a roslaunch process group started with start_new_session=True:
    SIGINT the whole group (graceful roslaunch shutdown), escalate to SIGTERM
    then SIGKILL if it lingers, so nothing is left orphaned. Copied in spirit
    from send_mapless_goal.py."""
    if proc is None or proc.poll() is not None:
        return
    try:
        pgid = os.getpgid(proc.pid)
    except OSError:
        return

    rospy.loginfo("Shutting down %s (pgid=%d) ...", label, pgid)
    for sig, wait_s in ((signal.SIGINT, 15.0), (signal.SIGTERM, 5.0),
                        (signal.SIGKILL, 2.0)):
        try:
            os.killpg(pgid, sig)
        except OSError:
            return  # group already gone
        try:
            proc.wait(timeout=wait_s)
            return
        except subprocess.TimeoutExpired:
            continue


def start_robot():
    """Bring up the STOCK husky_control/control.launch in its own process group
    (odom-frame EKF, no GPS/compass). Long-lived until teardown. Returns Popen.
    Faithful to send_mapless_goal.py's start_robot()."""
    rospy.loginfo("Robot bring-up: roslaunch husky_control control.launch "
                  "(STOCK - odom EKF, no GPS/compass)")
    return subprocess.Popen(
        ["roslaunch", "husky_control", "control.launch"],
        start_new_session=True,
    )


def stop_robot(proc):
    _stop_proc_group(proc, "robot (husky_control/control.launch)")


def wait_for_robot_description(proc):
    """Block until control.launch has published /robot_description so spawn_model
    has something to place. Bounded; bails early if roslaunch died. True on
    success. Faithful to send_mapless_goal.py."""
    rospy.loginfo("Waiting for /robot_description (up to %.0fs) ...",
                  ROBOT_DESCRIPTION_TIMEOUT_S)
    deadline = time.monotonic() + ROBOT_DESCRIPTION_TIMEOUT_S
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            rospy.logerr("husky_control/control.launch exited (rc=%s) before "
                         "setting /robot_description.", proc.returncode)
            return False
        if rospy.has_param("/robot_description"):
            if rospy.get_param("/robot_description", ""):
                rospy.loginfo("/robot_description is up.")
                return True
        time.sleep(1.0)
    rospy.logerr("/robot_description never appeared within %.0fs.",
                 ROBOT_DESCRIPTION_TIMEOUT_S)
    return False


def delete_existing_robot():
    """Remove any leftover model named "husky" so the spawn below can land at the
    fixed pose. Every failure mode is non-fatal (logged at info). Returns True
    only if an existing husky was genuinely removed. Faithful to
    send_mapless_goal.py; see that file for the full rationale."""
    rospy.loginfo("Pre-spawn cleanup: deleting any existing '%s' model (waiting "
                  "up to %.0fs for %s) ...",
                  ROBOT_MODEL_NAME, DELETE_SERVICE_TIMEOUT_S,
                  "/gazebo/delete_model")
    try:
        rospy.wait_for_service("/gazebo/delete_model",
                               timeout=DELETE_SERVICE_TIMEOUT_S)
    except rospy.ROSException as exc:
        rospy.loginfo("/gazebo/delete_model unavailable within %.0fs (%s) - "
                      "skipping cleanup and going straight to the spawn.",
                      DELETE_SERVICE_TIMEOUT_S, exc)
        return False

    try:
        delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        response = delete_model(model_name=ROBOT_MODEL_NAME)
    except rospy.ServiceException as exc:
        rospy.loginfo("/gazebo/delete_model call failed (%s) - carrying on to "
                      "the spawn; this is not fatal.", exc)
        return False

    if not response.success:
        rospy.loginfo("No existing '%s' to delete (expected on a first run): %s",
                      ROBOT_MODEL_NAME, response.status_message)
        return False

    rospy.loginfo("Deleted an existing '%s' left over from a previous run: %s",
                  ROBOT_MODEL_NAME, response.status_message)
    rospy.sleep(DELETE_SETTLE_S)
    return True


def spawn_robot():
    """Instantiate the robot_description into the live world at the fixed pose.
    A NON-ZERO EXIT IS EXPECTED under the heavy park world and must be tolerated
    -- the controller check is the authoritative success signal, never this exit
    code and never any Gazebo ground-truth pose service. Faithful to
    send_mapless_goal.py; see that file for the full rationale."""
    rospy.loginfo("Spawning STOCK Husky at x=%.2f y=%.2f z=%.2f yaw=%.4f ...",
                  SPAWN_X, SPAWN_Y, SPAWN_Z, SPAWN_YAW)
    rc = subprocess.call([
        "rosrun", "gazebo_ros", "spawn_model",
        "-x", str(SPAWN_X), "-y", str(SPAWN_Y),
        "-z", str(SPAWN_Z), "-Y", str(SPAWN_YAW),
        "-unpause", "-urdf", "-param", "robot_description", "-model", "husky",
    ])
    if rc != 0:
        rospy.loginfo("spawn_model exited rc=%d - EXPECTED under the heavy park "
                      "world (entity-appear timeout); the controller check is "
                      "the real success signal.", rc)


def wait_for_controllers():
    """Poll controller_manager until BOTH husky_joint_publisher and
    husky_velocity_controller are ( running ). Warn but do not abort otherwise
    (a stuck controller SILENTLY drops every cmd_vel). Returns True if both
    reached ( running ). Faithful to send_mapless_goal.py."""
    rospy.loginfo("Waiting for the Husky controllers (up to %.0fs) ...",
                  CONTROLLER_TIMEOUT_S)
    deadline = time.monotonic() + CONTROLLER_TIMEOUT_S
    listing = ""
    while time.monotonic() < deadline:
        try:
            listing = subprocess.run(
                ["rosrun", "controller_manager", "controller_manager", "list"],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                universal_newlines=True, timeout=15.0,
            ).stdout or ""
        except (subprocess.SubprocessError, OSError) as exc:
            rospy.logdebug("controller_manager list failed: %s", exc)
            listing = ""
        running = [line for line in listing.splitlines() if "( running )" in line]
        if (any("husky_joint_publisher" in line for line in running)
                and any("husky_velocity_controller" in line for line in running)):
            rospy.loginfo("Husky controllers are running.")
            return True
        time.sleep(1.0)

    rospy.logwarn(
        "The Husky controllers are NOT both ( running ). Last seen:\n%s\n"
        "husky_velocity_controller stuck in `initialized` means the spawner died "
        "part-way - the robot will look fine but SILENTLY IGNORE every cmd_vel. "
        "Usual cause: a leftover roslaunch/gzserver from an earlier run. Stop it "
        "and re-run. Re-check with: rosrun controller_manager "
        "controller_manager list. Continuing anyway.",
        listing.strip() or "  <controller_manager did not answer at all>")
    return False


def bring_up_robot():
    """Full robot bring-up: control.launch -> robot_description -> delete stale
    husky -> spawn_model -> controller check. Returns the control.launch Popen so
    the caller can tear it down even if a later step failed. Faithful to
    send_mapless_goal.py."""
    proc = start_robot()
    if wait_for_robot_description(proc):
        delete_existing_robot()
        spawn_robot()
        wait_for_controllers()
    return proc


class HuskyAutoDrive(object):
    """The planner-free driver / §4 attack victim. Holds the heading captured at
    loop start and cruises at whatever ~linear_speed currently reads, re-read
    fresh every tick."""

    def __init__(self, args):
        self.args = args
        self._start_wall = None
        self._initial_yaw = None
        self._latest_odom = None

        # Command output onto the twist_mux 'external' slot. queue_size=1: always
        # the freshest command, never a backlog.
        self._pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)
        # The ONLY pose/heading source: the stock odom-frame EKF. No GPS, no
        # compass, no ground truth.
        rospy.Subscriber(ODOM_TOPIC, Odometry, self._on_odom, queue_size=1)

        # CSV: opt-in via --csv. When open, once, header, flush every row so a
        # mid-run Ctrl-C still leaves a complete file. Same discipline as the
        # attack scripts.
        if args.csv:
            self._csv_file = open(args.csv, "w", newline="")
            self._csv = csv.writer(self._csv_file)
            self._csv.writerow(
                ["elapsed_time", "polled_linear_speed", "cmd_linear_x",
                 "cmd_angular_z", "fused_yaw"])
            self._csv_file.flush()
        else:
            self._csv_file = None
            self._csv = None

    def _on_odom(self, msg):
        """Cache the latest /odometry/filtered sample -- our only heading source."""
        self._latest_odom = msg

    def run(self):
        # Set the sane cruise default so the param EXISTS before any attack can
        # target it. --linear-speed lets the operator override the default.
        rospy.set_param("~linear_speed", self.args.linear_speed)
        rospy.loginfo("Set ~linear_speed = %.2f m/s (report cruise default %.2f). "
                      "Full param name for the §4 attack: %s/linear_speed",
                      self.args.linear_speed, REPORT_CRUISE_SPEED,
                      rospy.get_name())

        # Capture the initial heading ONCE, from the odom-frame EKF.
        rospy.loginfo("Waiting for %s to capture initial heading ...", ODOM_TOPIC)
        start = rospy.wait_for_message(ODOM_TOPIC, Odometry, timeout=30.0)
        self._latest_odom = start
        self._initial_yaw = yaw_of(start)
        rospy.loginfo("Initial fused yaw = %.4f rad (%.1f deg), frame=%s -- "
                      "holding this heading.",
                      self._initial_yaw, math.degrees(self._initial_yaw),
                      start.header.frame_id)

        self._start_wall = time.time()
        rate = rospy.Rate(self.args.rate)
        next_log = self._start_wall + 1.0

        rospy.loginfo(
            "DRIVE START (planner-free, report §4 victim): %.1f Hz, holding "
            "heading, driving at ~linear_speed%s",
            self.args.rate,
            (" for %.0f s" % self.args.duration)
            if self.args.duration > 0 else " until Ctrl-C")

        while not rospy.is_shutdown():
            if self.args.duration > 0 and \
                    (time.time() - self._start_wall) >= self.args.duration:
                rospy.loginfo("Duration reached -- stopping.")
                break

            # (a) RE-READ the speed param fresh every tick, caching nothing. This
            # live re-read is what makes the §4 param attack land.
            polled_speed = rospy.get_param("~linear_speed", self.args.linear_speed)

            # (b) Current yaw from the odom-frame EKF ONLY.
            current_yaw = yaw_of(self._latest_odom)

            # (c) Proportional correction that holds the initial heading. Sign per
            # CLAUDE.md: heading_error > 0 (target to the left) -> positive
            # angular.z (CCW/left), no negation.
            heading_error = wrap_to_pi(self._initial_yaw - current_yaw)
            angular_z = self.args.k_heading * heading_error

            # (d) Publish. Do NOT clamp the sign of polled_speed: a negative param
            # must drive backward so the report's -5.0 step is visible.
            cmd = Twist()
            cmd.linear.x = polled_speed
            cmd.angular.z = angular_z
            self._pub.publish(cmd)

            now = time.time()
            if now >= next_log:
                elapsed = now - self._start_wall
                rospy.loginfo(
                    "[t=%6.1fs] polled_speed=%7.3f  cmd=(lin %7.3f, ang %6.3f)  "
                    "fused_yaw=%.4f", elapsed, polled_speed, cmd.linear.x,
                    cmd.angular.z, current_yaw)
                if self._csv is not None:
                    self._csv.writerow(
                        ["%.3f" % elapsed, "%.4f" % polled_speed,
                         "%.4f" % cmd.linear.x, "%.4f" % cmd.angular.z,
                         "%.4f" % current_yaw])
                    self._csv_file.flush()
                next_log += 1.0

            rate.sleep()

    def shutdown(self):
        """Stop the robot with a single zero Twist, then close the CSV.
        Idempotent. Unlike the attack scripts (which send NO corrective message),
        this is a legitimate driver, so stopping its own robot on exit is correct
        housekeeping, not a second injection."""
        try:
            if self._pub is not None:
                self._pub.publish(Twist())  # zero velocity -- park the robot
        except Exception as exc:  # noqa: BLE001 -- log, never hide
            rospy.logwarn("Error publishing stop Twist: %s", exc)
        try:
            if self._csv_file is not None and not self._csv_file.closed:
                self._csv_file.flush()
                self._csv_file.close()
        except Exception as exc:  # noqa: BLE001 -- log, never hide
            rospy.logwarn("Error closing CSV: %s", exc)
        if self._csv_file is not None:
            rospy.loginfo("DRIVE STOPPED. CSV saved to %s", self.args.csv)
        else:
            rospy.loginfo("DRIVE STOPPED. CSV logging was disabled.")


def parse_args():
    p = argparse.ArgumentParser(
        description="Simulation-only planner-free Husky driver (report §4 "
                    "victim): holds heading from /odometry/filtered and cruises "
                    "at the re-read-every-tick ~linear_speed param.")
    p.add_argument("--rate", type=float, default=10.0,
                   help="control-loop rate in Hz (default 10)")
    p.add_argument("--linear-speed", type=float, default=REPORT_CRUISE_SPEED,
                   help="initial ~linear_speed cruise value in m/s "
                        "(default 0.5; the param the §4 attack overwrites)")
    p.add_argument("--k-heading", type=float, default=1.5,
                   help="proportional gain on heading error, rad/s per rad "
                        "(default 1.5)")
    p.add_argument("--duration", type=float, default=0.0,
                   help="seconds to run; 0 = until Ctrl-C (default 0)")
    p.add_argument("--csv", nargs="?", const="husky_auto_drive.csv", default=None,
                   help="write telemetry CSV to husky_auto_drive.csv; off unless "
                        "this flag is given (default: off)")
    args = p.parse_args()
    if args.rate <= 0:
        p.error("--rate must be > 0")
    return args


def main():
    args = parse_args()
    # NOT anonymous: the param namespace /husky_auto_drive/linear_speed must be
    # stable so the §4 attack can target it. anonymous=True would append a random
    # suffix and break that.
    rospy.init_node("husky_auto_drive")

    robot = None
    driver = None
    try:
        robot = bring_up_robot()
        driver = HuskyAutoDrive(args)
        # Register cleanup for every exit path (Ctrl-C, duration, rospy shutdown).
        rospy.on_shutdown(driver.shutdown)
        driver.run()
        return 0
    except rospy.ROSInterruptException:
        return 0
    finally:
        # Reverse order of startup: stop the driver (park the robot, close CSV)
        # first, then tear down the robot. Each guarded so one failure still lets
        # the other run.
        try:
            if driver is not None:
                driver.shutdown()
        finally:
            stop_robot(robot)


if __name__ == "__main__":
    sys.exit(main())
