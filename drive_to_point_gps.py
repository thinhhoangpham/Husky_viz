#!/usr/bin/env python3
"""Drive a Clearpath Husky through a sequence of (x, y) points in the WORLD
frame using ONLY real robot sensors: GPS for position, compass for heading.

The driver walks the hardcoded WAYPOINTS list in order. It heads for one
waypoint at a time and, the moment it is within TOLERANCE of it, ADVANCES to
the next one without stopping the run; it exits only after the LAST waypoint is
reached. It makes a single pass - it does not loop back to the first waypoint.

No Gazebo ground truth, no odometry, no EKF, no TF, no move_base, no latched
transforms, no cached pose. Both sensors are ABSOLUTE and neither drifts, so
every tick re-reads them and recomputes everything from scratch; error can
therefore never accumulate. That invariant is per-tick, not per-leg: advancing
to the next waypoint changes only WHICH target is being subtracted from the
live fix, never where the fix itself comes from.

Usage:
    python3 drive_to_point_gps.py

The route is hardcoded as WAYPOINTS below.
"""

import math
import sys
import threading

import rospy
import tf.transformations
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, NavSatFix

# ---------------------------------------------------------------------------
# GPS -> WORLD metres conversion.
#
# PROVENANCE: everything below is DERIVED, not measured. The reference fix comes
# from the GPS plugin's own DECLARED parameters, and the scale factors from the
# standard WGS84 ellipsoid. Nothing here was obtained by observing the
# simulator's internal state.
#
# Source of the reference fix -- the hector_gazebo_plugins gazebo_ros_gps plugin
# declaration in:
#     natural_environments_ros_opt/husky/husky_description/urdf/gps.urdf.xacro
#         line 33:  <referenceLatitude>49.9</referenceLatitude>
#         line 34:  <referenceLongitude>8.9</referenceLongitude>
#         line 35:  <referenceHeading>0</referenceHeading>
#         line 36:  <referenceAltitude>0</referenceAltitude>
#
# IF referenceLatitude/referenceLongitude ARE CHANGED IN THAT XACRO, REF_LAT and
# REF_LON HERE MUST BE UPDATED TO MATCH. Nothing detects a mismatch at runtime.
#
# The plugin maps local metres to degrees using the WGS84 meridional and normal
# radii of curvature evaluated at the reference latitude; those radii are
# computed below so the derivation stays visible and auditable in this file.
# ---------------------------------------------------------------------------
REF_LAT = 49.9  # deg, <referenceLatitude>  in gps.urdf.xacro line 33
REF_LON = 8.9   # deg, <referenceLongitude> in gps.urdf.xacro line 34

# WGS84 defining constants.
EQUATORIAL_RADIUS = 6378137.0
FLATTENING = 1.0 / 298.257223563
E2 = 2.0 * FLATTENING - FLATTENING ** 2

_SIN2_REF_LAT = math.sin(math.radians(REF_LAT)) ** 2
# Meridional (north-south) and normal (east-west) radii of curvature at REF_LAT.
RADIUS_NORTH = EQUATORIAL_RADIUS * (1.0 - E2) / (1.0 - E2 * _SIN2_REF_LAT) ** 1.5
RADIUS_EAST = (EQUATORIAL_RADIUS / math.sqrt(1.0 - E2 * _SIN2_REF_LAT)
               * math.cos(math.radians(REF_LAT)))

DEG_LAT_PER_METRE = math.degrees(1.0 / RADIUS_NORTH)  # deg latitude  per metre NORTH
DEG_LON_PER_METRE = math.degrees(1.0 / RADIUS_EAST)   # deg longitude per metre EAST

# The route. World frame, metres, visited in this order.
#
# PROVENANCE: the 5 waypoints from the bag topic /navigation/objetive_gps,
# converted to Gazebo WORLD coords via a least-squares fit of /navsat/fix
# against /gazebo/model_states (lat->world X, lon->world -Y; residuals < 5 mm).
#
# drive-park.sh hardcodes the SAME five pairs as MARKER_WAYPOINTS (line 92) to
# place its visual markers, so markers and goals coincide by construction.
# Nothing checks that at runtime: IF YOU CHANGE ONE LIST, CHANGE THE OTHER.
WAYPOINTS = (
    (38.26, 1.25),
    (27.11, 1.10),
    (1.16, -2.40),
    (-15.95, -3.33),
    (-30.77, -3.45),
)

TOLERANCE = 0.5    # m, arrival radius
MAX_LIN = 0.8      # m/s
MAX_ANG = 1.6      # rad/s

# PER-LEG budget, in seconds, RESET at the start of every waypoint leg - NOT a
# budget for the whole route. Kept at the original 180 s even though each leg is
# now shorter than the old single-target route: the longest leg here is ~26 m,
# which at MAX_LIN = 0.4 m/s takes ~65 s of pure driving, so 180 s still leaves
# roughly a 2.5x margin for turn-in-place time and terrain. The point of this
# number is to catch a robot that is genuinely stuck (wedged on terrain, pushing
# against something, commanded but not moving), not to enforce a schedule, so a
# generous value is correct. Tightening it would only trade real failures for
# false ones. Exceeding it on ANY leg is fatal.
TIMEOUT = 180.0    # s, per waypoint leg

RATE_HZ = 10.0
TURN_IN_PLACE_THRESHOLD = 0.25  # rad (~14 deg)
STARTUP_WAIT_S = 30.0           # max wait for the first fix + first compass msg
SENSOR_STALE_S = 3.0            # a frozen sensor must stop the robot
NO_FIX_WARN_PERIOD_S = 2.0      # throttle for STATUS_NO_FIX warnings


def clamp(value, low, high):
    return max(low, min(high, value))


def normalize_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def fix_to_world(latitude, longitude):
    """Convert a geodetic fix to (world_x, world_y) in metres."""
    world_x = (latitude - REF_LAT) / DEG_LAT_PER_METRE
    # EXPLICIT NEGATION: with <referenceHeading>0</referenceHeading> the plugin's
    # local frame is aligned so that world +x is NORTH and world +y is WEST.
    # Longitude therefore DECREASES as world y increases, hence the minus sign.
    world_y = -(longitude - REF_LON) / DEG_LON_PER_METRE
    return world_x, world_y


class DriveToPointGPS(object):
    def __init__(self):
        self._lock = threading.Lock()

        # Latest GOOD fix only. A STATUS_NO_FIX message never overwrites these.
        self._fix_xy = None          # (world_x, world_y)
        self._fix_stamp = None       # rospy time (seconds) of last GOOD fix
        self._yaw = None
        self._yaw_stamp = None
        self._last_no_fix_warn = 0.0

        # /cmd_vel is the lowest-priority twist_mux input and is the correct channel.
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.fix_sub = rospy.Subscriber("/navsat/fix", NavSatFix,
                                        self._fix_cb, queue_size=1)
        # /compass/data is an ABSOLUTE world heading (accurate to ~0.01 deg) and does
        # not drift. Do NOT use /imu/data: it is mounted rotated 90 deg and is wrong here.
        self.imu_sub = rospy.Subscriber("/compass/data", Imu,
                                        self._compass_cb, queue_size=1)

    # -- callbacks ----------------------------------------------------------
    def _fix_cb(self, msg):
        # status.status >= 0 means STATUS_FIX or better; < 0 is STATUS_NO_FIX.
        if msg.status.status < 0:
            now = rospy.get_time()
            with self._lock:
                warn = now - self._last_no_fix_warn >= NO_FIX_WARN_PERIOD_S
                if warn:
                    self._last_no_fix_warn = now
            if warn:
                rospy.logwarn("/navsat/fix reports STATUS_NO_FIX (%d); "
                              "keeping previous good fix", msg.status.status)
            return
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            rospy.logwarn_throttle(NO_FIX_WARN_PERIOD_S,
                                   "/navsat/fix contained NaN lat/lon; ignoring message")
            return
        xy = fix_to_world(msg.latitude, msg.longitude)
        with self._lock:
            self._fix_xy = xy
            self._fix_stamp = rospy.get_time()

    def _compass_cb(self, msg):
        q = msg.orientation
        yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        with self._lock:
            self._yaw = yaw
            self._yaw_stamp = rospy.get_time()

    def _snapshot(self):
        """Return (x, y, yaw, fix_stamp, yaw_stamp) of the latest sensor values."""
        with self._lock:
            xy = self._fix_xy
            return (xy[0] if xy else None,
                    xy[1] if xy else None,
                    self._yaw, self._fix_stamp, self._yaw_stamp)

    # -- lifecycle ----------------------------------------------------------
    def stop(self):
        """Publish a zero Twist. Safe to call repeatedly, including at shutdown."""
        try:
            self.cmd_pub.publish(Twist())
        except Exception as exc:  # publisher may already be torn down at shutdown
            rospy.logwarn("could not publish stop command: %s", exc)

    def wait_for_sensors(self):
        """Block until BOTH a valid fix and a compass message have arrived."""
        # Let the publisher register with subscribers, otherwise the first few
        # Twist messages are silently dropped.
        rospy.sleep(0.5)

        deadline = rospy.get_time() + STARTUP_WAIT_S
        rate = rospy.Rate(RATE_HZ)
        while not rospy.is_shutdown() and rospy.get_time() < deadline:
            cur_x, cur_y, cur_yaw, _, _ = self._snapshot()
            if cur_x is not None and cur_yaw is not None:
                rospy.loginfo(
                    "sensors ready | starting world position from GPS = (%.2f, %.2f) "
                    "yaw = %.1f deg  <-- sanity-check this before the robot moves",
                    cur_x, cur_y, math.degrees(cur_yaw))
                return True
            rate.sleep()

        cur_x, _, cur_yaw, _, _ = self._snapshot()
        missing = []
        if cur_x is None:
            missing.append("/navsat/fix (no valid NavSatFix with status >= 0)")
        if cur_yaw is None:
            missing.append("/compass/data (no sensor_msgs/Imu message)")
        rospy.logerr("startup gate failed after %.0f s; missing: %s",
                     STARTUP_WAIT_S, "; ".join(missing) or "unknown")
        return False

    def run(self):
        """Return a process exit code."""
        if not self.wait_for_sensors():
            self.stop()
            return 1

        if not WAYPOINTS:
            rospy.logerr("WAYPOINTS is empty; nothing to drive to")
            self.stop()
            return 1

        rate = rospy.Rate(RATE_HZ)
        last_log = 0.0

        total = len(WAYPOINTS)
        # The ONLY state carried across ticks: which leg we are on, and when that
        # leg began. No pose and no distance is ever cached - see the module
        # docstring; both are recomputed from the live snapshot every tick.
        index = 0
        target_x, target_y = WAYPOINTS[index]
        leg_start = rospy.get_time()
        rospy.loginfo("route: %d waypoints | heading for waypoint 1/%d = (%.2f, %.2f)",
                      total, total, target_x, target_y)

        while not rospy.is_shutdown():
            now = rospy.get_time()
            if now - leg_start > TIMEOUT:
                self.stop()
                cur_x, cur_y, _, _, _ = self._snapshot()
                remaining = (math.hypot(target_x - cur_x, target_y - cur_y)
                             if cur_x is not None else float("nan"))
                rospy.logerr("TIMEOUT after %.1f s on the leg to waypoint %d/%d: "
                             "target=(%.2f, %.2f) still %.2f m away",
                             TIMEOUT, index + 1, total, target_x, target_y, remaining)
                return 2

            # Recompute EVERYTHING from the latest sensor values. Nothing cached.
            cur_x, cur_y, cur_yaw, fix_stamp, yaw_stamp = self._snapshot()
            if cur_x is None or cur_yaw is None:
                self.stop()
                rospy.logerr("sensor data disappeared mid-run")
                return 3

            fix_age = now - fix_stamp
            yaw_age = now - yaw_stamp
            if fix_age > SENSOR_STALE_S or yaw_age > SENSOR_STALE_S:
                self.stop()
                rospy.logerr("STALE SENSOR: /navsat/fix age=%.1f s, /compass/data age=%.1f s "
                             "(limit %.1f s) - stopping rather than driving on stale data",
                             fix_age, yaw_age, SENSOR_STALE_S)
                return 3

            dx = target_x - cur_x
            dy = target_y - cur_y
            dist = math.hypot(dx, dy)
            target_bearing = math.atan2(dy, dx)              # absolute world bearing
            heading_error = normalize_angle(target_bearing - cur_yaw)

            if dist <= TOLERANCE:
                # Stop FIRST, on every arrival - including the intermediate ones.
                # The next leg usually starts with a turn in place, and this also
                # means an arrival can never leave a velocity command standing if
                # anything below (or a shutdown between ticks) cuts the run short.
                self.stop()
                rospy.loginfo("ARRIVED at waypoint %d/%d: world=(%.2f, %.2f) "
                              "target=(%.2f, %.2f) dist=%.2f m",
                              index + 1, total, cur_x, cur_y, target_x, target_y, dist)
                index += 1
                if index >= total:
                    rospy.loginfo("route complete: all %d waypoints visited", total)
                    return 0
                target_x, target_y = WAYPOINTS[index]
                # Fresh per-leg timeout budget for the leg that starts now.
                leg_start = rospy.get_time()
                rospy.loginfo("heading for waypoint %d/%d = (%.2f, %.2f)",
                              index + 1, total, target_x, target_y)
                rate.sleep()
                continue

            # SIGN CONVENTION: in ROS/Gazebo, yaw increases counter-clockwise and a
            # POSITIVE angular.z rotates counter-clockwise (left) as seen from above.
            # heading_error > 0 therefore means the target is to the robot's LEFT and
            # angular.z must be POSITIVE. No negation anywhere below. Do not add one.
            cmd = Twist()
            if abs(heading_error) > TURN_IN_PLACE_THRESHOLD:
                # Turn in place first: arcing off-course is easy on a skid-steer.
                cmd.linear.x = 0.0
                cmd.angular.z = clamp(2.0 * heading_error, -MAX_ANG, MAX_ANG)
            else:
                cmd.linear.x = min(MAX_LIN, 0.6 * dist)  # ease off near the goal
                cmd.angular.z = clamp(1.5 * heading_error, -MAX_ANG, MAX_ANG)
            self.cmd_pub.publish(cmd)

            if now - last_log >= 1.0:
                last_log = now
                rospy.loginfo(
                    "wp %d/%d | world=(%.2f, %.2f) yaw=%.1f deg | target=(%.2f, %.2f) "
                    "dist=%.2f m heading_err=%.1f deg | v=%.2f w=%.2f",
                    index + 1, total, cur_x, cur_y, math.degrees(cur_yaw),
                    target_x, target_y, dist,
                    math.degrees(heading_error), cmd.linear.x, cmd.angular.z)

            rate.sleep()

        self.stop()
        rospy.logwarn("rospy shutdown while driving to waypoint %d/%d (%.2f, %.2f)",
                      index + 1, total, target_x, target_y)
        return 4


def main():
    rospy.init_node("drive_to_point_gps", anonymous=True)
    driver = DriveToPointGPS()
    # Belt and braces: the robot must never be left with a non-zero velocity command.
    rospy.on_shutdown(driver.stop)
    try:
        return driver.run()
    except rospy.ROSInterruptException:
        rospy.logwarn("interrupted")
        return 130
    except KeyboardInterrupt:
        rospy.logwarn("SIGINT received")
        return 130
    finally:
        driver.stop()


if __name__ == "__main__":
    sys.exit(main())
