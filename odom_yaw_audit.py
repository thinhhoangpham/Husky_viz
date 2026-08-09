#!/usr/bin/env python3
"""Odom-EKF rotation-ratio audit (sensor-based, NO ground truth).

Publishes a fixed in-place spin on /cmd_vel for a set duration, and records
the yaw reported by the odom-frame EKF (/odometry/filtered_odom) against
/compass/data (absolute world yaw, the truth reference established as tracking
1:1). Prints the net yaw change for each and the ratio odom/compass.

A correct EKF gives ratio ~1.0. The pre-compass-fix config (fusing the
90deg-rotated /imu/data) under-reports and gives ~0.5.

Usage:
  python3 odom_yaw_audit.py --label pre  --ang 0.5 --duration 8 --csv audit_pre.csv
"""
import argparse, csv, math, time
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion


def yaw_of(q):
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def unwrap(prev, cur):
    """Accumulate continuous yaw across +/-pi wraps."""
    d = cur - prev
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


class Audit:
    def __init__(self, a):
        self.a = a
        self.odom_yaw = None
        self.comp_yaw = None
        self.odom_accum = 0.0
        self.comp_accum = 0.0
        self._odom_prev = None
        self._comp_prev = None
        self.rows = []
        rospy.Subscriber("/odometry/filtered_odom", Odometry, self._on_odom, queue_size=50)
        rospy.Subscriber("/compass/data", Imu, self._on_comp, queue_size=50)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def _on_odom(self, m):
        y = yaw_of(m.pose.pose.orientation)
        self.odom_yaw = y
        if self._odom_prev is not None:
            self.odom_accum += unwrap(self._odom_prev, y)
        self._odom_prev = y

    def _on_comp(self, m):
        y = yaw_of(m.orientation)
        self.comp_yaw = y
        if self._comp_prev is not None:
            self.comp_accum += unwrap(self._comp_prev, y)
        self._comp_prev = y

    def run(self):
        a = self.a
        # wait for both streams
        t0 = time.time()
        while (self.odom_yaw is None or self.comp_yaw is None) and time.time() - t0 < 15:
            time.sleep(0.1)
        if self.odom_yaw is None or self.comp_yaw is None:
            rospy.logerr("no odom/compass data (odom=%s compass=%s)", self.odom_yaw, self.comp_yaw)
            return
        # reset accumulators after both are live
        self.odom_accum = 0.0
        self.comp_accum = 0.0
        rospy.loginfo("[%s] spin start: ang=%.2f rad/s for %.1fs", a.label, a.ang, a.duration)
        tw = Twist()
        tw.angular.z = a.ang
        rate = rospy.Rate(20)
        start = time.time()
        while time.time() - start < a.duration and not rospy.is_shutdown():
            self.pub.publish(tw)
            el = time.time() - start
            self.rows.append((el, self.odom_accum, self.comp_accum, self.odom_yaw, self.comp_yaw))
            rate.sleep()
        # stop
        self.pub.publish(Twist())
        time.sleep(0.5)
        self.pub.publish(Twist())

        od = math.degrees(self.odom_accum)
        cp = math.degrees(self.comp_accum)
        ratio = (self.odom_accum / self.comp_accum) if abs(self.comp_accum) > 1e-6 else float("nan")
        print("=" * 60)
        print(f"[{a.label}] net odom-EKF yaw : {od:8.2f} deg")
        print(f"[{a.label}] net compass  yaw : {cp:8.2f} deg  (truth reference)")
        print(f"[{a.label}] ratio odom/compass: {ratio:6.3f}   (1.0 = correct, ~0.5 = halved)")
        print("=" * 60)
        if a.csv:
            with open(a.csv, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["elapsed", "odom_accum_rad", "compass_accum_rad", "odom_yaw", "compass_yaw"])
                w.writerows(self.rows)
            print(f"CSV -> {a.csv}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--label", default="run")
    p.add_argument("--ang", type=float, default=0.5, help="angular.z rad/s")
    p.add_argument("--duration", type=float, default=8.0)
    p.add_argument("--csv", default="")
    a = p.parse_args()
    rospy.init_node("odom_yaw_audit", anonymous=True, disable_signals=True)
    Audit(a).run()


if __name__ == "__main__":
    main()
