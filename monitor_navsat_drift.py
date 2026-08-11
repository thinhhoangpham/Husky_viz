#!/usr/bin/env python3
"""GPS slow-drift DEFENDER / ANALYST monitor.

  *** SIMULATION-ONLY SECURITY DEMONSTRATION. No real robot is involved. ***

THIS IS NOT AN ATTACKER. It is the legitimate defender/analyst observing the
EFFECT of a GPS slow-drift spoof (attack_navsat.py, which runs container-side as
a blind Tier-2 injector). This monitor runs ON THE HOST with the host ROS env,
where reading the robot's own internal robot_localization topics is entirely
legitimate -- the defender owns the robot and its estimator.

It subscribes to two internal MAP-frame estimates and logs them to a CSV so the
analyst can see whether the fused position walks away from the anchor as the
attack progresses:

    /odometry/filtered_map (nav_msgs/Odometry) -- the MAP-frame EKF output that
        move_base actually plans against. THE load-bearing signal: does it walk
        away as the spoof injects a growing offset?
    /odometry/abs_fix (nav_msgs/Odometry) -- navsat_transform's output. This
        distinguishes the two failure modes: if it TRACKS the injected offset,
        the lie propagated (attack works); if it COLLAPSES to (0,0), the
        flood/rejection failure mode is occurring (attack rejected).

The injected offset itself is NOT observable by the defender (the attacker does
not announce it), so it is not logged; we record a wall-clock elapsed time
instead. Correlate this CSV's elapsed axis with the attacker's --duration.

NO GROUND TRUTH (hard project rule; see CLAUDE.md). This monitor NEVER reads
Gazebo ground truth (no /gazebo/model_states, no gazebo_msgs). It observes only
the robot's own estimator outputs.

CSV columns:

    elapsed_time, fused_x, fused_y, gps_anchor_x, gps_anchor_y

USAGE (host-side, alongside the container-side attack)
------------------------------------------------------
    cd ~/Documents/Husky_viz
    export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311
    python3 monitor_navsat_drift.py --duration 40 --csv monitor_navsat_drift_run.csv
"""

import argparse
import csv
import threading
import time

import rospy
from nav_msgs.msg import Odometry


class NavSatDriftMonitor(object):
    def __init__(self, args):
        self.args = args
        self._lock = threading.Lock()
        self._fused_xy = None       # (x, y) from /odometry/filtered_map, map m
        self._gps_anchor_xy = None  # (x, y) from /odometry/abs_fix, map m
        self._stop = threading.Event()
        self._start_wall = None

        # Read-only defender subscriptions to the robot's own estimator outputs.
        rospy.Subscriber("/odometry/filtered_map", Odometry,
                         self._on_fused, queue_size=1)
        rospy.Subscriber("/odometry/abs_fix", Odometry,
                         self._on_gps_anchor, queue_size=1)

        # CSV: open once, write header, flush every row so a mid-run Ctrl-C
        # still leaves a complete, readable file.
        self._csv_file = open(args.csv, "w", newline="")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(
            ["elapsed_time", "fused_x", "fused_y",
             "gps_anchor_x", "gps_anchor_y"])
        self._csv_file.flush()

    # --- telemetry callbacks -------------------------------------------------
    def _on_fused(self, msg):
        with self._lock:
            self._fused_xy = (msg.pose.pose.position.x,
                              msg.pose.pose.position.y)

    def _on_gps_anchor(self, msg):
        with self._lock:
            self._gps_anchor_xy = (msg.pose.pose.position.x,
                                   msg.pose.pose.position.y)

    # --- telemetry row -------------------------------------------------------
    def _log_row(self):
        with self._lock:
            fused = self._fused_xy
            anchor = self._gps_anchor_xy
        elapsed = time.time() - self._start_wall

        fused_x = fused[0] if fused else float("nan")
        fused_y = fused[1] if fused else float("nan")
        anchor_x = anchor[0] if anchor else float("nan")
        anchor_y = anchor[1] if anchor else float("nan")

        rospy.loginfo(
            "[t=%6.1fs] fused=(%8.3f,%8.3f) gps_anchor=(%8.3f,%8.3f)",
            elapsed, fused_x, fused_y, anchor_x, anchor_y)
        self._csv.writerow(
            ["%.3f" % elapsed,
             "%.4f" % fused_x, "%.4f" % fused_y,
             "%.4f" % anchor_x, "%.4f" % anchor_y])
        self._csv_file.flush()

    # --- main loop -----------------------------------------------------------
    def run(self):
        self._start_wall = time.time()
        rate = rospy.Rate(1.0)  # ~1 Hz telemetry, as the old attack logged
        rospy.loginfo("MONITOR START: logging fused vs anchor to %s%s",
                      self.args.csv,
                      (" for %.0f s" % self.args.duration)
                      if self.args.duration > 0 else " until Ctrl-C")

        while not self._stop.is_set() and not rospy.is_shutdown():
            if self.args.duration > 0 and \
                    (time.time() - self._start_wall) >= self.args.duration:
                rospy.loginfo("Duration reached -- stopping.")
                break
            self._log_row()
            rate.sleep()

    def shutdown(self):
        """Stop logging and close the CSV. Idempotent."""
        self._stop.set()
        try:
            if not self._csv_file.closed:
                self._csv_file.flush()
                self._csv_file.close()
        except Exception as exc:  # noqa: BLE001 -- log, never hide
            rospy.logwarn("Error closing CSV: %s", exc)
        rospy.loginfo("MONITOR STOPPED. CSV saved to %s", self.args.csv)


def parse_args():
    p = argparse.ArgumentParser(
        description="Defender/analyst monitor: log the MAP-frame fused pose vs "
                    "the navsat_transform anchor to observe a GPS slow-drift "
                    "spoof. Host-side, read-only, NOT an attacker.")
    p.add_argument("--duration", type=float, default=0.0,
                   help="seconds to run; 0 = until Ctrl-C (default 0)")
    p.add_argument("--csv", default="monitor_navsat_drift.csv",
                   help="telemetry output CSV path "
                        "(default monitor_navsat_drift.csv)")
    return p.parse_args()


def main():
    args = parse_args()
    rospy.init_node("monitor_navsat_drift", anonymous=True)

    monitor = NavSatDriftMonitor(args)
    rospy.on_shutdown(monitor.shutdown)

    try:
        monitor.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        monitor.shutdown()


if __name__ == "__main__":
    main()
