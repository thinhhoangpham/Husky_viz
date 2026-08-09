#!/usr/bin/env python3
"""Record the GPS localization chain to a single CSV, one shared time axis.

  *** SIMULATION-ONLY. Reads three ROS topics; no ground truth (CLAUDE.md). ***

Purpose: empirically show how a GPS spoof propagates through the map-frame
localization chain, by logging all three stages side by side against one clock:

    /navsat/fix (sensor_msgs/NavSatFix)      RAW GPS -- the topic the attacker
        |                                    spoofs (attack_navsat_drift.py).
        v
    navsat_transform_node
        |
        v
    /odometry/gps (nav_msgs/Odometry)        map-frame position derived FROM the
        |                                    GPS fix.
        v
    map EKF (ekf_localization_map)
        |
        v
    /odometry/filtered_map (nav_msgs/Odometry)   FUSED map estimate -- what the
                                                 operator sees.

DESIGN -- latest-value sampling, NOT message-time sync
------------------------------------------------------
Each topic's newest message is cached in an instance variable under a lock
(rospy callbacks run in separate threads). A fixed-rate loop (default 5 Hz)
writes one row per tick from those cached values, so all three series share one
elapsed-time axis. This is deliberate: we want aligned rows on a shared clock,
not exact per-message time synchronization. A topic with no message yet writes
`nan` for its fields (matches operator/gcs_csv.py's nan convention).

Time column is seconds since the recorder started (first tick ~= 0). lat/lon are
~49.9 / ~8.9 and the spoof moves them in tiny increments, so they are written
with %.8f; x/y metres and elapsed_time with %.4f; navsat_status as an int.
"""

import argparse
import csv
import threading
import time

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix


class GpsChainRecorder(object):
    def __init__(self, args):
        self.args = args
        self._lock = threading.Lock()
        self._navsat = None          # (lat, lon, status) from /navsat/fix
        self._gps_odom = None        # (x, y) from /odometry/gps
        self._filtered_map = None    # (x, y) from /odometry/filtered_map
        self._stop = threading.Event()
        self._start_wall = None
        self._rows_written = 0

        # Read-only subscribers; queue_size=1 -> always the freshest value.
        rospy.Subscriber("/navsat/fix", NavSatFix,
                         self._on_navsat, queue_size=1)
        rospy.Subscriber("/odometry/gps", Odometry,
                         self._on_gps_odom, queue_size=1)
        rospy.Subscriber("/odometry/filtered_map", Odometry,
                         self._on_filtered_map, queue_size=1)

        # CSV: open once, header, flush every row so a mid-run kill leaves a
        # complete, readable file.
        self._csv_file = open(args.csv, "w", newline="")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(
            ["elapsed_time", "navsat_lat", "navsat_lon", "navsat_status",
             "gps_odom_x", "gps_odom_y", "filtered_map_x", "filtered_map_y"])
        self._csv_file.flush()

    # --- callbacks -----------------------------------------------------------
    def _on_navsat(self, msg):
        with self._lock:
            self._navsat = (msg.latitude, msg.longitude, msg.status.status)

    def _on_gps_odom(self, msg):
        with self._lock:
            self._gps_odom = (msg.pose.pose.position.x,
                              msg.pose.pose.position.y)

    def _on_filtered_map(self, msg):
        with self._lock:
            self._filtered_map = (msg.pose.pose.position.x,
                                  msg.pose.pose.position.y)

    # --- one CSV row ---------------------------------------------------------
    def _write_row(self):
        with self._lock:
            navsat = self._navsat
            gps_odom = self._gps_odom
            filtered = self._filtered_map
        elapsed = time.time() - self._start_wall

        if navsat is not None:
            lat = "%.8f" % navsat[0]
            lon = "%.8f" % navsat[1]
            status = "%d" % navsat[2]
        else:
            lat = lon = status = "nan"

        gps_x = "%.4f" % gps_odom[0] if gps_odom is not None else "nan"
        gps_y = "%.4f" % gps_odom[1] if gps_odom is not None else "nan"
        fmap_x = "%.4f" % filtered[0] if filtered is not None else "nan"
        fmap_y = "%.4f" % filtered[1] if filtered is not None else "nan"

        self._csv.writerow(
            ["%.4f" % elapsed, lat, lon, status,
             gps_x, gps_y, fmap_x, fmap_y])
        self._csv_file.flush()
        self._rows_written += 1

    # --- main loop -----------------------------------------------------------
    def run(self):
        self._start_wall = time.time()
        rate = rospy.Rate(self.args.rate)

        rospy.loginfo(
            "recording /navsat/fix, /odometry/gps, /odometry/filtered_map "
            "at %.1f Hz -> %s%s",
            self.args.rate, self.args.csv,
            (" for %.0f s" % self.args.duration)
            if self.args.duration > 0 else " until Ctrl-C")

        while not self._stop.is_set() and not rospy.is_shutdown():
            if self.args.duration > 0 and \
                    (time.time() - self._start_wall) >= self.args.duration:
                rospy.loginfo("Duration reached -- stopping.")
                break
            self._write_row()
            rate.sleep()

    def shutdown(self):
        """Flush and close the CSV. Idempotent."""
        self._stop.set()
        try:
            if not self._csv_file.closed:
                self._csv_file.flush()
                self._csv_file.close()
        except Exception as exc:  # noqa: BLE001 -- log, never hide
            rospy.logwarn("Error closing CSV: %s", exc)
        rospy.loginfo("RECORDING STOPPED. %d rows saved to %s",
                      self._rows_written, self.args.csv)


def parse_args():
    p = argparse.ArgumentParser(
        description="Record the GPS localization chain (/navsat/fix, "
                    "/odometry/gps, /odometry/filtered_map) to one CSV on a "
                    "shared time axis, via latest-value sampling.")
    p.add_argument("--duration", type=float, default=0.0,
                   help="seconds to record; 0 = until Ctrl-C (default 0)")
    p.add_argument("--rate", type=float, default=5.0,
                   help="sampling rate in Hz (default 5.0)")
    p.add_argument("--csv", default="gps_chain_record.csv",
                   help="output CSV path (default gps_chain_record.csv)")
    args = p.parse_args()
    if args.rate <= 0:
        p.error("--rate must be > 0")
    return args


def main():
    args = parse_args()
    # anonymous=True: reruns / concurrent recorders must not collide on name.
    rospy.init_node("record_gps_chain", anonymous=True)

    recorder = GpsChainRecorder(args)
    rospy.on_shutdown(recorder.shutdown)

    try:
        recorder.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        # on_shutdown may not fire on a clean duration exit -- ensure cleanup.
        recorder.shutdown()


if __name__ == "__main__":
    main()
