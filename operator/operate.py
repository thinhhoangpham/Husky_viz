#!/usr/bin/env python3
"""Remote ROS operator: send ONE move_base goal to a target (x, y) in the odom
frame, watch telemetry, and write a normal-baseline CSV.

Runs in the operator container (own IP, docker0). Assumes the robot is ALREADY
spawned and move_base is up (host-side spawn-robot-idle.sh). Does NO robot
bring-up — it is a pure remote peer.

CSV columns are the union of every BASELINE signal the repo's attack CSVs log,
on a shared elapsed_time clock, so each attack's plot can overlay its series.
Attack-injected columns (fake_yaw_deg, value_written, d_*) have no baseline and
stay in the attack CSVs.
"""
import argparse
import csv
import math
import os
import sys
import threading

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# goal_marker.py lives at the repo root; when run by path, sys.path[0] is this
# script's dir (operator/), so add the repo root so the import resolves.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from goal_marker import place_goal_marker

ODOM_TOPIC = "/odometry/filtered"
PLANNER_CMD_TOPIC = "/cmd_vel"                              # move_base output
CTRL_CMD_TOPIC = "/husky_velocity_controller/cmd_vel"      # controller input
STATUS_TEXT = {
    GoalStatus.PENDING: "PENDING", GoalStatus.ACTIVE: "ACTIVE",
    GoalStatus.SUCCEEDED: "SUCCEEDED", GoalStatus.ABORTED: "ABORTED",
    GoalStatus.REJECTED: "REJECTED", GoalStatus.PREEMPTED: "PREEMPTED",
    GoalStatus.LOST: "LOST",
}
CSV_HEADER = ["elapsed_time", "fused_x", "fused_y", "fused_yaw", "fused_yaw_deg",
              "planner_linear_x", "planner_angular_z",
              "ctrl_linear_x", "ctrl_angular_z", "ref_x", "ref_y"]


def yaw_of(odom):
    q = odom.pose.pose.orientation
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


class Operator(object):
    def __init__(self, args):
        self.args = args
        self._lock = threading.Lock()
        self._odom = None          # latest Odometry
        self._planner_cmd = (0.0, 0.0)  # (linear.x, angular.z) from /cmd_vel
        self._ctrl_cmd = (0.0, 0.0)     # from controller cmd_vel
        rospy.Subscriber(ODOM_TOPIC, Odometry, self._on_odom, queue_size=1)
        rospy.Subscriber(PLANNER_CMD_TOPIC, Twist, self._on_planner, queue_size=1)
        rospy.Subscriber(CTRL_CMD_TOPIC, Twist, self._on_ctrl, queue_size=1)
        self._csv_file = open(args.csv, "w", newline="")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(CSV_HEADER)
        self._csv_file.flush()

    def _on_odom(self, msg):
        with self._lock:
            self._odom = msg

    def _on_planner(self, msg):
        with self._lock:
            self._planner_cmd = (msg.linear.x, msg.angular.z)

    def _on_ctrl(self, msg):
        with self._lock:
            self._ctrl_cmd = (msg.linear.x, msg.angular.z)

    def _write_row(self, elapsed):
        with self._lock:
            odom = self._odom
            plx, paz = self._planner_cmd
            clx, caz = self._ctrl_cmd
        if odom is None:
            return None
        px = odom.pose.pose.position.x
        py = odom.pose.pose.position.y
        yaw = yaw_of(odom)
        self._csv.writerow(
            ["%.3f" % elapsed, "%.4f" % px, "%.4f" % py,
             "%.4f" % yaw, "%.4f" % math.degrees(yaw),
             "%.4f" % plx, "%.4f" % paz, "%.4f" % clx, "%.4f" % caz,
             "%.4f" % self.args.goal_x, "%.4f" % self.args.goal_y])
        self._csv_file.flush()
        return (px, py, yaw)

    def run(self):
        client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server ...")
        if not client.wait_for_server(rospy.Duration(60.0)):
            rospy.logerr("move_base action server not available.")
            return 1

        # Heading toward the goal from the current pose, so the robot faces its
        # target on arrival (same convention as send_mapless_goal's goal yaw).
        start = rospy.wait_for_message(ODOM_TOPIC, Odometry, timeout=30.0)
        sp = start.pose.pose.position
        gyaw = math.atan2(self.args.goal_y - sp.y, self.args.goal_x - sp.x)
        gq = quaternion_from_euler(0.0, 0.0, gyaw)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.args.goal_x
        goal.target_pose.pose.position.y = self.args.goal_y
        goal.target_pose.pose.orientation.x = gq[0]
        goal.target_pose.pose.orientation.y = gq[1]
        goal.target_pose.pose.orientation.z = gq[2]
        goal.target_pose.pose.orientation.w = gq[3]

        rospy.loginfo("Sending goal (frame=odom): x=%.3f y=%.3f",
                      self.args.goal_x, self.args.goal_y)
        # Visual: GREEN disc on the ground at the real goal (best-effort).
        place_goal_marker("goal_marker_real", self.args.goal_x,
                          self.args.goal_y, "0 1 0")
        start_t = rospy.Time.now()
        client.send_goal(goal)

        rate = rospy.Rate(1.0)
        deadline = start_t + rospy.Duration(self.args.timeout)
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_t).to_sec()
            pose = self._write_row(elapsed)
            state = client.get_state()
            if pose is not None:
                dist = math.hypot(self.args.goal_x - pose[0],
                                  self.args.goal_y - pose[1])
                rospy.loginfo("state=%s pos=(%.2f, %.2f) dist_to_goal=%.2f m",
                              STATUS_TEXT.get(state, state), pose[0], pose[1], dist)
            else:
                rospy.loginfo("state=%s (no odom yet)", STATUS_TEXT.get(state, state))
            if state in (GoalStatus.SUCCEEDED, GoalStatus.ABORTED,
                         GoalStatus.REJECTED, GoalStatus.PREEMPTED, GoalStatus.LOST):
                rospy.loginfo("Final move_base state: %s", STATUS_TEXT.get(state, state))
                return 0 if state == GoalStatus.SUCCEEDED else 2
            if rospy.Time.now() > deadline:
                rospy.logwarn("Timed out after %ss; last state=%s",
                              self.args.timeout, STATUS_TEXT.get(state, state))
                return 3
            rate.sleep()
        return 0

    def shutdown(self):
        if self._csv_file and not self._csv_file.closed:
            self._csv_file.flush()
            self._csv_file.close()
            rospy.loginfo("CSV saved to %s", self.args.csv)


def main():
    p = argparse.ArgumentParser(description="Remote operator: send one move_base goal.")
    p.add_argument("--goal-x", type=float, default=10.0, dest="goal_x",
                   help="target x in the odom frame (m), default 10.0")
    p.add_argument("--goal-y", type=float, default=0.0, dest="goal_y",
                   help="target y in the odom frame (m), default 0.0")
    p.add_argument("--csv", default="operator_run.csv",
                   help="baseline CSV output path (default operator_run.csv)")
    p.add_argument("--timeout", type=float, default=180.0,
                   help="give up after this many seconds (default 180)")
    args = p.parse_args()

    rospy.init_node("operator", anonymous=True)
    op = Operator(args)
    try:
        return op.run()
    finally:
        op.shutdown()


if __name__ == "__main__":
    sys.exit(main())
