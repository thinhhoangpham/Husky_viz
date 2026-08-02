#!/usr/bin/env python3
"""Spawn the STOCK Husky + mapless move_base into an ALREADY-RUNNING park world,
then IDLE (no goal) until Ctrl-C, tearing everything down on exit.

This is the ROBOT SIDE of the operator demo: it makes the robot ready so a
REMOTE operator (operator/operator.py, a separate container) can send a goal.
It deliberately does NOT send any goal itself — that is the operator's job.

Reuses send_mapless_goal.py's bring-up verbatim (imported, that file is not
modified). Teardown order mirrors send_mapless_goal.main(): planner then robot,
each guarded so one failure cannot leak the other.
"""
import signal
import rospy
from send_mapless_goal import (
    bring_up_robot, start_planner, stop_planner, stop_robot,
)


def main():
    rospy.init_node("spawn_robot_idle", anonymous=True)
    robot = None
    planner = None
    try:
        robot = bring_up_robot()
        planner = start_planner()
        rospy.loginfo("Robot spawned + mapless move_base up. IDLE — waiting for a "
                      "remote operator goal. Ctrl-C to tear down.")
        rospy.spin()          # idle until SIGINT/rospy shutdown
    finally:
        try:
            stop_planner(planner)
        finally:
            stop_robot(robot)


if __name__ == "__main__":
    main()
