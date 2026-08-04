#!/usr/bin/env python3
"""Spawn the STOCK Husky + move_base into an ALREADY-RUNNING park world, then
IDLE (no goal) until Ctrl-C, tearing everything down on exit.

The planner is OBSTACLE-AWARE by default; set HUSKY_MAPLESS=1 (any non-empty
value) to bring up the sensorless MAPLESS victim planner for the odom-attack
demo instead.

This is the ROBOT SIDE of the operator demo: it makes the robot ready so a
REMOTE operator (operator/operate.py, a separate container) can send a goal.
It deliberately does NOT send any goal itself — that is the operator's job.

Reuses send_mapless_goal.py's bring-up verbatim (imported, that file is not
modified). Teardown order mirrors send_mapless_goal.main(): planner then robot,
each guarded so one failure cannot leak the other.
"""
import os

import rospy
from send_mapless_goal import (
    bring_up_robot, start_planner, stop_planner, stop_robot,
    PLANNER_LAUNCH_OA, PLANNER_LAUNCH_MAPLESS,
)


def main():
    rospy.init_node("spawn_robot_idle", anonymous=True)
    # HUSKY_MAPLESS non-empty -> mapless victim planner; unset/empty -> obstacle-aware.
    use_mapless = bool(os.environ.get("HUSKY_MAPLESS", ""))
    chosen_launch = PLANNER_LAUNCH_MAPLESS if use_mapless else PLANNER_LAUNCH_OA
    robot = None
    planner = None
    try:
        robot = bring_up_robot()
        planner = start_planner(chosen_launch)
        rospy.loginfo("Robot spawned + %s move_base up. IDLE — waiting for a "
                      "remote operator goal. Ctrl-C to tear down.",
                      "MAPLESS" if use_mapless else "obstacle-aware")
        rospy.spin()          # idle until SIGINT/rospy shutdown
    finally:
        try:
            stop_planner(planner)
        finally:
            stop_robot(robot)


if __name__ == "__main__":
    main()
