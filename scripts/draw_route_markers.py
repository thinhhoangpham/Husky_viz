#!/usr/bin/env python3
"""Spawn a flat Gazebo disc at EVERY route waypoint, all at once.

goal_marker.place_goal_marker only ever shows the ONE active move_base goal, so
during a multi-waypoint run you cannot see the rest of the route. This drops a
disc on each waypoint up front, so the whole planned route is visible in Gazebo
from the start of the run.

Discs are flat, static and have NO <collision> -- the same construction
goal_marker uses -- so the lidar's height-gated obstacle layer never scans them
and they cannot affect navigation.

    python3 scripts/draw_route_markers.py                  # default 3-WP route
    python3 scripts/draw_route_markers.py 27.11 1.10 1.16 -2.40
    python3 scripts/draw_route_markers.py --clear          # remove them

Colour goes green -> amber -> blue along the route so the order is readable.
"""
import sys

import rospy

from goal_marker import place_goal_marker

# The drift-experiment route (WP0 = spawn, not marked).
DEFAULT_WPS = [(27.11, 1.10), (1.16, -2.40), (-15.95, -3.33)]

# Distinct per-waypoint colours, in route order.
COLOURS = ["0.1 0.85 0.2",    # WP1 green
           "1 0.7 0.05",      # WP2 amber
           "0.15 0.5 1"]      # WP3 blue

NAME = "route_wp_%d"


def main():
    args = [a for a in sys.argv[1:] if a != "--clear"]
    clear = "--clear" in sys.argv[1:]
    rospy.init_node("draw_route_markers", anonymous=True)

    if clear:
        from gazebo_msgs.srv import DeleteModel
        rospy.wait_for_service("/gazebo/delete_model", timeout=10)
        d = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        for i in range(len(COLOURS) + 6):
            try:
                d(NAME % (i + 1))
            except Exception:
                pass
        rospy.loginfo("[route] markers cleared")
        return

    if args:
        vals = [float(v) for v in args]
        wps = [(vals[i], vals[i + 1]) for i in range(0, len(vals) - 1, 2)]
    else:
        wps = DEFAULT_WPS

    for i, (x, y) in enumerate(wps):
        rgb = COLOURS[i % len(COLOURS)]
        place_goal_marker(NAME % (i + 1), x, y, rgb, frame="map")
        rospy.loginfo("[route] WP%d disc at (%.2f, %.2f) rgb=%s", i + 1, x, y, rgb)
    rospy.loginfo("[route] %d waypoint markers placed", len(wps))


if __name__ == "__main__":
    main()
