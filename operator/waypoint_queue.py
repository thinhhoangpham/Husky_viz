"""Pure, ROS-free queue of coordinate waypoints for the operator's `route` command.

Waypoints are bare map-frame (x, y) points -- the MISSION layer. The robot drives
them in order and advances when its own position estimate reaches the current
one. The landmark localizer is a separate layer underneath (it keeps that
position estimate honest under GPS denial); the two never exchange information,
so nothing here knows about landmarks, regions or perception.

This class holds the advance policy and nothing else: no rospy, no I/O. It is
unit-tested in isolation. operate.py does the ROS wiring (send goals, test the
robot's pose against the current waypoint).
"""


class WaypointQueue(object):
    def __init__(self, points):
        """points: ordered list of (x, y) map-frame waypoints to visit."""
        self._points = list(points)
        self._index = 0

    def current(self):
        """The (x, y) of the active (not-yet-reached) waypoint, or None when the
        route is finished."""
        if self._index < len(self._points):
            return self._points[self._index]
        return None

    def on_arrival(self, reached):
        """Report an arrival check at the current waypoint. Advance to the next
        waypoint only when the robot's own position estimate reached it
        (reached=True). A reached=False report is a no-op. Returns the new
        current()."""
        if reached and not self.done():
            self._index += 1
        return self.current()

    def done(self):
        """True once every waypoint has been reached."""
        return self._index >= len(self._points)
