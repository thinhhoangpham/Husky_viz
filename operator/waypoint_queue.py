"""Pure, ROS-free queue of named waypoints for the operator's `route` command.

The operator drives waypoints in order but must advance ONLY when the localizer
confirms arrival by PERCEPTION (a descriptor match published on
/landmark_arrival_confirmed), never on move_base SUCCEEDED or the fused pose --
the fused pose is exactly what a navsat spoofing attack controls.

This class holds that advance policy and nothing else: no rospy, no I/O. It is
unit-tested in isolation. operate.py does the ROS wiring (send goals, publish the
active waypoint + expected anchors, subscribe to the confirmation Bool).
"""


class WaypointQueue(object):
    def __init__(self, names):
        """names: ordered list of waypoint names to visit."""
        self._names = list(names)
        self._index = 0

    def current(self):
        """The name of the active (not-yet-confirmed) waypoint, or None when the
        route is finished."""
        if self._index < len(self._names):
            return self._names[self._index]
        return None

    def on_arrival(self, confirmed):
        """Report an arrival attempt at the current waypoint. Advance to the next
        waypoint ONLY when perception confirmed arrival (confirmed=True). A
        confirmed=False report (e.g. move_base said done but the localizer did
        not perceive the expected region) is a no-op. Returns the new current()."""
        if confirmed and not self.done():
            self._index += 1
        return self.current()

    def done(self):
        """True once every waypoint has been perception-confirmed."""
        return self._index >= len(self._names)
