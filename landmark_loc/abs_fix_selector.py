"""Live pose-source selector: forwards exactly ONE absolute-position source onto
/odometry/abs_fix (the map-EKF's absolute anchor), switchable at runtime by the
operator. Strict forward -- never substitutes the unselected source; raises a
:stale flag when the selected source goes silent past stale_timeout.

This is the "twist_mux for the pose anchor": navsat_transform publishes
/odometry/gps_fix, landmark_localizer publishes /odometry/landmark_fix, and this
node passes only the selected one to /odometry/abs_fix. See
docs/superpowers/specs/2026-08-11-live-pose-source-switch-design.md.

ROS imports live inside main()/callbacks so AbsFixArbiter is unit-testable
without a running master (same pattern as landmark_loc/localizer_node.py).
"""

# friendly-name -> input topic. Single source of truth; the operator mirrors it.
SOURCES = {
    "gps": "/odometry/gps_fix",
    "landmark": "/odometry/landmark_fix",
}
TOPIC_TO_NAME = {v: k for k, v in SOURCES.items()}

OUTPUT_TOPIC = "/odometry/abs_fix"
STATUS_TOPIC = "/abs_fix_mode"
SELECT_SERVICE = "/set_abs_fix_mode"
DEFAULT_STALE_TIMEOUT = 2.0


class AbsFixArbiter(object):
    """Pure arbitration logic, no ROS. Decides which source forwards and whether
    the selected source is stale. Knows nothing about the unselected source's
    freshness -- silence there must never affect output or status."""

    def __init__(self, stale_timeout=DEFAULT_STALE_TIMEOUT, initial="gps"):
        if initial not in SOURCES:
            raise ValueError("unknown initial source: %s" % initial)
        self.stale_timeout = stale_timeout
        self._selected = initial
        self._last_seen = {}  # friendly name -> float seconds of last message

    @property
    def selected_name(self):
        return self._selected

    def select(self, name):
        """Switch selected source by friendly name. Return the PREVIOUS friendly
        name. Return None (state unchanged) if name is unknown."""
        if name not in SOURCES:
            return None
        prev = self._selected
        self._selected = name
        return prev

    def note_message(self, name, now):
        """Record that source `name` produced a message at time `now`."""
        if name in SOURCES:
            self._last_seen[name] = now

    def should_forward(self, name):
        """True iff `name` is the currently selected source."""
        return name == self._selected

    def status(self, now):
        """Friendly name of the selected source, with ':stale' iff its last
        message is older than stale_timeout (or it never published). Only the
        SELECTED source's freshness matters."""
        last = self._last_seen.get(self._selected)
        stale = last is None or (now - last) > self.stale_timeout
        return self._selected + (":stale" if stale else "")
