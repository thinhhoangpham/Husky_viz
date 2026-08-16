# Bare import per ruling F4: the operator/ dir is on sys.path (conftest.py), and
# `operator` is a stdlib module name -- importing through the package dir would
# shadow it. Existing tests import bare too (e.g. `import gcs_commands`).
from waypoint_queue import WaypointQueue


def test_queue_advances_only_when_reached():
    q = WaypointQueue([(1.0, 2.0), (3.0, 4.0), (5.0, 6.0)])
    assert q.current() == (1.0, 2.0)
    q.on_arrival(reached=False)            # still en route
    assert q.current() == (1.0, 2.0)       # must NOT advance
    assert q.done() is False
    q.on_arrival(reached=True)
    assert q.current() == (3.0, 4.0)
    q.on_arrival(reached=True)
    assert q.current() == (5.0, 6.0)
    q.on_arrival(reached=True)
    assert q.current() is None
    assert q.done() is True


def test_on_arrival_returns_new_current():
    q = WaypointQueue([(0.0, 0.0), (7.0, 8.0)])
    assert q.on_arrival(reached=False) == (0.0, 0.0)
    assert q.on_arrival(reached=True) == (7.0, 8.0)
    assert q.on_arrival(reached=True) is None


def test_arrival_past_end_is_noop():
    q = WaypointQueue([(2.0, 2.0)])
    q.on_arrival(reached=True)
    assert q.done() is True
    q.on_arrival(reached=True)             # extra reports must not underflow/raise
    assert q.current() is None
    assert q.done() is True


def test_empty_route_is_immediately_done():
    q = WaypointQueue([])
    assert q.current() is None
    assert q.done() is True
