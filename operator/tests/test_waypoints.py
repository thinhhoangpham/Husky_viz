# Bare import per ruling F4: the operator/ dir is on sys.path (conftest.py), and
# `operator` is a stdlib module name -- importing through the package dir would
# shadow it. Existing tests import bare too (e.g. `import gcs_commands`).
from waypoint_queue import WaypointQueue


def test_queue_advances_only_on_confirmation():
    q = WaypointQueue(["pole_A", "bench_3", "pole_B"])
    assert q.current() == "pole_A"
    q.on_arrival(confirmed=False)          # move_base says done, perception does not
    assert q.current() == "pole_A"          # must NOT advance
    assert q.done() is False
    q.on_arrival(confirmed=True)
    assert q.current() == "bench_3"
    q.on_arrival(confirmed=True)
    assert q.current() == "pole_B"
    q.on_arrival(confirmed=True)
    assert q.current() is None
    assert q.done() is True


def test_on_arrival_returns_new_current():
    q = WaypointQueue(["a", "b"])
    assert q.on_arrival(confirmed=False) == "a"
    assert q.on_arrival(confirmed=True) == "b"
    assert q.on_arrival(confirmed=True) is None


def test_confirmation_past_end_is_noop():
    q = WaypointQueue(["only"])
    q.on_arrival(confirmed=True)
    assert q.done() is True
    q.on_arrival(confirmed=True)             # extra confirms must not underflow/raise
    assert q.current() is None
    assert q.done() is True


def test_empty_route_is_immediately_done():
    q = WaypointQueue([])
    assert q.current() is None
    assert q.done() is True
