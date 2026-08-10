import importlib.util, os
from types import SimpleNamespace

_spec = importlib.util.spec_from_file_location(
    "operate_mod",
    os.path.join(os.path.dirname(__file__), "..", "..", "operator", "operate.py"))
_stub_names = [
    "argparse", "csv", "math", "os", "sys", "threading", "time",
    "actionlib", "rospy",
]
# operate.py imports ROS packages at module scope, which aren't installed on
# this host. Stub them out just enough for module import to succeed; the
# module-level snap_to_free() function under test has no ROS dependency.
import sys as _sys
import types as _types


def _stub_module(name, attrs=None):
    if name in _sys.modules:
        return _sys.modules[name]
    m = _types.ModuleType(name)
    for k, v in (attrs or {}).items():
        setattr(m, k, v)
    _sys.modules[name] = m
    return m


_stub_module("rospy", {
    "Subscriber": lambda *a, **k: None,
    "Publisher": lambda *a, **k: None,
    "Time": SimpleNamespace(now=lambda: None),
    "loginfo": lambda *a, **k: None,
    "logwarn": lambda *a, **k: None,
    "ROSException": Exception,
    "wait_for_message": lambda *a, **k: None,
    "exceptions": _types.SimpleNamespace(ROSInterruptException=Exception),
    "Rate": lambda *a, **k: None,
    "is_shutdown": lambda: True,
    "init_node": lambda *a, **k: None,
})
_stub_module("actionlib", {"SimpleActionClient": object})
_GoalStatus = _types.SimpleNamespace(
    PENDING=0, ACTIVE=1, PREEMPTED=2, SUCCEEDED=3, ABORTED=4,
    REJECTED=5, PREEMPTING=6, RECALLING=7, RECALLED=8, LOST=9)
actionlib_msgs = _stub_module("actionlib_msgs")
actionlib_msgs_msg = _stub_module("actionlib_msgs.msg", {"GoalStatus": _GoalStatus, "GoalStatusArray": object})
geometry_msgs = _stub_module("geometry_msgs")
_stub_module("geometry_msgs.msg", {"Twist": object, "Pose": object})
gazebo_msgs = _stub_module("gazebo_msgs")
_stub_module("gazebo_msgs.srv", {"SpawnModel": object, "DeleteModel": object})
move_base_msgs = _stub_module("move_base_msgs")
_stub_module("move_base_msgs.msg", {
    "MoveBaseAction": object, "MoveBaseActionGoal": object, "MoveBaseGoal": object,
})
nav_msgs = _stub_module("nav_msgs")
_stub_module("nav_msgs.msg", {"Odometry": object, "OccupancyGrid": object})
std_msgs = _stub_module("std_msgs")
_stub_module("std_msgs.msg", {"Bool": object})
tf = _stub_module("tf")
_stub_module("tf.transformations", {
    "quaternion_from_euler": lambda *a, **k: (0, 0, 0, 1),
    "euler_from_quaternion": lambda *a, **k: (0, 0, 0),
})

_gcs_dir = os.path.join(os.path.dirname(__file__), "..", "..", "operator")
if _gcs_dir not in _sys.path:
    _sys.path.insert(0, _gcs_dir)

_m = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_m)
snap_to_free = _m.snap_to_free


def _grid(width, height, resolution, origin_x, origin_y, lethal_cells):
    data = [0] * (width * height)
    for (cx, cy) in lethal_cells:
        data[cy * width + cx] = 100
    info = SimpleNamespace(
        resolution=resolution, width=width, height=height,
        origin=SimpleNamespace(position=SimpleNamespace(x=origin_x, y=origin_y)))
    return data, info


def test_snap_free_point_unchanged():
    # 10x10 grid, 1m resolution, origin at (0,0), all free.
    data, info = _grid(10, 10, 1.0, 0.0, 0.0, lethal_cells=[])
    sx, sy = snap_to_free(data, info, 5.4, 5.4)
    # Already free -> radius-0 ring hits the same cell; center of that cell.
    assert (sx, sy) == (5.5, 5.5)


def test_snap_out_of_lethal_block():
    # 10x10 grid, 1m resolution. Lethal 3x3 block centered on cell (5,5).
    # With clearance_m=0.6 (1 cell at this resolution), the accepted cell
    # must have a full clear 3x3 neighborhood, pushing the result out to
    # ring radius 3 (not radius 2, which is only "not lethal itself" but
    # still touches the lethal block's neighborhood).
    lethal = [(cx, cy) for cx in range(4, 7) for cy in range(4, 7)]
    data, info = _grid(10, 10, 1.0, 0.0, 0.0, lethal_cells=lethal)
    sx, sy = snap_to_free(data, info, 5.5, 5.5)
    cell_x = int(sx)
    cell_y = int(sy)
    assert (cell_x, cell_y) not in lethal
    assert max(abs(cell_x - 5), abs(cell_y - 5)) == 3


def test_snap_out_of_lethal_block_has_full_clearance():
    # Same block as above; assert the snapped cell's clearance ring (radius
    # 1, from clearance_m=0.6 at 1m resolution) contains no lethal cell.
    lethal = [(cx, cy) for cx in range(4, 7) for cy in range(4, 7)]
    data, info = _grid(10, 10, 1.0, 0.0, 0.0, lethal_cells=lethal)
    sx, sy = snap_to_free(data, info, 5.5, 5.5)
    cell_x, cell_y = int(sx), int(sy)
    for dc in (-1, 0, 1):
        for dr in (-1, 0, 1):
            assert (cell_x + dc, cell_y + dr) not in lethal


def test_snap_skips_inflated_ring_for_truly_free_cell():
    # Reproduces the reported bug: a large lethal core surrounded by an
    # inflated ring (cost 99, below the old cost_thresh=50 rejection but
    # NOT free), with truly-free (cost 0) cells only further out. The old
    # behavior (cost < 50) would accept an inflated-adjacent cell that still
    # has no clearance; the fix must skip past the inflated ring entirely.
    width = height = 21
    res = 1.0
    data = [0] * (width * height)
    cx0, cy0 = 10, 10

    def set_cost(cx, cy, val):
        data[cy * width + cx] = val

    # Lethal core: 3x3 block (cost 100).
    for cx in range(cx0 - 1, cx0 + 2):
        for cy in range(cy0 - 1, cy0 + 2):
            set_cost(cx, cy, 100)
    # Inflated ring immediately around the core (cost 99): rings at
    # Chebyshev distance 2 and 3 from center.
    for rad in (2, 3):
        for dc in range(-rad, rad + 1):
            for dr in range(-rad, rad + 1):
                if max(abs(dc), abs(dr)) != rad:
                    continue
                set_cost(cx0 + dc, cy0 + dr, 99)
    # Everything at distance >= 4 stays cost 0 (already initialized).

    info = SimpleNamespace(
        resolution=res, width=width, height=height,
        origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0)))
    sx, sy = snap_to_free(data, info, float(cx0) + 0.5, float(cy0) + 0.5,
                           max_radius_m=10.0)
    cell_x, cell_y = int(sx), int(sy)
    dist = max(abs(cell_x - cx0), abs(cell_y - cy0))
    # Must land beyond the inflated ring (distance >= 4), not on an
    # inflated-adjacent cell.
    assert dist >= 4
    assert data[cell_y * width + cell_x] == 0


def test_snap_no_costmap_returns_unchanged_via_wrapper_contract():
    # Pure function contract: if searching finds nothing within max_radius_m
    # (e.g. entire grid lethal), the original point is returned unchanged.
    data, info = _grid(3, 3, 1.0, 0.0, 0.0, lethal_cells=[(x, y) for x in range(3) for y in range(3)])
    sx, sy = snap_to_free(data, info, 1.5, 1.5, max_radius_m=1.0)
    assert (sx, sy) == (1.5, 1.5)
