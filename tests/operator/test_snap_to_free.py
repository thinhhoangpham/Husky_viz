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
    lethal = [(cx, cy) for cx in range(4, 7) for cy in range(4, 7)]
    data, info = _grid(10, 10, 1.0, 0.0, 0.0, lethal_cells=lethal)
    sx, sy = snap_to_free(data, info, 5.5, 5.5)
    cell_x = int(sx)
    cell_y = int(sy)
    assert (cell_x, cell_y) not in lethal
    # Nearest free ring around (5,5) is radius 2 -> cells at distance 2.
    assert max(abs(cell_x - 5), abs(cell_y - 5)) == 2


def test_snap_no_costmap_returns_unchanged_via_wrapper_contract():
    # Pure function contract: if searching finds nothing within max_radius_m
    # (e.g. entire grid lethal), the original point is returned unchanged.
    data, info = _grid(3, 3, 1.0, 0.0, 0.0, lethal_cells=[(x, y) for x in range(3) for y in range(3)])
    sx, sy = snap_to_free(data, info, 1.5, 1.5, max_radius_m=1.0)
    assert (sx, sy) == (1.5, 1.5)
