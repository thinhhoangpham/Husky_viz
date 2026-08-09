import math

CSV_HEADER = [
    "elapsed_time", "fused_x", "fused_y", "fused_yaw", "fused_yaw_deg",
    "planner_linear_x", "planner_angular_z", "ctrl_linear_x", "ctrl_angular_z",
    "ref_x", "ref_y",
    "active_goal_x", "active_goal_y", "nav_status", "heartbeat_age",
    "operator_mode",
]

def _f(v):
    return "nan" if v is None else "%.4f" % v

def build_row(elapsed, pose, planner, ctrl, sent_goal, active_goal,
              nav_status, heartbeat_age, mode):
    px, py, yaw = pose
    plx, paz = planner
    clx, caz = ctrl
    sx, sy = (sent_goal if sent_goal else (None, None))
    ax, ay = (active_goal if active_goal else (None, None))
    return [
        "%.4f" % elapsed, _f(px), _f(py), _f(yaw),
        _f(math.degrees(yaw)) if yaw is not None else "nan",
        _f(plx), _f(paz), _f(clx), _f(caz),
        _f(sx), _f(sy), _f(ax), _f(ay),
        nav_status, _f(heartbeat_age), mode,
    ]
