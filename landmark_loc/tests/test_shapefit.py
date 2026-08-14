import math
import numpy as np
from landmark_loc.shapefit import fit_rectangle

BENCH_L, BENCH_W = 1.78, 0.80

def _rect_edge_points(cx, cy, yaw, length, width, n_per_edge=15, sides="all"):
    """Sample points on the rectangle outline at pose (cx,cy,yaw)."""
    hl, hw = length / 2.0, width / 2.0
    corners = [(-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw)]
    c, s = math.cos(yaw), math.sin(yaw)
    pts = []
    edges = [(0, 1), (1, 2), (2, 3), (3, 0)]
    use = edges if sides == "all" else [edges[0]]  # "near" = one long edge
    for a, b in use:
        for t in np.linspace(0, 1, n_per_edge):
            lx = corners[a][0] + t * (corners[b][0] - corners[a][0])
            ly = corners[a][1] + t * (corners[b][1] - corners[a][1])
            pts.append((cx + c * lx - s * ly, cy + s * lx + c * ly))
    return np.array(pts, float)

def test_full_outline_recovers_pose():
    pts = _rect_edge_points(5.0, 2.0, 0.6, BENCH_L, BENCH_W, sides="all")
    cx, cy, yaw, ok = fit_rectangle(pts, BENCH_L, BENCH_W)
    assert ok
    assert abs(cx - 5.0) < 0.1 and abs(cy - 2.0) < 0.1
    # yaw modulo pi (a rectangle is symmetric under 180deg)
    dyaw = (yaw - 0.6) % math.pi
    assert min(dyaw, math.pi - dyaw) < 0.1

def test_L_shape_two_edges_recovers_center():
    hl, hw = BENCH_L / 2, BENCH_W / 2
    # near long edge + one short end (an L), at pose (3,-1,0)
    pts = _rect_edge_points(3.0, -1.0, 0.0, BENCH_L, BENCH_W, sides="all")
    # keep only points on the near long edge and one end (simulate partial view)
    keep = (pts[:, 1] < -1.0 + 0.05) | (pts[:, 0] > 3.0 + hl - 0.05)
    cx, cy, yaw, ok = fit_rectangle(pts[keep], BENCH_L, BENCH_W)
    assert ok
    assert abs(cx - 3.0) < 0.2 and abs(cy - (-1.0)) < 0.2

def test_sparse_returns_not_ok():
    pts = np.array([[1.0, 1.0], [1.1, 1.0]])  # 2 points, too few
    cx, cy, yaw, ok = fit_rectangle(pts, BENCH_L, BENCH_W)
    assert not ok
