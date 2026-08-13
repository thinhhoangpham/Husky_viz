# landmark_loc/tests/test_classify.py
import math
import numpy as np
import pytest
from landmark_loc import classify
from landmark_loc.classify import Observation, to_observations
from landmark_loc.segment import Cluster


def _canopy_cluster(trunk_w=0.4, canopy_w=4.0, top_z=6.0):
    """Synthetic tree: a narrow trunk (z 0..2.4) under a wide canopy (z 2.5..top).
    Returns a Cluster whose `points` produce the intended vertical profile."""
    pts = []
    # trunk: narrow column of points from z=0 to 2.4
    for z in np.linspace(0.0, 2.4, 12):
        pts += [(0.0, 0.0, z), (trunk_w, 0.0, z)]
    # canopy: wide disc of points from z=2.5 to top_z
    for z in np.linspace(2.5, top_z, 10):
        for a in np.linspace(0, 2 * np.pi, 12, endpoint=False):
            r = canopy_w / 2.0
            pts.append((r * np.cos(a), r * np.sin(a), z))
    arr = np.array(pts, dtype=float)
    major = arr[:, 0].max() - arr[:, 0].min()
    minor = arr[:, 1].max() - arr[:, 1].min()
    height = arr[:, 2].max() - arr[:, 2].min()
    return Cluster(points=arr, centroid_xy=(1.0, 2.0),
                   major=float(major), minor=float(minor), height=float(height))


def test_classifies_each_type_from_shape():
    assert classify.classify_cluster(_pts_cluster(_lamp_post())) == "lamp"
    assert classify.classify_cluster(_pts_cluster(_box(0.68, 0.38, 1.04))) == "trash_bin_1"
    assert classify.classify_cluster(_pts_cluster(_box(1.78, 0.80, 0.9))) == "bench"
    assert classify.classify_cluster(_pts_cluster(_box(3.0, 1.32, 1.05))) == "garden_table"


def test_wide_canopy_over_trunk_is_tree():
    assert classify.classify_cluster(_canopy_cluster()) == "tree"


def test_min_canopy_width_is_tree():
    # width exactly at the 2.0 m threshold, high band -> tree
    assert classify.classify_cluster(_canopy_cluster(canopy_w=2.0, top_z=5.0)) == "tree"


def test_thin_tall_pole_is_not_tree():
    # a lamp-like column: narrow (<2 m) at every height -> NOT tree
    pts = []
    for z in np.linspace(0.0, 3.0, 30):
        pts += [(0.0, 0.0, z), (0.5, 0.0, z)]  # 0.5 m wide the whole way up
    arr = np.array(pts, dtype=float)
    c = Cluster(points=arr, centroid_xy=(1.0, 2.0), major=0.5, minor=0.5, height=3.0)
    assert classify.classify_cluster(c) != "tree"


def test_low_wide_object_is_not_tree():
    # a bench-like object: wide but only low (no band at z >= 2.5) -> NOT tree
    pts = []
    for z in np.linspace(0.0, 0.9, 6):
        for x in np.linspace(0.0, 2.5, 8):
            pts.append((x, 0.0, z))
    arr = np.array(pts, dtype=float)
    c = Cluster(points=arr, centroid_xy=(1.0, 2.0), major=2.5, minor=0.4, height=0.9)
    assert classify.classify_cluster(c) != "tree"


def test_tall_box_is_unknown():
    # a box-shaped cluster too tall for any known box type (bin/bench/table
    # all sit under _BOX_MAX_H=1.40) and too wide-footed to be a lamp post ->
    # matches no rule -> unknown. The OLD test encoded a "between bands"
    # aspect/size gap that no longer exists now that bin/bench/table foot
    # bands are contiguous (0.30-1.20-2.30+); this is the shape-equivalent
    # gap: no band covers this height.
    got = classify.classify_cluster(_pts_cluster(_box(1.9, 1.3, 2.0)))
    assert got == "unknown"


def test_to_observations_emits_tree_drops_unknown():
    bench_cluster = _pts_cluster(_box(1.78, 0.80, 0.9))
    unknown_cluster = _pts_cluster(_box(1.9, 1.3, 2.0))
    clusters = [
        bench_cluster,      # bench -> emitted
        _canopy_cluster(),  # tree  -> emitted
        unknown_cluster,    # too tall for any box band -> unknown, dropped
    ]
    obs = classify.to_observations(clusters)
    idents = sorted(o.identity for o in obs)
    assert "tree" in idents and "bench" in idents
    assert "unknown" not in idents

    # Tree position is derived from the TRUNK (base of points), not the
    # centroid_xy placeholder, and not the canopy blob mean.
    tree_cluster = clusters[1]
    trunk_x, trunk_y = classify._trunk_xy(tree_cluster.points)
    tree_r = classify.KNOWN_RADIUS["tree"]
    tr = math.hypot(trunk_x, trunk_y)
    tux, tuy = trunk_x / tr, trunk_y / tr
    tree_obs = [o for o in obs if o.identity == "tree"][0]
    assert tree_obs.x == pytest.approx(trunk_x + tree_r * tux)
    assert tree_obs.y == pytest.approx(trunk_y + tree_r * tuy)


def _lamp_post_at(cx, cy, height=2.5, w=0.14, n=40):
    """A thin-post lamp cluster whose points are centered at world (cx, cy),
    with centroid_xy matching that offset (mirrors a real lamp seen at that
    world position)."""
    pts = _lamp_post(height=height, w=w, n=n)
    pts[:, 0] += cx
    pts[:, 1] += cy
    return _pts_cluster(pts)


def test_to_observations_offset_is_view_invariant():
    """The same real lamp seen from two different robot bearings should yield
    observation positions at the same distance-from-origin (true center),
    along whatever bearing the raw centroid was on -- i.e. the offset always
    lands at D + R along the observed direction, not a fixed world point."""
    from landmark_loc.signatures import MESH_SIGNATURES as S
    R = S["lamp"]["minor"] / 2.0

    for theta in (0.3, 2.1):
        D = 5.0
        cx, cy = D * math.cos(theta), D * math.sin(theta)
        c = _lamp_post_at(cx, cy)
        obs = classify.to_observations([c])
        assert len(obs) == 1
        o = obs[0]
        assert o.identity == "lamp"
        assert math.hypot(o.x, o.y) == pytest.approx(D + R)
        # bearing unchanged
        assert math.atan2(o.y, o.x) == pytest.approx(theta)


def test_to_observations_centroid_at_origin_unchanged():
    # r ~ 0: no direction to push along, must not divide by zero and must
    # leave the position unchanged.
    c = _lamp_post_at(0.0, 0.0)
    obs = classify.to_observations([c])
    assert len(obs) == 1
    assert obs[0].x == pytest.approx(0.0, abs=1e-9)
    assert obs[0].y == pytest.approx(0.0, abs=1e-9)


def _offset_canopy_tree_cluster():
    """Synthetic tree: a narrow trunk column at (5.0, 2.0) plus a wide canopy
    blob offset to (7.0, 4.0). The naive centroid (mean of all points) is
    pulled toward the canopy (~6, 3); the trunk xy should stay at (5, 2)."""
    pts = []
    for z in np.linspace(0.0, 2.0, 12):
        pts += [(5.0, 2.0, z), (5.0 + 0.1, 2.0, z)]
    for z in np.linspace(2.5, 4.0, 10):
        for a in np.linspace(0, 2 * np.pi, 24, endpoint=False):
            r = 3.8 / 2.0
            pts.append((7.0 + r * np.cos(a), 4.0 + r * np.sin(a), z))
    arr = np.array(pts, dtype=float)
    major = arr[:, 0].max() - arr[:, 0].min()
    minor = arr[:, 1].max() - arr[:, 1].min()
    height = arr[:, 2].max() - arr[:, 2].min()
    return Cluster(points=arr, centroid_xy=(float(arr[:, 0].mean()), float(arr[:, 1].mean())),
                   major=float(major), minor=float(minor), height=float(height))


def test_trunk_xy_returns_trunk_not_blended_mean():
    c = _offset_canopy_tree_cluster()
    trunk_x, trunk_y = classify._trunk_xy(c.points)
    assert trunk_x == pytest.approx(5.0, abs=0.2)
    assert trunk_y == pytest.approx(2.0, abs=0.2)
    # NOT anywhere near the blended centroid (~6, 3)
    assert math.hypot(trunk_x - 6.0, trunk_y - 3.0) > 0.5


def test_trunk_xy_none_for_missing_points():
    assert classify._trunk_xy(None) is None
    assert classify._trunk_xy(np.zeros((0, 3))) is None


def test_trunk_xy_falls_back_when_too_few_low_points():
    # Only 2 points in the low band (< _TRUNK_BAND_MIN of 3) -> fallback signal.
    pts = np.array([
        (5.0, 2.0, 0.0),
        (5.0, 2.0, 0.5),
        (7.0, 4.0, 3.0),
        (7.0, 4.0, 3.5),
    ])
    assert classify._trunk_xy(pts) is None


def test_to_observations_tree_position_near_trunk_not_canopy():
    c = _offset_canopy_tree_cluster()
    assert classify.classify_cluster(c) == "tree"
    obs = classify.to_observations([c])
    assert len(obs) == 1
    o = obs[0]
    assert o.identity == "tree"
    dist_to_trunk = math.hypot(o.x - 5.0, o.y - 2.0)
    dist_to_canopy = math.hypot(o.x - 7.0, o.y - 4.0)
    assert dist_to_trunk < dist_to_canopy
    assert dist_to_trunk < 1.0  # trunk (5,2) pushed out by 0.45m radius


def test_to_observations_tree_falls_back_to_centroid_when_no_trunk_band():
    # Force a cluster classified as tree (via monkeypatch-free trick: build
    # points with a valid canopy profile) but with too few low-band points by
    # starting the base high with only 2 points below base+_TRUNK_BAND.
    pts = []
    # 2 low points only, at the very base
    pts += [(5.0, 2.0, 0.0), (5.0, 2.0, 0.05)]
    # wide canopy so it's classified as tree
    for z in np.linspace(2.5, 4.0, 10):
        for a in np.linspace(0, 2 * np.pi, 24, endpoint=False):
            r = 3.8 / 2.0
            pts.append((7.0 + r * np.cos(a), 4.0 + r * np.sin(a), z))
    arr = np.array(pts, dtype=float)
    major = arr[:, 0].max() - arr[:, 0].min()
    minor = arr[:, 1].max() - arr[:, 1].min()
    height = arr[:, 2].max() - arr[:, 2].min()
    centroid = (float(arr[:, 0].mean()), float(arr[:, 1].mean()))
    c = Cluster(points=arr, centroid_xy=centroid, major=float(major),
                minor=float(minor), height=float(height))
    assert classify.classify_cluster(c) == "tree"
    assert classify._trunk_xy(c.points) is None  # confirms fallback path taken

    obs = classify.to_observations([c])
    assert len(obs) == 1
    o = obs[0]
    # Falls back to centroid_xy + push-out; must not crash and must not equal
    # the (never-computed) trunk-based position.
    cx, cy = centroid
    r = math.hypot(cx, cy)
    radius = classify.KNOWN_RADIUS["tree"]
    ux, uy = cx / r, cy / r
    assert o.x == pytest.approx(cx + radius * ux)
    assert o.y == pytest.approx(cy + radius * uy)


def _bench_cluster_from_outline():
    # a bench outline at robot-frame (5,0), yaw 0 -> full rectangle points
    L, W = 1.78, 0.80
    hl, hw = L / 2, W / 2
    corners = [(-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw)]
    edges = [(0, 1), (1, 2), (2, 3), (3, 0)]
    pts = []
    for a, b in edges:
        for t in np.linspace(0, 1, 15):
            lx = corners[a][0] + t * (corners[b][0] - corners[a][0])
            ly = corners[a][1] + t * (corners[b][1] - corners[a][1])
            pts.append((5.0 + lx, 0.0 + ly))
    xy = np.array(pts)
    z = np.full((len(xy), 1), 0.4)
    p3 = np.hstack([xy, z])
    return Cluster(points=p3, centroid_xy=(float(xy[:, 0].mean()), float(xy[:, 1].mean())),
                   major=L, minor=W, height=0.4)


def test_to_observations_falls_back_to_pushout_when_rect_fit_fails(monkeypatch):
    # Guards the classify.py:151-157 `if ok:` fallback branch in to_observations:
    # when shapefit.fit_rectangle fails (ok=False) for an identity that IS
    # rect-fit-eligible (bench/garden_table), the observation must still be
    # emitted via centroid+pushout, not silently dropped.
    #
    # NOTE: under current thresholds this branch is unreachable with real data
    # -- shapefeat._MIN_SHAPE_PTS and shapefit._MIN_PTS are both exactly 6, so
    # any cluster with enough points to classify as "bench" also has enough
    # points to make fit_rectangle return ok=True (verified: fit_rectangle has
    # no other ok=False exit; even fully degenerate points >= 6 count return
    # ok=True). This test forces the failure via monkeypatch to guard the
    # fallback LOGIC against a future change to fit_rectangle (e.g. a real
    # convergence/residual check) that could start returning ok=False for real
    # data -- without this test, that change could silently drop observations.
    c = _pts_cluster(_box(1.78, 0.80, 0.9))
    c = Cluster(points=c.points, centroid_xy=(3.0, 4.0),
                major=c.major, minor=c.minor, height=c.height)
    assert classify.classify_cluster(c) == "bench"

    monkeypatch.setattr(classify.shapefit, "fit_rectangle",
                         lambda xy, L, W: (0.0, 0.0, 0.0, False))

    obs = classify.to_observations([c])
    assert len(obs) == 1
    o = obs[0]
    assert o.identity == "bench"

    cx, cy = c.centroid_xy
    r = math.hypot(cx, cy)
    radius = classify.KNOWN_RADIUS["bench"]
    ux, uy = cx / r, cy / r
    assert o.x == pytest.approx(cx + radius * ux)
    assert o.y == pytest.approx(cy + radius * uy)


def test_bench_observation_has_yaw_and_fit_center():
    c = _bench_cluster_from_outline()
    obs = to_observations([c])
    assert len(obs) == 1
    o = obs[0]
    assert o.identity == "bench"
    assert o.yaw is not None
    # fit center near the true (5,0), not just the centroid
    assert abs(o.x - 5.0) < 0.2 and abs(o.y - 0.0) < 0.2


def test_lamp_observation_yaw_is_none():
    # a compact lamp cluster: tall, narrow post (round type keeps
    # centroid+pushout, no rectangle fit, so no yaw)
    c = _lamp_post_at(3.0, 0.0)
    obs = to_observations([c])
    assert len(obs) == 1 and obs[0].identity == "lamp" and obs[0].yaw is None


# --- shape-rule tests (Task 2) ---
import numpy as np
from landmark_loc.segment import Cluster


def _pts_cluster(points):
    xy = points[:, :2]
    from landmark_loc.segment import _pca_extents
    major, minor = _pca_extents(xy)
    return Cluster(points=points, centroid_xy=(float(xy[:, 0].mean()), float(xy[:, 1].mean())),
                   major=float(major), minor=float(minor),
                   height=float(points[:, 2].max() - points[:, 2].min()))


def _lamp_post(height=2.5, w=0.14, n=40):
    pts = []
    for z in np.linspace(0.0, height, n):
        for a in np.linspace(0, 2 * np.pi, 6, endpoint=False):
            pts.append((0.5 * w * np.cos(a), 0.5 * w * np.sin(a), z))
    return np.array(pts, float)


def _box(major, minor, height, n=8):
    pts = []
    for z in np.linspace(0.0, height, 4):
        for x in np.linspace(-major / 2, major / 2, n):
            for y in (-minor / 2, minor / 2):
                pts.append((x, y, z))
    return np.array(pts, float)


def test_thin_tall_post_classifies_lamp():
    assert classify.classify_cluster(_pts_cluster(_lamp_post())) == "lamp"


def test_short_oblong_box_classifies_bin():
    # bin: 0.68 x 0.38 x 1.04, no tall thin post
    assert classify.classify_cluster(_pts_cluster(_box(0.68, 0.38, 1.04))) == "trash_bin_1"


def test_medium_low_box_classifies_bench():
    assert classify.classify_cluster(_pts_cluster(_box(1.78, 0.80, 0.9))) == "bench"


def test_long_low_box_classifies_table():
    assert classify.classify_cluster(_pts_cluster(_box(3.0, 1.32, 1.05))) == "garden_table"


def test_pole_fragment_is_not_bin():
    # a tiny pole fragment (the OLD phantom) must NOT be trash_bin now
    frag = _lamp_post(height=0.5, w=0.10, n=8)
    assert classify.classify_cluster(_pts_cluster(frag)) != "trash_bin_1"


def test_no_points_is_unknown():
    c = Cluster(points=None, centroid_xy=(1, 2), major=0.7, minor=0.4, height=1.0)
    assert classify.classify_cluster(c) == "unknown"
