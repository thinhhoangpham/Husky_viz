# landmark_loc/tests/test_classify.py
import math
import numpy as np
import pytest
from landmark_loc import classify
from landmark_loc.segment import Cluster


def _c(major, minor, height):
    return Cluster(points=None, centroid_xy=(1.0, 2.0),
                   major=major, minor=minor, height=height)


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


def test_classifies_each_type_from_ideal_dims():
    from landmark_loc.signatures import MESH_SIGNATURES as S
    for fam in ("bench", "garden_table", "lamp", "trash_bin_1"):
        sig = S[fam]
        got = classify.classify_cluster(_c(sig["major"], sig["minor"], sig["height"]))
        assert got == fam, f"{fam} misclassified as {got}"


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


def test_ideal_bin_is_still_bin():
    from landmark_loc.signatures import MESH_SIGNATURES as S
    s = S["trash_bin_1"]
    got = classify.classify_cluster(_c(s["major"], s["minor"], s["height"]))
    assert got == "trash_bin_1"


def test_ideal_lamp_is_still_lamp():
    from landmark_loc.signatures import MESH_SIGNATURES as S
    s = S["lamp"]
    got = classify.classify_cluster(_c(s["major"], s["minor"], s["height"]))
    assert got == "lamp"


def test_ambiguous_between_bands_is_unknown():
    # deliberately between bench and table aspect/size
    got = classify.classify_cluster(_c(major=1.9, minor=1.3, height=0.9))
    assert got == "unknown"


def test_to_observations_emits_tree_drops_unknown():
    bench_dims = _dims("bench")
    clusters = [
        Cluster(points=None, centroid_xy=(1.0, 2.0), major=bench_dims[0],
                minor=bench_dims[1], height=bench_dims[2]),
        _canopy_cluster(),                                            # tree -> emitted
        Cluster(points=None, centroid_xy=(3.0, 4.0), major=1.9,
                minor=1.3, height=0.9),                               # unknown -> dropped
    ]
    obs = classify.to_observations(clusters)
    idents = sorted(o.identity for o in obs)
    assert idents == ["bench", "tree"]

    # Both emitted observations are offset outward from the raw (1.0, 2.0)
    # centroid by their identity's known radius, along the same bearing.
    r0 = math.hypot(1.0, 2.0)
    ux, uy = 1.0 / r0, 2.0 / r0

    bench_obs = [o for o in obs if o.identity == "bench"][0]
    bench_r = classify.KNOWN_RADIUS["bench"]
    assert bench_obs.x == pytest.approx(1.0 + bench_r * ux)
    assert bench_obs.y == pytest.approx(2.0 + bench_r * uy)

    tree_obs = [o for o in obs if o.identity == "tree"][0]
    tree_r = classify.KNOWN_RADIUS["tree"]
    assert tree_obs.x == pytest.approx(1.0 + tree_r * ux)
    assert tree_obs.y == pytest.approx(2.0 + tree_r * uy)


def test_to_observations_offset_is_view_invariant():
    """The same real lamp seen from two different robot bearings should yield
    observation positions at the same distance-from-origin (true center),
    along whatever bearing the raw centroid was on -- i.e. the offset always
    lands at D + R along the observed direction, not a fixed world point."""
    from landmark_loc.signatures import MESH_SIGNATURES as S
    lamp_dims = _dims("lamp")
    R = S["lamp"]["minor"] / 2.0

    for theta in (0.3, 2.1):
        D = 5.0
        cx, cy = D * math.cos(theta), D * math.sin(theta)
        c = Cluster(points=None, centroid_xy=(cx, cy),
                    major=lamp_dims[0], minor=lamp_dims[1], height=lamp_dims[2])
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
    lamp_dims = _dims("lamp")
    c = Cluster(points=None, centroid_xy=(0.0, 0.0),
                major=lamp_dims[0], minor=lamp_dims[1], height=lamp_dims[2])
    obs = classify.to_observations([c])
    assert len(obs) == 1
    assert (obs[0].x, obs[0].y) == (0.0, 0.0)


def _dims(fam):
    from landmark_loc.signatures import MESH_SIGNATURES as S
    s = S[fam]
    return s["major"], s["minor"], s["height"]
