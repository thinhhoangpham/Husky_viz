# landmark_loc/tests/test_classify.py
import numpy as np
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
    tree_obs = [o for o in obs if o.identity == "tree"][0]
    assert (tree_obs.x, tree_obs.y) == (1.0, 2.0)


def _dims(fam):
    from landmark_loc.signatures import MESH_SIGNATURES as S
    s = S[fam]
    return s["major"], s["minor"], s["height"]
