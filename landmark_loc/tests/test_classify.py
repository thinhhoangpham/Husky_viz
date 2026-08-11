# landmark_loc/tests/test_classify.py
from landmark_loc import classify
from landmark_loc.segment import Cluster


def _c(major, minor, height):
    return Cluster(points=None, centroid_xy=(1.0, 2.0),
                   major=major, minor=minor, height=height)


def test_classifies_each_type_from_ideal_dims():
    from landmark_loc.signatures import MESH_SIGNATURES as S
    for fam in ("bench", "garden_table", "lamp", "trash_bin_1"):
        sig = S[fam]
        got = classify.classify_cluster(_c(sig["major"], sig["minor"], sig["height"]))
        assert got == fam, f"{fam} misclassified as {got}"


def test_round_tall_trunk_is_tree_not_lamp():
    # tree trunk: small round footprint but tall with trunk radius > lamp pole
    got = classify.classify_cluster(_c(major=0.45, minor=0.42, height=4.0))
    assert got == "tree"


def test_real_trunks_inside_lamp_height_band_are_tree_not_lamp():
    # Adversarial: trunks whose height sits INSIDE the lamp height band and whose
    # footprint falls inside the lamp footprint band. They must be excluded from
    # identity (tree), never emitted as a phantom lamp landmark.
    for h in (2.5, 3.0, 3.5):
        got = classify.classify_cluster(_c(major=0.6, minor=0.5, height=h))
        assert got == "tree", f"trunk at height {h} misclassified as {got}"
    # exact reviewer example
    assert classify.classify_cluster(_c(major=0.6, minor=0.5, height=3.5)) == "tree"


def test_short_trunks_under_crop_are_tree_not_bin():
    # The localizer crops z in [-0.73, 1.2] -> clusters <= 1.93 m, so real trees
    # appear SHORT. These trunk/stump footprints must classify as tree (dropped),
    # NOT as phantom bins.
    short_trunks = [
        (0.9, 0.85, 1.8),   # tree_8 trunk (radius ~0.45 -> dia ~0.9)
        (0.6, 0.55, 1.6),   # arbolpartes4 trunk (radius ~0.30 -> dia ~0.6)
        (0.5, 0.4, 1.5),    # stump
    ]
    for mj, mn, h in short_trunks:
        got = classify.classify_cluster(_c(major=mj, minor=mn, height=h))
        assert got == "tree", f"short trunk {mj}/{mn}/{h} misclassified as {got}"


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


def test_to_observations_drops_tree_and_unknown():
    clusters = [
        _c(*_dims("bench")),
        _c(major=0.45, minor=0.42, height=4.0),   # tree
        _c(major=1.9, minor=1.3, height=0.9),     # unknown
    ]
    obs = classify.to_observations(clusters)
    assert len(obs) == 1 and obs[0].identity == "bench"
    assert obs[0].x == 1.0 and obs[0].y == 2.0


def _dims(fam):
    from landmark_loc.signatures import MESH_SIGNATURES as S
    s = S[fam]
    return s["major"], s["minor"], s["height"]
