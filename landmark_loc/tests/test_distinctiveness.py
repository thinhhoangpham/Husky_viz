import numpy as np
from landmark_loc.distinctiveness import nearest_distances, unique_names


def _d(*rows):
    a = np.zeros((18, 4))
    for bi, val in rows:
        a[bi, 0] = val
    return a


def test_repeated_shapes_score_low():
    # three identical "benches", one tall "pole"
    descs = {
        "bench_a": _d((0, 1.0)),
        "bench_b": _d((0, 1.0)),
        "bench_c": _d((0, 1.0)),
        "pole": _d((0, 1.0), (14, 1.0)),
    }
    nd = nearest_distances(descs)
    assert nd["bench_a"] == 0.0  # identical twin exists
    assert nd["pole"] > 0.5      # nothing like it
    uniq = unique_names(descs, threshold=0.5)
    assert uniq == {"pole"}


def test_single_entry_has_no_neighbour():
    nd = nearest_distances({"only": _d((0, 1.0))})
    assert nd["only"] == float("inf")
