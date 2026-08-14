import math
from experiments.ab_matcher import compare
from landmark_loc.classify import Observation
from landmark_loc.catalog import MapLandmark


def test_compare_reports_both_arms():
    cat = [MapLandmark("lampA", "lamp", 10.0, 0.0),
           MapLandmark("benchB", "bench", 13.0, 4.0),
           MapLandmark("binC", "trash_bin_1", 8.0, 5.0)]
    c, s = 1.0, 0.0
    obs = [Observation(lm.identity, lm.x, lm.y) for lm in cat]  # true pose = origin
    out = compare(obs, cat, (0.0, 0.0, 0.0), 1.0)
    assert "typed" in out and "typeless" in out
    assert out["typed"]["n_inliers"] == 3
    assert out["typeless"]["n_inliers"] == 3
    # agreement flag: did both pick the same catalog names?
    assert out["agree"] is True
