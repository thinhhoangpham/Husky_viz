"""Tests for the best-score classifier (landmark_loc.score).

The cascade's guarantees that MUST survive the move to best-score are tested
here in terms of the scorer, not by re-testing the cascade:

  * every type still classifies its canonical synthetic shape (mirrors
    test_classify.py's per-type cases),
  * the two HARD vetoes (min points, ground anchoring) still reject,
  * confidence is a real number in [0, 1], and below-floor becomes "unknown",
  * THE TREE-FIRST PROBLEM: the cascade gave `tree` an unfair advantage by
    testing it FIRST. Under best-score there is no first, so a canopy must
    OUTSCORE bin/bench/table on the merits. That is the single most likely
    regression in this change and is tested explicitly below.
"""
import numpy as np
import pytest

from landmark_loc import classify, score
from landmark_loc.segment import Cluster, _pca_extents
from map_tools.park_types import PARK_TYPES

# plain import, not relative: landmark_loc/tests has no __init__.py, so pytest
# imports these modules top-level with the test dir on sys.path.
from test_classify import _canopy_cluster, _pts_cluster, _box, _lamp_post


def _cluster(pts):
    xy = pts[:, :2]
    major, minor = _pca_extents(xy)
    return Cluster(points=pts,
                   centroid_xy=(float(xy[:, 0].mean()), float(xy[:, 1].mean())),
                   major=float(major), minor=float(minor),
                   height=float(pts[:, 2].max() - pts[:, 2].min()))


# --- canonical shapes: same expectations as the cascade -----------------

@pytest.mark.parametrize("expected,pts", [
    ("lamp", _lamp_post()),
    ("trash_bin_1", _box(0.68, 0.38, 1.04)),
    ("bench", _box(1.78, 0.80, 0.9)),
    ("garden_table", _box(3.0, 1.32, 1.05)),
])
def test_canonical_shape_scores_as_its_own_type(expected, pts):
    ident, conf = score.classify_cluster(_pts_cluster(pts))
    assert ident == expected
    assert conf > 0.0


def test_canopy_over_trunk_is_tree():
    ident, conf = score.classify_cluster(_canopy_cluster())
    assert ident == "tree"
    assert conf > 0.0


def test_tall_box_matching_no_type_is_unknown():
    # Same cluster the cascade calls unknown: too tall for any box band.
    ident, _conf = score.classify_cluster(_pts_cluster(_box(1.9, 1.3, 2.0)))
    assert ident == "unknown"


# --- D4: the tree-first problem -----------------------------------------

@pytest.mark.parametrize("canopy_w", [2.5, 3.0, 4.0, 4.75, 6.0])
def test_tree_outscores_every_furniture_type(canopy_w):
    """A canopy must WIN on score, not on cascade position.

    Asserted as a strict inequality against each furniture type individually,
    so this fails loudly if a future band widening lets a canopy sneak into
    bench/garden_table -- which cascade ORDER, not any scoring property, is
    what prevented before this change.
    """
    c = _canopy_cluster(canopy_w=canopy_w)
    scores = score.score_cluster(c)
    for other in ("trash_bin_1", "bench", "garden_table", "lamp"):
        assert scores["tree"] > scores[other], (
            "canopy_w=%s: tree %.3f did not beat %s %.3f"
            % (canopy_w, scores["tree"], other, scores[other]))
    assert score.classify_cluster(c)[0] == "tree"


def test_wide_canopy_reaching_the_ground_is_still_tree():
    """The adversarial case: a canopy whose cluster also spans down to ground
    level, so the ground veto cannot help and the footprint is table-sized.
    The cascade got this right only by testing tree first."""
    pts = []
    for z in np.linspace(2.5, 3.0, 4):
        for a in np.linspace(0, 2 * np.pi, 16, endpoint=False):
            pts.append((1.5 * np.cos(a), 1.5 * np.sin(a), z))
    for z in np.linspace(-0.4, 2.4, 10):
        pts += [(0.0, 0.0, z), (0.3, 0.0, z)]
    c = _cluster(np.array(pts, float))
    assert classify.classify_cluster(c) == "tree"      # cascade's answer
    assert score.classify_cluster(c)[0] == "tree"      # must not regress


def test_narrow_canopy_below_the_floor_is_not_a_tree():
    """Below the 2.0 m canopy floor the tree score must be 0 -- the margin
    formula is one-sided, and a lamp head must not read as a tiny tree."""
    scores = score.score_cluster(_pts_cluster(_lamp_post()))
    assert scores["tree"] == 0.0


# --- hard vetoes ---------------------------------------------------------

def test_too_few_points_is_vetoed():
    pts = np.array([(0.0, 0.0, 0.0), (0.1, 0.0, 0.1), (0.2, 0.0, 0.2)], float)
    c = _cluster(pts)
    assert score.veto(c) == "too-few-points"
    assert score.classify_cluster(c)[0] == "unknown"


def test_floating_cluster_is_vetoed_from_ground_types():
    """A bench-shaped cluster floating 4 m up is the phantom-furniture case
    the ground gate exists for: identical dimensions, wrong physics."""
    pts = _box(1.78, 0.80, 0.9)
    pts[:, 2] += 4.0
    c = _cluster(pts)
    assert score.veto(c) == "not-ground-anchored"
    scores = score.score_cluster(c)
    assert scores["bench"] == 0.0
    assert score.classify_cluster(c)[0] == "unknown"


def test_ground_veto_does_not_apply_to_tree():
    """A cluster holding only a canopy is legitimately a tree sighting, so the
    ground veto must not reach it -- same asymmetry the cascade has."""
    pts = []
    for z in np.linspace(3.0, 5.0, 6):
        for a in np.linspace(0, 2 * np.pi, 16, endpoint=False):
            pts.append((2.0 * np.cos(a), 2.0 * np.sin(a), z))
    c = _cluster(np.array(pts, float))
    assert score.veto(c) == "not-ground-anchored"
    assert score.score_cluster(c)["tree"] > 0.0
    assert score.classify_cluster(c)[0] == "tree"


# --- confidence contract -------------------------------------------------

def _all_test_clusters():
    from landmark_loc.ab_compare import load_fixture_percepts
    out = list(load_fixture_percepts())
    out += [_pts_cluster(_lamp_post()), _pts_cluster(_box(0.68, 0.38, 1.04)),
            _pts_cluster(_box(1.78, 0.80, 0.9)), _pts_cluster(_box(3.0, 1.32, 1.05)),
            _canopy_cluster()]
    return out


def test_confidence_is_in_unit_interval():
    for c in _all_test_clusters():
        ident, conf = score.classify_cluster(c)
        assert 0.0 <= conf <= 1.0, "%s scored %r" % (ident, conf)
        for name, s in score.score_cluster(c).items():
            assert 0.0 <= s <= 1.0, "%s scored %r" % (name, s)


def test_below_floor_becomes_unknown(monkeypatch):
    """Raising a type's floor above its score must flip it to unknown, and the
    returned score is the (rejected) best score, not 0 -- that number is what
    the RViz label and [diag] surface, so a too-tight floor stays visible."""
    c = _pts_cluster(_box(1.78, 0.80, 0.9))
    ident, conf = score.classify_cluster(c)
    assert ident == "bench" and conf > 0.0

    raised = tuple(
        t.__class__(**{**t.__dict__, "score_floor": 1.01}) if t.identity == "bench" else t
        for t in score.SCOREABLE)
    monkeypatch.setattr(score, "SCOREABLE", raised)
    ident2, conf2 = score.classify_cluster(c)
    assert ident2 == "unknown"
    assert conf2 == pytest.approx(conf)


def test_every_scoreable_type_declares_its_scoring_data():
    """The registry is the single source of the per-type numbers; a type that
    opts into scoring without them would fail only at classification time."""
    for t in score.SCOREABLE:
        assert t.score_family in ("band", "profile")
        assert 0.0 <= t.score_floor <= 1.0
        if t.score_family == "band":
            assert set(t.score_bands) == {"major", "height", "aspect"}
            for _q, (_centre, half) in t.score_bands.items():
                assert half > 0
        else:
            assert t.score_margin > 0


def test_scoreable_covers_exactly_the_catalog_types():
    """Every catalog identity must be scoreable, or the score detector would
    silently be unable to ever emit it."""
    catalog = {t.identity for t in PARK_TYPES if t.is_catalog}
    assert {t.identity for t in score.SCOREABLE} == catalog


# --- the captured-cluster oracle, against the SCORER ---------------------

def _fixture_clusters():
    from landmark_loc.ab_compare import load_fixture_percepts
    return load_fixture_percepts()


@pytest.mark.parametrize("i,expected", sorted(
    {0: "lamp", 4: "lamp", 10: "lamp", 11: "lamp", 14: "lamp", 12: "bench"}.items()))
def test_captured_cluster_scored_correctly(i, expected):
    assert score.classify_cluster(_fixture_clusters()[i])[0] == expected


@pytest.mark.parametrize("i", (1, 3, 9))
def test_captured_fragment_not_phantom_furniture(i):
    got = score.classify_cluster(_fixture_clusters()[i])[0]
    assert got not in {"lamp", "trash_bin_1", "bench", "garden_table"}


def test_scorer_unknown_rate_matches_cascade():
    """The unknown rate is a SAFETY metric: a scorer that rejects FEWER
    percepts than the cascade is emitting candidate phantom landmarks."""
    labels = [score.classify_cluster(c)[0] for c in _fixture_clusters()]
    assert sum(1 for l in labels if l == "unknown") == 9


# --- A3: confidence surfaced to the operator -----------------------------
# These cover the pure formatting/plumbing helpers, which import cleanly
# without rospy (localizer_node imports rospy lazily, inside functions).

def test_marker_text_omits_confidence_when_there_is_none():
    """The cascade's constant 1.0 is not evidence, so it must not be shown."""
    from landmark_loc import localizer_node as ln
    assert ln.marker_text("bench", None) == "bench"
    assert ln.marker_text("unknown", None) == "unknown"


def test_marker_text_shows_confidence_when_scored():
    from landmark_loc import localizer_node as ln
    assert ln.marker_text("bench", 0.8244) == "bench 0.82"
    # a rejected percept still shows how close it came
    assert ln.marker_text("unknown", 0.04) == "unknown 0.04"


def test_cluster_confidences_is_none_for_the_cascade():
    from landmark_loc import localizer_node as ln
    from landmark_loc import detector
    percepts = _fixture_clusters()
    det = detector.get_detector("cascade")
    labels, obs = det.detect(percepts)
    assert ln.cluster_confidences(labels, obs, "cascade") is None
    assert ln.confidence_summary(obs, "cascade") == ""


def test_cluster_confidences_aligns_scores_with_their_clusters():
    from landmark_loc import localizer_node as ln
    from landmark_loc import detector
    percepts = _fixture_clusters()
    det = detector.get_detector("score")
    labels, obs = det.detect(percepts)
    confs = ln.cluster_confidences(labels, obs, "score")
    assert len(confs) == len(percepts)
    for i, (lab, conf) in enumerate(zip(labels, confs)):
        if lab == "unknown":
            assert conf is None
        else:
            # the confidence shown for cluster i must be cluster i's own score
            assert conf == pytest.approx(score.classify_cluster(percepts[i])[1])


def test_confidence_summary_reports_each_detection():
    from landmark_loc import localizer_node as ln
    from landmark_loc import detector
    _labels, obs = detector.get_detector("score").detect(_fixture_clusters())
    line = ln.confidence_summary(obs, "score")
    assert line.startswith(" conf=[")
    # one "identity:score" entry per emitted observation ...
    assert line.count(":") == len(obs)
    # ... and the weakest one called out separately
    assert "min=%.2f" % min(o.confidence for o in obs) in line


def test_score_detector_observations_carry_real_confidence():
    from landmark_loc import detector
    _labels, obs = detector.get_detector("score").detect(_fixture_clusters())
    assert obs, "expected some accepted observations"
    assert all(0.0 < o.confidence <= 1.0 for o in obs)
    # and they are NOT the cascade's constant 1.0
    assert any(o.confidence < 1.0 for o in obs)


def test_cascade_observations_keep_constant_confidence():
    from landmark_loc import detector
    _labels, obs = detector.get_detector("cascade").detect(_fixture_clusters())
    assert all(o.confidence == 1.0 for o in obs)
