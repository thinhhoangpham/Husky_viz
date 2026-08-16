"""Step 2 acceptance: the A/B harness itself is sound.

Two properties are being proved here, and they pull in opposite directions:
  (1) IDENTITY -- comparing a detector with itself must report ZERO
      disagreements and zero position delta. If it reported spurious
      difference, every future experiment would be noise.
  (2) SENSITIVITY -- a stub detector that deliberately differs on chosen
      percepts must produce EXACTLY those disagreement rows. If it missed real
      difference, the harness would silently bless a regression.
"""
import pytest

from landmark_loc import ab_compare, classify, detector


@pytest.fixture(scope="module")
def percepts():
    return ab_compare.load_fixture_percepts()


def _cascade():
    return detector.get_detector("cascade")


def test_fixture_loads_the_fifteen_captured_clusters(percepts):
    assert len(percepts) == 15
    assert all(p.points is not None and len(p.points) for p in percepts)


# --- (1) identity case: the harness reports no difference where none exists --

def test_identical_detectors_report_zero_disagreements(percepts):
    cmp = ab_compare.compare(percepts, _cascade(), _cascade())
    assert cmp.total == 15
    assert cmp.disagreements == []
    assert cmp.agreements == 15


def test_identical_detectors_report_zero_position_delta(percepts):
    cmp = ab_compare.compare(percepts, _cascade(), _cascade())
    # every accepted percept is compared, not silently skipped
    assert len(cmp.pos_deltas) == 15 - cmp.unknown_count("a")
    assert cmp.max_pos_delta == 0.0
    assert cmp.mean_pos_delta == 0.0


def test_summary_counts_are_consistent(percepts):
    cmp = ab_compare.compare(percepts, _cascade(), _cascade())
    assert cmp.agreements + len(cmp.disagreements) == cmp.total
    assert sum(cmp.label_counts("a").values()) == cmp.total
    assert sum(cmp.label_counts("b").values()) == cmp.total


def test_unknown_rate_matches_known_cascade_baseline(percepts):
    """9/15 unknown is the captured-regression baseline; the harness must agree."""
    cmp = ab_compare.compare(percepts, _cascade(), _cascade())
    assert cmp.unknown_count("a") == 9
    assert cmp.unknown_count("b") == 9
    assert cmp.unknown_delta == 0


def test_label_counts_match_the_raw_classifier(percepts):
    cmp = ab_compare.compare(percepts, _cascade(), _cascade())
    expected = {}
    for p in percepts:
        lab = classify.classify_cluster(p)
        expected[lab] = expected.get(lab, 0) + 1
    assert cmp.label_counts("a") == expected


def test_rows_carry_confidence_for_accepted_and_none_for_rejected(percepts):
    cmp = ab_compare.compare(percepts, _cascade(), _cascade())
    for r in cmp.rows:
        if r.label_a == ab_compare.UNKNOWN:
            assert r.conf_a is None
        else:
            assert r.conf_a == 1.0


# --- (2) sensitivity: a stub that differs must show up exactly ---------------

class _StubDetector(object):
    """A detector defined HERE, deliberately NOT registered in DETECTORS.

    `compare()` takes instances, not names, precisely so an experimental
    implementation can be A/B'd before (or without ever) being registered.
    """

    name = "stub"

    def __init__(self, overrides):
        #: percept index -> label to force (including "unknown")
        self.overrides = overrides
        self._inner = detector.get_detector("cascade")

    def detect(self, percepts, frame_id=None, stamp=None):
        labels = [self.overrides.get(i, self._inner.label(p))
                  for i, p in enumerate(percepts)]
        obs = classify.to_observations(percepts, frame_id=frame_id, stamp=stamp,
                                       labels=labels)
        return labels, obs


def test_stub_disagreements_are_exactly_the_forced_rows(percepts):
    base = [classify.classify_cluster(p) for p in percepts]
    # flip one accepted percept to a different type, and one to unknown
    accepted = [i for i, l in enumerate(base) if l != ab_compare.UNKNOWN]
    rejected = [i for i, l in enumerate(base) if l == ab_compare.UNKNOWN]
    flip, drop, promote = accepted[0], accepted[1], rejected[0]
    stub = _StubDetector({flip: "bench", drop: ab_compare.UNKNOWN,
                          promote: "lamp"})

    cmp = ab_compare.compare(percepts, _cascade(), stub)
    assert sorted(r.index for r in cmp.disagreements) == sorted(
        {flip, drop, promote})
    assert cmp.agreements == 15 - 3
    by_i = {r.index: r for r in cmp.rows}
    assert by_i[flip].label_a == base[flip] and by_i[flip].label_b == "bench"
    assert by_i[drop].label_b == ab_compare.UNKNOWN and by_i[drop].conf_b is None
    assert by_i[promote].label_a == ab_compare.UNKNOWN
    assert by_i[promote].conf_a is None and by_i[promote].conf_b == 1.0


def test_disagreement_rows_carry_feature_context(percepts):
    base = [classify.classify_cluster(p) for p in percepts]
    flip = next(i for i, l in enumerate(base) if l != ab_compare.UNKNOWN)
    cmp = ab_compare.compare(percepts, _cascade(), _StubDetector({flip: "bench"}))
    row = cmp.disagreements[0]
    assert set(row.features) == {"n", "foot_major", "foot_minor", "aspect",
                                 "height", "z_min"}
    assert row.features["n"] == len(percepts[flip].points)
    assert row.features["aspect"] == pytest.approx(
        row.features["foot_major"] / max(row.features["foot_minor"], 1e-6))
    # and it reaches the printed table
    table = ab_compare.format_table(cmp)
    assert "foot_major" in table and "DISAGREE" in table


def test_agreement_rows_carry_no_feature_context(percepts):
    cmp = ab_compare.compare(percepts, _cascade(), _cascade())
    assert all(r.features == {} for r in cmp.rows)


def test_unknown_rate_delta_flags_a_less_conservative_b(percepts):
    """B accepting what A rejected is a SAFETY regression and must be loud."""
    base = [classify.classify_cluster(p) for p in percepts]
    promote = [i for i, l in enumerate(base) if l == ab_compare.UNKNOWN][:2]
    cmp = ab_compare.compare(percepts, _cascade(),
                             _StubDetector({i: "lamp" for i in promote}))
    assert cmp.unknown_delta == -2
    assert "REJECTS FEWER" in ab_compare.format_summary(cmp)


def test_more_conservative_b_is_not_flagged_as_a_safety_warning(percepts):
    base = [classify.classify_cluster(p) for p in percepts]
    drop = next(i for i, l in enumerate(base) if l != ab_compare.UNKNOWN)
    cmp = ab_compare.compare(percepts, _cascade(),
                             _StubDetector({drop: ab_compare.UNKNOWN}))
    assert cmp.unknown_delta == 1
    assert "REJECTS FEWER" not in ab_compare.format_summary(cmp)


# --- position delta ---------------------------------------------------------

class _ShiftedDetector(object):
    """Same labels as the cascade, but every position shifted by `dx`."""

    name = "shifted"

    def __init__(self, dx):
        self.dx = dx
        self._inner = detector.get_detector("cascade")

    def detect(self, percepts, frame_id=None, stamp=None):
        labels, obs = self._inner.detect(percepts, frame_id=frame_id, stamp=stamp)
        return labels, [classify.Observation(o.identity, o.x + self.dx, o.y,
                                             o.yaw, o.confidence, o.frame_id,
                                             o.stamp) for o in obs]


def test_position_delta_is_reported_for_same_label_percepts(percepts):
    cmp = ab_compare.compare(percepts, _cascade(), _ShiftedDetector(0.25))
    assert cmp.disagreements == []          # labels identical
    assert cmp.max_pos_delta == pytest.approx(0.25)
    assert cmp.mean_pos_delta == pytest.approx(0.25)
    assert "0.250000" in ab_compare.format_summary(cmp)


def test_position_delta_only_where_both_accepted_same_label(percepts):
    base = [classify.classify_cluster(p) for p in percepts]
    flip = next(i for i, l in enumerate(base) if l != ab_compare.UNKNOWN)
    cmp = ab_compare.compare(percepts, _cascade(), _StubDetector({flip: "bench"}))
    assert {r.index for r in cmp.rows if r.pos_delta is not None} == {
        i for i, l in enumerate(base) if l != ab_compare.UNKNOWN} - {flip}


# --- report / CLI plumbing --------------------------------------------------

def test_compare_rejects_a_detector_that_skips_percepts(percepts):
    class _Bad(object):
        name = "bad"

        def detect(self, ps, frame_id=None, stamp=None):
            return ["unknown"], []

    with pytest.raises(ValueError):
        ab_compare.compare(percepts, _cascade(), _Bad())


def test_report_contains_every_percept_and_the_unknown_label(percepts):
    cmp = ab_compare.compare(percepts, _cascade(), _cascade())
    report = ab_compare.format_report(cmp)
    assert ab_compare.UNKNOWN in report, "unknown must be visible in the table"
    for r in cmp.rows:
        assert ("\n%4d |" % r.index) in "\n" + report
    assert "UNKNOWN RATE" in report and "9/15" in report


def test_cli_runs_on_the_fixture_without_ros(capsys):
    assert ab_compare.main(["--a", "cascade", "--b", "cascade"]) == 0
    out = capsys.readouterr().out
    assert "disagreements: 0" in out and "9/15" in out


def test_cli_rejects_an_unregistered_detector_name():
    with pytest.raises(SystemExit):
        ab_compare.main(["--a", "cascade", "--b", "no_such_detector"])


def test_cli_json_output_is_machine_readable(capsys):
    import json
    assert ab_compare.main(["--json"]) == 0
    data = json.loads(capsys.readouterr().out)
    assert data["total"] == 15 and data["disagreements"] == 0
    assert data["unknown_a"] == 9 and len(data["rows"]) == 15


def test_harness_is_ros_free():
    """The harness path must import cleanly with no rospy anywhere."""
    import sys
    for mod in ("landmark_loc.ab_compare", "landmark_loc.detector",
                "landmark_loc.classify", "landmark_loc.segment",
                "landmark_loc.shapefeat"):
        __import__(mod)
        assert "rospy" not in sys.modules.get(mod).__dict__


def test_harness_is_generic_over_the_registry(percepts):
    """Nothing here may be hardcoded to a specific detector name."""
    for name, cls in detector.DETECTORS.items():
        # The A/B harness compares PERCEPT detectors. A region-based anchor
        # detector (percept_based=False) has a different contract
        # (match(cloud, prior_xy)) and cannot be constructed argument-free, so
        # it is not part of this harness -- skip it. See detector.Detector.
        if not getattr(cls, "percept_based", True):
            continue
        cmp = ab_compare.compare(percepts, detector.get_detector(name),
                                 detector.get_detector(name))
        assert cmp.total == len(percepts)
        assert cmp.disagreements == []
