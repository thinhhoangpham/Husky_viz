"""Step 1 acceptance: the detector seam is behavior-neutral.

The 'cascade' detector must be a pure indirection over the pre-existing
classify.classify_cluster / classify.to_observations logic -- identical
labels and identical observations on the captured fixture clusters.
"""
import pytest
from landmark_loc import classify, detector
from landmark_loc.tests.test_captured_regression import _load


@pytest.fixture(scope="module")
def captured():
    clusters, _ = _load()
    return [clusters[i] for i in sorted(clusters)]


def test_default_detector_is_cascade():
    det = detector.get_detector()
    assert det.name == "cascade"
    assert detector.DEFAULT_DETECTOR == "cascade"


def test_unknown_mode_is_not_silently_accepted():
    with pytest.raises(KeyError):
        detector.get_detector("no_such_classifier")


def test_cascade_labels_match_legacy_cascade(captured):
    det = detector.get_detector("cascade")
    for c in captured:
        assert det.label(c) == classify.classify_cluster(c)


def test_cascade_observations_match_legacy(captured):
    det = detector.get_detector("cascade")
    assert det.observe(captured) == classify.to_observations(captured)


def test_cascade_rejects_unknown_percepts(captured):
    """`unknown` is first-class: rejected percepts are DROPPED, not emitted."""
    det = detector.get_detector("cascade")
    labels = [det.label(c) for c in captured]
    assert "unknown" in labels
    kept = [l for l in labels if l != "unknown"]
    obs = det.observe(captured)
    assert [o.identity for o in obs] == kept


def test_observation_carries_confidence(captured):
    det = detector.get_detector("cascade")
    obs = det.observe(captured)
    assert obs, "expected at least one accepted observation"
    for o in obs:
        assert o.confidence == 1.0


def test_observation_confidence_defaults_and_is_last_positional():
    o = classify.Observation("lamp", 1.0, 2.0)
    assert o.confidence == 1.0
    assert classify.Observation("lamp", 1.0, 2.0, 0.5, 0.25).confidence == 0.25


# --- Step 1b: frame/stamp on the contract, and one classification pass ---

def test_observations_carry_caller_supplied_frame_and_stamp(captured):
    det = detector.get_detector("cascade")
    obs = det.observe(captured, frame_id="os0_lidar", stamp=1234.5)
    assert obs, "expected at least one accepted observation"
    for o in obs:
        assert o.frame_id == "os0_lidar"
        assert o.stamp == 1234.5


def test_frame_and_stamp_default_to_unstated(captured):
    """Omitting them must stay legal -- they are additive, not required."""
    for o in detector.get_detector("cascade").observe(captured):
        assert o.frame_id is None and o.stamp is None


def test_frame_and_stamp_are_last_positional():
    """Existing positional construction must keep working unchanged."""
    o = classify.Observation("lamp", 1.0, 2.0)
    assert (o.yaw, o.confidence, o.frame_id, o.stamp) == (None, 1.0, None, None)
    o = classify.Observation("lamp", 1.0, 2.0, 0.5, 0.25, "cam", 7.0)
    assert o.yaw == 0.5 and o.confidence == 0.25
    assert o.frame_id == "cam" and o.stamp == 7.0


def test_detect_labels_every_percept_but_observes_only_accepted(captured):
    det = detector.get_detector("cascade")
    labels, obs = det.detect(captured, frame_id="os0_lidar", stamp=1.0)
    assert len(labels) == len(captured)
    assert "unknown" in labels
    assert [o.identity for o in obs] == [l for l in labels if l != "unknown"]
    assert len(obs) < len(labels)


def test_detect_single_pass_matches_two_call_path(captured):
    """The one-pass call must be IDENTICAL to label()-then-observe()."""
    det = detector.get_detector("cascade")
    labels, obs = det.detect(captured, frame_id="os0_lidar", stamp=9.5)
    assert labels == [det.label(c) for c in captured]
    assert obs == det.observe(captured, frame_id="os0_lidar", stamp=9.5)


def test_detect_classifies_each_percept_once(captured):
    """Guards the double-inference fix: a future ML detector must not pay twice."""
    det = detector.get_detector("cascade")
    calls = []
    real = classify.classify_cluster

    def counting(cluster, margins=classify.DEFAULT_MARGINS):
        calls.append(cluster)
        return real(cluster, margins)

    orig = classify.classify_cluster
    classify.classify_cluster = counting
    try:
        det.detect(captured)
    finally:
        classify.classify_cluster = orig
    assert len(calls) == len(captured)


def test_markers_from_precomputed_labels_match_old_path(captured):
    """Marker output must be byte-identical in content to the old det.label loop."""
    rospy = pytest.importorskip("rospy")
    pytest.importorskip("visualization_msgs.msg")
    from landmark_loc import localizer_node

    det = detector.get_detector("cascade")
    stamp = rospy.Time(12, 500000000)
    labels, _ = det.detect(captured, frame_id="os0_lidar", stamp=stamp.to_sec())
    new = localizer_node.build_observed_markers(captured, "os0_lidar", stamp, labels)
    old = localizer_node.build_observed_markers(
        captured, "os0_lidar", stamp, [det.label(c) for c in captured])
    assert len(new.markers) == len(captured) + 1  # + the DELETEALL
    assert [str(m) for m in new.markers] == [str(m) for m in old.markers]
    # and the labels shown are still one per cluster, unknowns included
    assert [m.text for m in new.markers[1:]] == labels
