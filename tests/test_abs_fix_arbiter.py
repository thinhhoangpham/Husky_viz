import pytest
from landmark_loc.abs_fix_selector import AbsFixArbiter, SOURCES, TOPIC_TO_NAME


def test_defaults_to_gps():
    a = AbsFixArbiter()
    assert a.selected_name == "gps"
    assert a.should_forward("gps") is True
    assert a.should_forward("landmark") is False


def test_topic_maps_are_consistent():
    assert SOURCES["gps"] == "/odometry/gps_fix"
    assert SOURCES["landmark"] == "/odometry/landmark_fix"
    assert TOPIC_TO_NAME["/odometry/gps_fix"] == "gps"
    assert TOPIC_TO_NAME["/odometry/landmark_fix"] == "landmark"


def test_select_switches_and_returns_previous():
    a = AbsFixArbiter()
    prev = a.select("landmark")
    assert prev == "gps"
    assert a.selected_name == "landmark"
    assert a.should_forward("landmark") is True
    assert a.should_forward("gps") is False


def test_select_unknown_is_rejected_and_state_unchanged():
    a = AbsFixArbiter()
    assert a.select("galileo") is None
    assert a.selected_name == "gps"


def test_only_selected_source_forwards():
    a = AbsFixArbiter(initial="gps")
    assert a.should_forward("gps") is True
    assert a.should_forward("landmark") is False
    a.select("landmark")
    assert a.should_forward("gps") is False
    assert a.should_forward("landmark") is True


def test_status_fresh_when_selected_recently_published():
    a = AbsFixArbiter(stale_timeout=2.0)
    a.note_message("gps", now=100.0)
    assert a.status(now=101.0) == "gps"


def test_status_stale_after_timeout_on_selected():
    a = AbsFixArbiter(stale_timeout=2.0)
    a.note_message("gps", now=100.0)
    assert a.status(now=103.0) == "gps:stale"


def test_status_stale_when_selected_never_published():
    a = AbsFixArbiter(stale_timeout=2.0)
    assert a.status(now=100.0) == "gps:stale"


def test_unselected_source_silence_does_not_affect_status():
    a = AbsFixArbiter(stale_timeout=2.0)
    a.note_message("gps", now=100.0)        # selected source fresh
    # landmark (unselected) never publishes -> must NOT make status stale
    assert a.status(now=101.0) == "gps"


def test_stale_clears_when_selected_resumes():
    a = AbsFixArbiter(stale_timeout=2.0)
    a.note_message("gps", now=100.0)
    assert a.status(now=103.0) == "gps:stale"
    a.note_message("gps", now=103.5)
    assert a.status(now=104.0) == "gps"


def test_switching_uses_new_source_freshness():
    a = AbsFixArbiter(stale_timeout=2.0)
    a.note_message("landmark", now=50.0)     # landmark last seen long ago
    a.note_message("gps", now=100.0)
    a.select("landmark")                      # now selected: landmark, stale
    assert a.status(now=100.1) == "landmark:stale"
    a.note_message("landmark", now=100.2)
    assert a.status(now=100.3) == "landmark"
