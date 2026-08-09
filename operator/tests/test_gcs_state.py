import pytest
from gcs_state import GcsState

def test_defaults():
    s = GcsState()
    assert s.mode == "AUTO"
    assert s.sent_goal is None
    assert s.active_goal is None
    assert s.nav_status == "NONE"
    assert s.estop_engaged is False

def test_set_mode_valid_and_invalid():
    s = GcsState()
    s.set_mode("MANUAL")
    assert s.mode == "MANUAL"
    with pytest.raises(ValueError):
        s.set_mode("FLYING")

def test_estop_engage_release():
    s = GcsState()
    s.engage_estop()
    assert s.estop_engaged is True and s.mode == "ESTOP"
    s.release_estop()
    assert s.estop_engaged is False and s.mode == "AUTO"
