LAUNCH = "/home/thinh/Documents/Husky_viz/launch/move_base_landmark.launch"
RUNBOOK = "/home/thinh/Documents/Husky_viz/RUN-MAP-NAV.md"


def test_launch_is_clean_move_base_mirror():
    txt = open(LAUNCH).read()
    assert 'type="move_base"' in txt
    assert 'type="map_server"' in txt
    # landmark mode must NOT start navsat_transform
    assert "navsat_transform" not in txt
    # the localizer is a loose python node run from the runbook, NOT a <node> here
    assert "localizer_node.py" not in txt


def test_runbook_starts_localizer():
    txt = open(RUNBOOK).read()
    # the runbook (Option B) carries the localizer command
    assert "localizer_node.py" in txt


def test_runbook_offers_both_modes():
    txt = open(RUNBOOK).read()
    assert "move_base_gps_map.launch" in txt
    assert "move_base_landmark.launch" in txt
