LAUNCH = "/home/thinh/Documents/Husky_viz/launch/move_base_landmark.launch"
RUNBOOK = "/home/thinh/Documents/Husky_viz/RUN-MAP-NAV.md"


def test_launch_starts_localizer_and_move_base():
    txt = open(LAUNCH).read()
    assert "localizer_node.py" in txt
    assert 'type="move_base"' in txt
    assert 'type="map_server"' in txt
    # landmark mode must NOT start navsat_transform
    assert "navsat_transform" not in txt


def test_runbook_offers_both_modes():
    txt = open(RUNBOOK).read()
    assert "move_base_gps_map.launch" in txt
    assert "move_base_landmark.launch" in txt
