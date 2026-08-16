from landmark_loc.waypoint_anchor import confirm_arrival, choose_anchor, fault_offset


def test_arrival_confirmed_when_expected_pole_seen():
    assert confirm_arrival((10.0, 5.0), {"pole_A"},
                           [("pole_A", 10.3, 5.1)], radius=1.0) is True


def test_arrival_rejected_when_pole_absent_or_far():
    assert confirm_arrival((10.0, 5.0), {"pole_A"}, [], radius=1.0) is False
    assert confirm_arrival((10.0, 5.0), {"pole_A"},
                           [("pole_A", 30.0, 5.0)], radius=1.0) is False


def test_pole_beats_waypoint_beats_hold():
    assert choose_anchor((0, 0), (7, 7), (3, 3)) == ((7, 7), "pole")
    assert choose_anchor((0, 0), None, (3, 3)) == ((3, 3), "waypoint")
    assert choose_anchor((0, 0), None, None) == ((0, 0), "hold")


def test_fault_offset():
    assert abs(fault_offset((0, 0), (3, 4)) - 5.0) < 1e-9
