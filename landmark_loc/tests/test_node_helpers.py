from landmark_loc import localizer_node as ln


def test_covariance_shrinks_with_matches():
    c2 = ln.covariance_for(2, base_var=1.0)
    c6 = ln.covariance_for(6, base_var=1.0)
    # index 0 = x var, index 7 = y var (row-major 6x6)
    assert c6[0] < c2[0] and c6[7] < c2[7]
    assert len(c2) == 36


def test_covariance_marks_orientation_unfused_large():
    c = ln.covariance_for(4, base_var=1.0)
    # yaw variance (index 35) must be large (orientation not fused from here)
    assert c[35] >= 1e3


def test_is_landmark_mode_dormant_on_gps_and_none():
    assert ln._is_landmark_mode("gps") is False
    assert ln._is_landmark_mode(None) is False
    assert ln._is_landmark_mode("") is False


def test_is_landmark_mode_active_on_landmark_and_stale():
    assert ln._is_landmark_mode("landmark") is True
    assert ln._is_landmark_mode("landmark:stale") is True


from landmark_loc.localizer_node import _jump_ok


def test_jump_bootstrap_accepts():
    assert _jump_ok((5.0, 0.0), None, (0.0, 0.0), 3.0) is True


def test_jump_within_reach_accepts():
    # last pub (10,0); odom moved (-2,0); expected (8,0); fix (8.3,0.1) close -> ok
    assert _jump_ok((8.3, 0.1), (10.0, 0.0), (-2.0, 0.0), 3.0) is True


def test_backward_teleport_rejected():
    # last pub (10,0); odom moved (-2,0) forward; expected ~(8,0);
    # fix jumps BACKWARD to (18,0) -> 10m from expected -> reject
    assert _jump_ok((18.0, 0.0), (10.0, 0.0), (-2.0, 0.0), 3.0) is False
