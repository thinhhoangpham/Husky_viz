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


def test_should_reanchor_rejects_low_count():
    assert ln.should_reanchor(3, (10.0, 10.0), (10.0, 10.0), jump_max=5.0) is False


def test_should_reanchor_rejects_big_jump():
    assert ln.should_reanchor(4, (20.0, 0.0), (0.0, 0.0), jump_max=5.0) is False


def test_should_reanchor_accepts_confident_close_fix():
    assert ln.should_reanchor(4, (2.0, 1.0), (0.0, 0.0), jump_max=5.0) is True


def test_should_reanchor_jump_boundary_is_inclusive():
    assert ln.should_reanchor(6, (5.0, 0.0), (0.0, 0.0), jump_max=5.0) is True
    assert ln.should_reanchor(6, (5.0001, 0.0), (0.0, 0.0), jump_max=5.0) is False
