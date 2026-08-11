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
