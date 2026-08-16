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


# --- region-anchor wiring helpers (T20) ------------------------------------

import numpy as np


def test_update_anchor_region_fix_wins():
    # A region fix (pole_sighting) wins over a confirmed waypoint and hold.
    xy, src = ln._update_anchor((1.0, 1.0), (5.0, 6.0), (9.0, 9.0))
    assert xy == (5.0, 6.0) and src == "pole"


def test_update_anchor_confirmed_waypoint_when_no_region_fix():
    xy, src = ln._update_anchor((1.0, 1.0), None, (9.0, 9.0))
    assert xy == (9.0, 9.0) and src == "waypoint"


def test_update_anchor_holds_prev_when_nothing():
    xy, src = ln._update_anchor((1.0, 1.0), None, None)
    assert xy == (1.0, 1.0) and src == "hold"


def test_cloud_to_map_frame_known_prior_and_point():
    # Prior at (10, 20) heading 90deg (pi/2). A point 1m in front of the robot
    # (base-frame x=1,y=0) rotates to map +y, so map = (10, 21). A point to the
    # robot's left (x=0,y=1) rotates to map -x, so map = (9, 20). z passes
    # through unchanged.
    prior = (10.0, 20.0, np.pi / 2)
    pts = np.array([[1.0, 0.0, 2.5],
                    [0.0, 1.0, -0.3]], dtype=float)
    out = ln.cloud_to_map_frame(pts, prior)
    expected = np.array([[10.0, 21.0, 2.5],
                         [9.0, 20.0, -0.3]], dtype=float)
    assert np.allclose(out, expected)


def test_cloud_to_map_frame_empty():
    out = ln.cloud_to_map_frame(np.empty((0, 3)), (1.0, 2.0, 0.5))
    assert out.shape == (0, 3)
