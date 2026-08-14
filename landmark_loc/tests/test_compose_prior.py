import math
from landmark_loc.localizer_node import compose_prior


def _close(a, b, tol=1e-9):
    return abs(a - b) <= tol


def test_zero_motion_position_matches_anchor():
    anchor = (10.0, -5.0, 0.7)
    x, y, yaw = compose_prior(anchor, (3.0, 3.0, 0.2), (3.0, 3.0, 0.2), 1.1)
    assert _close(x, 10.0) and _close(y, -5.0) and _close(yaw, 1.1)


def test_pure_translation_anchor_yaw_zero():
    # anchor yaw 0 -> odom displacement applies directly in map axes
    anchor = (10.0, -5.0, 0.0)
    x, y, yaw = compose_prior(anchor, (0.0, 0.0, 0.0), (2.0, 1.0, 0.0), 0.0)
    assert _close(x, 12.0) and _close(y, -4.0) and _close(yaw, 0.0)


def test_pure_translation_anchor_yaw_90deg():
    # anchor facing +90deg: odom +x (forward) maps to map +y
    anchor = (0.0, 0.0, math.pi / 2)
    x, y, yaw = compose_prior(anchor, (0.0, 0.0, 0.0), (2.0, 0.0, 0.0), math.pi / 2)
    assert _close(x, 0.0, 1e-9) and _close(y, 2.0) and _close(yaw, math.pi / 2)


def test_heading_comes_from_compass_not_odom_rotation():
    # Same anchor/odom-baseline, odom_now yaw changes but heading_yaw (compass)
    # is fixed -> position ignores odom yaw (rotation only affects heading via
    # the anchor rotation baseline, not the returned pyaw), and pyaw must equal
    # heading_yaw exactly, independent of odom yaw.
    anchor = (4.0, 7.0, 0.3)
    compass_heading = 0.8
    x1, y1, yaw1 = compose_prior(anchor, (1.0, 1.0, 0.0), (1.0, 1.0, 0.5), compass_heading)
    x2, y2, yaw2 = compose_prior(anchor, (1.0, 1.0, 0.0), (1.0, 1.0, -1.7), compass_heading)
    assert _close(yaw1, compass_heading)
    assert _close(yaw2, compass_heading)
    assert _close(yaw1, yaw2)
    # position is unaffected by odom yaw here since odom position didn't move
    assert _close(x1, 4.0) and _close(y1, 7.0)
    assert _close(x2, 4.0) and _close(y2, 7.0)


def test_combined_matches_hand_computed():
    # anchor at (0,0,0); odom moves from (0,0,0) to (1,2, pi/2)
    anchor = (0.0, 0.0, 0.0)
    x, y, yaw = compose_prior(anchor, (0.0, 0.0, 0.0), (1.0, 2.0, math.pi / 2), 2.4)
    assert _close(x, 1.0) and _close(y, 2.0) and _close(yaw, 2.4)


def test_anchor_offset_and_odom_offset():
    # odom baseline non-zero: only the DELTA since baseline matters (position);
    # heading is the passed compass yaw regardless.
    anchor = (100.0, 200.0, 0.0)
    x, y, yaw = compose_prior(anchor, (5.0, 5.0, 0.0), (8.0, 9.0, 0.0), -1.0)
    assert _close(x, 103.0) and _close(y, 204.0) and _close(yaw, -1.0)
