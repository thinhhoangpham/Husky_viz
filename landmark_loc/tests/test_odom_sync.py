import math

from landmark_loc.localizer_node import odom_at


def test_empty_buffer_returns_none():
    assert odom_at([], 5.0) is None


def test_t_before_first_sample_clamps_to_first():
    buf = [(1.0, 10.0, 20.0, 0.5), (2.0, 11.0, 21.0, 0.6)]
    assert odom_at(buf, 0.0) == (10.0, 20.0, 0.5)


def test_t_after_last_sample_clamps_to_last():
    buf = [(1.0, 10.0, 20.0, 0.5), (2.0, 11.0, 21.0, 0.6)]
    assert odom_at(buf, 5.0) == (11.0, 21.0, 0.6)


def test_t_between_samples_interpolates_linearly():
    buf = [(1.0, 0.0, 0.0, 0.0), (2.0, 10.0, 20.0, 0.0)]
    x, y, yaw = odom_at(buf, 1.5)
    assert x == 5.0
    assert y == 10.0
    assert yaw == 0.0


def test_yaw_interpolation_wraps_short_way():
    buf = [(1.0, 0.0, 0.0, 3.0), (2.0, 0.0, 0.0, -3.0)]
    _, _, yaw = odom_at(buf, 1.5)
    # shortest path from 3.0 to -3.0 wraps through +-pi, not through 0
    assert abs(abs(yaw) - math.pi) < 0.05
