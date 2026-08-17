import math
import numpy as np
from landmark_loc import derotate


def test_level_quat_gives_zero_roll_pitch():
    # identity quaternion => no tilt
    r, p = derotate.roll_pitch_from_quat(0.0, 0.0, 0.0, 1.0)
    assert abs(r) < 1e-9
    assert abs(p) < 1e-9


def test_pure_pitch_recovered():
    # quaternion for +0.3 rad pitch about y
    half = 0.15
    qx, qy, qz, qw = 0.0, math.sin(half), 0.0, math.cos(half)
    r, p = derotate.roll_pitch_from_quat(qx, qy, qz, qw)
    assert abs(r) < 1e-6
    assert abs(p - 0.3) < 1e-6


def test_level_rotation_is_identity_when_level():
    R = derotate.level_rotation(0.0, 0.0)
    assert np.allclose(R, np.eye(3))


def test_level_rotation_flattens_a_tilted_ground_plane():
    # a ground plane tilted by 0.3 rad pitch: points on z = tan(0.3)*x in the
    # BODY frame. After de-rotation their z should be ~constant (flat).
    pitch = 0.3
    xs = np.linspace(-10, 10, 50)
    pts = np.zeros((50, 3))
    pts[:, 0] = xs
    pts[:, 2] = math.tan(pitch) * xs
    R = derotate.level_rotation(0.0, pitch)
    out = (R @ pts.T).T
    assert out[:, 2].std() < 1e-6  # z is now flat


def test_derotate_cloud_empty():
    out = derotate.derotate_cloud(np.zeros((0, 3)), 0.1, 0.2)
    assert out.shape == (0, 3)


def test_derotate_cloud_preserves_count():
    pts = np.random.RandomState(0).randn(17, 3)
    out = derotate.derotate_cloud(pts, 0.1, -0.2)
    assert out.shape == (17, 3)
