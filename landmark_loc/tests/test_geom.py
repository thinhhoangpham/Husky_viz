import math
from landmark_loc.geom import rigid_transform_2d


def test_identity_transform():
    src = [[0, 0], [1, 0], [0, 1]]
    tx, ty, yaw, rms = rigid_transform_2d(src, src)
    assert abs(tx) < 1e-9 and abs(ty) < 1e-9 and abs(yaw) < 1e-9 and rms < 1e-9


def test_pure_translation():
    src = [[0, 0], [1, 0], [0, 1]]
    dst = [[2, 3], [3, 3], [2, 4]]
    tx, ty, yaw, rms = rigid_transform_2d(src, dst)
    assert abs(tx - 2) < 1e-6 and abs(ty - 3) < 1e-6 and abs(yaw) < 1e-6 and rms < 1e-6


def test_rotation_90():
    src = [[1, 0], [0, 1], [-1, 0]]
    dst = [[0, 1], [-1, 0], [0, -1]]   # rotate +90deg about origin
    tx, ty, yaw, rms = rigid_transform_2d(src, dst)
    assert abs(yaw - math.pi / 2) < 1e-6 and rms < 1e-6
