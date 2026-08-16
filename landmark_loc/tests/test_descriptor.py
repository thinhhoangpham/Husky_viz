import numpy as np
from landmark_loc.descriptor import voxel_shape


def _rng(seed):
    return np.random.RandomState(seed)


def test_linear_points_classify_linear():
    # A thin stick along x: large spread in x, tiny in y/z.
    p = _rng(0).randn(200, 3) * np.array([1.0, 0.01, 0.01])
    lin, pla, sph = voxel_shape(p)
    assert lin > 0.8
    assert lin > pla and lin > sph
    assert abs((lin + pla + sph) - 1.0) < 1e-6


def test_planar_points_classify_planar():
    # A sheet in x-y: spread in x and y, tiny in z.
    p = _rng(1).randn(200, 3) * np.array([1.0, 1.0, 0.01])
    lin, pla, sph = voxel_shape(p)
    assert pla > 0.5
    assert pla > lin and pla > sph


def test_isotropic_points_classify_spherical():
    p = _rng(2).randn(200, 3)
    lin, pla, sph = voxel_shape(p)
    assert sph > 0.4
    assert sph >= lin and sph >= pla


def test_degenerate_is_spherical_default():
    assert voxel_shape(np.zeros((2, 3))) == (0.0, 0.0, 1.0)
    assert voxel_shape(np.zeros((0, 3))) == (0.0, 0.0, 1.0)
