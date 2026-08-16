import numpy as np
from landmark_loc.descriptor import voxel_shape, describe, descriptor_distance


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


def _lattice_pole(seed, height=16.0):
    # Criss-crossing thin members over 0..height: many linear voxels.
    r = _rng(seed)
    zs = r.uniform(0, height, 4000)
    xs = r.choice([-0.25, 0.25], 4000) + r.randn(4000) * 0.02
    ys = r.randn(4000) * 0.02
    a = np.column_stack([xs, ys, zs])
    b = np.column_stack([ys, xs, zs])  # members in the other orientation
    return np.vstack([a, b])


def _bench():
    r = _rng(9)
    xs = r.uniform(-0.9, 0.9, 800)
    ys = r.uniform(-0.4, 0.4, 800)
    zs = r.uniform(0.0, 0.94, 800)
    return np.column_stack([xs, ys, zs])


def test_describe_shape_and_bands():
    d = describe(_lattice_pole(3))
    assert d.shape == (18, 4)
    # a tall pole has non-empty high bands; a bench would not
    assert d[14, :3].sum() > 0  # ~14-15 m band has material


def test_pole_far_from_bench():
    dp = describe(_lattice_pole(3))
    db = describe(_bench())
    dpp = describe(_lattice_pole(4))  # different sampling of the same shape
    # same-shape distance must be much smaller than cross-shape distance
    assert descriptor_distance(dp, dpp) < descriptor_distance(dp, db)
    assert descriptor_distance(dp, db) > 1.0


def test_empty_bands_are_zero():
    d = describe(_bench())
    # bench has no material above ~1 m -> bands 2..17 all zero
    assert np.allclose(d[2:], 0.0)


def test_partial_view_matches_full():
    full = _lattice_pole(5)
    # keep only the near HALF (y < 0): a single-face view
    partial = full[full[:, 1] < 0]
    df = describe(full)
    dp = describe(partial)
    db = describe(_bench())
    # partial view still much closer to its full self than to a bench
    assert descriptor_distance(df, dp) < 0.5 * descriptor_distance(df, db)


def test_decimation_stable():
    full = _lattice_pole(6)
    decim = full[::4]  # 1/4 the density, standing in for range
    assert descriptor_distance(describe(full), describe(decim)) \
        < descriptor_distance(describe(full), describe(_bench()))


def _sparse_lattice_pole(seed, n_points=200, height=16.5):
    # Realistic-density stand-in: a real Ouster return on a 16.5 m pole at
    # ~15 m range is orders of magnitude sparser than the dense synthetic
    # cloud _lattice_pole() produces (thousands of points). This is a LOWER
    # BOUND sanity check on voxel/min_voxel_pts, not a substitute for
    # validation against real Ouster returns -- that happens later against
    # captured lidar data (see Task 13 in the implementation plan).
    dense = _lattice_pole(seed, height=height)
    idx = np.random.RandomState(seed + 100).choice(
        dense.shape[0], size=n_points, replace=False)
    return dense[idx]


def test_sparse_pole_still_closer_to_dense_pole_than_bench():
    dense = _lattice_pole(7)
    sparse = _sparse_lattice_pole(7, n_points=200)
    db = describe(_bench())
    dd = describe(dense)
    ds = describe(sparse)
    assert descriptor_distance(dd, ds) < descriptor_distance(dd, db)
