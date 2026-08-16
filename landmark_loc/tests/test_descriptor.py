import numpy as np
from landmark_loc.descriptor import (
    voxel_shape, describe, descriptor_distance, window,
    describe_region, region_distance,
)


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


def test_window_selects_and_recenters():
    pts = np.array([[10.,10.,1.],[10.5,10.,1.],[30.,30.,1.]])
    w = window(pts, 10.0, 10.0, 2.0)
    assert len(w) == 2                       # third point is 28 m away
    assert abs(w[:,0].mean()) < 1.0 and abs(w[:,1].mean()) < 1.0   # recentred near origin
    assert np.allclose(w[:,2], 1.0)          # z untouched


# --- Region descriptor: horizontal arrangement -----------------------------
#
# The region descriptor's job is to separate two IDENTICAL structures that sit
# in DIFFERENT neighbourhoods -- something the vertical descriptor above cannot
# do, because it is translation-invariant within the window. The bounds below
# are pinned to MEASURED distances (recorded in describe_region's comments), not
# loose habit values, so a subtly-wrong arrangement grid cannot pass them.

def _tower(cx, cy, seed):
    # A 16 m lattice tower (like _lattice_pole but placeable) centred on (cx,cy).
    r = _rng(seed); z = r.uniform(0, 16, 3000)
    x = r.choice([-0.25, 0.25], 3000) + r.randn(3000) * 0.02 + cx
    y = r.randn(3000) * 0.02 + cy
    return np.column_stack([x, y, z])


def test_identical_structure_same_empty_neighbourhood_matches():
    # Two independent samplings of the SAME centred tower with NOTHING around
    # either -> descriptors must be nearly identical. Measured same-structure
    # distance is ~0.018 (sampling noise only); assert < 0.1, leaving ~5x
    # headroom over the measured value while still ruling out any real signal.
    a = _tower(0, 0, 1)
    b = _tower(0, 0, 2)
    d = region_distance(describe_region(a), describe_region(b))
    assert d < 0.1


def test_neighbour_in_different_direction_separates():
    # Same central tower; one window has an identical neighbour 8 m to the EAST,
    # the other the SAME neighbour 8 m to the NORTH. The arrangement grid must
    # light up different sectors, so the cross-neighbourhood distance must be
    # not merely > the match threshold but MANY times the same-structure
    # distance -- otherwise a near-miss implementation could pass both tests.
    a = _tower(0, 0, 1)
    b = _tower(0, 0, 2)
    same = region_distance(describe_region(a), describe_region(b))

    base = _tower(0, 0, 1)
    east = np.vstack([base, _tower(8, 0, 3)])
    north = np.vstack([base, _tower(0, 8, 4)])
    cross = region_distance(describe_region(east), describe_region(north))

    assert cross > 1.0                # comfortably above the match threshold
    assert cross >= 20.0 * same       # measured ratio ~150x; >=20x rules out near-miss


def test_region_descriptor_responds_to_direction():
    # The arrangement block itself (not just the summed distance) must differ
    # when the SAME neighbour moves from +x to +y. Compare each against the
    # lone-centre descriptor: the neighbour's mass lands in DIFFERENT sectors,
    # so the two "with neighbour" descriptors must be far apart.
    lone = describe_region(_tower(0, 0, 1))
    east = describe_region(np.vstack([_tower(0, 0, 1), _tower(8, 0, 3)]))
    north = describe_region(np.vstack([_tower(0, 0, 1), _tower(0, 8, 4)]))
    # both neighbours perturb the descriptor away from the lone centre...
    assert region_distance(lone, east) > 1.0
    assert region_distance(lone, north) > 1.0
    # ...and in genuinely different directions, so east != north.
    assert region_distance(east, north) > 1.0


def test_region_descriptor_is_deterministic():
    # No RNG inside describe_region: identical points -> bit-identical vector.
    pts = _tower(0, 0, 5)
    assert np.array_equal(describe_region(pts), describe_region(pts))
    assert np.array_equal(describe_region(pts.copy()), describe_region(pts))
