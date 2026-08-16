"""NDT-style shape descriptor: points -> per-height-band voxel-shape statistics.

Pure numpy, no ROS. Shared by map extraction (mesh-sampled points) and the
runtime detector (lidar points) so the two sides cannot drift. See the design
doc's "Descriptor" section: local shape from covariance eigenvalue RATIOS is a
LOCAL property, so a near-face voxel classifies correctly without the far side.
"""
import numpy as np


def voxel_shape(points):
    """Return (linearity, planarity, sphericity) for an (N,3) point set.

    From the covariance eigenvalues l1>=l2>=l3 (Demantke et al.):
        linearity   = (l1 - l2) / l1
        planarity   = (l2 - l3) / l1
        sphericity  =  l3       / l1
    Normalised to sum to 1. Degenerate input -> (0, 0, 1).
    """
    p = np.asarray(points, dtype=float)
    if p.shape[0] < 3:
        return (0.0, 0.0, 1.0)
    cov = np.cov((p - p.mean(axis=0)).T)
    evals = np.linalg.eigvalsh(cov)  # ascending
    l3, l2, l1 = float(evals[0]), float(evals[1]), float(evals[2])
    if l1 <= 1e-12:
        return (0.0, 0.0, 1.0)
    lin = (l1 - l2) / l1
    pla = (l2 - l3) / l1
    sph = l3 / l1
    s = lin + pla + sph
    if s <= 1e-12:
        return (0.0, 0.0, 1.0)
    return (lin / s, pla / s, sph / s)


EXTENT_WEIGHT = 0.25


def describe(points, band_height=1.0, voxel=0.5, min_voxel_pts=5, n_bands=18):
    """Per-height-band voxel-shape descriptor. Shape (n_bands, 4).

    Defaults (voxel=0.5, min_voxel_pts=5) pass the partial-view, decimation,
    and realistic-sparsity robustness characterization
    (test_partial_view_matches_full, test_decimation_stable,
    test_sparse_pole_still_closer_to_dense_pole_than_bench) as-is -- no
    tuning was needed. A half-diameter voxel is coarse enough that even a
    1/4-density, single-face, or ~200-point-total sample of the lattice pole
    still fills >=5 points per occupied cell, so the shape ratios stay
    comparable to the full-density, full-view descriptor. The sparsity test
    is a lower bound only (verified to fail at n=50 points, confirming it
    genuinely exercises the threshold) -- it is not a substitute for
    validation against real Ouster returns, which happens later in-sim.

    Bands are measured above the cluster's OWN minimum z (so ground offset
    does not matter). In each band, points are bucketed into `voxel`-sized
    x/y cells; each cell with >= min_voxel_pts contributes a voxel_shape, and
    the band records the MEAN shape over its voxels plus the band's horizontal
    extent. Bands with no qualifying voxel are all-zero.
    """
    out = np.zeros((n_bands, 4), dtype=float)
    p = np.asarray(points, dtype=float)
    if p.shape[0] == 0:
        return out
    z0 = p[:, 2].min()
    for bi in range(n_bands):
        lo = z0 + bi * band_height
        hi = lo + band_height
        band = p[(p[:, 2] >= lo) & (p[:, 2] < hi)]
        if band.shape[0] == 0:
            continue
        cells = np.floor(band[:, :2] / voxel).astype(int)
        buckets = {}
        for i, key in enumerate(map(tuple, cells)):
            buckets.setdefault(key, []).append(i)
        qualifying = [idxs for idxs in buckets.values() if len(idxs) >= min_voxel_pts]
        if not qualifying:
            # A handful of scattered points with no voxel meeting
            # min_voxel_pts is not a measured shape -- leave the WHOLE row
            # zero, extent included. Writing extent from noise-driven points
            # would give a sparse lidar band a nonzero value the dense
            # mesh-sampled side lacks, injecting spurious distance into the
            # exact column (weighted by EXTENT_WEIGHT) that partial/sparse
            # views most need to agree on.
            continue
        out[bi, 3] = float(np.hypot(
            band[:, 0].max() - band[:, 0].min(),
            band[:, 1].max() - band[:, 1].min()))
        out[bi, :3] = np.mean([voxel_shape(band[idxs]) for idxs in qualifying], axis=0)
    return out


def window(cloud, cx, cy, radius):
    """Return the subset of (N,3) `cloud` within `radius` of (cx,cy) in x/y.

    Recentred so the window centre is at x=y=0; z is left absolute. This is
    what makes a map window and a runtime window comparable regardless of
    where each was cut from in world coordinates.
    """
    p = np.asarray(cloud, dtype=float)
    mask = np.hypot(p[:, 0] - cx, p[:, 1] - cy) <= radius
    out = p[mask].copy()
    out[:, 0] -= cx
    out[:, 1] -= cy
    return out


def descriptor_distance(a, b):
    """Weighted L2 over flattened (n_bands, 4) descriptors; extent down-weighted."""
    a = np.array(a, dtype=float, copy=True)
    b = np.array(b, dtype=float, copy=True)
    a[:, 3] *= EXTENT_WEIGHT
    b[:, 3] *= EXTENT_WEIGHT
    return float(np.linalg.norm(a.ravel() - b.ravel()))


# --- Region descriptor: WHERE structure sits, not just WHAT it is -----------

# The vertical descriptor `describe()` answers "what does the structure at this
# spot look like?" -- it is translation-invariant within the window, so two
# IDENTICAL structures produce IDENTICAL vertical descriptors no matter what is
# around them. That is exactly why it cannot tell apart two copies of the same
# object standing in different neighbourhoods: a lone pole and a pole with a
# tree 8 m to its east have the same vertical descriptor.
#
# The ARRANGEMENT block fixes that. It divides the window into angular sectors x
# radial rings about the window centre (already at the origin, because window()
# recentred the cloud), and records per cell how much structure sits in that
# direction-and-distance. A neighbour to the east lights up the east cells; the
# same neighbour to the north lights up the north cells; the two windows now
# differ. Empty cells are legitimate zeros -- a direction with nothing in it is
# information (cf. empty height bands in describe()), so we never sentinel them.

ARRANGEMENT_WEIGHT = 1.0
# Chosen 1.0. Measured on the T17 test structures (a 16 m lattice tower at the
# centre, +/- an identical tower neighbour 8 m to the east vs the north):
#   same structure / same empty surroundings : region_distance ~= 0.018
#   same centre, neighbour east vs north      : region_distance ~= 2.9
# a >150x separation. The arrangement block's occupied-mass term (a fraction of
# total window points per cell) already carries a strong direction signal, so
# even weight 1.0 leaves both the "matches" and "separates" tests passing with
# large margin without the arrangement swamping the vertical block or vice
# versa. Larger weights only widen the (already huge) gap; 1.0 keeps the two
# blocks numerically comparable, which is the honest default.

# Number of scalars in the flattened vertical block: describe() returns
# (n_bands, 4) and we reuse its default n_bands=18 -> 72. region_distance reads
# this to split a region vector back into its vertical and arrangement halves.
_VERTICAL_LEN = 18 * 4


def _arrangement_grid(points, n_sectors, n_rings, radius):
    """Sector x ring occupancy+shape grid about the origin. Shape (S, R, 4).

    Per cell: column 0 is the OCCUPIED-POINT MASS (that cell's share of the
    window's points, so it is density- and count-normalised the way the vertical
    block's shape ratios are), columns 1..3 are the MEAN voxel-shape mix of the
    cell's points. Sectors are measured from atan2(y, x) in ABSOLUTE map x/y
    (see rotation note in describe_region); rings bin radius uniformly over
    [0, radius). Points beyond `radius` are dropped -- the caller's window()
    already clipped to `radius`, this only guards a mismatched radius argument.
    """
    grid = np.zeros((n_sectors, n_rings, 4), dtype=float)
    p = np.asarray(points, dtype=float)
    if p.shape[0] == 0:
        return grid

    x, y = p[:, 0], p[:, 1]
    # Sector index: wrap atan2's (-pi, pi] to [0, 2pi) so sector 0 starts at +x
    # and sectors advance counter-clockwise -- a fixed, absolute frame.
    ang = np.mod(np.arctan2(y, x), 2.0 * np.pi)
    si = np.clip((ang / (2.0 * np.pi) * n_sectors).astype(int), 0, n_sectors - 1)
    rad = np.hypot(x, y)
    ri = (rad / radius * n_rings).astype(int)
    in_range = ri < n_rings  # drop anything at/after the outer radius

    total = p.shape[0]
    for s in range(n_sectors):
        for r in range(n_rings):
            cell_mask = in_range & (si == s) & (ri == r)
            cnt = int(cell_mask.sum())
            grid[s, r, 0] = cnt / total
            # voxel_shape needs >=3 points to be meaningful (it returns the
            # spherical default below that); leave shape columns zero for near-
            # empty cells rather than feeding it noise.
            if cnt >= 3:
                grid[s, r, 1:4] = voxel_shape(p[cell_mask])
    return grid


def _normalise_orientation(points):
    """Rotation-sensitivity seam (currently a NO-OP -- returns points as-is).

    The arrangement grid is built in ABSOLUTE map x/y. That is deliberate: the
    world exposes an absolute compass heading at runtime, so an absolute-yaw
    grid is both usable and MORE discriminating than a rotation-invariant one
    (it can separate "neighbour to the east" from "neighbour to the north",
    which a rotation-invariant grid cannot). This hook is the single place to
    swap in rotation invariance -- e.g. rotate the window so its dominant
    horizontal direction points along +x -- if in-sim shows the compass is
    unreliable. Kept as a no-op so the descriptor is not hard-coupled to
    absolute yaw.
    """
    return points


def describe_region(points, n_sectors=8, n_rings=3, radius=12.0, **describe_kwargs):
    """Region descriptor: vertical shape CONCATENATED with a horizontal grid.

    `points` is an ALREADY-WINDOWED, already-recentred (M,3) cloud -- the caller
    cuts it with window(cx, cy, radius) first, so the window centre is at the
    origin and the sector angles are measured about (0,0). describe_region does
    NOT window again; the same function therefore serves both the map side
    (window cut from the scene cloud) and the runtime side (window cut from live
    lidar) with no change.

    Returns a 1-D float vector:
        [ describe(points).ravel()  |  _arrangement_grid(...).ravel() ]
    i.e. the vertical descriptor (WHAT sits here) followed by the arrangement
    grid (WHERE structure sits around here). `region_distance` weights the two
    halves; see ARRANGEMENT_WEIGHT.

    Deterministic: no RNG here or in anything it calls, so identical `points`
    yield an identical vector.
    """
    p = _normalise_orientation(np.asarray(points, dtype=float))
    vertical = describe(p, **describe_kwargs).ravel()
    arrangement = _arrangement_grid(p, n_sectors, n_rings, radius).ravel()
    return np.concatenate([vertical, arrangement])


def region_distance(a, b):
    """Weighted L2 between two describe_region vectors.

    Splits each vector into its vertical block (first _VERTICAL_LEN scalars) and
    its arrangement block (the rest). Within the vertical block the extent
    column is down-weighted by EXTENT_WEIGHT exactly as descriptor_distance
    does, so the vertical half of region_distance is consistent with the
    standalone vertical metric. The whole arrangement block is scaled by
    ARRANGEMENT_WEIGHT so arrangement and vertical shape both count.
    """
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)

    va = a[:_VERTICAL_LEN].reshape(-1, 4).copy()
    vb = b[:_VERTICAL_LEN].reshape(-1, 4).copy()
    va[:, 3] *= EXTENT_WEIGHT
    vb[:, 3] *= EXTENT_WEIGHT
    vertical_diff = va.ravel() - vb.ravel()

    arrangement_diff = (a[_VERTICAL_LEN:] - b[_VERTICAL_LEN:]) * ARRANGEMENT_WEIGHT

    return float(np.linalg.norm(np.concatenate([vertical_diff, arrangement_diff])))
