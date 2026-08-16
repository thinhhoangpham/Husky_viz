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
