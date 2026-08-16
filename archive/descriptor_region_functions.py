"""ARCHIVED (2026-08-16): retired region-descriptor functions.

These came from `landmark_loc/descriptor.py`. They implemented the PER-REGION
descriptor approach: `window()` cut a neighbourhood out of a cloud, and
`describe_region()` concatenated the vertical shape descriptor with a
sector x ring arrangement grid so that two geometrically identical objects in
DIFFERENT neighbourhoods would still be distinguishable.

Why retired: the literature survey in
`docs/research/lidar-place-recognition-survey.md` established that per-object
and per-region descriptor matching cannot work in this park. The world contains
23 identical trees and 16 identical benches -- geometrically identical by
construction -- so no descriptor can disambiguate instances. That is an
identifiability limit, not a tuning problem. The project pivoted to Scan
Context (whole-scan descriptors) instead.

Kept for reference only. NOT imported by any live code. `voxel_shape()`,
`describe()`, `descriptor_distance()` and `EXTENT_WEIGHT` remain in
`landmark_loc/descriptor.py`; this module imports them from there.
"""
import numpy as np

from landmark_loc.descriptor import voxel_shape, describe, EXTENT_WEIGHT


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
#
# This is a FIXED module constant, and region_distance ASSERTS the incoming
# vector length matches _VERTICAL_LEN + the arrangement length before slicing
# (Finding A). describe_region forwards **describe_kwargs to describe(), so a
# caller passing n_bands != 18 would produce a longer/shorter vertical block;
# with a fixed split point that would silently mis-slice and return a
# meaningless distance. The assertion turns that into a loud ValueError. If
# n_bands ever genuinely needs to vary, carry the split point explicitly
# instead of relaxing this guard.
_VERTICAL_LEN = 18 * 4

# The full region-descriptor length with the default grid: 72 vertical +
# 8 sectors * 3 rings * 4 = 96 arrangement = 168. region_distance asserts the
# incoming vectors are exactly this length, so a vector built with a non-default
# n_bands (or a non-default grid) fails loudly rather than mis-slicing. Note a
# bare (size - 72) % 4 check is NOT enough: n_bands=10 gives length 136 and
# 136 - 72 = 64 is a multiple of 4, so only pinning the exact total catches it.
_DEFAULT_REGION_LEN = _VERTICAL_LEN + 8 * 3 * 4


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

    describe_region OWNS its `radius` (Finding B): it clips the input to points
    within `radius` of the origin BEFORE computing either block. The window is
    already recentred to the origin, so that clip is `hypot(x, y) <= radius`.
    This makes the descriptor independent of whatever radius the caller's
    window() used: the arrangement grid's mass denominator (total points) and
    its per-cell counts are then guaranteed to be the same point set, so a
    caller cutting a wider window cannot silently deflate every cell's mass.

    Deterministic: no RNG here or in anything it calls, so identical `points`
    yield an identical vector.
    """
    p = _normalise_orientation(np.asarray(points, dtype=float))
    if p.shape[0]:
        p = p[np.hypot(p[:, 0], p[:, 1]) <= radius]
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

    # Guard against a fixed-split mis-slice (Finding A): both vectors must have
    # the length describe_region produces with the default n_bands=18. A vector
    # built with a different n_bands has a different vertical length; slicing it
    # at _VERTICAL_LEN would silently mix vertical and arrangement scalars and
    # return a meaningless distance. Fail loudly instead.
    if a.shape != b.shape:
        raise ValueError(
            "region_distance: mismatched vector lengths %d vs %d"
            % (a.size, b.size))
    if a.size != _DEFAULT_REGION_LEN:
        raise ValueError(
            "region_distance: vector length %d != expected %d (72 vertical + "
            "96 arrangement). A non-default n_bands or grid is unsupported by "
            "this fixed-split metric."
            % (a.size, _DEFAULT_REGION_LEN))

    va = a[:_VERTICAL_LEN].reshape(-1, 4).copy()
    vb = b[:_VERTICAL_LEN].reshape(-1, 4).copy()
    va[:, 3] *= EXTENT_WEIGHT
    vb[:, 3] *= EXTENT_WEIGHT
    vertical_diff = va.ravel() - vb.ravel()

    arrangement_diff = (a[_VERTICAL_LEN:] - b[_VERTICAL_LEN:]) * ARRANGEMENT_WEIGHT

    return float(np.linalg.norm(np.concatenate([vertical_diff, arrangement_diff])))
