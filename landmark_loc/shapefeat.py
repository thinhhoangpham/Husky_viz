"""Viewpoint-stable shape features for landmark classification.

Pure numpy on (N,3) point arrays in the lidar frame. Unlike a bounding box, these
features describe an object's STRUCTURE (a thin post that rises high, a low wide
box, an oblong column), which a partial lidar view still exhibits. Thresholds are
measured from the object meshes and live captured clusters (see the plan/spec).
"""
import numpy as np

LOW_BAND = 0.8      # m: height of the near-base band used to measure the post/base
HIGH_Z = 1.3        # m: a thin band ABOVE this height is the lamp-post signature
                    # (lamp post rises past 1.3 m; bin/bench/table do not)
THIN_DIAG = 0.4     # m: footprint diagonal below this is "thin" (lamp post ~0.14 m)
_BAND = 0.5         # m: z-band thickness for the high-band scan
_BAND_MIN_PTS = 3   # a band needs this many points to measure its width
_MIN_SHAPE_PTS = 6  # fewer points than this: shape is unmeasurable


def foot_diag(xy):
    if len(xy) == 0:
        return 0.0
    return float(np.hypot(xy[:, 0].max() - xy[:, 0].min(),
                          xy[:, 1].max() - xy[:, 1].min()))


def pca_extents(xy):
    if len(xy) < 2:
        return 0.0, 0.0
    c = xy.mean(axis=0)
    cen = xy - c
    cov = np.cov(cen.T)
    _, evec = np.linalg.eigh(cov)
    proj = cen @ evec
    span = proj.max(axis=0) - proj.min(axis=0)
    return float(max(span)), float(min(span))


def circle_roundness(xy):
    """Kasa circle fit. Returns (radius, radial_rms / radius). Low ratio => round."""
    if len(xy) < 3:
        return 0.0, 1.0
    x, y = xy[:, 0], xy[:, 1]
    A = np.c_[2 * x, 2 * y, np.ones(len(x))]
    b = x ** 2 + y ** 2
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy, c = sol
    r = float(np.sqrt(max(c + cx ** 2 + cy ** 2, 1e-9)))
    d = np.hypot(x - cx, y - cy)
    rms = float(np.sqrt(np.mean((d - r) ** 2)))
    return r, rms / max(r, 1e-3)


def post_width(points):
    """Footprint diagonal of the near-base band [z0, z0+LOW_BAND). Isolates the
    post/base from a head or canopy above. 0.0 if too few points."""
    if points is None or len(points) < _MIN_SHAPE_PTS:
        return 0.0
    z0 = float(points[:, 2].min())
    band = points[(points[:, 2] >= z0) & (points[:, 2] < z0 + LOW_BAND)]
    if len(band) < _BAND_MIN_PTS:
        return 0.0
    return foot_diag(band[:, :2])


def has_thin_high_band(points):
    """True if a thin (foot_diag < THIN_DIAG) band exists above HIGH_Z. This is the
    lamp signature: a skinny post rising high. Nothing else in the park does this."""
    if points is None or len(points) < _MIN_SHAPE_PTS:
        return False
    top = float(points[:, 2].max())
    z = HIGH_Z
    while z < top:
        band = points[(points[:, 2] >= z) & (points[:, 2] < z + _BAND)]
        if len(band) >= _BAND_MIN_PTS and foot_diag(band[:, :2]) < THIN_DIAG:
            return True
        z += _BAND
    return False


def foot_extents(points):
    """PCA extents (major, minor) over the full footprint. 0,0 if too few points."""
    if points is None or len(points) < _MIN_SHAPE_PTS:
        return 0.0, 0.0
    return pca_extents(points[:, :2])
