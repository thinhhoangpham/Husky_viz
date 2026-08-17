"""Terrain localization by GRADIENT correlation of a local ground grid against a
prior DTM. Gradients (not raw heights) so a constant elevation offset -- unknown
robot height, sloped ground -- cancels exactly. North-aligned by compass upstream,
so no rotation search here. See design spec section 6.
"""
import numpy as np
import yaml


class Dtm(object):
    __slots__ = ("z", "resolution", "origin_x", "origin_y")

    def __init__(self, z, resolution, origin_x, origin_y):
        self.z = z
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y


def load_dtm(npy_path, yaml_path):
    z = np.load(npy_path)
    m = yaml.safe_load(open(yaml_path))
    return Dtm(z.astype(np.float32), float(m["resolution"]),
               float(m["origin_x"]), float(m["origin_y"]))


def gradient(z):
    """NaN-aware forward differences. gx[i,j]=z[i,j+1]-z[i,j], gy[i,j]=
    z[i+1,j]-z[i,j]; NaN where either operand is NaN (last row/col also NaN)."""
    h, w = z.shape
    gx = np.full_like(z, np.nan)
    gy = np.full_like(z, np.nan)
    gx[:, :w - 1] = z[:, 1:] - z[:, :w - 1]
    gy[:h - 1, :] = z[1:, :] - z[:h - 1, :]
    return gx, gy


def _score_at(lgx, lgy, pgx, pgy, r0, c0):
    """Normalized gradient-match score for placing the local grid's (0,0) at
    prior cell (r0,c0). Correlation over cells finite in BOTH; None if <25 pairs.
    Score = 1 / (1 + mean squared gradient difference), in (0,1]."""
    h, w = lgx.shape
    ph, pw = pgx.shape
    if r0 < 0 or c0 < 0 or r0 + h > ph or c0 + w > pw:
        return None
    px = pgx[r0:r0 + h, c0:c0 + w]
    py = pgy[r0:r0 + h, c0:c0 + w]
    m = np.isfinite(lgx) & np.isfinite(px) & np.isfinite(lgy) & np.isfinite(py)
    if m.sum() < 25:
        return None
    d = (lgx[m] - px[m]) ** 2 + (lgy[m] - py[m]) ** 2
    return 1.0 / (1.0 + float(d.mean()))


def match_terrain(local_ground, local_res, prior, prior_xy, search_radius_m):
    """Slide the local ground gradient over the prior gradient within
    search_radius_m of prior_xy. Return (x, y, score): the map-frame CENTER of
    the best placement of the local grid, and its score. None if no placement
    has enough valid overlap. Assumes local_res == prior.resolution.
    """
    if abs(local_res - prior.resolution) > 1e-6:
        raise ValueError("local/prior resolution mismatch: %r vs %r"
                         % (local_res, prior.resolution))
    lgx, lgy = gradient(local_ground)
    pgx, pgy = gradient(prior.z)
    h, w = local_ground.shape
    res = prior.resolution
    # prior cell that the search centers on (place local-grid center at prior_xy)
    cx_cell = int(round((prior_xy[0] - prior.origin_x) / res - w / 2.0))
    cy_cell = int(round((prior_xy[1] - prior.origin_y) / res - h / 2.0))
    rad = int(round(search_radius_m / res))
    best = None
    for dr in range(-rad, rad + 1):
        for dc in range(-rad, rad + 1):
            s = _score_at(lgx, lgy, pgx, pgy, cy_cell + dr, cx_cell + dc)
            if s is None:
                continue
            if best is None or s > best[0]:
                best = (s, cy_cell + dr, cx_cell + dc)
    if best is None:
        return None
    s, r0, c0 = best
    # map-frame center of the placed local grid
    x = prior.origin_x + (c0 + w / 2.0) * res
    y = prior.origin_y + (r0 + h / 2.0) * res
    return (x, y, s)
