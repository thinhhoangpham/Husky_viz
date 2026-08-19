"""Terrain slope costmap generator.

Turns a DTM height raster into a map_server PGM/YAML pair encoding terrain
steepness, so move_base's global costmap routes around steep ground.

Pure NumPy + stdlib -- NO ROS imports. Runs offline, unit-testable without a
roscore.

Spec: docs/superpowers/specs/2026-08-19-terrain-slope-costmap-design.md
"""
import numpy as np


def slope_degrees(heights, resolution):
    """Slope magnitude in degrees from a height raster.

    heights: 2D array of metres, row 0 = lowest y. NaN = no mesh coverage.
    resolution: metres per cell (square cells).

    Returns float64 degrees, same shape. NaN in -> NaN out. The result is
    UNSIGNED: an 18 deg climb and an 18 deg descent both return 18.0.
    """
    z = np.asarray(heights, dtype=np.float64)
    # np.gradient(z, dy, dx) -> (d/dy, d/dx) in metres per metre.
    grad_y, grad_x = np.gradient(z, resolution, resolution)
    slope = np.degrees(np.arctan(np.hypot(grad_x, grad_y)))
    # np.gradient's central difference at a NaN cell reads its NEIGHBOURS, not
    # the cell itself, so a lone NaN does not naturally propagate into the
    # gradient there. Mask explicitly so NaN input always yields NaN output.
    slope[np.isnan(z)] = np.nan
    return slope


# map_server sentinel for "no information". Written to the PGM as pixel 205,
# which falls between free_thresh and occupied_thresh so map_server emits -1.
UNKNOWN_OCC = -1
UNKNOWN_PIXEL = 205


def slope_to_occupancy(slope_deg, warn_deg=10.0, lethal_deg=18.0):
    """Map slope in degrees onto ROS occupancy 0..100, with -1 for unknown.

    < warn_deg          -> 0        free, no penalty
    warn_deg..lethal_deg -> 1..99    crossable but priced (linear ramp)
    >= lethal_deg       -> 100      lethal, planner routes around
    NaN                 -> -1       unknown (NOT lethal -- no mesh there)

    Thresholds are ABSOLUTE DEGREES on purpose. Deriving them from the data's
    own percentiles would invent a lethal band on flat terrain like the park.
    """
    if lethal_deg <= warn_deg:
        raise ValueError("lethal_deg (%r) must exceed warn_deg (%r)"
                         % (lethal_deg, warn_deg))

    s = np.asarray(slope_deg, dtype=np.float64)
    known = np.isfinite(s)

    occ = np.full(s.shape, UNKNOWN_OCC, dtype=np.int16)
    occ[known] = 0

    band = known & (s >= warn_deg) & (s < lethal_deg)
    frac = (s[band] - warn_deg) / (lethal_deg - warn_deg)   # 0.0 .. <1.0
    occ[band] = 1 + (frac * 98.0).round().astype(np.int16)  # 1 .. 99

    occ[known & (s >= lethal_deg)] = 100
    return occ


class GridSpec(object):
    """Geometry of a raster: where cell (0,0) sits in the world and how big
    cells are. Row 0 is the LOWEST y (the .npy convention used by extract_dtm).
    """

    def __init__(self, origin_x, origin_y, resolution, width, height):
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.resolution = float(resolution)
        self.width = int(width)
        self.height = int(height)

    def cell_centres(self):
        """World (x, y) of every cell centre, as (xs[width], ys[height])."""
        xs = self.origin_x + (np.arange(self.width) + 0.5) * self.resolution
        ys = self.origin_y + (np.arange(self.height) + 0.5) * self.resolution
        return xs, ys

    def __repr__(self):
        return ("GridSpec(origin=(%.3f, %.3f), res=%.3f, %dx%d)"
                % (self.origin_x, self.origin_y, self.resolution,
                   self.width, self.height))


def resample_nearest(src, src_grid, dst_grid, fill=UNKNOWN_OCC):
    """Nearest-neighbour resample from one grid geometry to another.

    The DTM and the occupancy map do NOT share a grid (different resolution
    AND different origin). Without this step every slope cell lands in the
    wrong place in the costmap.

    Nearest-neighbour, not interpolation: the values are already-classified
    occupancy, and averaging a lethal cell with a free one would invent a
    meaningless intermediate.
    """
    src = np.asarray(src)
    xs, ys = dst_grid.cell_centres()

    cols = np.floor((xs - src_grid.origin_x) / src_grid.resolution).astype(int)
    rows = np.floor((ys - src_grid.origin_y) / src_grid.resolution).astype(int)

    col_ok = (cols >= 0) & (cols < src_grid.width)
    row_ok = (rows >= 0) & (rows < src_grid.height)

    out = np.full((dst_grid.height, dst_grid.width), fill, dtype=src.dtype)
    if not col_ok.any() or not row_ok.any():
        return out

    inside = np.outer(row_ok, col_ok)
    picked = src[np.clip(rows, 0, src_grid.height - 1)[:, None],
                 np.clip(cols, 0, src_grid.width - 1)[None, :]]
    out[inside] = picked[inside]
    return out
