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
