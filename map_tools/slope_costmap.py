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
