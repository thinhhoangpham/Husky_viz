"""2.5D terrain grid: bin a map-frame cloud into per-cell min_z, then extract a
local ground surface by morphological opening. TERRAIN ONLY -- this does not
detect objects (the classifier does that). See design spec sections 4-6.

Grid convention matches map_tools/dtm_raster.py:DtmGrid -- z[row, col], row 0 =
lowest y, origin_(x,y) = cell (0,0) corner, resolution metres/cell. Absent cells
are NaN, never 0.0 (a fake zero is a fabricated flat plateau).
"""
import numpy as np


def bin_min_z(points_map, resolution, origin_x, origin_y, width, height):
    """(height, width) float32 of min z per cell; NaN where no point falls."""
    z = np.full((height, width), np.nan, dtype=np.float32)
    p = np.asarray(points_map, dtype=float)
    if len(p) == 0:
        return z
    cols = ((p[:, 0] - origin_x) / resolution).astype(int)
    rows = ((p[:, 1] - origin_y) / resolution).astype(int)
    inb = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
    cols, rows, zz = cols[inb], rows[inb], p[inb, 2]
    # min per cell: sort so lowest z last-writes each (row,col)
    order = np.argsort(-zz)  # descending z; np assignment keeps the last write
    flat = rows[order] * width + cols[order]
    zf = z.reshape(-1)
    zf[flat] = zz[order]
    return z


def _min_filter(z, k):
    """NaN-aware min over a (2k+1) square window."""
    h, w = z.shape
    out = np.full_like(z, np.nan)
    for r in range(h):
        r0, r1 = max(0, r - k), min(h, r + k + 1)
        for c in range(w):
            c0, c1 = max(0, c - k), min(w, c + k + 1)
            win = z[r0:r1, c0:c1]
            v = win[np.isfinite(win)]
            if v.size:
                out[r, c] = v.min()
    return out


def _max_filter(z, k):
    """NaN-aware max over a (2k+1) square window."""
    h, w = z.shape
    out = np.full_like(z, np.nan)
    for r in range(h):
        r0, r1 = max(0, r - k), min(h, r + k + 1)
        for c in range(w):
            c0, c1 = max(0, c - k), min(w, c + k + 1)
            win = z[r0:r1, c0:c1]
            v = win[np.isfinite(win)]
            if v.size:
                out[r, c] = v.max()
    return out


def morphological_ground(min_z, window_cells):
    """Opening (erode=min then dilate=max) removes up-poking objects and leaves
    the slope-following ground surface. `window_cells` is the half-width k; it
    must exceed the widest object and stay under the terrain feature scale.
    """
    k = int(window_cells)
    return _max_filter(_min_filter(min_z, k), k)
