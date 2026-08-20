"""Rasterise a triangle soup into a Digital Terrain Model: a world-frame,
axis-aligned grid holding the MAX surface z over each cell.

Max (not mean, not last-writer-wins) because a DTM cell answers "how high is
the ground here"; where a fold or overhang puts two surfaces over one cell,
the drivable/visible one is the upper. Max is also order-independent, so the
output does not depend on triangle ordering in the mesh file.

Cells no triangle covers stay NaN. That distinction is load-bearing: 0.0 would
be a plausible-looking height and would forge a flat plateau outside the mesh,
which is exactly the kind of fake terrain a localiser would then try to match
against.

Pure geometry -- no COLLADA, no SDF, no ROS. Callers hand in triangles already
in world coordinates (see extract_dtm.py, which applies the mesh <scale> and
the model's effective <pose> first).
"""
import numpy as np


class DtmGrid(object):
    """A height raster. `z[row, col]`, row 0 = LOWEST y, matching the row
    convention in map_tools/occupancy_grid.py so the two are read the same way.
    """

    __slots__ = ("z", "resolution", "origin_x", "origin_y")

    def __init__(self, z, resolution, origin_x, origin_y):
        self.z = z
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y

    @property
    def height(self):
        return self.z.shape[0]

    @property
    def width(self):
        return self.z.shape[1]

    def cell_centers(self):
        """(X, Y) arrays of each cell's world-frame center, same shape as z."""
        cols = np.arange(self.width, dtype=np.float64)
        rows = np.arange(self.height, dtype=np.float64)
        x = self.origin_x + (cols + 0.5) * self.resolution
        y = self.origin_y + (rows + 0.5) * self.resolution
        return np.meshgrid(x, y)

    def stats(self):
        """(valid_count, total_count, z_min, z_max). z_min/z_max are NaN when
        nothing is valid, rather than raising -- an empty DTM is a legitimate
        (if useless) result and the caller reports it."""
        finite = np.isfinite(self.z)
        n_valid = int(finite.sum())
        total = int(self.z.size)
        if n_valid == 0:
            return 0, total, float("nan"), float("nan")
        vals = self.z[finite]
        return n_valid, total, float(vals.min()), float(vals.max())


def grid_bounds(tris, resolution, origin=None, shape=None):
    """Derive (origin_x, origin_y, width, height) covering `tris`.

    Snaps the origin DOWN to a multiple of `resolution` so that two grids built
    at the same resolution from different meshes (terrain and water) share cell
    boundaries and can be compared cell-for-cell. Pass `origin`/`shape` to force
    an existing grid's geometry instead -- that is how the lake water layer is
    made to line up with the lake DTM.
    """
    if origin is not None and shape is not None:
        return origin[0], origin[1], shape[1], shape[0]

    pts = tris.reshape(-1, 3)
    min_x, min_y = pts[:, 0].min(), pts[:, 1].min()
    max_x, max_y = pts[:, 0].max(), pts[:, 1].max()
    origin_x = np.floor(min_x / resolution) * resolution
    origin_y = np.floor(min_y / resolution) * resolution
    width = int(np.ceil((max_x - origin_x) / resolution))
    height = int(np.ceil((max_y - origin_y) / resolution))
    # A degenerate (zero-extent) mesh still needs one cell to land in.
    return origin_x, origin_y, max(width, 1), max(height, 1)


def rasterize(tris, resolution=0.25, origin=None, shape=None, chunk=20000):
    """Rasterise (M,3,3) world-frame triangles into a DtmGrid of max z.

    Method: for each triangle, walk the cells of its 2D bounding box and keep
    those whose CENTER lies inside the triangle (barycentric test in xy), then
    interpolate z at that center with the same barycentric weights. Sampling at
    cell centers -- rather than marking every cell the triangle touches -- is
    what makes the height meaningful: the stored z is the surface height at a
    defined point, not at some unspecified spot in the cell.

    Triangles are processed in chunks and each chunk is vectorised over its own
    bounding-box grid. Chunking bounds peak memory: these terrain meshes carry
    ~250k-750k triangles, and a single allocation across all of them at once
    would be tens of GB.

    Degenerate triangles (zero xy area -- vertical walls seen edge-on, or
    collapsed faces) are skipped: they have no barycentric basis, and a
    zero-width surface covers no cell center anyway.
    """
    tris = np.asarray(tris, dtype=np.float64)
    if tris.ndim != 3 or tris.shape[1:] != (3, 3):
        raise ValueError("tris must have shape (M, 3, 3), got %r"
                         % (tris.shape,))
    if resolution <= 0:
        raise ValueError("resolution must be positive, got %r" % (resolution,))

    if len(tris) == 0:
        if origin is None or shape is None:
            raise ValueError("cannot infer grid bounds from zero triangles; "
                             "pass origin= and shape=")
        ox, oy, w, h = grid_bounds(tris, resolution, origin, shape)
        return DtmGrid(np.full((h, w), np.nan), resolution, ox, oy)

    ox, oy, width, height = grid_bounds(tris, resolution, origin, shape)
    z = np.full((height, width), np.nan, dtype=np.float64)

    for start in range(0, len(tris), chunk):
        _rasterize_chunk(tris[start:start + chunk], z, resolution, ox, oy)

    return DtmGrid(z.astype(np.float32), resolution, ox, oy)


def _rasterize_chunk(tris, z, res, ox, oy):
    """Accumulate max-z from one chunk of triangles into `z` in place."""
    height, width = z.shape

    v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]

    # Signed double-area in xy. Zero => degenerate in plan view, no basis.
    e1 = v1 - v0
    e2 = v2 - v0
    denom = e1[:, 0] * e2[:, 1] - e1[:, 1] * e2[:, 0]
    keep = denom != 0.0
    if not keep.any():
        return
    v0, v1, v2 = v0[keep], v1[keep], v2[keep]
    e1, e2, denom = e1[keep], e2[keep], denom[keep]

    xs = tris[keep][:, :, 0]
    ys = tris[keep][:, :, 1]

    # Cell-index range whose CENTERS can fall in each triangle's bbox. A center
    # sits at origin + (i + 0.5) * res, so solving for i and rounding inward
    # gives the first/last candidate index -- no need to test cells whose
    # centers are outside the bbox at all.
    col0 = np.ceil((xs.min(axis=1) - ox) / res - 0.5).astype(np.int64)
    col1 = np.floor((xs.max(axis=1) - ox) / res - 0.5).astype(np.int64)
    row0 = np.ceil((ys.min(axis=1) - oy) / res - 0.5).astype(np.int64)
    row1 = np.floor((ys.max(axis=1) - oy) / res - 0.5).astype(np.int64)

    np.clip(col0, 0, width - 1, out=col0)
    np.clip(col1, 0, width - 1, out=col1)
    np.clip(row0, 0, height - 1, out=row0)
    np.clip(row1, 0, height - 1, out=row1)

    n_col = col1 - col0 + 1
    n_row = row1 - row0 + 1
    # A triangle can be thinner than a cell and enclose no center at all.
    valid = (n_col > 0) & (n_row > 0)
    if not valid.any():
        return

    # Group triangles by identical bbox cell-extent so each group can be
    # evaluated as one dense (n_tri, n_row, n_col) block. Terrain meshes are
    # regular grids, so nearly all triangles share one small extent and this
    # collapses to a couple of large vectorised passes.
    idx = np.flatnonzero(valid)
    key = n_row[idx] * (width + 1) + n_col[idx]
    order = np.argsort(key, kind="stable")
    idx = idx[order]
    key = key[order]
    starts = np.flatnonzero(np.r_[True, key[1:] != key[:-1]])
    group_bounds = np.r_[starts, len(idx)]

    for g in range(len(starts)):
        sel = idx[group_bounds[g]:group_bounds[g + 1]]
        nr = int(n_row[sel[0]])
        nc = int(n_col[sel[0]])

        # World-frame centers of every candidate cell for every triangle here.
        dc = np.arange(nc, dtype=np.float64)
        dr = np.arange(nr, dtype=np.float64)
        cx = ox + (col0[sel][:, None, None] + dc[None, None, :] + 0.5) * res
        cy = oy + (row0[sel][:, None, None] + dr[None, :, None] + 0.5) * res

        px = cx - v0[sel][:, 0][:, None, None]
        py = cy - v0[sel][:, 1][:, None, None]

        d = denom[sel][:, None, None]
        # Barycentric coordinates of the cell center w.r.t. the triangle.
        b1 = (px * e2[sel][:, 1][:, None, None]
              - py * e2[sel][:, 0][:, None, None]) / d
        b2 = (py * e1[sel][:, 0][:, None, None]
              - px * e1[sel][:, 1][:, None, None]) / d
        b0 = 1.0 - b1 - b2

        # Inclusive bounds: a center exactly on a shared edge belongs to both
        # neighbours. Since we reduce with max, double-counting is harmless and
        # it avoids dropping centers that land precisely on an edge -- common
        # here because these meshes are regular grids aligned to round numbers.
        inside = (b0 >= 0.0) & (b1 >= 0.0) & (b2 >= 0.0)
        if not inside.any():
            continue

        zc = (b0 * v0[sel][:, 2][:, None, None]
              + b1 * v1[sel][:, 2][:, None, None]
              + b2 * v2[sel][:, 2][:, None, None])

        ti, ri, ci = np.nonzero(inside)
        rows = row0[sel][ti] + ri
        cols = col0[sel][ti] + ci
        flat = rows * z.shape[1] + cols
        vals = zc[ti, ri, ci]

        # np.maximum.at handles repeated targets correctly (plain fancy-index
        # assignment would keep only the last write, not the max). NaN in the
        # destination must be treated as "empty", so fill those first.
        cur = z.reshape(-1)
        first = np.isnan(cur[flat])
        if first.any():
            # Seed empty cells with -inf so the max below adopts the real value.
            cur[flat[first]] = -np.inf
        np.maximum.at(cur, flat, vals)


# --- Metadata readers -------------------------------------------------------
# These parse the sidecar files that accompany a rasterised DTM (and the
# object-map PGM it has to stay aligned with). They live here rather than in a
# tool module so any consumer of a DTM can read its geometry without importing
# a CLI. Restored from the deleted map_tools/slope_costmap.py (commit 4b022a3);
# nothing slope-related came with them.

def read_dtm_yaml(path):
    """Parse the flat `key: value` yaml written by extract_dtm.py.

    Deliberately not PyYAML: these files are generated by hand-rolled writers
    with a fixed shape, and map_tools has no yaml dependency today.
    """
    out = {}
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#") or ":" not in line:
                continue
            key, _, value = line.partition(":")
            value = value.strip()
            try:
                out[key.strip()] = int(value)
            except ValueError:
                try:
                    out[key.strip()] = float(value)
                except ValueError:
                    out[key.strip()] = value
    return out


def read_pgm_dimensions(path):
    """Parse just the header of a binary P5 PGM and return (width, height).

    Reads only the header tokens (magic, width, height, maxval) -- never the
    pixel body -- so this is cheap even on a large map.
    """
    import os

    if not os.path.exists(path):
        raise IOError("PGM not found: %s" % path)

    with open(path, "rb") as fh:
        tokens = []
        # P5 header: magic, width, height, maxval -- whitespace-separated,
        # comments starting with '#' allowed between tokens.
        while len(tokens) < 4:
            chunk = fh.read(1)
            if not chunk:
                raise IOError("truncated PGM header in %s" % path)
            if chunk in b" \t\r\n":
                continue
            if chunk == b"#":
                fh.readline()
                continue
            token = chunk
            while True:
                c = fh.read(1)
                if not c or c in b" \t\r\n":
                    break
                token += c
            tokens.append(token)

    magic, width, height, _maxval = tokens
    if magic != b"P5":
        raise ValueError("%s is not a binary P5 PGM (magic=%r)"
                         % (path, magic))
    return int(width), int(height)
