"""Clip the object occupancy map to the terrain (DTM) footprint.

The object map (maps/<world>_map.pgm) is sized from object positions plus a
5 m margin pad (see extract_park_map.py), independent of the terrain mesh's
own extent. Where the object map falls SHORT of the terrain, the slope
costmap layer (map_tools/slope_costmap.py) -- which sizes itself off the
object map -- silently discards real terrain slope data outside that
boundary.

This tool crops the object map's PGM to the DTM's world-coordinate
footprint, snapping the crop to the object map's OWN cell grid so no cell is
resampled or shifted -- a pure integer crop, never interpolation.

Pure NumPy + stdlib -- NO ROS imports. Runs offline, unit-testable without a
roscore.
"""
import argparse
import os
import sys

import numpy as np

from map_tools.slope_costmap import read_dtm_yaml, read_pgm_dimensions


def read_map_yaml(path):
    """Parse the object map's map_server yaml: (origin_x, origin_y, resolution)."""
    meta = read_dtm_yaml(path)
    origin = meta["origin"]
    if isinstance(origin, str):
        parts = origin.strip("[]").split(",")
        ox, oy = float(parts[0]), float(parts[1])
    else:
        raise ValueError("could not parse origin from %s" % path)
    return ox, oy, float(meta["resolution"])


def read_pgm(path):
    """Read a binary P5 PGM into a numpy array, row 0 = LOWEST y (flips the
    file's row 0 = HIGHEST y convention back to the repo's internal one)."""
    with open(path, "rb") as fh:
        tokens = []
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
            raise ValueError("%s is not a binary P5 PGM (magic=%r)" % (path, magic))
        width, height = int(width), int(height)
        body = fh.read(width * height)
    pixels = np.frombuffer(body, dtype=np.uint8).reshape(height, width)
    # File row 0 = highest y -> flip so row 0 = lowest y, matching the repo's
    # internal convention (occupancy_grid.py, slope_costmap.py).
    return pixels[::-1, :].copy()


def write_pgm(path, pixels):
    """Binary P5 PGM. `pixels` row 0 = LOWEST y; flip on write so the file's
    row 0 = HIGHEST y, matching map_server's bottom-left origin convention."""
    pixels = np.asarray(pixels, dtype=np.uint8)
    height, width = pixels.shape
    with open(path, "wb") as fh:
        fh.write(b"P5\n%d %d\n255\n" % (width, height))
        fh.write(pixels[::-1, :].tobytes())


def write_yaml(path, image_name, resolution, origin_x, origin_y):
    """Match map_tools/occupancy_grid.py's write_yaml field order/format."""
    with open(path, "w") as fh:
        fh.write("image: %s\n" % image_name)
        fh.write("resolution: %.6f\n" % resolution)
        fh.write("origin: [%.6f, %.6f, 0.0]\n" % (origin_x, origin_y))
        fh.write("negate: 0\n")
        fh.write("occupied_thresh: 0.65\n")
        fh.write("free_thresh: 0.196\n")


class ClipResult(object):
    def __init__(self, orig_width, orig_height, clip_width, clip_height,
                 clip_origin_x, clip_origin_y, col0, row0,
                 discarded_count, orig_occupied_count, unchanged):
        self.orig_width = orig_width
        self.orig_height = orig_height
        self.clip_width = clip_width
        self.clip_height = clip_height
        self.clip_origin_x = clip_origin_x
        self.clip_origin_y = clip_origin_y
        self.col0 = col0
        self.row0 = row0
        self.discarded_count = discarded_count
        self.orig_occupied_count = orig_occupied_count
        self.unchanged = unchanged

    @property
    def discarded_pct(self):
        if self.orig_occupied_count == 0:
            return 0.0
        return 100.0 * self.discarded_count / self.orig_occupied_count


UNKNOWN = 205


def mask_unknown_cells(pixels, dtm_heights, map_ox, map_oy, map_res,
                        dtm_ox, dtm_oy, dtm_res):
    """Return a copy of `pixels` with UNKNOWN (205) written at every cell
    that has no terrain: either its DTM cell is NaN, or it falls outside the
    DTM's footprint entirely. Cells with valid terrain are left unchanged,
    whether they were free or occupied.

    `pixels` and `dtm_heights` are both row 0 = LOWEST y (internal
    convention). An OCCUPIED object cell with no terrain underneath it is
    also masked to unknown -- an object marker floating over off-mesh void
    (e.g. a prop beyond the terrain edge) is not meaningful ground info
    either, and masking it keeps the rule simple and total: "no terrain ->
    unknown", with no special case for occupied vs free.
    """
    height, width = pixels.shape
    dtm_height, dtm_width = dtm_heights.shape

    out = pixels.copy()

    rows = np.arange(height)
    cols = np.arange(width)
    world_y = map_oy + (rows + 0.5) * map_res
    world_x = map_ox + (cols + 0.5) * map_res

    dtm_row = np.floor((world_y - dtm_oy) / dtm_res).astype(np.int64)
    dtm_col = np.floor((world_x - dtm_ox) / dtm_res).astype(np.int64)

    in_bounds = ((dtm_row >= 0) & (dtm_row < dtm_height))[:, None] & \
                ((dtm_col >= 0) & (dtm_col < dtm_width))[None, :]

    has_terrain = np.zeros((height, width), dtype=bool)
    r_clip = np.clip(dtm_row, 0, dtm_height - 1)
    c_clip = np.clip(dtm_col, 0, dtm_width - 1)
    sampled = dtm_heights[r_clip[:, None], c_clip[None, :]]
    has_terrain = in_bounds & ~np.isnan(sampled)

    out[~has_terrain] = UNKNOWN
    return out


def _edt_1d_sq(f):
    """Exact squared distance transform of a 1-D array of ANY-distance
    seeds, using the Felzenszwalt & Huttenlocher lower-envelope-of-parabolas
    algorithm. `f` holds 0 at seed positions and a large sentinel (>= any
    possible true squared distance) elsewhere. Operates along axis -1;
    caller loops/transposes for 2-D. Pure numpy/stdlib, O(n) per line."""
    n = f.shape[0]
    INF = 1e20
    d = np.empty(n, dtype=np.float64)
    v = np.zeros(n, dtype=np.int64)      # locations of parabolas in envelope
    z = np.empty(n + 1, dtype=np.float64)  # boundaries between parabolas
    k = 0
    v[0] = 0
    z[0] = -INF
    z[1] = INF
    for q in range(1, n):
        while True:
            p = v[k]
            s = ((f[q] + q * q) - (f[p] + p * p)) / (2.0 * q - 2.0 * p)
            if s <= z[k]:
                k -= 1
                if k < 0:
                    k = 0
                    break
            else:
                break
        k += 1
        v[k] = q
        z[k] = s
        z[k + 1] = INF

    k = 0
    for q in range(n):
        while z[k + 1] < q:
            k += 1
        p = v[k]
        d[q] = (q - p) * (q - p) + f[p]
    return d


def _euclidean_distance_to_seeds(seed_mask):
    """Exact Euclidean distance (in CELLS) from every cell to the nearest
    True cell in `seed_mask`, via the separable squared-EDT algorithm
    (columns then rows). No scipy dependency."""
    height, width = seed_mask.shape
    INF = 1e20
    f = np.where(seed_mask, 0.0, INF)

    # Pass 1: down each column.
    for c in range(width):
        f[:, c] = _edt_1d_sq(f[:, c])

    # Pass 2: across each row.
    for r in range(height):
        f[r, :] = _edt_1d_sq(f[r, :])

    return np.sqrt(f)


def erode_unknown_cells(pixels, metres, resolution):
    """Return a copy of `pixels` with UNKNOWN (205) additionally written at
    every cell whose centre lies within `metres` (Euclidean, true distance
    computed from `resolution`) of any cell that is already UNKNOWN in the
    input. Only ever adds unknown cells -- never reverts one to free/occupied.

    Chosen behaviour for OCCUPIED cells inside the erosion band: they are
    also turned to UNKNOWN, for the same reason `mask_unknown_cells` already
    treats "no terrain" as total -- an object marker sitting right at the
    edge of a void is not trustworthy ground info either, and the whole
    point of --erode is a keep-out margin the planner must not enter, so an
    occupied cell there should not be quietly treated as a safer boundary
    than free space is.
    """
    if metres <= 0:
        return pixels.copy()

    seed_mask = pixels == UNKNOWN
    if not seed_mask.any():
        return pixels.copy()

    dist_cells = _euclidean_distance_to_seeds(seed_mask) * resolution

    out = pixels.copy()
    out[dist_cells <= metres] = UNKNOWN
    return out


def compute_clip(pixels, map_ox, map_oy, map_res, dtm_ox, dtm_oy, dtm_res,
                  dtm_width, dtm_height):
    """Compute the crop window and stats. Returns (clipped_pixels, ClipResult).

    `pixels` is row 0 = LOWEST y, matching the internal (non-file) convention.
    """
    height, width = pixels.shape
    occupied = pixels == 0
    orig_occupied_count = int(occupied.sum())

    # DTM world-coordinate footprint.
    dtm_max_x = dtm_ox + dtm_width * dtm_res
    dtm_max_y = dtm_oy + dtm_height * dtm_res

    # Sub-rectangle of the object map that overlaps the DTM footprint,
    # snapped OUTWARD to whole object-map cells so the crop is a pure
    # crop -- never resampled/shifted, and never accidentally excludes a
    # partially-covered edge cell.
    col0 = max(0, int(math_floor((dtm_ox - map_ox) / map_res)))
    row0 = max(0, int(math_floor((dtm_oy - map_oy) / map_res)))
    col1 = min(width, int(math_ceil((dtm_max_x - map_ox) / map_res)))
    row1 = min(height, int(math_ceil((dtm_max_y - map_oy) / map_res)))

    clip_width = max(0, col1 - col0)
    clip_height = max(0, row1 - row0)

    unchanged = (col0 == 0 and row0 == 0 and clip_width == width
                 and clip_height == height)

    if unchanged:
        clipped = pixels
        clip_ox, clip_oy = map_ox, map_oy
    else:
        clipped = pixels[row0:row1, col0:col1]
        clip_ox = map_ox + col0 * map_res
        clip_oy = map_oy + row0 * map_res

    clipped_occupied_count = int((clipped == 0).sum())
    discarded_count = orig_occupied_count - clipped_occupied_count

    result = ClipResult(
        orig_width=width, orig_height=height,
        clip_width=clipped.shape[1], clip_height=clipped.shape[0],
        clip_origin_x=clip_ox, clip_origin_y=clip_oy,
        col0=col0, row0=row0,
        discarded_count=discarded_count,
        orig_occupied_count=orig_occupied_count,
        unchanged=unchanged,
    )
    return clipped, result


def math_floor(x):
    import math
    return math.floor(x + 1e-9)  # tiny epsilon guards against fp round-down


def math_ceil(x):
    import math
    return math.ceil(x - 1e-9)  # tiny epsilon guards against fp round-up


def build(world, maps_dir="maps", out_suffix="_clipped", dry_run=False,
          mask_unknown=False, no_crop=False, erode=0.0):
    if erode > 0 and not mask_unknown:
        raise ValueError("--erode requires --mask-unknown (eroding is "
                          "meaningless if no cells are marked unknown yet)")

    map_yaml = os.path.join(maps_dir, "%s_map.yaml" % world)
    map_pgm = os.path.join(maps_dir, "%s_map.pgm" % world)
    dtm_yaml = os.path.join(maps_dir, "%s_dtm.yaml" % world)

    map_ox, map_oy, map_res = read_map_yaml(map_yaml)
    dtm_meta = read_dtm_yaml(dtm_yaml)

    pixels = read_pgm(map_pgm)

    if no_crop:
        # Skip the rectangular crop entirely: output keeps the source
        # object map's exact width, height, origin and resolution -- a
        # pure pixel-wise transform, same geometry in and out. This is
        # what the ROS costmap needs: the mask-unknown output must stay
        # on the SAME grid as the other StaticLayer inputs (e.g. the
        # slope layer), or costmap_2d::LayeredCostmap::resizeMap
        # truncates whichever map arrived first.
        clipped = pixels
        result = ClipResult(
            orig_width=pixels.shape[1], orig_height=pixels.shape[0],
            clip_width=pixels.shape[1], clip_height=pixels.shape[0],
            clip_origin_x=map_ox, clip_origin_y=map_oy,
            col0=0, row0=0,
            discarded_count=0,
            orig_occupied_count=int((pixels == 0).sum()),
            unchanged=True,
        )
    else:
        clipped, result = compute_clip(
            pixels, map_ox, map_oy, map_res,
            dtm_meta["origin_x"], dtm_meta["origin_y"], dtm_meta["resolution"],
            dtm_meta["width"], dtm_meta["height"])

    if mask_unknown:
        dtm_npy = os.path.join(maps_dir, "%s_dtm.npy" % world)
        dtm_heights = np.load(dtm_npy)
        clipped = mask_unknown_cells(
            clipped, dtm_heights, result.clip_origin_x, result.clip_origin_y,
            map_res, dtm_meta["origin_x"], dtm_meta["origin_y"],
            dtm_meta["resolution"])

        if erode > 0:
            before_unknown = int((clipped == UNKNOWN).sum())
            clipped = erode_unknown_cells(clipped, erode, map_res)
            result.eroded_count = int((clipped == UNKNOWN).sum()) - before_unknown
        else:
            result.eroded_count = 0
    else:
        result.eroded_count = 0

    out_base = os.path.join(maps_dir, "%s_map%s" % (world, out_suffix))
    if not dry_run:
        write_pgm(out_base + ".pgm", clipped)
        write_yaml(out_base + ".yaml", "%s_map%s.pgm" % (world, out_suffix),
                   map_res, result.clip_origin_x, result.clip_origin_y)

    return result


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Clip a world's object occupancy map to its terrain "
                    "(DTM) footprint.")
    ap.add_argument("world", help="world name, e.g. lake or park")
    ap.add_argument("--maps-dir", default="maps")
    ap.add_argument("--out-suffix", default="_clipped")
    ap.add_argument("--dry-run", action="store_true",
                    help="report the numbers without writing files")
    ap.add_argument("--mask-unknown", action="store_true",
                    help="write pixel 205 (unknown) at every cell with no "
                         "terrain (NaN DTM or outside the DTM footprint), "
                         "regardless of the source object map's value there")
    ap.add_argument("--no-crop", action="store_true",
                    help="skip the rectangular crop; output keeps the "
                         "source object map's exact width, height, origin "
                         "and resolution (needed so --mask-unknown output "
                         "stays grid-aligned with other costmap layers)")
    ap.add_argument("--erode", type=float, default=0.0,
                    help="after --mask-unknown, additionally mark as "
                         "unknown every cell within this many METRES "
                         "(true Euclidean distance) of any no-terrain "
                         "cell, so the robot footprint cannot overhang "
                         "the terrain edge. Requires --mask-unknown.")
    args = ap.parse_args(argv)

    try:
        result = build(args.world, args.maps_dir, args.out_suffix,
                       args.dry_run, args.mask_unknown, args.no_crop,
                       args.erode)
    except (ValueError, IOError, OSError) as exc:
        sys.stderr.write("error: %s\n" % exc)
        return 1

    print("clip map to terrain: %s%s" % (args.world, " (dry run)" if args.dry_run else ""))
    print("  original dims : %dx%d" % (result.orig_width, result.orig_height))
    print("  clipped dims  : %dx%d" % (result.clip_width, result.clip_height))
    if result.unchanged:
        print("  no-op: object map already within the terrain footprint")
    print("  occupied cells discarded : %d / %d (%.2f%%)"
          % (result.discarded_count, result.orig_occupied_count,
             result.discarded_pct))
    if args.erode > 0:
        print("  eroded to unknown        : %d additional cells (--erode %.2f m)"
              % (result.eroded_count, args.erode))
    if not args.dry_run:
        print("  -> %s_map%s.{pgm,yaml}"
              % (os.path.join(args.maps_dir, args.world), args.out_suffix))
    return 0


if __name__ == "__main__":
    sys.exit(main())
