"""Terrain slope costmap generator.

Turns a DTM height raster into a map_server PGM/YAML pair encoding terrain
steepness, so move_base's global costmap routes around steep ground.

Pure NumPy + stdlib -- NO ROS imports. Runs offline, unit-testable without a
roscore.

Spec: docs/superpowers/specs/2026-08-19-terrain-slope-costmap-design.md
"""
import argparse
import os
import sys

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


def occupancy_to_pixels(occ):
    """ROS occupancy 0..100 (plus -1 unknown) -> map_server PGM pixels.

    THE PGM IS INVERTED: map_server reads occ = (255 - pixel) / 255 * 100, so
    pixel 0 is fully occupied and 255 is free. Getting this backwards makes
    flat ground lethal and steep ground free. Same convention as
    map_tools/occupancy_grid.py.

    Unknown (-1) is written as pixel 205, which lands between free_thresh
    (0.196) and occupied_thresh (0.65) so map_server reports -1 for it.
    """
    occ = np.asarray(occ)
    px = np.round(255.0 - np.clip(occ, 0, 100) * 255.0 / 100.0)
    px = px.astype(np.uint8)
    px[occ < 0] = UNKNOWN_PIXEL
    return px


def write_pgm(path, pixels):
    """Binary P5 PGM.

    Row 0 of `pixels` is the LOWEST y (the .npy convention). The PGM format
    puts the TOP of the image first, and map_server's origin is the
    bottom-left corner -- so flip vertically on write.
    """
    pixels = np.asarray(pixels, dtype=np.uint8)
    height, width = pixels.shape
    with open(path, "wb") as fh:
        fh.write(b"P5\n%d %d\n255\n" % (width, height))
        fh.write(pixels[::-1, :].tobytes())


def write_yaml(path, image_name, grid, meta):
    """map_server YAML, plus the provenance a reader needs to interpret the PGM.

    The thresholds go in as comments: the PGM alone cannot say what angle a
    given pixel came from, so the pair (pgm, yaml) has to carry the mapping.
    """
    with open(path, "w") as fh:
        fh.write("# Terrain SLOPE costmap. Generated by "
                 "map_tools/slope_costmap.py -- do not hand-edit.\n")
        fh.write("# Pixel 255 = free (<warn_deg), 0 = lethal (>=lethal_deg),\n")
        fh.write("# 205 = unknown (no mesh coverage). Values between ramp\n")
        fh.write("# linearly across the warn..lethal band.\n")
        for key in sorted(meta):
            fh.write("# %s: %s\n" % (key, meta[key]))
        fh.write("image: %s\n" % image_name)
        fh.write("resolution: %.6f\n" % grid.resolution)
        fh.write("origin: [%.6f, %.6f, 0.0]\n"
                 % (grid.origin_x, grid.origin_y))
        fh.write("negate: 0\n")
        fh.write("occupied_thresh: 0.65\n")
        fh.write("free_thresh: 0.196\n")
        # mode: scale is REQUIRED and is the whole point of the graded band.
        # map_server's DEFAULT mode is `trinary`, which collapses every pixel to
        # one of three values: >occupied_thresh -> 100, <free_thresh -> 0, and
        # EVERYTHING IN BETWEEN -> -1 (unknown). That destroys the 10-18 deg
        # ramp before costmap_2d ever sees it -- measured: a PGM holding 102
        # distinct pixel values published as just {-1, 0, 100}, with the graded
        # band misreported as unknown. `scale` keeps the full 0..100 range.
        fh.write("mode: scale\n")


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


def _map_grid_from_yaml(path):
    """GridSpec for the occupancy map. width/height are not in the yaml, so
    they are supplied by the caller from the PGM or the DTM footprint."""
    meta = read_dtm_yaml(path)
    origin = meta["origin"]
    if isinstance(origin, str):
        parts = origin.strip("[]").split(",")
        ox, oy = float(parts[0]), float(parts[1])
    else:
        raise ValueError("could not parse origin from %s" % path)
    return ox, oy, float(meta["resolution"])


def read_pgm_dimensions(path):
    """Parse just the header of a binary P5 PGM and return (width, height).

    Reads only the header tokens (magic, width, height, maxval) -- never the
    pixel body -- so this is cheap even on a large map.
    """
    if not os.path.exists(path):
        raise IOError("object map PGM not found: %s (the slope layer must be "
                       "sized to match it, not the DTM footprint)" % path)

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


def build(world, maps_dir="maps", warn_deg=10.0, lethal_deg=18.0):
    """Full pipeline: DTM -> slope -> occupancy -> map grid -> PGM/YAML/NPY."""
    dtm_npy = os.path.join(maps_dir, "%s_dtm.npy" % world)
    dtm_yaml = os.path.join(maps_dir, "%s_dtm.yaml" % world)
    map_yaml = os.path.join(maps_dir, "%s_map.yaml" % world)
    map_pgm = os.path.join(maps_dir, "%s_map.pgm" % world)

    heights = np.load(dtm_npy)
    dmeta = read_dtm_yaml(dtm_yaml)
    src = GridSpec(dmeta["origin_x"], dmeta["origin_y"], dmeta["resolution"],
                   dmeta["width"], dmeta["height"])

    slope = slope_degrees(heights, src.resolution)
    occ_src = slope_to_occupancy(slope, warn_deg, lethal_deg)

    # Destination grid: MUST be pixel-identical to the object map's PGM, not
    # sized from the DTM footprint. move_base loads both as StaticLayers on
    # the same non-rolling costmap; if their dimensions differ, whichever
    # layer's map arrives last resizes the master costmap and truncates the
    # other (costmap_2d::StaticLayer::incomingMap -> resizeMap). Cells the
    # DTM does not cover are filled UNKNOWN_OCC by resample_nearest, which is
    # the correct behaviour for map area outside the DTM.
    ox, oy, res = _map_grid_from_yaml(map_yaml)
    dst_width, dst_height = read_pgm_dimensions(map_pgm)
    dst = GridSpec(ox, oy, res, dst_width, dst_height)

    occ_dst = resample_nearest(occ_src, src, dst, fill=UNKNOWN_OCC)

    base = os.path.join(maps_dir, "%s_slope" % world)
    np.save(base + ".npy", slope.astype(np.float32))
    write_pgm(base + ".pgm", occupancy_to_pixels(occ_dst))
    write_yaml(base + ".yaml", "%s_slope.pgm" % world, dst, {
        "world": world,
        "source_dtm": os.path.normpath(dtm_npy),
        "aligned_to": os.path.normpath(map_yaml),
        "warn_deg": warn_deg,
        "lethal_deg": lethal_deg,
        "max_slope_deg": "%.4f" % float(np.nanmax(slope)),
    })

    known = occ_dst >= 0
    n = float(known.sum()) or 1.0
    stats = {
        "free_pct": 100.0 * ((occ_dst == 0) & known).sum() / n,
        "graded_pct": 100.0 * ((occ_dst > 0) & (occ_dst < 100)).sum() / n,
        "lethal_pct": 100.0 * (occ_dst == 100).sum() / n,
        "unknown_pct": 100.0 * (~known).sum() / occ_dst.size,
        "max_slope_deg": float(np.nanmax(slope)),
    }
    return stats


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Build a terrain slope costmap from a world's DTM.")
    ap.add_argument("world", help="world name, e.g. lake or park")
    ap.add_argument("--maps-dir", default="maps")
    ap.add_argument("--warn-deg", type=float, default=10.0,
                    help="slope at which cost starts ramping up (default 10)")
    ap.add_argument("--lethal-deg", type=float, default=18.0,
                    help="slope treated as impassable (default 18)")
    args = ap.parse_args(argv)

    try:
        stats = build(args.world, args.maps_dir,
                      args.warn_deg, args.lethal_deg)
    except (ValueError, IOError, OSError) as exc:
        sys.stderr.write("error: %s\n" % exc)
        return 1

    print("slope costmap: %s  (warn %.1f deg, lethal %.1f deg)"
          % (args.world, args.warn_deg, args.lethal_deg))
    print("  max slope : %.2f deg" % stats["max_slope_deg"])
    print("  free      : %6.2f%% of known cells" % stats["free_pct"])
    print("  graded    : %6.2f%%" % stats["graded_pct"])
    print("  LETHAL    : %6.2f%%" % stats["lethal_pct"])
    print("  unknown   : %6.2f%% of grid" % stats["unknown_pct"])
    if stats["lethal_pct"] == 0.0 and stats["graded_pct"] == 0.0:
        print("  NOTE: terrain is flat -- this layer is a no-op, as expected "
              "for a world like the park.")
    print("  -> %s_slope.{npy,pgm,yaml}"
          % os.path.join(args.maps_dir, args.world))
    return 0


if __name__ == "__main__":
    sys.exit(main())
