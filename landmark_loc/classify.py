"""Rule-based classification of a lidar cluster into a park landmark type.

Bands are centered on the mesh-derived signatures (signatures.MESH_SIGNATURES)
and widened by DEFAULT_MARGINS to absorb partial views. Deliberately
CONSERVATIVE: a cluster matching zero or more-than-one type is 'unknown' and is
dropped downstream. Trees are recognized by their vertical profile (a wide
canopy band above a narrow trunk) and EMITTED as the 'tree' landmark type.
"""
import math
from dataclasses import dataclass
from landmark_loc.signatures import MESH_SIGNATURES
from landmark_loc import shapefit
from landmark_loc import shapefeat

# Shape-signature thresholds (Task 2). Each pinned from measured live/mesh
# values; see .superpowers/sdd/2026-08-13-shape-classifier/task-2-brief.md.
_LAMP_POST_MAX = 0.35    # m: lamp post foot_diag ~0.14; captured lamps 0.12-0.51 low
# 1.4 -> 1.2 (Task 4): mesh bin height is 1.041 m (mesh_bounds.bounds3d). A
# lidar sees the TOP of a short solid object, so a bin's apparent height is
# view-stable and won't shrink much below the mesh value -- 1.041 + ~0.15 m
# slop = 1.2 is a tighter, better-justified ceiling than the old 1.4, and it
# shrinks the tall-fragment phantom window before the lamps start (~2.0-2.5 m).
# PROVISIONAL: pin against a CLEAN in-sim bin capture -- captured cluster [13]
# (h=1.679) is foliage-contaminated (segmenter fused overhanging tree canopy
# onto the bin body), so no clean bin height was captured this session.
_BIN_MAX_H = 1.2         # m: mesh bin height 1.041 + ~0.15 view-stability slop
_BIN_FOOT_MIN = 0.30     # m: bin foot_major ~0.68; keep off sub-0.3 noise fragments
_BIN_FOOT_MAX = 1.20     # m: below bench 1.78 major, above bin 0.68 with margin
# Genuine footprint-shape discriminator (Task 4): a real bin's footprint is
# close to square/oblong (captured bin [13] aspect 1.33; mesh aspect 1.79),
# while flat elongated ground-scatter fragments [1]/[3] measure aspect
# 2.38-2.41 despite sitting inside the same major/height band. 2.0 sits
# cleanly between the two groups; independently load-bearing (not merely a
# side effect of any particular _BIN_MAX_H value).
_BIN_ASPECT_MAX = 2.0
# 1.4 -> 1.2 (Task 4): the taller of the two box objects is garden_table at
# mesh height 1.085 m (bench is 0.942 m). Box height is view-STABLE (the
# lidar sees the tabletop/seat surface of a solid low object), so
# 1.085 + ~0.12 m slop = 1.2 is a tighter, mesh-grounded ceiling than the old
# 1.40 -- closes the 1.09-1.40 m window where tall fragments could slip into
# the bench/table classes. Same logic as _BIN_MAX_H above.
# PROVISIONAL: pin against clean captures in-sim -- no bench/table cluster
# near this ceiling was captured this session (captured bench [12] is 0.44 m,
# far under both 1.2 and 1.4).
_BOX_MAX_H = 1.20        # m: table mesh height 1.085 + ~0.12 view-stability slop; bench mesh 0.942 also under this
_BENCH_MAJOR_MIN = 1.20  # m: bench major 1.78; near-edge foreshortening floor
_TABLE_MAJOR_MIN = 2.30  # m: table major 3.00; splits table (>=2.3) from bench (<2.3)

# Tolerances. major/minor in metres, aspect is a ratio, height in metres.
# Pinned against live lidar (Task 1 in-sim NOTE).
DEFAULT_MARGINS = {
    "major": 0.8,      # +/- m on the major horizontal extent
    "minor": 0.6,      # +/- m on the minor horizontal extent
    "height": 1.0,     # +/- m on height (principled: absorbs partial vertical
                       # views; lamp<->bin stay separated by their 2.11 m gap)
    "aspect_split": 1.8,   # major/minor above this = elongated (bench-like)
}

# Tree = a wide canopy band ABOVE a narrow trunk. Keys on the vertical PROFILE
# (view-robust), not the canopy's absolute size (which varies 3.7-4.75 m in-sim).
# Thresholds measured live (13 trees, 4 viewpoints): canopy begins by z~2.75
# (p50 2.25), canopy width 2.9-4.75 m; lamps stay <1 m wide at every height.
_TREE_CANOPY_MIN_Z = 2.5      # a wide band at/above this height is a canopy
_TREE_CANOPY_MIN_WIDTH = 2.0  # canopy horizontal width floor (lamp head < 1 m)
_TREE_BAND = 0.5              # z-band thickness for the profile scan
_TREE_BAND_MIN_PTS = 3        # a band needs this many points to measure width
_TRUNK_BAND = 1.0             # trunk region height above cluster base, used to
                               # estimate a tree's stable position from the base
                               # (trunk) rather than the wandering canopy mean

# Known object radius by identity, metres. The lidar only sees the near face of
# an object, so a cluster's raw centroid sits ~one radius TOWARD the robot from
# the object's true center (used in to_observations to push the observation
# back out to the true center). Round/near-round types: half the minor
# horizontal extent from the mesh signature. Trees are not in MESH_SIGNATURES
# (they're identified by vertical profile, not a size band), so the trunk
# radius is hardcoded from the extractor's RADII table (tree_8 = 0.45 m).
KNOWN_RADIUS = {
    "lamp": MESH_SIGNATURES["lamp"]["minor"] / 2.0,
    "trash_bin_1": MESH_SIGNATURES["trash_bin_1"]["minor"] / 2.0,
    "bench": MESH_SIGNATURES["bench"]["minor"] / 2.0,
    "garden_table": MESH_SIGNATURES["garden_table"]["minor"] / 2.0,
    "tree": 0.45,  # tree_8 trunk radius (extractor RADII); arbolpartes4 not separately modeled here
}

# Real rectangle footprints (length, width) in metres, from the mesh signatures.
# Only elongated types get the ICP shape fit; round types keep centroid+pushout.
_RECT_FOOTPRINT = {"bench": (1.78, 0.80), "garden_table": (3.00, 1.32)}


@dataclass
class Observation:
    identity: str
    x: float
    y: float
    yaw: float = None


def _band_width(pts, z0, z1):
    """Horizontal bbox-diagonal width of points whose z is in [z0, z1)."""
    m = (pts[:, 2] >= z0) & (pts[:, 2] < z1)
    if int(m.sum()) < _TREE_BAND_MIN_PTS:
        return 0.0
    xy = pts[m][:, :2]
    return float(((xy[:, 0].max() - xy[:, 0].min()) ** 2
                  + (xy[:, 1].max() - xy[:, 1].min()) ** 2) ** 0.5)


def _is_tree(cluster, margins):
    """True when the cluster has a wide canopy band at z >= _TREE_CANOPY_MIN_Z.

    A lamp is narrow (<_TREE_CANOPY_MIN_WIDTH) at every height, a bench/bin has no
    band that high, so neither can satisfy this. Uses the cluster's raw points
    (the vertical profile), not its bbox. Synthetic clusters with points=None are
    never trees (the four rigid-type tests build those)."""
    pts = cluster.points
    if pts is None or len(pts) == 0:
        return False
    top = float(pts[:, 2].max())
    z = _TREE_CANOPY_MIN_Z
    while z < top:
        if _band_width(pts, z, z + _TREE_BAND) >= _TREE_CANOPY_MIN_WIDTH:
            return True
        z += _TREE_BAND
    return False


def _trunk_xy(points):
    """Horizontal mean of a tree cluster's low (trunk) band, near its base.

    The canopy centroid wanders by metres depending on which portion the
    lidar sees; the trunk base is a stable, precise point that matches the
    catalog. Returns None if points is missing/empty; returns None if the
    low band has too few points (caller falls back to centroid_xy)."""
    if points is None or len(points) == 0:
        return None
    z0 = float(points[:, 2].min())
    band = points[(points[:, 2] >= z0) & (points[:, 2] < z0 + _TRUNK_BAND)]
    if len(band) < 3:
        return None
    return float(band[:, 0].mean()), float(band[:, 1].mean())


def classify_cluster(cluster, margins=DEFAULT_MARGINS):
    # Tree wins first: a real trunk+canopy must be excluded from the other types.
    if _is_tree(cluster, margins):
        return "tree"
    pts = cluster.points
    if pts is None or len(pts) < shapefeat._MIN_SHAPE_PTS:
        return "unknown"
    # lamp: the only object with a thin post rising above HIGH_Z.
    if shapefeat.has_thin_high_band(pts) and shapefeat.post_width(pts) < _LAMP_POST_MAX:
        return "lamp"
    foot_major, foot_minor = shapefeat.foot_extents(pts)
    height = cluster.height
    aspect = foot_major / max(foot_minor, 1e-6)
    # trash_bin: short compact oblong box, no tall thin post. The aspect cap
    # keeps flat elongated ground fragments (aspect >2) out of the footprint
    # band a real bin also occupies (see _BIN_ASPECT_MAX comment above).
    if (height < _BIN_MAX_H and _BIN_FOOT_MIN <= foot_major < _BIN_FOOT_MAX
            and aspect < _BIN_ASPECT_MAX):
        return "trash_bin_1"
    # garden_table: low box, long.
    if height < _BOX_MAX_H and foot_major >= _TABLE_MAJOR_MIN:
        return "garden_table"
    # bench: low box, medium length.
    if height < _BOX_MAX_H and _BENCH_MAJOR_MIN <= foot_major < _TABLE_MAJOR_MIN:
        return "bench"
    return "unknown"


def to_observations(clusters, margins=DEFAULT_MARGINS):
    out = []
    for c in clusters:
        ident = classify_cluster(c, margins)
        if ident == "unknown":
            continue
        yaw = None
        if ident in _RECT_FOOTPRINT and c.points is not None and len(c.points) > 0:
            L, W = _RECT_FOOTPRINT[ident]
            fx, fy, fyaw, ok = shapefit.fit_rectangle(c.points[:, :2], L, W)
            if ok:
                out.append(Observation(identity=ident, x=fx, y=fy, yaw=fyaw))
                continue
            # fall through to centroid+pushout on a failed fit
        # The lidar only sees the near surface, so the raw centroid sits one
        # radius toward the robot from the true center. Push it back out along
        # the robot->object direction (robot is at the origin in this frame)
        # to estimate the view-invariant true center.
        if ident == "tree":
            trunk = _trunk_xy(c.points)
            cx, cy = trunk if trunk is not None else c.centroid_xy
        else:
            cx, cy = c.centroid_xy
        r = math.hypot(cx, cy)
        radius = KNOWN_RADIUS.get(ident, 0.0)
        if r > 1e-6 and radius > 0.0:
            ux, uy = cx / r, cy / r
            ox, oy = cx + radius * ux, cy + radius * uy
        else:
            ox, oy = cx, cy
        out.append(Observation(identity=ident, x=ox, y=oy, yaw=yaw))
    return out
