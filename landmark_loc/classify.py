"""Rule-based classification of a lidar cluster into a park landmark type.

Bands are centered on the mesh-derived signatures (signatures.MESH_SIGNATURES)
and widened by DEFAULT_MARGINS to absorb partial views. Deliberately
CONSERVATIVE: a cluster matching zero or more-than-one type is 'unknown' and is
dropped downstream. Trees are recognized by their vertical profile (a wide
canopy band above a narrow trunk) and EMITTED as the 'tree' landmark type.
"""
from dataclasses import dataclass
from landmark_loc.signatures import MESH_SIGNATURES, SIGNATURE_FAMILIES

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


@dataclass
class Observation:
    identity: str
    x: float
    y: float


def _matches(cluster, fam, m):
    sig = MESH_SIGNATURES[fam]
    if abs(cluster.major - sig["major"]) > m["major"]:
        return False
    if abs(cluster.minor - sig["minor"]) > m["minor"]:
        return False
    if abs(cluster.height - sig["height"]) > m["height"]:
        return False
    aspect = cluster.major / max(cluster.minor, 1e-3)
    sig_aspect = sig["major"] / max(sig["minor"], 1e-3)
    elongated = aspect >= m["aspect_split"]
    sig_elongated = sig_aspect >= m["aspect_split"]
    return elongated == sig_elongated


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


def classify_cluster(cluster, margins=DEFAULT_MARGINS):
    # Tree gate first and it wins: a real trunk must be excluded from identity
    # even when it also satisfies a family band (e.g. the lamp band).
    if _is_tree(cluster, margins):
        return "tree"
    hits = [fam for fam in SIGNATURE_FAMILIES if _matches(cluster, fam, margins)]
    if len(hits) == 1:
        return hits[0]
    return "unknown"


def to_observations(clusters, margins=DEFAULT_MARGINS):
    out = []
    for c in clusters:
        ident = classify_cluster(c, margins)
        if ident == "unknown":
            continue
        out.append(Observation(identity=ident,
                               x=c.centroid_xy[0], y=c.centroid_xy[1]))
    return out
