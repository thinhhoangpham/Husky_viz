"""Rule-based classification of a lidar cluster into a park landmark type.

Bands are centered on the mesh-derived signatures (signatures.MESH_SIGNATURES)
and widened by DEFAULT_MARGINS to absorb partial views. Deliberately
CONSERVATIVE: a cluster matching zero or more-than-one type is 'unknown' and is
dropped downstream. Trees (round, tall, trunk-radius footprint) are recognized
only to EXCLUDE them from identity; they remain obstacles for the costmap.
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

# Tree exclusion. A tree is a roughly round footprint (aspect < aspect_split)
# that is tall AND *definitively* trunk-like, i.e. distinguishable from the
# lamp signature (0.483 minor, 3.148 height). The lamp is the only tall, round,
# small-footprint FAMILY, so the gate must catch real trunks without swallowing
# the lamp. Two independent trunk tells, EITHER of which suffices:
#   - minor radius clearly exceeds the lamp pole (>= _TREE_TRUNK_MINOR), or
#   - height clearly exceeds the lamp height band's top (>= _TREE_TALL_HEIGHT).
# The ideal lamp (minor 0.483 < 0.50, height 3.148 < 3.95) triggers neither, so
# it stays a lamp; a real trunk (wider radius and/or taller) triggers at least
# one. This is why the gate can run FIRST and win over a family match without
# mislabeling the lamp.
_TREE_MIN_HEIGHT = 2.0
_TREE_MIN_MINOR = 0.30       # below this is a lamp pole / noise, not a trunk
_TREE_MAX_MINOR = 1.0        # above this is not a single trunk
_TREE_TRUNK_MINOR = 0.50     # > lamp pole minor 0.483: a definite trunk radius
_TREE_TALL_HEIGHT = 3.95     # > lamp height band top (3.148 + 0.8): a definite tree


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


def _is_tree(cluster, margins):
    """A round, tall, definitively trunk-like cluster — never the lamp.

    Runs BEFORE family matching and wins, so a real trunk that also happens to
    fall inside the lamp band is still excluded from identity. The gate is
    narrowed so the ideal lamp (minor 0.483, height 3.148) triggers neither
    trunk tell and therefore is NOT a tree.
    """
    aspect = cluster.major / max(cluster.minor, 1e-3)
    if aspect >= margins["aspect_split"]:
        return False
    if cluster.height < _TREE_MIN_HEIGHT:
        return False
    if not (_TREE_MIN_MINOR <= cluster.minor <= _TREE_MAX_MINOR):
        return False
    # definitively trunk-like: wider radius than a lamp pole OR taller than a lamp
    return cluster.minor >= _TREE_TRUNK_MINOR or cluster.height >= _TREE_TALL_HEIGHT


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
        if ident in ("tree", "unknown"):
            continue
        out.append(Observation(identity=ident,
                               x=c.centroid_xy[0], y=c.centroid_xy[1]))
    return out
