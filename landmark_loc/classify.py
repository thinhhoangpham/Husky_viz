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
# Pinned against live lidar (Task 1 in-sim NOTE). Starting values below.
DEFAULT_MARGINS = {
    "major": 0.8,      # +/- m on the major horizontal extent
    "minor": 0.6,      # +/- m on the minor horizontal extent
    "height": 0.8,     # +/- m on height (< 0.85 so a 4 m tree trunk is not
                       # claimed by the 3.15 m lamp; still self-classifies all four)
    "aspect_split": 1.8,   # major/minor above this = elongated (bench-like)
}

# Tree exclusion: a roughly round footprint (aspect < aspect_split) with a
# trunk-scale minor extent AND tall. Distinguishes trunk from lamp pole by the
# larger trunk radius and from bins by height.
_TREE_MIN_HEIGHT = 2.0
_TREE_MIN_MINOR = 0.30   # trunk radius > lamp pole
_TREE_MAX_MINOR = 1.0


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


def classify_cluster(cluster, margins=DEFAULT_MARGINS):
    # A unique family match wins: the mesh signatures (incl. the lamp's head-
    # widened footprint) take precedence over the tree gate, whose minor band
    # overlaps the lamp pole. The tree gate below only catches clusters that no
    # single family claims.
    hits = [fam for fam in SIGNATURE_FAMILIES if _matches(cluster, fam, margins)]
    if len(hits) == 1:
        return hits[0]
    # tree gate: round + tall + trunk-radius, distinct from any lamp/bin match
    aspect = cluster.major / max(cluster.minor, 1e-3)
    if (aspect < margins["aspect_split"]
            and cluster.height >= _TREE_MIN_HEIGHT
            and _TREE_MIN_MINOR <= cluster.minor <= _TREE_MAX_MINOR):
        return "tree"
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
