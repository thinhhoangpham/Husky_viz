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

# Tree exclusion. A tree is a ROUND footprint (aspect < aspect_split) with a
# trunk-scale minor radius. It is identified by that FOOTPRINT, not by height:
# the localizer crops points to z in [-0.73, 1.2], so every cluster is <= 1.93 m
# tall and real trunks appear SHORT. A height floor would make the gate dead
# (all trees fall through to the bin band, bin height 1.041 +/- 1.0). So there
# is NO minimum-height requirement for the footprint tells.
#
# The gate must catch real trunks (arbolpartes4 radius ~0.30 -> ~0.6 m minor,
# tree_8 radius ~0.45 -> ~0.9 m minor, stumps down to ~0.4 m minor) WITHOUT
# swallowing the four landmark families. Real trunk minors (~0.4-0.9) overlap
# the bin (0.382) and lamp (0.483) footprints, so three independent trunk tells,
# ANY of which suffices:
#   1. wide trunk: minor >= _TREE_TRUNK_MINOR (0.50, wider than bin 0.382 AND
#      lamp 0.483) -> catches tree_8, arbolpartes4, tall trunks. Bin/lamp spared.
#   2. round stump: clearly round (aspect <= _TREE_STUMP_ASPECT) AND not
#      lamp-tall (height < _LAMP_BAND_BOTTOM). Catches thin stumps (minor ~0.4)
#      whose footprint sits in the bin band but whose aspect (~1.25) is far
#      rounder than the bin (1.79). Spares the bin (aspect 1.79 > 1.5) and the
#      lamp (genuinely tall: 3.148 >= lamp band bottom; the crop caps trunks at
#      1.93 m so no trunk can be lamp-tall).
#   3. tall trunk: height >= _TREE_TALL_HEIGHT (> lamp band top) -> catches thin,
#      very tall trunks in unclipped views. Lamp (3.148 < 3.95) spared.
# Bench/table are elongated (aspect >= aspect_split) so tell (1)-(3) never see
# them.
_TREE_MIN_MINOR = 0.30       # below this is a lamp pole / noise, not a trunk
_TREE_MAX_MINOR = 1.0        # above this is not a single trunk
_TREE_TRUNK_MINOR = 0.50     # > bin 0.382 and lamp 0.483: a definite trunk radius
_TREE_STUMP_ASPECT = 1.5     # rounder than the bin (aspect 1.79): a round stump
_LAMP_BAND_BOTTOM = 2.148    # lamp 3.148 - height margin 1.0; a trunk (<=1.93 m
                             # under the crop) is never this tall, the lamp is
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
    """A round, trunk-footprint cluster — never a landmark family.

    Runs BEFORE family matching and wins, so a real trunk that also falls inside
    a family band (bin or lamp) is still excluded from identity. Identified by
    the ROUND, trunk-scale FOOTPRINT — NOT by a height floor, because the crop
    caps clusters at ~1.93 m and real trunks appear short. See the module-level
    comment for the three tells and why each spares the four families.
    """
    aspect = cluster.major / max(cluster.minor, 1e-3)
    if aspect >= margins["aspect_split"]:
        return False  # elongated: bench / table
    if not (_TREE_MIN_MINOR <= cluster.minor <= _TREE_MAX_MINOR):
        return False
    # (1) wide trunk radius
    if cluster.minor >= _TREE_TRUNK_MINOR:
        return True
    # (3) thin but clearly taller than any lamp
    if cluster.height >= _TREE_TALL_HEIGHT:
        return True
    # (2) round stump: rounder than a bin and not lamp-tall
    return (aspect <= _TREE_STUMP_ASPECT
            and cluster.height < _LAMP_BAND_BOTTOM)


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
