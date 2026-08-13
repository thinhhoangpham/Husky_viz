"""Load the known landmark catalog and gate it to the robot's plausible view.

The catalog is maps/park_places.yaml (name -> map-frame x,y). Each name is
mapped to a landmark identity via map_tools.sdf_parse.classify. gate() prunes
the catalog to landmarks a robot at the prior pose could currently see, so
association is a local problem, not a global search.
"""
import math
from dataclasses import dataclass
import yaml
from map_tools.sdf_parse import classify as _family_of

_IDENTITY_FAMILIES = {"bench", "garden_table", "lamp", "trash_bin_1", "tree"}

# map raw world-file family -> matcher identity (tree_8 model -> generic 'tree')
_FAMILY_TO_IDENTITY = {"tree_8": "tree"}


def _identity_of(name):
    fam = _family_of(name)
    return _FAMILY_TO_IDENTITY.get(fam, fam)


@dataclass
class MapLandmark:
    name: str
    identity: str
    x: float
    y: float
    yaw: float = None


def load(places_path):
    with open(places_path) as fh:
        data = yaml.safe_load(fh)
    out = []
    for name, xy in data.items():
        fam = _identity_of(name)
        if fam not in _IDENTITY_FAMILIES:
            continue
        out.append(MapLandmark(name, fam, float(xy["x"]), float(xy["y"]),
                               float(xy["yaw"]) if "yaw" in xy else None))
    return out


def gate(landmarks, prior_xyz, max_range, fov_halfwidth):
    px, py, pyaw = prior_xyz
    c, s = math.cos(-pyaw), math.sin(-pyaw)
    kept = []
    for lm in landmarks:
        dx, dy = lm.x - px, lm.y - py
        # rotate world delta into robot frame (robot forward = +x)
        rx = c * dx - s * dy
        ry = s * dx + c * dy
        rng = math.hypot(rx, ry)
        if rng > max_range or rng < 1e-6:
            continue
        bearing = math.atan2(ry, rx)
        if abs(bearing) <= fov_halfwidth:
            kept.append(lm)
    return kept
