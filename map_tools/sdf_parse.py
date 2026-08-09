"""Read obstacle positions from a Gazebo SDF world file (park.world).

Reads the <state world_name=...> snapshot for POSES and the model-name prefix
for CLASSIFICATION. Trees: the trunk is link_0 (verified in-sim 2026-08-09) --
for the multi-link arbolpartes4 that link is offset ~1 m from the model <pose>,
so we read link_0's pose, not the model pose. tree_8 is single-link so link_0
== model pose; reading link_0 is correct for both.
"""
import re
from dataclasses import dataclass

# Ordered longest-prefix-first so "trash_bin_1" is not shadowed, etc.
_FAMILY_PREFIXES = (
    "arbolpartes4",
    "tree_8",
    "garden_table",
    "trash_bin_1",
    "bench",
    "lamp",
)


def classify(name):
    for fam in _FAMILY_PREFIXES:
        if name == fam or name.startswith(fam + "_"):
            return fam
    return "skip"


@dataclass
class Model:
    name: str
    family: str
    world_x: float
    world_y: float
    yaw: float


# The <state world_name='default'> block holds each model's runtime pose and,
# nested, each link's pose. We parse THAT block. For arbolpartes4 we need
# link_0's pose (the trunk); for every other family the model pose == the link
# pose so either works and we use link_0 uniformly.
_MODEL_RE = re.compile(r"<model name='([^']+)'>")
_LINK_RE = re.compile(r"<link name='([^']+)'>")
_POSE_RE = re.compile(
    r"<pose[^>]*>\s*([-\d.eE]+)\s+([-\d.eE]+)\s+[-\d.eE]+\s+"
    r"[-\d.eE]+\s+[-\d.eE]+\s+([-\d.eE]+)\s*</pose>")


def _state_block(text):
    start = text.index("<state world_name=")
    end = text.index("</state>", start)
    return text[start:end]


def parse_models(world_path):
    """Return the obstacle Models (family != 'skip') from park.world.

    Position is the trunk/link_0 world pose from the <state> block.
    """
    with open(world_path, "r") as fh:
        text = fh.read()
    state = _state_block(text)

    models = []
    # Split the state block into per-model chunks.
    idxs = [m.start() for m in _MODEL_RE.finditer(state)]
    idxs.append(len(state))
    for i in range(len(idxs) - 1):
        chunk = state[idxs[i]:idxs[i + 1]]
        name = _MODEL_RE.search(chunk).group(1)
        family = classify(name)
        if family == "skip":
            continue
        # Find link_0's pose within this model chunk.
        link_x = link_y = None
        link_yaw = 0.0
        for lm in _LINK_RE.finditer(chunk):
            if lm.group(1) == "link_0":
                after = chunk[lm.end():]
                pm = _POSE_RE.search(after)
                if pm:
                    link_x = float(pm.group(1))
                    link_y = float(pm.group(2))
                    link_yaw = float(pm.group(3))
                break
        if link_x is None:
            # Fallback: model pose (first pose in the chunk).
            pm = _POSE_RE.search(chunk)
            link_x = float(pm.group(1))
            link_y = float(pm.group(2))
            link_yaw = float(pm.group(3))
        models.append(Model(name, family, link_x, link_y, link_yaw))
    return models
