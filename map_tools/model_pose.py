"""Resolve a Gazebo model's EFFECTIVE world pose from an SDF .world file.

Why this module exists: a .world file can specify the same model's pose TWICE,
and the two can disagree by metres.

  1. The <model name='X'> ... <pose> in the world's static definition section.
  2. A <state world_name='...'> block near the end of the file, which holds a
     saved runtime snapshot -- <model name='X'><pose>...</pose>.

When a <state> entry exists for a model, Gazebo applies it ON LOAD, so it is
the pose the simulation actually runs with. The definition <pose> is then dead
data. Reading only the definition pose silently places geometry wherever the
world was authored rather than where it is simulated.

This is not hypothetical -- measured in this repo's own worlds:

  lake.world  terreno_lago  definition (-5.97581, 12.5754, 0)
                            STATE      ( 0,        0,      5)
              => 13.9 m horizontal and 5.0 m vertical error if state is ignored
  lake.world  lago          definition (-22.0983,  5.73074,  0)
                            STATE      (-12.6254,  5.66516, -0.828242)
              => 9.5 m horizontal error
  park.world  camino_parque definition ( 0, 0, 0)
                            STATE      ( 0.038597, -1.15723, 2.93639)
              => 2.9 m vertical error

Note that park.world's `terreno_parque` has NO state entry, so its definition
pose (0 0 0) is correct -- which is exactly why the precedence must be applied
unconditionally rather than assumed one way or the other per world.

This is the same class of defect as ignoring COLLADA <node> <matrix> transforms
(see mesh_bounds.py, where that bug put a bench footprint ~1.2 m off). Both are
"a second transform exists elsewhere in the file and overrides what you read".

Scope: this module resolves a MODEL's pose by name. It deliberately does not
classify models or read link poses -- map_tools/sdf_parse.py already does that
for catalog objects (it reads link_0 out of the state block). This module is
for models that are not in the type registry at all, such as the terrain.
"""
import re

# A 6-DOF SDF pose: x y z roll pitch yaw. Gazebo writes floats that may be in
# exponent form ("2e-06") or signed, hence the permissive character class.
_NUM = r"([-+]?[\d.]+(?:[eE][-+]?\d+)?)"
_POSE_RE = re.compile(
    r"<pose[^>]*>\s*" + r"\s+".join([_NUM] * 6) + r"\s*</pose>")


class Pose(object):
    """A model's world pose. Only the fields the DTM rasteriser needs."""

    __slots__ = ("x", "y", "z", "roll", "pitch", "yaw", "source")

    def __init__(self, x, y, z, roll, pitch, yaw, source):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        # "state" or "definition" -- kept so callers/tests can assert WHICH
        # pose won, not merely that some pose was returned.
        self.source = source

    def __repr__(self):
        return ("Pose(x=%.6g, y=%.6g, z=%.6g, roll=%.6g, pitch=%.6g, "
                "yaw=%.6g, source=%r)"
                % (self.x, self.y, self.z, self.roll, self.pitch, self.yaw,
                   self.source))


def _model_block(text, name, start=0, end=None):
    """Return the text of <model name='NAME'> ... </model> starting at/after
    `start`, or None. Nested <model> elements are handled by depth counting:
    a model block can contain child models, so the FIRST </model> is not
    necessarily the right closing tag.
    """
    if end is None:
        end = len(text)
    open_re = re.compile(r"<model name='%s'>" % re.escape(name))
    m = open_re.search(text, start, end)
    if m is None:
        return None

    # Walk model open/close tags from the match, tracking nesting depth.
    tag_re = re.compile(r"<model\b[^>]*>|</model>")
    depth = 0
    pos = m.start()
    while True:
        t = tag_re.search(text, pos, end)
        if t is None:
            # Malformed/truncated world file: no matching close tag.
            return None
        if t.group(0).startswith("</"):
            depth -= 1
            if depth == 0:
                return text[m.start():t.end()]
        else:
            depth += 1
        pos = t.end()


def _state_bounds(text):
    """(start, end) of the <state world_name=...> block, or None if absent."""
    try:
        start = text.index("<state world_name=")
    except ValueError:
        return None
    try:
        end = text.index("</state>", start)
    except ValueError:
        return None
    return start, end


def _model_pose_in(text, name, start=0, end=None):
    """The model's OWN <pose>, i.e. the direct child of <model name='NAME'>.

    A model block also contains <link>, <inertial>, <visual> and <collision>
    poses; those are LOCAL to the model and must not be mistaken for it. In a
    state block the model's pose is the first <pose> in the block (it precedes
    the <link> entries), but in a definition block the model's pose often comes
    LAST, after the link. So neither "first" nor "last" is universally right.

    Instead, strip out every nested element that can carry its own <pose>, then
    take the first <pose> that survives -- that is the model's own by
    construction, regardless of ordering.
    """
    block = _model_block(text, name, start, end)
    if block is None:
        return None

    # Drop the opening <model ...> tag so a pose attribute on it can't match,
    # then remove nested pose-bearing elements entirely.
    body = block.split(">", 1)[1] if ">" in block else block
    for tag in ("link", "inertial", "visual", "collision", "joint",
                "sensor", "model"):
        body = re.sub(r"<%s\b.*?</%s>" % (tag, tag), "", body,
                      flags=re.DOTALL)

    m = _POSE_RE.search(body)
    if m is None:
        return None
    return tuple(float(g) for g in m.groups())


def resolve_pose(world_text, name):
    """Return the effective Pose for model `name`, or None if it has neither.

    PRECEDENCE (the whole point of this module):
        <state> pose   if the state block has an entry for this model,
        <model> pose   otherwise.

    See the module docstring for the measured metre-scale errors that result
    from getting this backwards.
    """
    bounds = _state_bounds(world_text)
    if bounds is not None:
        s_start, s_end = bounds
        vals = _model_pose_in(world_text, name, s_start, s_end)
        if vals is not None:
            return Pose(*(vals + ("state",)))
        # Fall through: this model has no state entry (e.g. park's
        # terreno_parque), so the definition pose is the live one.
        def_end = s_start
    else:
        def_end = len(world_text)

    vals = _model_pose_in(world_text, name, 0, def_end)
    if vals is None:
        return None
    return Pose(*(vals + ("definition",)))


def resolve_pose_from_file(world_path, name):
    """resolve_pose() for a world on disk."""
    with open(world_path, "r") as fh:
        return resolve_pose(fh.read(), name)
