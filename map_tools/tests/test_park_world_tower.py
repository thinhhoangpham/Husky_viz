"""The single distinctive object: exactly ONE water tower in park.world.

The whole single-distinctive-blob design rests on this object having no twin.
Six identical power-line poles is precisely the mistake that sank the earlier
per-object shape-matching attempt (descriptor distance between identical
instances is exactly 0), so the count assertion below is the point of the test,
not a formality.

NOTE: park.world lives under the git-ignored natural_environments_ros_opt/ tree,
so this test guards an edit that is NOT version controlled. The exact pose is
`20.0 14.0 2.99 0 -0 0` -- recorded here and in the commit message so the world
edit is reproducible from the repo alone.
"""
import os
from map_tools.sdf_parse import parse_models

WORLD = os.path.join(os.path.dirname(__file__), "..", "..",
                     "natural_environments_ros_opt", "natural_enviroment",
                     "worlds", "park.world")


def test_exactly_one_tower():
    towers = [m for m in parse_models(WORLD) if m.family == "water_tower"]
    assert len(towers) == 1, "the whole design depends on there being exactly ONE"
    assert round(towers[0].world_x, 1) == 20.0
    assert round(towers[0].world_y, 1) == 14.0
