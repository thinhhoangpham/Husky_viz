import os
from map_tools.sdf_parse import parse_models

WORLD = os.path.join(os.path.dirname(__file__), "..", "..",
                     "natural_environments_ros_opt", "natural_enviroment",
                     "worlds", "park.world")


def test_three_poles_present():
    models = parse_models(WORLD)
    poles = [m for m in models if m.family == "postescable"]
    assert len(poles) == 3
    got = sorted((round(m.world_x, 2), round(m.world_y, 2)) for m in poles)
    want = sorted([(-16.33, 5.28), (40.01, 17.26), (-0.51, -24.25)])
    assert got == want
