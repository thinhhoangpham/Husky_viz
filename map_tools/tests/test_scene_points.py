import os

import numpy as np

from map_tools.sdf_parse import parse_models
from map_tools.scene_points import scene_cloud

WORLD = os.path.join(os.path.dirname(__file__), "..", "..",
                     "natural_environments_ros_opt", "natural_enviroment",
                     "worlds", "park.world")


def test_scene_cloud_spans_the_park_and_is_tall():
    ms = parse_models(WORLD)
    cloud = scene_cloud(ms, per_object_n=500, seed=0)
    assert cloud.shape[1] == 3 and len(cloud) > 10000
    # park extent is roughly x in [-50,48], y in [-26,23]; poles reach ~16 m + ground z~3
    assert cloud[:, 0].min() < -40 and cloud[:, 0].max() > 40
    assert cloud[:, 2].max() > 15.0        # the tall added structures are present
    # determinism
    assert np.array_equal(cloud, scene_cloud(ms, per_object_n=500, seed=0))
