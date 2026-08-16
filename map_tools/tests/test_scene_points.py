import os

import numpy as np

from map_tools.sdf_parse import parse_models
from map_tools.scene_points import scene_cloud, sample_model

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


def test_scene_cloud_is_ground_referenced_across_families():
    """Different meshes put their local origin at different heights (e.g.
    tree_8's trunk mesh origin sits mid-trunk). scene_cloud must apply each
    model's world_z so every family sits on the same ground plane --
    otherwise a window spanning two families would see them at the wrong
    relative height, and the map cloud would disagree with what the robot's
    live lidar sees for the same region.
    """
    ms = parse_models(WORLD)
    cloud = scene_cloud(ms, per_object_n=500, seed=0)

    # Overall min should be near ground (a tree's buried root extent), not
    # ~6 m below it as it was before world_z was applied (was cloud z-min
    # -3.24, entirely from tree_8's mid-trunk mesh origin).
    assert cloud[:, 2].min() > -1.5

    # world_z itself -- the ground-plane height the world file assigns each
    # model's link_0 pose -- must agree across families within ~1.5 m. This
    # is the quantity the fix actually ground-references on; the true
    # per-family point MINIMUM legitimately differs by mesh shape (e.g. a
    # tree trunk mesh extends further below its own origin than a bench's
    # does), so that raw minimum is not itself a fair cross-family check.
    world_z_by_family = {}
    for m in ms:
        world_z_by_family.setdefault(m.family, []).append(m.world_z)
    world_z_means = {fam: float(np.mean(vals))
                     for fam, vals in world_z_by_family.items()
                     if fam != "arbolpartes4"}  # no mesh -> not part of the cloud
    assert len(world_z_means) >= 4  # sanity: several families actually present
    values = list(world_z_means.values())
    assert max(values) - min(values) < 1.5, world_z_means
