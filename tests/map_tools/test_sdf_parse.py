import os
from map_tools.sdf_parse import classify, parse_models

def test_classify_families():
    assert classify("arbolpartes4") == "arbolpartes4"
    assert classify("arbolpartes4_clone_10") == "arbolpartes4"
    assert classify("tree_8") == "tree_8"
    assert classify("tree_8_clone_2_clone_7_clone_1") == "tree_8"
    assert classify("bench_clone_0_clone_clone") == "bench"
    assert classify("garden_table") == "garden_table"
    assert classify("lamp") == "lamp"
    assert classify("trash_bin_1") == "trash_bin_1"
    assert classify("parque") == "skip"
    assert classify("camino_parque") == "skip"
    assert classify("Untitled2") == "skip"

WORLD = os.path.join(os.path.dirname(__file__), "..", "..",
                     "natural_environments_ros_opt", "natural_enviroment",
                     "worlds", "park.world")

def test_parse_models_counts():
    models = parse_models(WORLD)
    fams = [m.family for m in models]
    assert fams.count("arbolpartes4") == 15
    assert fams.count("tree_8") == 23
    assert "skip" not in fams  # skipped models are dropped, not returned

def test_arbolpartes4_trunk_is_offset_from_model_pose():
    # The first arbolpartes4 model <pose> in the <state> block is
    # (36.8181, -20.8082); its link_0 (trunk) pose is (36.9169, -19.6925).
    # parse_models MUST return the LINK pose, not the model pose.
    models = parse_models(WORLD)
    tree = next(m for m in models if m.name == "arbolpartes4")
    assert abs(tree.world_x - 36.9169) < 0.01
    assert abs(tree.world_y - (-19.6925)) < 0.5  # check magnitude: |y| ≈ 19.6925
    # Guard the bug: it must NOT be the model pose y = -20.8082
    assert abs(tree.world_y - (-20.8082)) > 0.5
