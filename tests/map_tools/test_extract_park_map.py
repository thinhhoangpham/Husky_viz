import os
from map_tools.sdf_parse import parse_models, Model
from map_tools.extract_park_map import RADII, build_grid, build_places

WORLD = os.path.join(os.path.dirname(__file__), "..", "..",
                     "natural_environments_ros_opt", "natural_enviroment",
                     "worlds", "park.world")

def test_radii_cover_all_obstacle_families():
    for fam in ("arbolpartes4", "tree_8", "bench", "garden_table",
                "lamp", "trash_bin_1"):
        assert fam in RADII and RADII[fam] > 0

def test_build_grid_marks_a_known_tree_and_leaves_far_ground_free():
    models = parse_models(WORLD)
    g = build_grid(models, resolution=0.15, margin=5.0)
    tree = next(m for m in models if m.name == "arbolpartes4")
    assert g.is_occupied(tree.world_x, tree.world_y) is True
    # A point 50 m away from any obstacle is free. Use grid corner-ish empty.
    assert g.is_occupied(tree.world_x + 40.0, tree.world_y + 40.0) is False

def test_build_places_has_named_furniture_not_trees():
    models = parse_models(WORLD)
    places = build_places(models)
    assert any(n.startswith("bench") for n in places)
    assert any(n.startswith("lamp") for n in places)
    assert not any(n.startswith("arbolpartes4") for n in places)
    assert not any(n.startswith("tree_8") for n in places)
    # Each entry has numeric x and y.
    sample = next(iter(places.values()))
    assert isinstance(sample["x"], float) and isinstance(sample["y"], float)

def test_bench_footprint_covers_full_length():
    models = parse_models(WORLD)
    g = build_grid(models, resolution=0.15)
    bench = next(m for m in models if m.name == "bench")
    # The bench yaw is ~-1.563 (long axis ~ along world y). Sample a point ~0.7 m
    # from the bench center along its long axis; a disc (old behavior) would miss
    # it, an oriented box covers it. Long half-extent ~0.89 m.
    import math
    L = 0.85  # within the ~0.89 m half-length
    px = bench.world_x + L * math.cos(bench.yaw + math.pi / 2)
    py = bench.world_y + L * math.sin(bench.yaw + math.pi / 2)
    assert g.is_occupied(px, py) is True

def test_lamp_and_bin_still_marked():
    # Regression guard: lamp (0.095 m wide) and trash_bin_1 (~0.10x0.06 m) are
    # sub-cell at 0.15 m resolution -- boxing them finds zero/near-zero cell
    # centers inside the footprint and the object nearly vanishes from the map.
    # They must stay discs (as before Task 8), not boxes.
    models = parse_models(WORLD)
    g = build_grid(models, resolution=0.15)
    lamp = next(m for m in models if m.name == "lamp")
    binm = next(m for m in models if m.name == "trash_bin_1")
    assert g.is_occupied(lamp.world_x, lamp.world_y) is True
    assert g.is_occupied(binm.world_x, binm.world_y) is True

def test_main_writes_three_files(tmp_path):
    from map_tools.extract_park_map import main
    out = tmp_path / "maps"
    rc = main(["--world", WORLD, "--out-dir", str(out)])
    assert rc == 0
    assert (out / "park_map.pgm").exists()
    assert (out / "park_map.yaml").exists()
    assert (out / "park_places.yaml").exists()
