import os
from map_tools.sdf_parse import parse_models, Model
from map_tools.extract_park_map import RADII, build_grid, build_objects

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

def test_build_objects_has_named_furniture_and_trees_not_bushes():
    models = parse_models(WORLD)
    objects = build_objects(models)
    assert any(n.startswith("bench") for n in objects)
    assert any(n.startswith("lamp") for n in objects)
    # tree_8 IS a valid catalog object (is_object=True); trees are objects too.
    assert any(n.startswith("tree_8") for n in objects)
    # arbolpartes4 bushes are obstacle-only, never catalog objects.
    assert not any(n.startswith("arbolpartes4") for n in objects)
    # Each entry has numeric x and y.
    sample = next(iter(objects.values()))
    assert isinstance(sample["x"], float) and isinstance(sample["y"], float)

def test_bench_box_is_stamped_at_shifted_geometry_center_not_link_origin():
    # Bench_1.dae's COLLADA node transforms translate the geometry ~1.4 m from
    # the mesh origin (see mesh_bounds.footprint). The box must be stamped at
    # the rotated true geometry center (~36.2, -0.56 for this model's pose),
    # not at the link_0 pose (~37.61, -0.16) -- that was the ~1.2 m bug.
    models = parse_models(WORLD)
    g = build_grid(models, resolution=0.15)
    bench = next(m for m in models if m.name == "bench")
    assert abs(bench.world_x - 37.61) < 0.05
    assert abs(bench.world_y - (-0.16)) < 0.05

    # True (shifted) geometry center is occupied.
    assert g.is_occupied(36.2, -0.56) is True
    # The link origin itself is no longer necessarily the box center -- it's
    # ~1.4 m away from the true geometry, well outside the box's ~0.9 m
    # long half-extent.
    assert g.is_occupied(bench.world_x, bench.world_y) is False


def test_bench_footprint_covers_full_length():
    models = parse_models(WORLD)
    g = build_grid(models, resolution=0.15)
    bench = next(m for m in models if m.name == "bench")
    # The bench yaw is ~-1.563 (long axis ~ along world y). Sample a point ~0.7 m
    # from the SHIFTED bench center along its long axis; a disc (old behavior)
    # would miss it, an oriented box covers it. Long half-extent ~0.89 m.
    import math
    from map_tools.extract_park_map import _box_extents
    hx, hy, cx, cy = _box_extents("bench")
    cx_w = bench.world_x + (cx * math.cos(bench.yaw) - cy * math.sin(bench.yaw))
    cy_w = bench.world_y + (cx * math.sin(bench.yaw) + cy * math.cos(bench.yaw))
    L = 0.85  # within the ~0.89 m half-length
    px = cx_w + L * math.cos(bench.yaw + math.pi / 2)
    py = cy_w + L * math.sin(bench.yaw + math.pi / 2)
    assert g.is_occupied(px, py) is True

def test_garden_table_footprint_covers_true_length():
    # garden_table's real footprint is ~1.319 x 3.000 m (scale 1.0, not the
    # 0.15 wrongly applied before) -- long half-extent ~1.5 m. A point 1.2 m
    # out along the long axis must be covered; the old (buggy, scale=0.15)
    # ~0.2 m dot would have missed it entirely.
    import math
    models = parse_models(WORLD)
    g = build_grid(models, resolution=0.15)
    table = next(m for m in models if m.name == "garden_table")
    L = 1.2
    px = table.world_x + L * math.cos(table.yaw + math.pi / 2)
    py = table.world_y + L * math.sin(table.yaw + math.pi / 2)
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
    assert (out / "park_objects.yaml").exists()


# --- grid extent is defined by the TERRAIN, not by object positions ----------

DTM = os.path.join(os.path.dirname(__file__), "..", "..", "maps", "park_dtm.yaml")


def _map_bounds(g):
    return (g.origin_x, g.origin_y,
            g.origin_x + g.width * g.resolution,
            g.origin_y + g.height * g.resolution)


def test_terrain_extent_uses_full_dtm_grid_including_nan_cells():
    # The FULL grid footprint, not the finite-cell bbox: NaN cells (water,
    # off-mesh void) must fall INSIDE the map so they can be marked unknown.
    from map_tools.extract_park_map import terrain_extent
    from map_tools.dtm_raster import read_dtm_yaml
    d = read_dtm_yaml(DTM)
    x0, y0, x1, y1 = terrain_extent(DTM)
    assert x0 == d["origin_x"] and y0 == d["origin_y"]
    assert abs(x1 - (d["origin_x"] + d["width"] * d["resolution"])) < 1e-9
    assert abs(y1 - (d["origin_y"] + d["height"] * d["resolution"])) < 1e-9


def test_grid_contains_the_whole_terrain_footprint():
    models = parse_models(WORLD)
    g = build_grid(models, resolution=0.15, dtm_yaml=DTM)
    from map_tools.extract_park_map import terrain_extent
    tx0, ty0, tx1, ty1 = terrain_extent(DTM)
    gx0, gy0, gx1, gy1 = _map_bounds(g)
    assert gx0 <= tx0 and gy0 <= ty0 and gx1 >= tx1 and gy1 >= ty1


def test_grid_extends_past_terrain_to_keep_an_outlying_object():
    # An object beyond the terrain footprint must NOT be dropped -- the grid
    # takes the union, so it stays inside the map.
    from map_tools.extract_park_map import terrain_extent
    tx0, ty0, tx1, ty1 = terrain_extent(DTM)
    far = Model(name="stray", family="lamp",
                world_x=tx1 + 20.0, world_y=ty0 + 1.0, yaw=0.0)
    g = build_grid(list(parse_models(WORLD)) + [far], resolution=0.15, dtm_yaml=DTM)
    gx0, gy0, gx1, gy1 = _map_bounds(g)
    assert gx1 > tx1 + 20.0
    assert g.is_occupied(far.world_x, far.world_y) is True


def test_grid_ignores_object_bounds_that_are_inside_the_terrain():
    # Objects clustered well inside the terrain must not shrink the map:
    # the terrain, not the outermost object, sets the extent.
    from map_tools.extract_park_map import terrain_extent
    tx0, ty0, tx1, ty1 = terrain_extent(DTM)
    tight = [Model(name="a", family="lamp", world_x=0.0, world_y=0.0, yaw=0.0)]
    g = build_grid(tight, resolution=0.15, dtm_yaml=DTM)
    gx0, gy0, gx1, gy1 = _map_bounds(g)
    assert gx0 <= tx0 and gx1 >= tx1 and gy0 <= ty0 and gy1 >= ty1


def test_no_dtm_falls_back_to_object_sizing_with_a_warning(capsys):
    models = parse_models(WORLD)
    g = build_grid(models, resolution=0.15, dtm_yaml=None)
    assert "WARNING" in capsys.readouterr().err
    xs = [m.world_x for m in models]
    gx0, _, gx1, _ = _map_bounds(g)
    assert gx0 <= min(xs) and gx1 >= max(xs)
