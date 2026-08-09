import math
from map_tools.occupancy_grid import Grid

def test_grid_dimensions_and_origin():
    g = Grid(min_x=-10.0, min_y=-10.0, max_x=10.0, max_y=10.0, resolution=0.5)
    assert g.width == 40
    assert g.height == 40
    assert g.origin_x == -10.0
    assert g.origin_y == -10.0

def test_stamp_disc_marks_center_and_within_radius():
    g = Grid(-5, -5, 5, 5, 0.5)
    g.stamp_disc(0.0, 0.0, 1.0)
    assert g.is_occupied(0.0, 0.0) is True
    assert g.is_occupied(0.8, 0.0) is True     # within radius
    assert g.is_occupied(3.0, 0.0) is False    # outside radius

def test_free_by_default():
    g = Grid(-5, -5, 5, 5, 0.5)
    assert g.is_occupied(2.0, 2.0) is False

def test_write_pgm_and_yaml(tmp_path):
    g = Grid(-5, -5, 5, 5, 0.5)
    g.stamp_disc(0.0, 0.0, 0.5)
    pgm = tmp_path / "m.pgm"
    yaml = tmp_path / "m.yaml"
    g.write_pgm(str(pgm))
    g.write_yaml(str(yaml), "m.pgm")
    header = pgm.read_bytes()[:15]
    assert header.startswith(b"P5\n20 20\n255\n")
    txt = yaml.read_text()
    assert "resolution: 0.500000" in txt
    assert "origin: [-5.000000, -5.000000, 0.0]" in txt
    assert "image: m.pgm" in txt

def test_stamp_box_axis_aligned():
    g = Grid(-5, -5, 5, 5, 0.1)
    # A 2 x 0.4 m box (half 1.0 x 0.2) at origin, yaw 0: long axis = x.
    g.stamp_box(0.0, 0.0, 0.0, 1.0, 0.2)
    assert g.is_occupied(0.9, 0.0) is True    # inside along long axis
    assert g.is_occupied(0.0, 0.1) is True    # inside along short axis
    assert g.is_occupied(0.0, 0.5) is False   # outside short axis
    assert g.is_occupied(1.5, 0.0) is False   # outside long axis

def test_stamp_box_rotated_90deg():
    g = Grid(-5, -5, 5, 5, 0.1)
    # Same box rotated 90 deg: long axis now y.
    g.stamp_box(0.0, 0.0, math.pi / 2, 1.0, 0.2)
    assert g.is_occupied(0.0, 0.9) is True    # long axis now along y
    assert g.is_occupied(0.9, 0.0) is False   # was inside at yaw 0, now outside
