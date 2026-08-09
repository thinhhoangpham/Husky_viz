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
