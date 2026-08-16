"""Analytic surface sampling for the primitive-built water tower.

The tower has no mesh file, so sample_surface cannot describe it. Sampling its
primitive surfaces analytically gives the map-side point set in the same form
describe() consumes, keeping map and observation in one space.
"""
import numpy as np
from map_tools.primitive_sample import sample_cylinder_stack

TOWER = [(1.0, 4.0, 2.0), (2.5, 5.0, 8.5)]   # pedestal, tank


def test_shape_and_determinism():
    a = sample_cylinder_stack(TOWER, n=2000, seed=0)
    b = sample_cylinder_stack(TOWER, n=2000, seed=0)
    assert a.shape == (2000, 3)
    assert np.array_equal(a, b)


def test_geometry_matches_the_spec():
    p = sample_cylinder_stack(TOWER, n=4000, seed=1)
    assert p[:, 2].min() > -0.1 and p[:, 2].max() < 11.1      # ~11 m tall
    top = p[p[:, 2] > 6.0]
    assert np.hypot(top[:, 0], top[:, 1]).max() > 2.3         # tank is wide
    low = p[p[:, 2] < 3.5]
    assert np.hypot(low[:, 0], low[:, 1]).max() < 1.2         # pedestal is narrow


def test_different_seeds_give_different_points():
    a = sample_cylinder_stack(TOWER, n=1000, seed=0)
    b = sample_cylinder_stack(TOWER, n=1000, seed=1)
    assert not np.array_equal(a, b)


def test_points_lie_on_the_surfaces_not_inside():
    """Surface sampling, not volume filling: every point must sit on a cap
    plane or on a lateral wall of one of the cylinders."""
    p = sample_cylinder_stack(TOWER, n=4000, seed=2)
    r = np.hypot(p[:, 0], p[:, 1])
    on_surface = np.zeros(len(p), dtype=bool)
    for radius, length, zc in TOWER:
        z0, z1 = zc - length / 2.0, zc + length / 2.0
        within_z = (p[:, 2] > z0 - 1e-9) & (p[:, 2] < z1 + 1e-9)
        lateral = within_z & (np.abs(r - radius) < 1e-9)
        cap = (r < radius + 1e-9) & (
            (np.abs(p[:, 2] - z0) < 1e-9) | (np.abs(p[:, 2] - z1) < 1e-9))
        on_surface |= lateral | cap
    assert on_surface.all()


def test_area_weighting_puts_most_points_on_the_tank():
    """Faces are sampled in proportion to their true areas. The tank's total
    surface (2*pi*2.5*5 + 2*pi*2.5^2 = 117.8 m^2) dominates the pedestal's
    (2*pi*1*4 + 2*pi*1^2 = 31.4 m^2), so ~79% of points should land on it."""
    p = sample_cylinder_stack(TOWER, n=20000, seed=3)
    tank_area = 2 * np.pi * 2.5 * 5 + 2 * np.pi * 2.5 ** 2
    pedestal_area = 2 * np.pi * 1.0 * 4 + 2 * np.pi * 1.0 ** 2
    expected = tank_area / (tank_area + pedestal_area)
    # NOTE the >= : the tank's bottom cap lies at exactly z=6.0 and holds ~13%
    # of all points (a 2.5 m-radius disc is a large face). A strict > 6.0 drops
    # that whole cap and reads 0.66 instead of 0.79.
    on_tank = (p[:, 2] >= 6.0).mean()
    assert abs(on_tank - expected) < 0.02
