"""Tests for the runtime region anchor detector (T19).

Strong assertions (exact location returned, tight margins), not loose bounds --
a repeated lesson in this project. Deterministic seeds throughout.

The map fixtures ALWAYS carry a `_meta` block: the detector reads it to describe
the live window with the same parameters the map was built with (ruling R9).
The fixtures use window_radius=8 (the real map's value, NOT describe_region's
default of 12), which the `_meta`-radius test below relies on.
"""
import numpy as np
import yaml
import pytest

from landmark_loc.region_detector import RegionDetector
from landmark_loc.descriptor import window, describe_region

MAP_RADIUS = 8.0  # matches maps/park_regions.yaml _meta.window_radius


def _tower(cx, cy, seed):
    """A tall thin twin-rail structure centred at (cx, cy). Deterministic."""
    r = np.random.RandomState(seed)
    z = r.uniform(0, 16, 3000)
    x = r.choice([-0.25, 0.25], 3000) + r.randn(3000) * 0.02 + cx
    y = r.randn(3000) * 0.02 + cy
    return np.column_stack([x, y, z])


def _describe_at(cloud, cx, cy, radius=MAP_RADIUS):
    """Describe the region of `cloud` around (cx, cy) at the map's parameters."""
    win = window(cloud, cx, cy, radius)
    return describe_region(win, n_sectors=8, n_rings=3, radius=radius).tolist()


def _write_map(path, locations, meta=True):
    """Write a park_regions-style yaml.

    `locations` is a list of (id, x, y, descriptor). `meta` toggles the _meta
    block so the missing-meta case can be exercised.
    """
    data = {}
    if meta:
        data["_meta"] = {"window_radius": MAP_RADIUS, "grid_step": 5.0,
                         "n_sectors": 8, "n_rings": 3}
    for loc_id, x, y, desc in locations:
        data[loc_id] = {"x": x, "y": y, "descriptor": desc}
    with open(path, "w") as f:
        yaml.safe_dump(data, f)


def _single_tower_map(path, loc_x=10.0, loc_y=0.0):
    """Map with one distinctive location: a tower described at (loc_x, loc_y)."""
    desc = _describe_at(_tower(loc_x, loc_y, 1), loc_x, loc_y)
    _write_map(path, [("loc_000", loc_x, loc_y, desc)])


def test_matches_region_near_prior(tmp_path):
    """Live tower at a map loc, prior nearby -> returns that map loc."""
    p = tmp_path / "r.yaml"
    _single_tower_map(str(p))
    det = RegionDetector(str(p), match_threshold=1.0, prior_gate=25.0)

    cloud = _tower(10, 0, 2)  # same structure, different RNG seed, at the map loc
    out = det.match(cloud, (10.0, 0.0))

    assert out is not None
    loc_id, map_x, map_y, conf = out
    assert loc_id == "loc_000"                # the exact map location's id
    assert (map_x, map_y) == (10.0, 0.0)      # the exact map location
    # Same structure, different RNG seed than the map's -> a small but nonzero
    # distance (~0.018). conf = 1 - dist/threshold, so a strong match at
    # threshold 1.0 -> conf ~ 0.982. Tight bound: still well above any partial.
    assert conf > 0.97


def test_prior_gate_rejects_far_lookalike(tmp_path):
    """Identical structure but prior far from any map loc -> None.

    The look-alike sits at (200, 0); the only map loc is at (10, 0), 190 m away,
    well outside the 25 m prior gate. Without the gate the identical descriptor
    would match; the gate is what rejects it.
    """
    p = tmp_path / "r.yaml"
    _single_tower_map(str(p))
    det = RegionDetector(str(p), match_threshold=1.0, prior_gate=25.0)

    cloud = _tower(200, 0, 2)
    assert det.match(cloud, (200.0, 0.0)) is None


def test_meta_radius_is_enforced_not_default(tmp_path):
    """Prove the detector describes the live window at radius 8, NOT 12.

    The map location's descriptor is an EMPTY neighbourhood (no structure within
    8 m). The live cloud has a tower 10 m from the prior: inside describe_region's
    default radius (12) but OUTSIDE the map's window_radius (8).

    If the detector wrongly used radius 12, the tower would enter the live
    descriptor, making it differ sharply from the empty map descriptor -> the
    match would be REJECTED (distance above threshold) or wrong. Because the
    detector uses the _meta radius (8), the tower is excluded, the live window is
    also empty, the descriptors agree, and the match is accepted at the map loc.
    """
    p = tmp_path / "r.yaml"
    # Map loc at (0,0) described from an EMPTY cloud at radius 8.
    empty_desc = describe_region(np.zeros((0, 3)), n_sectors=8, n_rings=3,
                                 radius=MAP_RADIUS).tolist()
    _write_map(str(p), [("loc_000", 0.0, 0.0, empty_desc)])
    det = RegionDetector(str(p), match_threshold=1.0, prior_gate=25.0)

    # Sanity: the same tower described at radius 12 vs 8 must actually DIFFER,
    # otherwise this test would pass trivially regardless of the radius used.
    cloud = _tower(10.0, 0.0, 3)
    from landmark_loc.descriptor import region_distance
    d12 = region_distance(_describe_at(cloud, 0.0, 0.0, radius=12.0), empty_desc)
    d8 = region_distance(_describe_at(cloud, 0.0, 0.0, radius=8.0), empty_desc)
    assert d12 > 1.0        # at radius 12 the tower enters -> far from empty
    assert d8 < 1e-9        # at radius 8 the tower is excluded -> identical

    out = det.match(cloud, (0.0, 0.0))
    assert out is not None            # radius 8 in force: window is empty, matches
    loc_id, map_x, map_y, conf = out
    assert loc_id == "loc_000"
    assert (map_x, map_y) == (0.0, 0.0)
    assert conf > 0.99                # dist ~ 0 -> confidence ~ 1


def test_missing_meta_raises(tmp_path):
    """A map with no _meta block -> RegionDetector refuses to guess a radius."""
    p = tmp_path / "r.yaml"
    desc = _describe_at(_tower(0.0, 0.0, 1), 0.0, 0.0)
    _write_map(str(p), [("loc_000", 0.0, 0.0, desc)], meta=False)

    with pytest.raises(ValueError, match="_meta"):
        RegionDetector(str(p), match_threshold=1.0)


def test_get_detector_registers_region(tmp_path):
    """get_detector('region', ...) constructs a RegionDetector via **kwargs."""
    from landmark_loc.detector import get_detector
    p = tmp_path / "r.yaml"
    _single_tower_map(str(p))
    det = get_detector("region", regions_path=str(p), match_threshold=1.0)
    assert isinstance(det, RegionDetector)
    assert det.name == "region"
    assert det.window_radius == MAP_RADIUS
