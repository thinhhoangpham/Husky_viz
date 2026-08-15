"""Guards for the single park-type registry (map_tools.park_types).

These lock in the deliberate per-type ASYMMETRIES so a future edit to the
registry cannot silently flatten them (which would drift map-building and
lidar-detection apart -- the exact failure the registry exists to prevent).
"""
from map_tools.park_types import (
    PARK_TYPES,
    BY_PREFIX,
    classify_prefix,
    PREFIXES_LONGEST_FIRST,
)


def test_arbolpartes4_is_obstacle_only():
    t = BY_PREFIX["arbolpartes4"]
    # Has a footprint radius + world prefix (it IS stamped as an obstacle)...
    assert t.disc_radius > 0
    assert t.world_prefix == "arbolpartes4"
    # ...but is never a place, never in the catalog, and has no mesh signature.
    assert t.is_place is False
    assert t.is_catalog is False
    assert t.mesh is None
    assert t.signature is None
    assert t.detect_radius is None


def test_tree_has_no_signature_but_hardcoded_detect_radius():
    t = BY_PREFIX["tree_8"]
    assert t.identity == "tree"          # tree_8 model -> generic 'tree'
    assert t.is_place is True
    assert t.is_catalog is True
    assert t.mesh is None                # identified by vertical profile
    assert t.signature is None
    assert t.detect_radius == 0.45       # trunk radius, hardcoded not derived
    assert t.box_stamped is False


def test_lamp_and_bin_are_disc_not_box():
    for prefix in ("lamp", "trash_bin_1"):
        t = BY_PREFIX[prefix]
        assert t.mesh is not None            # have a mesh signature
        assert t.signature is not None
        assert t.is_place is True
        assert t.box_stamped is False        # stamped as a disc, not a box


def test_bench_and_table_are_box_stamped():
    for prefix in ("bench", "garden_table"):
        t = BY_PREFIX[prefix]
        assert t.box_stamped is True
        assert t.mesh is not None
        assert t.rect_footprint is not None


def test_every_catalog_type_has_a_detect_radius():
    for t in PARK_TYPES:
        if t.is_catalog:
            assert t.detect_radius is not None, t.world_prefix
            assert t.detect_radius > 0


def test_every_box_stamped_type_has_a_mesh():
    for t in PARK_TYPES:
        if t.box_stamped:
            assert t.mesh is not None, t.world_prefix


def test_prefix_lookup_is_longest_first():
    # Longer prefixes come before any shorter prefix that is a substring start.
    order = list(PREFIXES_LONGEST_FIRST)
    lengths = [len(p) for p in order]
    assert lengths == sorted(lengths, reverse=True)
    # Concretely: tree_8 / trash_bin_1 must not be shadowed by a shorter prefix.
    assert classify_prefix("tree_8") == "tree_8"
    assert classify_prefix("tree_8_3") == "tree_8"
    assert classify_prefix("trash_bin_1") == "trash_bin_1"
    assert classify_prefix("trash_bin_1_2") == "trash_bin_1"
    # Non-matching names are skipped.
    assert classify_prefix("streetlight_9") == "skip"
