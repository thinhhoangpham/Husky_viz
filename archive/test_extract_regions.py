"""ARCHIVED (2026-08-16). This test no longer runs: the `--regions` code path
it exercises was removed from `map_tools/extract_park_map.py` when the
per-region descriptor approach was retired (see
`docs/research/lidar-place-recognition-survey.md`). Kept for reference only.

End-to-end: the location-grid distinctiveness extractor should surface
distinctive locations that cluster near the six known pole world-positions.

This runs the real --regions extraction ONCE (main builds the ~72 s scene cloud
a single time) and asserts on the written yaml -- it never rebuilds the scene
cloud per cell.
"""
import math

import yaml

from map_tools import extract_park_map

# The six known distinctive structures (postescable poles) in park.world,
# map-frame metres. Distinctive grid locations should cluster near these.
POLES = [
    (-42.5, 1.0), (-14.33, 6.99), (13.84, 12.98),
    (42.01, 18.96), (-27.0, -23.0), (1.8, -23.0),
]


def test_distinctive_locations_cluster_near_the_added_structures(tmp_path):
    extract_park_map.main(["--out-dir", str(tmp_path), "--regions"])
    with open(tmp_path / "park_regions.yaml") as fh:
        doc = yaml.safe_load(fh)

    # _meta is a reserved, additive key; split it off so the location assertions
    # see only loc_NNN entries.
    meta = doc.pop("_meta")
    regs = doc

    # The map must be self-describing: the runtime (T19) reads these to describe
    # its live window with the SAME parameters. A radius mismatch is silent (a
    # window at any radius yields the same 168-length vector, so region_distance's
    # length guard cannot catch it), so pin the recorded params to what the
    # extractor actually used -- change a literal without regenerating and this
    # fails.
    assert meta["window_radius"] == extract_park_map.WINDOW_RADIUS
    assert meta["n_sectors"] == extract_park_map.N_SECTORS
    assert meta["n_rings"] == extract_park_map.N_RINGS
    assert meta["grid_step"] == extract_park_map.GRID_STEP
    # No location id may collide with the reserved key.
    assert all(not lid.startswith("_") for lid in regs)

    assert len(regs) >= 4, "expected several distinctive spots, got %d" % len(regs)

    for loc_id, r in regs.items():
        d = min(math.hypot(r["x"] - px, r["y"] - py) for px, py in POLES)
        assert d < 20.0, (
            "distinctive location %s at (%.1f, %.1f) is %.1f m from the nearest "
            "pole -- not near any known structure" % (loc_id, r["x"], r["y"], d))
        assert len(r["descriptor"]) == 168
