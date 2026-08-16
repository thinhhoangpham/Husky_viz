"""End-to-end: the location-grid distinctiveness extractor should surface
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
        regs = yaml.safe_load(fh)

    assert len(regs) >= 4, "expected several distinctive spots, got %d" % len(regs)

    for loc_id, r in regs.items():
        d = min(math.hypot(r["x"] - px, r["y"] - py) for px, py in POLES)
        assert d < 20.0, (
            "distinctive location %s at (%.1f, %.1f) is %.1f m from the nearest "
            "pole -- not near any known structure" % (loc_id, r["x"], r["y"], d))
        assert len(r["descriptor"]) == 168
