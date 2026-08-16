"""Runtime region anchor detector: match the live neighbourhood to the map.

WHERE THIS FITS
---------------
The offline step (`extract_park_map.py --regions`) wrote `maps/park_regions.yaml`:
a set of DISTINCTIVE map locations, each with an (x, y), a 168-number region
descriptor (`descriptor.describe_region`), and a `_meta` block recording the
EXACT parameters those descriptors were built with. This class is the runtime
half: given the live lidar cloud and the robot's prior (x, y), it describes the
region around the prior and finds which distinctive map location it matches,
producing the position fix the localizer anchors on.

Unlike the percept-based `Detector` implementations in `detector.py` (cluster
in, `Observation` out), this detector is REGION-based: its entry point is
`match(cloud, prior_xy)`, not `detect(percepts, ...)`. It is registered in the
same `DETECTORS` registry so `get_detector("region", ...)` selects it, but it
does not implement the percept contract -- the two live side by side because the
localizer picks exactly one anchor source per run.

THE _META RADIUS CONTRACT (correctness-critical -- ruling R9)
-------------------------------------------------------------
The descriptor built from the LIVE cloud MUST use the EXACT window_radius /
n_sectors / n_rings recorded in the map's `_meta`. The map was built at
window_radius=8, which is NOT `describe_region`'s default (12). If the live
window were described at radius 12, the 168-length guard in `region_distance`
would NOT catch it (both vectors are length 168) and every match distance would
be silently wrong -- structure between 8 m and 12 m of the prior would enter the
live descriptor but not the map's, injecting spurious distance into every
comparison.

Therefore `__init__` reads `_meta` and stores window_radius / n_sectors /
n_rings, and `match` passes those exact values to `window` and `describe_region`.
If `_meta` is ABSENT (an old map), `__init__` RAISES -- it does NOT fall back to
a default radius, because a wrong radius fails silently and that is precisely
what this rule prevents.
"""
import yaml

from landmark_loc.descriptor import window, describe_region, region_distance


# Provisional acceptance threshold on region_distance(live, map) for the SAME
# place. PROVISIONAL, pending T21 in-sim tuning against real Ouster returns.
#
# This is NOT the map's nearest-OTHER separation (3.66..4.50 between distinct
# map locations). It is a DIFFERENT quantity: the max distance between the LIVE
# descriptor and the CORRECT map descriptor for the same spot. That gap is
# driven by mesh-vs-lidar and partial-view differences, which are only
# measurable in-sim (T21). The map descriptors are mesh-built, so any threshold
# picked from them alone is not trustworthy (the T18 soft-threshold caveat --
# smooth distribution, 0.039 margin -- applies here too). 1.0 sits comfortably
# below the ~3.66 nearest-other separation (so a look-alike at a DIFFERENT map
# location stays rejected) while leaving headroom for live-vs-mesh drift; T21
# must confirm or replace it.
DEFAULT_MATCH_THRESHOLD = 1.0


class RegionDetector(object):
    """Match the region around the prior to a distinctive map location.

    `RegionDetector(regions_path, match_threshold, prior_gate=25.0)`.

    - `regions_path`: path to `park_regions.yaml` (must contain `_meta`).
    - `match_threshold`: accept a match only if `region_distance` is below this.
    - `prior_gate`: only map locations whose stored (x, y) is within this many
      metres of the prior are considered. This is what stops a far-away
      look-alike (identical structure at a distant location) from being chosen.
    """

    name = "region"

    #: Region-based, NOT percept-based: its entry point is match(cloud,
    #: prior_xy), not the label/observe/detect percept pipeline. The A/B percept
    #: harness reads this flag to skip this detector (it cannot be constructed
    #: with no args, nor compared over percepts). See detector.Detector.
    percept_based = False

    def __init__(self, regions_path, match_threshold=DEFAULT_MATCH_THRESHOLD,
                 prior_gate=25.0):
        with open(regions_path) as f:
            data = yaml.safe_load(f) or {}

        meta = data.get("_meta")
        if meta is None:
            raise ValueError(
                "RegionDetector: %s has no `_meta` block. It was built by an "
                "old extractor that did not record the describe_region window "
                "parameters. Regenerate the map with `extract_park_map.py "
                "--regions`; refusing to guess a window radius, because a wrong "
                "radius fails silently (the 168-length guard cannot catch it)."
                % regions_path)

        # Store the EXACT build parameters so the live window is described the
        # same way the map was -- see the _meta radius contract above.
        self.window_radius = float(meta["window_radius"])
        self.n_sectors = int(meta["n_sectors"])
        self.n_rings = int(meta["n_rings"])

        self.match_threshold = float(match_threshold)
        self.prior_gate = float(prior_gate)

        # Locations are the entries whose id starts with "loc_"; skip "_meta"
        # (and anything else non-loc) when iterating.
        self.locations = [
            (k, float(v["x"]), float(v["y"]), v["descriptor"])
            for k, v in data.items()
            if isinstance(k, str) and k.startswith("loc_")
        ]

    def match(self, cloud, prior_xy):
        """Match the live region around `prior_xy` to a map location.

        Returns `(loc_id, map_x, map_y, confidence)` for the best accepted
        match, or `None` if no gated candidate is close enough. `loc_id` is the
        map location's id (e.g. "loc_080") -- callers cross-check it against the
        operator's expected anchors so arrival is confirmed only when the place
        the matcher fixed on is the place the operator expected there, not merely
        a distinctive region that happens to sit nearby.

        Steps:
          1. Cut+describe the live window at the prior, using the `_meta` params
             (window_radius / n_sectors / n_rings from the map).
          2. Among map locations within `prior_gate` of the prior, pick the one
             with the smallest `region_distance` to the live descriptor.
          3. Accept if that distance < `match_threshold`; the confidence is
             `1 - dist / match_threshold` (1.0 at a perfect match, ->0 at the
             threshold). Otherwise return None.
        """
        prior_x, prior_y = float(prior_xy[0]), float(prior_xy[1])

        win = window(cloud, prior_x, prior_y, self.window_radius)
        desc = describe_region(win, n_sectors=self.n_sectors,
                               n_rings=self.n_rings, radius=self.window_radius)

        best_dist = None
        best_loc = None
        for loc_id, loc_x, loc_y, loc_desc in self.locations:
            # Prior gate: only consider map locations near the prior.
            if (loc_x - prior_x) ** 2 + (loc_y - prior_y) ** 2 > self.prior_gate ** 2:
                continue
            dist = region_distance(desc, loc_desc)
            if best_dist is None or dist < best_dist:
                best_dist = dist
                best_loc = (loc_id, loc_x, loc_y)

        if best_loc is None or best_dist >= self.match_threshold:
            return None

        confidence = 1.0 - best_dist / self.match_threshold
        return (best_loc[0], best_loc[1], best_loc[2], confidence)
