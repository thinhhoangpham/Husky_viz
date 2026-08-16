# Terrain in the descriptor map — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax.

**Goal:** Make live-lidar region descriptors comparable to the offline map by sampling the world's terrain into the descriptor map, so both sides contain the ground the sensor actually returns.

**Architecture:** The map extractor currently samples only *object* meshes; the terrain families (`parque`, `camino_parque`, `terreno_lago`) classify as `skip` and never reach the scene cloud. This plan registers them as sampled-but-not-catalogued geometry, samples them at an area-appropriate density, and regenerates the descriptor map. No conditional per-world logic: flat terrain contributes a constant that cancels in the distance calculation, sloped terrain contributes real structure.

**Spec:** `docs/superpowers/specs/2026-08-16-unique-landmark-waypoint-localization-design.md`

---

## Why — the measured failure this fixes

The in-sim validation (2026-08-16) drove a 5-waypoint route with the region
localizer active and produced **zero matches for the entire drive**, including
while standing 0.4 m from a distinctive location.

Measured at that spot:

| | descriptor distance | true distance |
|---|---|---|
| `loc_144` — the correct region, underfoot | **6.63** | 0.4 m |
| `loc_116` — nearest-ranked | **5.75** | **25.8 m away** |
| `match_threshold` | 1.0 | |

Not a threshold problem: the *ranking* is meaningless — the nearest descriptor
belongs to a region 26 m away. Raising the threshold would only manufacture
confident wrong matches.

The cause, measured directly on the live cloud:

```
window pts = 13237   below z=3.3 (ground) = 12920 (98%)   above = 317
```

**98% of the live window is ground. The map side contains 0% ground.** The two
descriptors are describing different things. That single asymmetry accounts for
the scale gap (live L2 norm 35.4 vs map 27.1; band-0 extent 19.3 vs 4.26).

## Why sample terrain rather than filter it out

Both fix the asymmetry. Sampling is better:

- **Uniform rule, no per-world logic.** Flat ground appears identically in every
  window, so it contributes an equal term to both vectors being compared and
  **cancels** in the distance. Sloped ground differs place to place and
  **informs**. The same code does the right thing without knowing which world it
  is in. Measured relief: park **0.007 m** (z-scale 0.01 — a flat plane), lake
  **2.43 m** (z-scale 4 — real basin walls and shoreline).
- **It is what the sensor actually returns.** Filtering discards 98% of the real
  measurement to match an impoverished map. Sampling makes the map honest.
- **It unlocks terrain as a landmark.** Shoreline, banks and slopes become
  distinctive structure in the lake world — the case where ground relief carries
  the most information.

The cost is mild dilution: some descriptor cells hold ground instead of being
empty, so an object-driven difference is a smaller *fraction* of each vector's
magnitude. Second-order, and not a correctness issue. Accepted.

## Verified facts this plan is built on

- `park.world` loads `model://terreno_parque/terreno_parque_lowpoly.dae` at
  scale `50 25 0.01` for **both** visual and collision.
- `camino_parque` (the path) uses full-res `camino_parque.dae` (85 MB) for
  collision, `camino_parque_lowpoly.dae` for visual, scale `50 25 0.1`.
- Mesh sizes: `terreno_parque.dae` **304 MB**, `terreno_parque_lowpoly.dae`
  **8.7 MB**, `camino_parque.dae` **85 MB**, `camino_parque_lowpoly.dae`
  **3.0 MB**, `terreno_lago_lowpoly.dae` **87 MB**, `lago.dae` **255 MB**.
  **Always sample the `_lowpoly` variant.** Full-res parsing is prohibitive and
  lowpoly is what the world renders anyway.
- `classify_prefix` currently returns `skip` for `parque`, `camino_parque`,
  `terreno_parque`, `terreno_lago`, `lago`.
- Scene cloud build is already ~72 s; terrain must not make it pathological.
- Current map: 11 distinctive locations, `_meta {window_radius: 8.0,
  grid_step: 5.0, n_sectors: 8, n_rings: 3}`.

## Global Constraints

- Python 3.8, numpy, PyYAML, pytest. NO new third-party dependencies.
- NEVER Gazebo ground truth. Parsing the static world file is fine.
- `map_tools/park_types.py` stays the single registry.
- The descriptor must be computed identically on both sides — do not add a
  map-only or live-only preprocessing step.
- Determinism: fixed seeds, so the committed map artifact is stable.
- Terrain is **sampled geometry, not a catalog landmark**: it must reach the
  scene cloud but must NOT become a named object or an occupancy-grid footprint.
  Do not change `park_map.pgm` / `park_objects.yaml` content.

---

### Task 1: Registry support for sampled-but-not-catalogued terrain

**Files:**
- Modify: `map_tools/park_types.py`
- Test: `map_tools/tests/test_terrain_types.py`

**Interfaces:**
- Produces: terrain entries resolvable by `classify_prefix` and a way for the
  scene assembler to know "sample this, but it is not a landmark".

- [ ] **Step 1: failing test**

```python
# map_tools/tests/test_terrain_types.py
from map_tools.park_types import classify_prefix, BY_PREFIX, LAKE_BY_PREFIX

def test_park_terrain_is_classified_not_skipped():
    assert classify_prefix("parque") == "parque"
    assert classify_prefix("camino_parque") == "camino_parque"

def test_terrain_is_sampled_but_not_catalogued():
    for p in ("parque", "camino_parque"):
        t = BY_PREFIX[p]
        assert t.is_terrain is True          # sampled into the scene cloud
        assert t.is_catalog is False         # never a matcher landmark
        assert t.is_object is False          # never a named goal
        assert t.mesh is not None            # has a lowpoly mesh to sample
```

- [ ] **Step 2: run, expect fail** — `python3 -m pytest map_tools/tests/test_terrain_types.py -v`
- [ ] **Step 3: implement.** Add an `is_terrain: bool = False` field to `ParkType`, appended LAST with a default (the dataclass documents this convention at `park_types.py:59-62` — positional construction elsewhere must keep working). Add entries for `parque` and `camino_parque` to `PARK_TYPES` and `terreno_lago` to `LAKE_TYPES`, each with `is_terrain=True, is_catalog=False, is_object=False, box_stamped=False, disc_radius=0.0`, and `mesh` pointing at the **lowpoly** `.dae` with the world scale from the table above. Do NOT register `lago` (the water box has no collision — the lidar returns nothing from it; it stays `skip`).
- [ ] **Step 4: run, expect pass.** Also run `map_tools/tests/ landmark_loc/tests/ -q` — `park_map.pgm`/`park_objects.yaml` tests MUST still pass unchanged (terrain is not catalogued, so counts must not move).
- [ ] **Step 5: commit** — `git add map_tools/park_types.py map_tools/tests/test_terrain_types.py && git commit -m "feat(map): register terrain as sampled-but-not-catalogued geometry"`

---

### Task 2: Area-proportional terrain sampling in the scene cloud

**Files:**
- Modify: `map_tools/scene_points.py`
- Test: `map_tools/tests/test_scene_points.py` (extend)

**Interfaces:**
- Consumes: Task 1's `is_terrain` flag, existing `sample_surface`/`sample_triangles`.
- Produces: `scene_cloud` includes terrain points at a density expressed in
  **points per square metre**, not a flat per-object count.

**Why a different density rule:** an object mesh is a few m²; the terrain sheet
spans ~100 × 50 m. Sampling it with the same `per_object_n` as a bench leaves the
ground invisible in an 8 m window; sampling it at bench *density* would generate
millions of points. Terrain needs its own knob.

- [ ] **Step 1: failing test**

```python
# append to map_tools/tests/test_scene_points.py
def test_terrain_points_present_at_useful_density():
    ms = parse_models(WORLD)
    cloud = scene_cloud(ms, per_object_n=300, seed=0)
    # ground sits at z~2.99 in the park; objects rise above it
    ground = cloud[cloud[:, 2] < 3.2]
    assert len(ground) > 5000, "terrain must be sampled into the scene cloud"
    # an 8 m window anywhere on the map should contain ground
    import numpy as np
    from landmark_loc.descriptor import window
    w = window(cloud, 0.0, 0.0, 8.0)
    wg = w[w[:, 2] < 3.2]
    assert len(wg) > 100, "an 8 m window must contain a meaningful ground sample"
```

- [ ] **Step 2: run, expect fail**
- [ ] **Step 3: implement.** Add `terrain_pts_per_m2` (a module constant, start at **2.0** — justify below) and, in `scene_cloud`, dispatch terrain models to a sampler that computes `n = ceil(area * terrain_pts_per_m2)` from the mesh's world-scaled footprint rather than using `per_object_n`. Reuse `sample_triangles`; do not duplicate the sampling math. **Choose the constant by measurement, not by guess:** the live lidar puts ~12,900 ground points in an 8 m window (~201 m²), i.e. ~64 pts/m². Matching that exactly would add ~320k points across the park and blow the build time — the descriptor only needs enough ground for the per-band statistics to be stable, not photographic parity. Start at 2.0 pts/m² (~10k park-wide, ~400 per window), REPORT the resulting window count and build time, and raise only if Task 4 shows the descriptor is unstable.
- [ ] **Step 4: run, expect pass.** Report the new `scene_cloud` build time (was ~72 s) — if it exceeds ~180 s, lower the density and say so.
- [ ] **Step 5: commit** — `git add map_tools/scene_points.py map_tools/tests/test_scene_points.py && git commit -m "feat(map): sample terrain at area-proportional density into the scene cloud"`

---

### Task 3: Regenerate the descriptor map and re-measure distinctiveness

**Files:**
- Modify: `maps/park_regions.yaml` (regenerated artifact)
- Test: `map_tools/tests/test_extract_regions.py` (may need threshold expectations updated)

- [ ] **Step 1:** Run `python3 -m map_tools.extract_park_map --regions` and capture the printed nearest-distance distribution and chosen threshold.
- [ ] **Step 2:** Compare against the pre-terrain baseline (11 locations, nearest 3.66–4.50, threshold 3.623). **Expect the distribution to shift** — every window now contains ground, so absolute distances change. What matters is whether a **gap** still exists and whether distinctive locations still emerge in sensible places.
- [ ] **Step 3:** If the existing test's assertions (>=4 locations, each within 20 m of a pole) still hold, leave them. If the distinctive set has moved, **report the new set and where it clusters** — do NOT loosen the test to force a pass. A materially different distinctive set is a real finding about what terrain contributes.
- [ ] **Step 4:** Commit the regenerated map with the measured distribution in the commit message.

---

### Task 4: Live-vs-map descriptor comparison (the actual acceptance test)

**Files:** none committed — this is a measurement task run **from the main conversation**, not a subagent, because it requires the simulator.

This is the task that decides whether the plan worked.

- [ ] **Step 1:** Bring up the sim per `RUN-MAP-NAV.md` Steps 0–2 with
      `_classifier:=region`, and the operator per Step 3 (restart the container
      first if it is stale).
- [ ] **Step 2:** With the robot parked **on** a distinctive location, measure the
      live descriptor against every map descriptor, exactly as the failing
      diagnosis did. **Acceptance:** the correct region must be **nearest**, and
      its distance must be well below the distances to far-away regions.
- [ ] **Step 3:** Set `match_threshold` from that measurement (halfway between the
      correct-region distance and the next-nearest), replacing the provisional 1.0.
- [ ] **Step 4:** Re-run the 5-waypoint route
      (`route 40.84 8.19 35.84 13.19 5.84 13.19 -9.16 13.19 -14.16 3.19`) and
      confirm region matches now land while driving.
- [ ] **Step 5:** Run the navsat spoof attack during the drive and confirm the
      descriptor-derived pose does not follow it.

**If Step 2 still fails**, terrain was not the whole gap and the remaining cause
is the mesh-vs-lidar difference (map samples whole objects; lidar sees near faces
only). The fallback is then to build the map from a **survey drive** — record real
lidar at each grid location with a trusted GPS pose and build descriptors from
those returns, so both sides are the same kind of data. Do not attempt that until
this plan's result is measured.

---

## Risks

1. **Build time.** Terrain adds points to a 72 s build. Mitigated by
   area-proportional density and the lowpoly meshes; Task 2 reports the number.
2. **Dilution.** Ground fills descriptor cells that were empty, shrinking the
   object-driven share of each vector. Accepted as second-order; Task 3's
   distribution and Task 4's measurement will show whether it bites.
3. **The distinctive set may move.** With ground in every window, which locations
   are distinctive can change — possibly for the better in the lake world
   (shoreline), possibly reducing the park's set. Task 3 reports it rather than
   forcing the old answer.
4. **Terrain may not be the whole gap.** The mesh-vs-lidar near-face difference
   remains. Task 4 Step 2 is the honest test; the survey-drive fallback is named.
