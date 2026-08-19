# Terrain Slope Costmap — Design

Date: 2026-08-19
Status: design approved pending user review
Supersedes: nothing (prior 2026-08-18 draft was deleted; this is a fresh design)

## Problem

`move_base` currently plans over trees, benches and live lidar returns, but it is
blind to terrain steepness. On sloped ground it will happily route the Husky
straight up or down a bank steep enough to slip, scrub or roll it. The goal is a
costmap layer that makes the planner avoid steep uphill **and** downhill.

## Scope

- **Universal.** The generator takes a world name and works on any world that has
  a DTM. It is not lake-specific and not park-specific.
- **Lake is the demo** because it is the only world with meaningful relief.
- **Park is the flat-terrain correctness case**: the same code, same thresholds,
  must produce an all-free layer rather than crashing or fabricating cost.

Out of scope, deliberately:
- Local-costmap slope. The local costmap is odom-frame and rolling; slope is
  static world knowledge and belongs in the map-frame global costmap.
- Signed slope (uphill vs downhill weighted differently). See Known Limitations.
- Object-height fusion. See Future Work.

## Measured facts (verified, not assumed)

Computed from the real DTMs with `np.gradient` at 0.25 m resolution:

| World | Relief | Max slope | >10° | >15° | >18° |
|---|---|---|---|---|---|
| park | 0.0069 m | 0.87° | 0% | 0% | 0% |
| lake | 2.42 m | 24.3° | 14.4% | 2.5% | 0.77% |

Lake slope percentiles: p50 3.6°, p75 7.4°, p90 11.4°, p95 13.3°, p99 17.3°.

All percentages above and below are **of valid (non-NaN) cells**, not of the whole
grid. Lake lethal is 0.77% of valid cells = 0.53% of all cells.

`maps/lake_dtm.npy` is 398x200 at 0.25 m, origin (-49.75, -25.0), with 31% NaN
(the water hole — `lago.dae` has no mesh there).
`maps/park_dtm.npy` is 400x201 at 0.25 m, origin (-50.0, -26.75), 1% NaN.

**The DTM grid does not match the map grid.** `lake_map.yaml` / `park_map.yaml`
are 0.15 m with different origins. The slope raster MUST be resampled onto the
map grid or every slope cell lands in the wrong place.

## Architecture

```
maps/<world>_dtm.npy  (float32 heights)
        |
        v
map_tools/slope_costmap.py          <-- offline, one pass, no ROS
        |  np.gradient -> degrees
        |  degrees -> occupancy 0-100 (thresholds) -> inverted pixel
        |  resample onto <world>_map.yaml grid
        v
maps/<world>_slope.npy   (float32 DEGREES, DTM grid)   <-- diagnostics + future fusion
maps/<world>_slope.pgm   (inverted occupancy, map grid)  <-- fed to move_base
maps/<world>_slope.yaml  (grid meta + thresholds)
        |
        v
map_server  --> /slope_map
        |
        v
costmap_2d::StaticLayer instance "slope"  in the map-frame global costmap
```

### Why StaticLayer and not a custom C++ layer

A custom `GradientLayer` reading the float `.npy` directly is the cleaner
architecture — full 0–254 cost, no occupancy round-trip. It costs a catkin build,
a pluginlib export + XML, and a compile/debug cycle. Stock `StaticLayer` needs
none of that. Quantization is not the binding constraint (see below), so the
custom layer buys precision the data cannot supply.

### Cost mapping

Driven by the measured lake distribution:

| Slope | Occupancy | PGM pixel | Resulting costmap cost | Meaning |
|---|---|---|---|---|
| < 10° (`--warn-deg`) | 0 | 255 | 0 | free, no penalty |
| 10–18° | 1–99 linear ramp | 252→3 | 3–251 | crossable but priced |
| > 18° (`--lethal-deg`) | 100 | 0 | 254 | lethal; routed around |
| NaN (no mesh) | -1 (unknown) | 205 | NO_INFORMATION | unknown; NOT lethal |

**The PGM is inverted.** `map_server` reads `occ = (255 - pixel)/255 * 100`, so
pixel 0 = fully occupied and 255 = free — matching the convention already
documented in `map_tools/occupancy_grid.py`. The generator therefore computes
occupancy 0–100 from degrees and inverts on write. Getting this backwards would
make flat ground lethal and steep ground free.

Round-trip verified numerically: occ 0→px255→cost 0; occ 50→px128→cost 127;
occ 100→px0→cost 254. Pixel 205 falls between `free_thresh` and
`occupied_thresh`, so `map_server` emits -1 (unknown) for it.

Thresholds are CLI flags so retuning is a 2-second regenerate. They are
**absolute degrees, never percentile-derived** — a percentile stretch would turn
park's 0.87° max into a fake lethal band.

Resulting composition:
- lake: 85.6% free, 13.6% graded, 0.77% lethal
- park: 100% free, 0% graded, 0% lethal

### PGM viability

PGM stores one byte per pixel — no angles, no floats, no units. It does not need
to: the degrees→cost decision happens offline in the generator, and what lands in
the file is already a cost. This is the same split the repo already uses
(`park_dtm.npy` holds float heights; `park_map.pgm` holds occupancy).

Quantization is not a limit here. The 10–18° band spans ~253 levels = 0.03°/step,
roughly 30x finer than the uncertainty of slope derived from a 0.25 m DTM.

`<world>_slope.npy` preserves the true degrees. It is **not** fed to move_base and
has zero influence on planning; it exists for inspection, verification, threshold
retuning, and as the seam for future fusion.

## Costmap configuration

Added to the map-frame global costmap (`config/costmap_global_gps_map.yaml`),
layer order:

```
static  ->  slope  ->  obstacles  ->  inflation
```

```yaml
slope:
  map_topic: /slope_map
  subscribe_to_updates: false
  trinary_costmap: false      # REQUIRED - see below
  lethal_cost_threshold: 100
  track_unknown_space: false  # NaN/off-mesh -> free, not lethal
  unknown_cost_value: -1     # map_server emits -1 for the unknown band, NOT 255
  use_maximum: true           # REQUIRED - see below
```

### Two parameters that are load-bearing

Both verified against `/opt/ros/noetic/include/costmap_2d/` source, not memory.

**`trinary_costmap: false`** — `StaticLayer::interpretValue()` collapses every
non-lethal value to `FREE_SPACE` when trinary is true (its default). Leaving it
default would silently flatten the entire 10–18° graded band to zero, leaving the
planner with a cliff edge and nothing else.

**`use_maximum: true`** — by default `StaticLayer` **overwrites** the cells it
touches. Since `slope` runs after `static`, the default would erase tree and
bench costs wherever the two overlap, wiping a tree on flat ground to free. With
`use_maximum` the composite is `max(tree, slope, lidar)` per cell.

### Inflation behaviour

`InflationLayer::computeCost()` takes only distance-to-nearest-`LETHAL_OBSTACLE`.
It never inspects what made a cell lethal. Consequences:

- **>18° lethal cells ARE inflated**, identically to trees — same 0.5 m
  exponential skirt. This falls out for free.
- **The 10–18° graded band is NOT inflated** — those cells are below the lethal
  seed threshold. This is correct: they are drivable, and inflating 14% of the
  lake would smear keep-out cost across terrain the robot is allowed to cross.

Inflation runs last, so it seeds from the union of trees + steep terrain + lidar.

## Testing

**Unit (pytest, `map_tools/tests/test_slope_costmap.py`):**
- slope of a synthetic constant-gradient ramp equals the analytic angle
- threshold boundaries: 9.9° -> 0, 10.1° -> >0, 17.9° -> <254, 18.1° -> 254
- NaN in -> unknown pixel 205 out, never 0 (lethal)
- resample alignment: a known DTM cell lands at the correct map-grid index
- park-like input (near-zero relief) -> all-free, no crash, no fabricated cost

**Live (lake world):**
Run the full documented startup procedure, then set a goal across a bank and
confirm in RViz that the global path bows around the steep ribbon. Judge arrival
and path shape by the robot's actual position in Gazebo, per the standing project
rule — never by the fused pose or by move_base's own SUCCEEDED.

Verify on the live run (assumptions, not yet confirmed):
- two static layers on different topics coexist without a geometry complaint
- the unknown water wedge does not become a planned-through shortcut; if the
  existing water/static layer does not block it, move the demo goal to the
  land-side bank
- 0.77% lethal is a thin barrier. If the path threads it, drop `--lethal-deg` to
  15 (~2.5% lethal). Retune the threshold; do not fake the result.

## Known limitations

- **Slope magnitude is unsigned.** 18° up and 18° down get identical cost. This
  matches the stated requirement (avoid both) but cannot express "climbing is
  worse than descending".
- **Retuning requires regeneration.** The PGM cannot reinterpret itself; changing
  a threshold means rerunning the generator.
- **Single scalar cost channel.** Costs merge by max, so a 12° slope under a 0.5 m
  rock reads the same as either alone. Fine while every contributor means "do not
  go here"; insufficient for graded multi-quantity fusion.

## Future work — object-height fusion

The single-scalar ceiling above is the reason `<world>_slope.npy` is a
first-class output rather than a diagnostic afterthought. When object heights
arrive, the right model is a multi-layer grid (one layer per physical quantity,
in native units, fused at query time) converted to a costmap at the end —
`grid_map` + `grid_map_costmap_2d`, both already installed on this box.

At that point the degrees→cost mapping in this design is discarded and redone as
part of fusion. What survives is the extraction: slope in degrees on a known
grid. That is a contained, deliberate loss, accepted to hit today's deadline.
