# Terrain slope costmap: Tasks 1-5 report

All five tasks built one file, `map_tools/slope_costmap.py`, and its test file
`map_tools/tests/test_slope_costmap.py`, incrementally, following TDD per the
briefs. Test command throughout:

```
cd /home/thinh/Documents/Husky_viz && python3 -m pytest map_tools/tests/test_slope_costmap.py -v
```

## Task 1 — Slope from heights

Implemented `slope_degrees(heights, resolution)` using `np.gradient` +
`np.hypot` + `arctan`, as specified.

Failing-test run confirmed `ModuleNotFoundError: No module named
'map_tools.slope_costmap'` before implementation existed.

**Deviation from the brief's literal code:** the brief's Step 3 snippet does
not mask NaN explicitly — it relies on `np.gradient` propagating NaN through
the arithmetic. That assumption is false on numpy 1.24.4 (installed here):
`np.gradient`'s central difference at a NaN cell reads its *neighbours*, not
the cell itself, so a lone NaN at `heights[4,4]` does not appear in the
gradient's own `[4,4]` output — `test_nan_is_preserved_not_zero` failed with
`out[4,4] == 0.0`, not NaN. I added one line after computing slope:
`slope[np.isnan(z)] = np.nan`, explicitly masking, then re-ran. This keeps the
interface and behavior contract from the brief; only the implementation
detail changed to make the stated behavior actually true on this numpy.

Verified: `python3 -c "..."` confirmed `np.gradient` on a flat 8x8 array with
one NaN produces `gx[4,4]=0.0, gy[4,4]=0.0`, not NaN — this is why the
explicit mask was necessary.

Post-fix run: 5/5 passing.

Commit: `307a5c8` — "feat(map_tools): slope magnitude in degrees from a DTM raster"

## Task 2 — Degrees to occupancy

Implemented `slope_to_occupancy(slope_deg, warn_deg=10.0, lethal_deg=18.0)`
plus module constants `UNKNOWN_OCC = -1`, `UNKNOWN_PIXEL = 205`, exactly per
the brief's Step 3 code (no deviation needed).

Failing-test run confirmed `ImportError: cannot import name
'slope_to_occupancy'`.

Post-implementation run: 13/13 passing (5 from Task 1 + 8 new).

Commit: `a35bbff` — "feat(map_tools): map slope degrees to ROS occupancy with absolute thresholds"

## Task 3 — Resample onto the map grid

Implemented `GridSpec` and `resample_nearest(src, src_grid, dst_grid,
fill=UNKNOWN_OCC)` exactly per the brief's Step 3 code.

Failing-test run confirmed `ImportError: cannot import name 'GridSpec'`.

Post-implementation run: 17/17 passing.

Commit: `d7eb497` — "feat(map_tools): nearest-neighbour resample between DTM and map grids"

## Task 4 — Write the inverted PGM and YAML

Implemented `occupancy_to_pixels`, `write_pgm`, `write_yaml` exactly per the
brief's Step 3 code.

Failing-test run confirmed `ImportError: cannot import name
'occupancy_to_pixels'`.

Post-implementation run: 25/25 passing, including the roundtrip test against
the map_server inversion formula and the row-flip test — both inversions
(value inversion, row flip) verified correct on first implementation attempt.

Commit: `abbf092` — "feat(map_tools): inverted PGM + YAML writers for the slope layer"

## Task 5 — CLI, wired end to end

Implemented `read_dtm_yaml`, `_map_grid_from_yaml`, `build`, `main` exactly
per the brief's Step 3 code. Moved `import argparse / os / sys` to the top of
the module alongside `import numpy as np`, as the brief's closing instruction
required.

Failing-test run confirmed `ImportError: cannot import name 'read_dtm_yaml'`.

Post-implementation run: 32/32 passing (all tasks combined).

Ruling 1 (origin parsed as a string then re-split in `_map_grid_from_yaml`)
and Ruling 2 (`build()` sizes the destination grid from the DTM footprint via
`span / resolution`, not from any existing map PGM's dimensions) were both
implemented as specified — the brief's code already encodes both rulings
verbatim, so no additional decision was required here.

Commit: `0ee65fd` — "feat(map_tools): slope_costmap CLI, DTM to map_server layer end to end"

## Summary

- Final test run: 32/32 passing.
- No ROS imports anywhere in `map_tools/slope_costmap.py` — verified by
  inspection; only `numpy`, `argparse`, `os`, `sys` are imported.
- Thresholds are absolute degrees (`warn_deg=10.0`, `lethal_deg=18.0`
  defaults), never derived from data percentiles.
- Flat terrain: `test_flat_world_produces_no_cost_at_all` and
  `test_build_on_flat_world_produces_all_free` both confirm an all-free
  output, no crash, no fabricated cost.
- Only surprise: the NaN-propagation assumption in Task 1's brief snippet
  does not hold on numpy 1.24.4 and needed one explicit masking line to make
  the stated NaN-preservation contract true. Everything else in the five
  briefs' Step 3 code worked as given, first try.

---

## Follow-up fix (2026-08-19): align slope grid to object map PGM

**Defect:** `build()` sized the destination `GridSpec` from the DTM footprint
(`span_x/res`, `span_y/res`) instead of the object map's actual PGM
dimensions, producing a mismatch (lake: slope 664x334 vs map 716x321; park:
slope 667x335 vs map 722x392). With `static_map: true, rolling_window: false`,
costmap_2d's `StaticLayer::incomingMap` resizes the whole layered costmap
(`resizeMap(..., size_locked=true)`) whenever an incoming map's dimensions
differ from the master, so two static layers of different sizes fight over
the master costmap's dimensions and truncate each other.

**Fix:** added `read_pgm_dimensions()` — a header-only binary P5 PGM parser
(reads the four whitespace-separated tokens `P5 width height maxval`, never
touches the pixel body) — and changed `build()` to size `dst` from
`read_pgm_dimensions(maps/<world>_map.pgm)` instead of
`ceil(span/res)`. Origin and resolution are still taken from
`<world>_map.yaml`, unchanged. A missing `<world>_map.pgm` now raises
`IOError` naming the file, rather than silently falling back to the DTM
footprint.

### Tests added (`map_tools/tests/test_slope_costmap.py`)

- `test_build_sizes_slope_grid_to_object_map_not_dtm_footprint` — builds a
  fake world where the DTM footprint is 40x60 cells @ res=0.25 (which the old
  code would size to `ceil(40*0.25/0.15) x ceil(60*0.25/0.15)` = 67x100) but
  the object map PGM is deliberately set to 70x50; asserts the produced slope
  PGM is exactly 70x50, matching the map PGM and NOT the footprint-derived
  size. This genuinely fails against the pre-fix `build()`.
- `test_build_raises_clear_error_when_object_map_pgm_missing` — removes
  `<world>_map.pgm` and asserts `build()` raises `IOError` with the filename
  `nomap_map.pgm` in the message, rather than falling back to the DTM
  footprint.
- Extended the `_fake_world` test helper to also write a valid P5
  `<name>_map.pgm` (via the existing `write_pgm`), with optional
  `map_width`/`map_height` overrides so tests can force a footprint/map-PGM
  size mismatch. All previously-passing tests continued to pass after this
  helper change.

### Commands run and output

```
python3 -m pytest map_tools/tests/test_slope_costmap.py -v
```
Result: **34 passed** (32 previously-existing + 2 new).

```
python3 -m map_tools.slope_costmap lake
python3 -m map_tools.slope_costmap park
```
```
slope costmap: lake  max slope 24.25 deg, free 85.11%, graded 14.10%, LETHAL 0.79%, unknown 36.69%
slope costmap: park  max slope 0.87 deg, free 100.00%, graded 0.00%, LETHAL 0.00%, unknown 21.88% (flat terrain no-op note)
```

Dimension check:
```
lake map   716 321
lake slope 716 321
park map   722 392
park slope 722 392
```
Before: lake slope was 664x334, park slope was 667x335 — both now match
their object map's PGM exactly.

Inversion / flat-terrain gate:
```
park  722x392 free=100.00% lethal=0.00%
lake  716x321 free=85.11% lethal=0.79%
inversion OK
```
Park remains 100% free — the flat-terrain correctness gate did not regress.

**Commit:** `fix(map_tools): size slope grid to the object map so both static layers align`
