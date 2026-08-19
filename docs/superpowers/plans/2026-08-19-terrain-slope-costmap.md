# Terrain Slope Costmap Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give `move_base` a terrain-slope layer so the global planner routes around steep uphill and downhill ground.

**Architecture:** An offline generator turns a DTM height raster into slope in degrees, maps degrees to occupancy via two absolute thresholds, resamples onto the existing map grid, and writes a standard `map_server` PGM/YAML pair. A second `map_server` publishes it on `/slope_map`, where a stock `costmap_2d::StaticLayer` instance merges it into the map-frame global costmap. No custom C++ layer, no catkin build.

**Tech Stack:** Python 3 + NumPy (pure, no ROS imports in the generator), pytest, ROS Noetic `map_server` + `costmap_2d`.

**Spec:** `docs/superpowers/specs/2026-08-19-terrain-slope-costmap-design.md`

## Global Constraints

- **Universal, not world-specific.** The generator takes a world name; `slope_costmap.py lake` and `slope_costmap.py park` both work. No world names hardcoded in logic.
- **Thresholds are absolute degrees, never percentile-derived.** Defaults `--warn-deg 10.0`, `--lethal-deg 18.0`. A percentile stretch would turn park's 0.87° max into a fake lethal band.
- **Park must produce an all-free layer** — not a crash, not fabricated cost. This is the flat-terrain correctness case.
- **PGM is inverted.** `map_server` reads `occ = (255 - pixel)/255 * 100`. Pixel 0 = occupied, 255 = free, 205 = unknown. Follow `map_tools/occupancy_grid.py`.
- **PGM row 0 is the TOP of the image = HIGHEST y.** DTM `.npy` row 0 is the LOWEST y. Flip vertically on write.
- **No ROS imports in `map_tools/slope_costmap.py`.** It is offline and must be unit-testable without a roscore.
- **No Gazebo ground truth** anywhere, in code or verification (standing project rule).
- Generator writes only into `maps/`. Follow the CLI/`main()`/`sys.exit(main())` shape of `map_tools/extract_dtm.py`.

---

## File Structure

| File | Responsibility |
|---|---|
| `map_tools/slope_costmap.py` (create) | Pure functions: heights→slope, slope→occupancy, resample, PGM/YAML write. Plus CLI `main()`. |
| `map_tools/tests/test_slope_costmap.py` (create) | Unit tests for every pure function. |
| `maps/<world>_slope.{npy,pgm,yaml}` (generated) | Outputs. Not hand-edited. |
| `config/costmap_global_gps_map.yaml` (modify) | Add the `slope` StaticLayer instance + params. |
| `launch/move_base_gps_map.launch` (modify) | Add the second `map_server` publishing `/slope_map`. |
| `RUN-MAP-NAV.md` (modify) | Document generation + the demo step. |

---

### Task 1: Slope from heights

**Files:**
- Create: `map_tools/slope_costmap.py`
- Test: `map_tools/tests/test_slope_costmap.py`

**Interfaces:**
- Consumes: nothing.
- Produces: `slope_degrees(heights: np.ndarray, resolution: float) -> np.ndarray` — float64 slope in degrees, same shape as input, NaN preserved where input is NaN.

- [ ] **Step 1: Write the failing test**

```python
import numpy as np
import pytest
from map_tools.slope_costmap import slope_degrees


def test_flat_terrain_is_zero_slope():
    heights = np.full((10, 10), 3.0, dtype=np.float32)
    out = slope_degrees(heights, 0.25)
    assert np.allclose(out, 0.0)


def test_constant_ramp_matches_analytic_angle():
    # 1 m rise per 1 m run along +x => 45 degrees, regardless of resolution.
    res = 0.25
    cols = np.arange(20) * res
    heights = np.tile(cols, (20, 1)).astype(np.float32)
    out = slope_degrees(heights, res)
    # Interior cells only: np.gradient uses one-sided differences at edges.
    assert np.allclose(out[1:-1, 1:-1], 45.0, atol=1e-6)


def test_known_shallow_gradient():
    # 0.1 m rise per 1.0 m run => atan(0.1) = 5.7106 degrees
    res = 0.5
    cols = np.arange(12) * res * 0.1
    heights = np.tile(cols, (12, 1)).astype(np.float32)
    out = slope_degrees(heights, res)
    assert out[5, 5] == pytest.approx(np.degrees(np.arctan(0.1)), abs=1e-6)


def test_nan_is_preserved_not_zero():
    heights = np.full((8, 8), 2.0, dtype=np.float32)
    heights[4, 4] = np.nan
    out = slope_degrees(heights, 0.25)
    assert np.isnan(out[4, 4])


def test_slope_is_direction_agnostic():
    # Uphill and downhill of equal steepness get the SAME magnitude.
    res = 0.25
    up = np.tile(np.arange(16) * res, (16, 1)).astype(np.float32)
    down = up[:, ::-1].copy()
    a = slope_degrees(up, res)
    b = slope_degrees(down, res)
    assert np.allclose(a[1:-1, 1:-1], b[1:-1, 1:-1])
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest map_tools/tests/test_slope_costmap.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'map_tools.slope_costmap'`

- [ ] **Step 3: Write minimal implementation**

Create `map_tools/slope_costmap.py`:

```python
"""Terrain slope costmap generator.

Turns a DTM height raster into a map_server PGM/YAML pair encoding terrain
steepness, so move_base's global costmap routes around steep ground.

Pure NumPy + stdlib -- NO ROS imports. Runs offline, unit-testable without a
roscore.

Spec: docs/superpowers/specs/2026-08-19-terrain-slope-costmap-design.md
"""
import numpy as np


def slope_degrees(heights, resolution):
    """Slope magnitude in degrees from a height raster.

    heights: 2D array of metres, row 0 = lowest y. NaN = no mesh coverage.
    resolution: metres per cell (square cells).

    Returns float64 degrees, same shape. NaN in -> NaN out. The result is
    UNSIGNED: an 18 deg climb and an 18 deg descent both return 18.0.
    """
    z = np.asarray(heights, dtype=np.float64)
    # np.gradient(z, dy, dx) -> (d/dy, d/dx) in metres per metre.
    grad_y, grad_x = np.gradient(z, resolution, resolution)
    return np.degrees(np.arctan(np.hypot(grad_x, grad_y)))
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest map_tools/tests/test_slope_costmap.py -v`
Expected: PASS (5 tests)

- [ ] **Step 5: Commit**

```bash
cd /home/thinh/Documents/Husky_viz
git add map_tools/slope_costmap.py map_tools/tests/test_slope_costmap.py
git commit -m "feat(map_tools): slope magnitude in degrees from a DTM raster"
```

---

### Task 2: Degrees to occupancy

**Files:**
- Modify: `map_tools/slope_costmap.py`
- Test: `map_tools/tests/test_slope_costmap.py`

**Interfaces:**
- Consumes: `slope_degrees()` from Task 1.
- Produces: `slope_to_occupancy(slope_deg: np.ndarray, warn_deg: float = 10.0, lethal_deg: float = 18.0) -> np.ndarray` — int16, values 0..100 for known cells and `-1` for unknown (NaN input).

Note the boundary rule, which the tests pin: `< warn` is free, `>= lethal` is 100, and the band in between ramps 1..99. Read `>= lethal` as lethal so that exactly-18.0 is blocked.

- [ ] **Step 1: Write the failing test**

```python
from map_tools.slope_costmap import slope_to_occupancy, UNKNOWN_OCC


def test_below_warn_is_free():
    s = np.array([[0.0, 5.0, 9.9]])
    out = slope_to_occupancy(s, warn_deg=10.0, lethal_deg=18.0)
    assert list(out[0]) == [0, 0, 0]


def test_at_or_above_lethal_is_full_occupancy():
    s = np.array([[18.0, 18.1, 24.3, 90.0]])
    out = slope_to_occupancy(s, warn_deg=10.0, lethal_deg=18.0)
    assert list(out[0]) == [100, 100, 100, 100]


def test_graded_band_is_strictly_between():
    s = np.array([[10.0, 14.0, 17.9]])
    out = slope_to_occupancy(s, warn_deg=10.0, lethal_deg=18.0)
    assert all(1 <= v <= 99 for v in out[0]), list(out[0])


def test_graded_band_is_monotonic():
    s = np.linspace(10.0, 17.9, 40).reshape(1, -1)
    out = slope_to_occupancy(s, warn_deg=10.0, lethal_deg=18.0)[0]
    assert list(out) == sorted(out)


def test_band_midpoint_is_near_half():
    s = np.array([[14.0]])  # exact midpoint of 10..18
    out = slope_to_occupancy(s, warn_deg=10.0, lethal_deg=18.0)
    assert 45 <= out[0, 0] <= 55


def test_nan_becomes_unknown_never_lethal():
    s = np.array([[np.nan, 3.0]])
    out = slope_to_occupancy(s, warn_deg=10.0, lethal_deg=18.0)
    assert out[0, 0] == UNKNOWN_OCC == -1
    assert out[0, 1] == 0


def test_flat_world_produces_no_cost_at_all():
    # The park case: 0.87 deg max relief must yield a uniformly free layer.
    s = np.full((50, 50), 0.87)
    out = slope_to_occupancy(s, warn_deg=10.0, lethal_deg=18.0)
    assert out.max() == 0


def test_thresholds_are_honoured_not_hardcoded():
    s = np.array([[12.0]])
    assert slope_to_occupancy(s, warn_deg=10.0, lethal_deg=18.0)[0, 0] < 100
    assert slope_to_occupancy(s, warn_deg=5.0, lethal_deg=11.0)[0, 0] == 100
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest map_tools/tests/test_slope_costmap.py -v`
Expected: FAIL — `ImportError: cannot import name 'slope_to_occupancy'`

- [ ] **Step 3: Write minimal implementation**

Append to `map_tools/slope_costmap.py`:

```python
# map_server sentinel for "no information". Written to the PGM as pixel 205,
# which falls between free_thresh and occupied_thresh so map_server emits -1.
UNKNOWN_OCC = -1
UNKNOWN_PIXEL = 205


def slope_to_occupancy(slope_deg, warn_deg=10.0, lethal_deg=18.0):
    """Map slope in degrees onto ROS occupancy 0..100, with -1 for unknown.

    < warn_deg          -> 0        free, no penalty
    warn_deg..lethal_deg -> 1..99    crossable but priced (linear ramp)
    >= lethal_deg       -> 100      lethal, planner routes around
    NaN                 -> -1       unknown (NOT lethal -- no mesh there)

    Thresholds are ABSOLUTE DEGREES on purpose. Deriving them from the data's
    own percentiles would invent a lethal band on flat terrain like the park.
    """
    if lethal_deg <= warn_deg:
        raise ValueError("lethal_deg (%r) must exceed warn_deg (%r)"
                         % (lethal_deg, warn_deg))

    s = np.asarray(slope_deg, dtype=np.float64)
    known = np.isfinite(s)

    occ = np.full(s.shape, UNKNOWN_OCC, dtype=np.int16)
    occ[known] = 0

    band = known & (s >= warn_deg) & (s < lethal_deg)
    frac = (s[band] - warn_deg) / (lethal_deg - warn_deg)   # 0.0 .. <1.0
    occ[band] = 1 + (frac * 98.0).round().astype(np.int16)  # 1 .. 99

    occ[known & (s >= lethal_deg)] = 100
    return occ
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest map_tools/tests/test_slope_costmap.py -v`
Expected: PASS (13 tests total)

- [ ] **Step 5: Commit**

```bash
cd /home/thinh/Documents/Husky_viz
git add map_tools/slope_costmap.py map_tools/tests/test_slope_costmap.py
git commit -m "feat(map_tools): map slope degrees to ROS occupancy with absolute thresholds"
```

---

### Task 3: Resample onto the map grid

**Files:**
- Modify: `map_tools/slope_costmap.py`
- Test: `map_tools/tests/test_slope_costmap.py`

**Interfaces:**
- Consumes: `slope_to_occupancy()` from Task 2.
- Produces:
  - `class GridSpec` with attributes `origin_x, origin_y, resolution, width, height` and constructor `GridSpec(origin_x, origin_y, resolution, width, height)`.
  - `resample_nearest(src: np.ndarray, src_grid: GridSpec, dst_grid: GridSpec, fill=UNKNOWN_OCC) -> np.ndarray` — same dtype as `src`, shaped `(dst_grid.height, dst_grid.width)`. Cells falling outside the source get `fill`.

This is the task that prevents every slope cell landing in the wrong place: the DTM is 0.25 m on its own origin, the map is 0.15 m on a different origin. Both use row 0 = lowest y.

- [ ] **Step 1: Write the failing test**

```python
from map_tools.slope_costmap import GridSpec, resample_nearest


def test_identical_grids_roundtrip_unchanged():
    g = GridSpec(origin_x=-1.0, origin_y=-2.0, resolution=0.25, width=8, height=4)
    src = np.arange(32, dtype=np.int16).reshape(4, 8)
    out = resample_nearest(src, g, g)
    assert np.array_equal(out, src)


def test_upsample_preserves_world_position_of_a_marked_cell():
    # Source: 1 m cells, origin (0,0), 4x4. Mark the cell covering world (2.5, 1.5).
    src_grid = GridSpec(0.0, 0.0, 1.0, 4, 4)
    src = np.zeros((4, 4), dtype=np.int16)
    src[1, 2] = 100                      # row=1 -> y in [1,2), col=2 -> x in [2,3)
    dst_grid = GridSpec(0.0, 0.0, 0.5, 8, 8)   # same extent, finer cells
    out = resample_nearest(src, src_grid, dst_grid)
    # World (2.5, 1.5) must still be 100 in the destination.
    col = int((2.5 - dst_grid.origin_x) / dst_grid.resolution)
    row = int((1.5 - dst_grid.origin_y) / dst_grid.resolution)
    assert out[row, col] == 100
    # And a cell far away must not be.
    assert out[0, 0] == 0


def test_offset_origin_is_accounted_for():
    src_grid = GridSpec(-10.0, -10.0, 1.0, 20, 20)
    src = np.zeros((20, 20), dtype=np.int16)
    src[15, 12] = 100                    # world x in [2,3), y in [5,6)
    dst_grid = GridSpec(0.0, 0.0, 1.0, 10, 10)   # different origin
    out = resample_nearest(src, src_grid, dst_grid)
    assert out[5, 2] == 100
    assert out.sum() == 100              # exactly one cell carried over


def test_cells_outside_source_get_fill():
    src_grid = GridSpec(0.0, 0.0, 1.0, 2, 2)
    src = np.zeros((2, 2), dtype=np.int16)
    dst_grid = GridSpec(0.0, 0.0, 1.0, 4, 4)     # extends past the source
    out = resample_nearest(src, src_grid, dst_grid, fill=-1)
    assert out[0, 0] == 0        # inside source
    assert out[3, 3] == -1       # outside source
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest map_tools/tests/test_slope_costmap.py -v`
Expected: FAIL — `ImportError: cannot import name 'GridSpec'`

- [ ] **Step 3: Write minimal implementation**

Append to `map_tools/slope_costmap.py`:

```python
class GridSpec(object):
    """Geometry of a raster: where cell (0,0) sits in the world and how big
    cells are. Row 0 is the LOWEST y (the .npy convention used by extract_dtm).
    """

    def __init__(self, origin_x, origin_y, resolution, width, height):
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.resolution = float(resolution)
        self.width = int(width)
        self.height = int(height)

    def cell_centres(self):
        """World (x, y) of every cell centre, as (xs[width], ys[height])."""
        xs = self.origin_x + (np.arange(self.width) + 0.5) * self.resolution
        ys = self.origin_y + (np.arange(self.height) + 0.5) * self.resolution
        return xs, ys

    def __repr__(self):
        return ("GridSpec(origin=(%.3f, %.3f), res=%.3f, %dx%d)"
                % (self.origin_x, self.origin_y, self.resolution,
                   self.width, self.height))


def resample_nearest(src, src_grid, dst_grid, fill=UNKNOWN_OCC):
    """Nearest-neighbour resample from one grid geometry to another.

    The DTM and the occupancy map do NOT share a grid (different resolution
    AND different origin). Without this step every slope cell lands in the
    wrong place in the costmap.

    Nearest-neighbour, not interpolation: the values are already-classified
    occupancy, and averaging a lethal cell with a free one would invent a
    meaningless intermediate.
    """
    src = np.asarray(src)
    xs, ys = dst_grid.cell_centres()

    cols = np.floor((xs - src_grid.origin_x) / src_grid.resolution).astype(int)
    rows = np.floor((ys - src_grid.origin_y) / src_grid.resolution).astype(int)

    col_ok = (cols >= 0) & (cols < src_grid.width)
    row_ok = (rows >= 0) & (rows < src_grid.height)

    out = np.full((dst_grid.height, dst_grid.width), fill, dtype=src.dtype)
    if not col_ok.any() or not row_ok.any():
        return out

    inside = np.outer(row_ok, col_ok)
    picked = src[np.clip(rows, 0, src_grid.height - 1)[:, None],
                 np.clip(cols, 0, src_grid.width - 1)[None, :]]
    out[inside] = picked[inside]
    return out
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest map_tools/tests/test_slope_costmap.py -v`
Expected: PASS (17 tests total)

- [ ] **Step 5: Commit**

```bash
cd /home/thinh/Documents/Husky_viz
git add map_tools/slope_costmap.py map_tools/tests/test_slope_costmap.py
git commit -m "feat(map_tools): nearest-neighbour resample between DTM and map grids"
```

---

### Task 4: Write the inverted PGM and YAML

**Files:**
- Modify: `map_tools/slope_costmap.py`
- Test: `map_tools/tests/test_slope_costmap.py`

**Interfaces:**
- Consumes: `GridSpec`, `UNKNOWN_OCC`, `UNKNOWN_PIXEL` from Tasks 2-3.
- Produces:
  - `occupancy_to_pixels(occ: np.ndarray) -> np.ndarray` — uint8, inverted.
  - `write_pgm(path: str, pixels: np.ndarray) -> None` — binary P5, flipped so row 0 of the file is the HIGHEST y.
  - `write_yaml(path: str, image_name: str, grid: GridSpec, meta: dict) -> None`

**This is the highest-risk task in the plan.** Two independent inversions are in play and getting either backwards silently makes flat ground lethal:
1. **Value inversion:** pixel 0 = occupied, 255 = free.
2. **Row flip:** the `.npy` has row 0 = lowest y; the PGM file has row 0 = highest y.

- [ ] **Step 1: Write the failing test**

```python
import os
from map_tools.slope_costmap import (
    occupancy_to_pixels, write_pgm, write_yaml, UNKNOWN_PIXEL,
)


def test_free_occupancy_becomes_white_pixel():
    assert occupancy_to_pixels(np.array([[0]], dtype=np.int16))[0, 0] == 255


def test_lethal_occupancy_becomes_black_pixel():
    assert occupancy_to_pixels(np.array([[100]], dtype=np.int16))[0, 0] == 0


def test_unknown_becomes_the_unknown_pixel():
    out = occupancy_to_pixels(np.array([[-1]], dtype=np.int16))
    assert out[0, 0] == UNKNOWN_PIXEL == 205


def test_inversion_roundtrips_through_map_server_formula():
    # map_server: occ = (255 - px) / 255 * 100
    for occ_in in [0, 1, 25, 50, 75, 99, 100]:
        px = int(occupancy_to_pixels(np.array([[occ_in]], dtype=np.int16))[0, 0])
        occ_back = round((255 - px) / 255.0 * 100)
        assert occ_back == occ_in, (occ_in, px, occ_back)


def test_graded_band_is_monotonically_darker():
    occ = np.array([[0, 25, 50, 75, 100]], dtype=np.int16)
    px = occupancy_to_pixels(occ)[0]
    assert list(px) == sorted(px, reverse=True)


def test_pgm_row_zero_is_highest_y(tmp_path):
    # occ row 0 = LOWEST y. In the file, the FIRST row must be the HIGHEST y.
    occ = np.array([[0, 0], [100, 100]], dtype=np.int16)   # row 1 = high y = lethal
    px = occupancy_to_pixels(occ)
    path = str(tmp_path / "t.pgm")
    write_pgm(path, px)
    with open(path, "rb") as fh:
        data = fh.read()
    body = data.split(b"255\n", 1)[1]
    assert body[0] == 0 and body[1] == 0        # first file row = high y = black
    assert body[2] == 255 and body[3] == 255    # last file row = low y = white


def test_pgm_header_is_binary_p5_with_right_dimensions(tmp_path):
    px = np.zeros((3, 7), dtype=np.uint8)
    path = str(tmp_path / "t.pgm")
    write_pgm(path, px)
    with open(path, "rb") as fh:
        head = fh.read(20)
    assert head.startswith(b"P5\n7 3\n255\n")


def test_yaml_carries_grid_and_thresholds(tmp_path):
    g = GridSpec(-55.4915, -30.9713, 0.15, 100, 80)
    path = str(tmp_path / "t.yaml")
    write_yaml(path, "t.pgm", g, {"warn_deg": 10.0, "lethal_deg": 18.0,
                                  "world": "lake"})
    text = open(path).read()
    assert "image: t.pgm" in text
    assert "resolution: 0.150000" in text
    assert "origin: [-55.491500, -30.971300, 0.0]" in text
    assert "warn_deg" in text and "18.0" in text
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest map_tools/tests/test_slope_costmap.py -v`
Expected: FAIL — `ImportError: cannot import name 'occupancy_to_pixels'`

- [ ] **Step 3: Write minimal implementation**

Append to `map_tools/slope_costmap.py`:

```python
def occupancy_to_pixels(occ):
    """ROS occupancy 0..100 (plus -1 unknown) -> map_server PGM pixels.

    THE PGM IS INVERTED: map_server reads occ = (255 - pixel) / 255 * 100, so
    pixel 0 is fully occupied and 255 is free. Getting this backwards makes
    flat ground lethal and steep ground free. Same convention as
    map_tools/occupancy_grid.py.

    Unknown (-1) is written as pixel 205, which lands between free_thresh
    (0.196) and occupied_thresh (0.65) so map_server reports -1 for it.
    """
    occ = np.asarray(occ)
    px = np.round(255.0 - np.clip(occ, 0, 100) * 255.0 / 100.0)
    px = px.astype(np.uint8)
    px[occ < 0] = UNKNOWN_PIXEL
    return px


def write_pgm(path, pixels):
    """Binary P5 PGM.

    Row 0 of `pixels` is the LOWEST y (the .npy convention). The PGM format
    puts the TOP of the image first, and map_server's origin is the
    bottom-left corner -- so flip vertically on write.
    """
    pixels = np.asarray(pixels, dtype=np.uint8)
    height, width = pixels.shape
    with open(path, "wb") as fh:
        fh.write(b"P5\n%d %d\n255\n" % (width, height))
        fh.write(pixels[::-1, :].tobytes())


def write_yaml(path, image_name, grid, meta):
    """map_server YAML, plus the provenance a reader needs to interpret the PGM.

    The thresholds go in as comments: the PGM alone cannot say what angle a
    given pixel came from, so the pair (pgm, yaml) has to carry the mapping.
    """
    with open(path, "w") as fh:
        fh.write("# Terrain SLOPE costmap. Generated by "
                 "map_tools/slope_costmap.py -- do not hand-edit.\n")
        fh.write("# Pixel 255 = free (<warn_deg), 0 = lethal (>=lethal_deg),\n")
        fh.write("# 205 = unknown (no mesh coverage). Values between ramp\n")
        fh.write("# linearly across the warn..lethal band.\n")
        for key in sorted(meta):
            fh.write("# %s: %s\n" % (key, meta[key]))
        fh.write("image: %s\n" % image_name)
        fh.write("resolution: %.6f\n" % grid.resolution)
        fh.write("origin: [%.6f, %.6f, 0.0]\n"
                 % (grid.origin_x, grid.origin_y))
        fh.write("negate: 0\n")
        fh.write("occupied_thresh: 0.65\n")
        fh.write("free_thresh: 0.196\n")
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest map_tools/tests/test_slope_costmap.py -v`
Expected: PASS (25 tests total)

- [ ] **Step 5: Commit**

```bash
cd /home/thinh/Documents/Husky_viz
git add map_tools/slope_costmap.py map_tools/tests/test_slope_costmap.py
git commit -m "feat(map_tools): inverted PGM + YAML writers for the slope layer"
```

---

### Task 5: CLI, wired end to end

**Files:**
- Modify: `map_tools/slope_costmap.py`
- Test: `map_tools/tests/test_slope_costmap.py`

**Interfaces:**
- Consumes: everything from Tasks 1-4.
- Produces:
  - `read_dtm_yaml(path: str) -> dict` — parses the `key: value` lines of a DTM yaml (it is written by hand in `extract_dtm.py`, so parse it plainly rather than adding a PyYAML dependency).
  - `build(world: str, maps_dir: str, warn_deg: float, lethal_deg: float) -> dict` — does the whole pipeline, writes the three files, returns a stats dict with keys `free_pct`, `graded_pct`, `lethal_pct`, `unknown_pct`, `max_slope_deg`.
  - `main(argv=None) -> int`

CLI contract: `python3 -m map_tools.slope_costmap <world> [--maps-dir maps] [--warn-deg 10.0] [--lethal-deg 18.0]`

- [ ] **Step 1: Write the failing test**

```python
from map_tools.slope_costmap import read_dtm_yaml, build, main


def _fake_world(tmp_path, name, heights, res=0.25, ox=-5.0, oy=-4.0):
    """Write a minimal <name>_dtm.{npy,yaml} + <name>_map.yaml pair."""
    maps = tmp_path / "maps"
    maps.mkdir(exist_ok=True)
    np.save(str(maps / ("%s_dtm.npy" % name)), heights.astype(np.float32))
    (maps / ("%s_dtm.yaml" % name)).write_text(
        "# comment line that must be skipped\n"
        "layer: terrain\n"
        "resolution: %f\norigin_x: %f\norigin_y: %f\n"
        "width: %d\nheight: %d\n"
        % (res, ox, oy, heights.shape[1], heights.shape[0]))
    (maps / ("%s_map.yaml" % name)).write_text(
        "image: %s_map.pgm\nresolution: 0.150000\n"
        "origin: [%f, %f, 0.0]\nnegate: 0\n"
        "occupied_thresh: 0.65\nfree_thresh: 0.196\n" % (name, ox, oy))
    return maps


def test_read_dtm_yaml_skips_comments_and_types_values(tmp_path):
    p = tmp_path / "d.yaml"
    p.write_text("# a comment\nlayer: terrain\nresolution: 0.250000\n"
                 "width: 400\nnote: free text here\n")
    got = read_dtm_yaml(str(p))
    assert got["layer"] == "terrain"
    assert got["resolution"] == 0.25
    assert got["width"] == 400
    assert got["note"] == "free text here"


def test_build_on_flat_world_produces_all_free(tmp_path):
    heights = np.full((40, 60), 3.0)
    maps = _fake_world(tmp_path, "flatland", heights)
    stats = build("flatland", str(maps), warn_deg=10.0, lethal_deg=18.0)
    assert stats["lethal_pct"] == 0.0
    assert stats["graded_pct"] == 0.0
    assert stats["free_pct"] == 100.0
    assert os.path.exists(str(maps / "flatland_slope.pgm"))
    assert os.path.exists(str(maps / "flatland_slope.yaml"))
    assert os.path.exists(str(maps / "flatland_slope.npy"))


def test_build_writes_degrees_not_cost_in_the_npy(tmp_path):
    res = 0.25
    heights = np.tile(np.arange(60) * res, (40, 1))   # 45 degree ramp
    maps = _fake_world(tmp_path, "ramp", heights, res=res)
    build("ramp", str(maps), warn_deg=10.0, lethal_deg=18.0)
    saved = np.load(str(maps / "ramp_slope.npy"))
    assert saved.dtype == np.float32
    assert saved[20, 30] == pytest.approx(45.0, abs=1e-4)


def test_build_marks_steep_ground_lethal(tmp_path):
    res = 0.25
    heights = np.tile(np.arange(60) * res, (40, 1))   # 45 deg everywhere
    maps = _fake_world(tmp_path, "steep", heights, res=res)
    stats = build("steep", str(maps), warn_deg=10.0, lethal_deg=18.0)
    assert stats["lethal_pct"] > 90.0


def test_build_propagates_nan_as_unknown(tmp_path):
    heights = np.full((40, 60), 3.0)
    heights[:20, :] = np.nan
    maps = _fake_world(tmp_path, "holey", heights)
    stats = build("holey", str(maps), warn_deg=10.0, lethal_deg=18.0)
    assert stats["unknown_pct"] > 0.0
    assert stats["lethal_pct"] == 0.0     # NaN must never become lethal


def test_main_returns_zero_on_success(tmp_path):
    maps = _fake_world(tmp_path, "cli", np.full((40, 60), 3.0))
    assert main(["cli", "--maps-dir", str(maps)]) == 0


def test_main_rejects_inverted_thresholds(tmp_path):
    maps = _fake_world(tmp_path, "cli2", np.full((40, 60), 3.0))
    assert main(["cli2", "--maps-dir", str(maps),
                 "--warn-deg", "20", "--lethal-deg", "10"]) != 0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest map_tools/tests/test_slope_costmap.py -v`
Expected: FAIL — `ImportError: cannot import name 'read_dtm_yaml'`

- [ ] **Step 3: Write minimal implementation**

Append to `map_tools/slope_costmap.py`:

```python
import argparse
import os
import sys


def read_dtm_yaml(path):
    """Parse the flat `key: value` yaml written by extract_dtm.py.

    Deliberately not PyYAML: these files are generated by hand-rolled writers
    with a fixed shape, and map_tools has no yaml dependency today.
    """
    out = {}
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#") or ":" not in line:
                continue
            key, _, value = line.partition(":")
            value = value.strip()
            try:
                out[key.strip()] = int(value)
            except ValueError:
                try:
                    out[key.strip()] = float(value)
                except ValueError:
                    out[key.strip()] = value
    return out


def _map_grid_from_yaml(path):
    """GridSpec for the occupancy map. width/height are not in the yaml, so
    they are supplied by the caller from the PGM or the DTM footprint."""
    meta = read_dtm_yaml(path)
    origin = meta["origin"]
    if isinstance(origin, str):
        parts = origin.strip("[]").split(",")
        ox, oy = float(parts[0]), float(parts[1])
    else:
        raise ValueError("could not parse origin from %s" % path)
    return ox, oy, float(meta["resolution"])


def build(world, maps_dir="maps", warn_deg=10.0, lethal_deg=18.0):
    """Full pipeline: DTM -> slope -> occupancy -> map grid -> PGM/YAML/NPY."""
    dtm_npy = os.path.join(maps_dir, "%s_dtm.npy" % world)
    dtm_yaml = os.path.join(maps_dir, "%s_dtm.yaml" % world)
    map_yaml = os.path.join(maps_dir, "%s_map.yaml" % world)

    heights = np.load(dtm_npy)
    dmeta = read_dtm_yaml(dtm_yaml)
    src = GridSpec(dmeta["origin_x"], dmeta["origin_y"], dmeta["resolution"],
                   dmeta["width"], dmeta["height"])

    slope = slope_degrees(heights, src.resolution)
    occ_src = slope_to_occupancy(slope, warn_deg, lethal_deg)

    # Destination grid: the occupancy map's origin+resolution, sized to cover
    # the same world footprint as the DTM.
    ox, oy, res = _map_grid_from_yaml(map_yaml)
    span_x = src.width * src.resolution
    span_y = src.height * src.resolution
    dst = GridSpec(ox, oy, res,
                   int(np.ceil(span_x / res)), int(np.ceil(span_y / res)))

    occ_dst = resample_nearest(occ_src, src, dst, fill=UNKNOWN_OCC)

    base = os.path.join(maps_dir, "%s_slope" % world)
    np.save(base + ".npy", slope.astype(np.float32))
    write_pgm(base + ".pgm", occupancy_to_pixels(occ_dst))
    write_yaml(base + ".yaml", "%s_slope.pgm" % world, dst, {
        "world": world,
        "source_dtm": os.path.normpath(dtm_npy),
        "aligned_to": os.path.normpath(map_yaml),
        "warn_deg": warn_deg,
        "lethal_deg": lethal_deg,
        "max_slope_deg": "%.4f" % float(np.nanmax(slope)),
    })

    known = occ_dst >= 0
    n = float(known.sum()) or 1.0
    stats = {
        "free_pct": 100.0 * ((occ_dst == 0) & known).sum() / n,
        "graded_pct": 100.0 * ((occ_dst > 0) & (occ_dst < 100)).sum() / n,
        "lethal_pct": 100.0 * (occ_dst == 100).sum() / n,
        "unknown_pct": 100.0 * (~known).sum() / occ_dst.size,
        "max_slope_deg": float(np.nanmax(slope)),
    }
    return stats


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Build a terrain slope costmap from a world's DTM.")
    ap.add_argument("world", help="world name, e.g. lake or park")
    ap.add_argument("--maps-dir", default="maps")
    ap.add_argument("--warn-deg", type=float, default=10.0,
                    help="slope at which cost starts ramping up (default 10)")
    ap.add_argument("--lethal-deg", type=float, default=18.0,
                    help="slope treated as impassable (default 18)")
    args = ap.parse_args(argv)

    try:
        stats = build(args.world, args.maps_dir,
                      args.warn_deg, args.lethal_deg)
    except (ValueError, IOError, OSError) as exc:
        sys.stderr.write("error: %s\n" % exc)
        return 1

    print("slope costmap: %s  (warn %.1f deg, lethal %.1f deg)"
          % (args.world, args.warn_deg, args.lethal_deg))
    print("  max slope : %.2f deg" % stats["max_slope_deg"])
    print("  free      : %6.2f%% of known cells" % stats["free_pct"])
    print("  graded    : %6.2f%%" % stats["graded_pct"])
    print("  LETHAL    : %6.2f%%" % stats["lethal_pct"])
    print("  unknown   : %6.2f%% of grid" % stats["unknown_pct"])
    if stats["lethal_pct"] == 0.0 and stats["graded_pct"] == 0.0:
        print("  NOTE: terrain is flat -- this layer is a no-op, as expected "
              "for a world like the park.")
    print("  -> %s_slope.{npy,pgm,yaml}"
          % os.path.join(args.maps_dir, args.world))
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

Move the `import argparse/os/sys` lines to the top of the file with the existing `import numpy as np`.

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest map_tools/tests/test_slope_costmap.py -v`
Expected: PASS (32 tests total)

- [ ] **Step 5: Commit**

```bash
cd /home/thinh/Documents/Husky_viz
git add map_tools/slope_costmap.py map_tools/tests/test_slope_costmap.py
git commit -m "feat(map_tools): slope_costmap CLI, DTM to map_server layer end to end"
```

---

### Task 6: Generate both worlds and sanity-check the numbers

**Files:**
- Create (generated): `maps/lake_slope.{npy,pgm,yaml}`, `maps/park_slope.{npy,pgm,yaml}`

**Interfaces:**
- Consumes: the CLI from Task 5.
- Produces: the map files the launch file will serve.

This is the gate that catches a silent inversion before it reaches the robot. The expected numbers come from the spec's measured table — if the run disagrees, STOP and diagnose rather than adjusting the expectation.

- [ ] **Step 1: Generate the lake layer**

```bash
cd /home/thinh/Documents/Husky_viz
python3 -m map_tools.slope_costmap lake
```

Expected, from the spec's measured distribution:
- max slope ≈ **24.3 deg**
- LETHAL ≈ **0.77%** of known cells
- graded ≈ **13.6%**
- free ≈ **85.6%**
- unknown ≈ **31%** of grid

- [ ] **Step 2: Generate the park layer**

```bash
cd /home/thinh/Documents/Husky_viz
python3 -m map_tools.slope_costmap park
```

Expected: max slope ≈ **0.87 deg**, LETHAL **0.00%**, graded **0.00%**, free **100.00%**, plus the "terrain is flat -- this layer is a no-op" note. Anything else means the absolute thresholds have been compromised.

- [ ] **Step 3: Verify the PGM inversion against map_server's own formula**

```bash
cd /home/thinh/Documents/Husky_viz
python3 - <<'EOF'
import numpy as np
for w, expect_flat in [("park", True), ("lake", False)]:
    with open("maps/%s_slope.pgm" % w, "rb") as fh:
        data = fh.read()
    head, body = data.split(b"255\n", 1)
    wd, ht = map(int, head.split(b"\n")[1].split())
    px = np.frombuffer(body, np.uint8).reshape(ht, wd)
    occ = np.where(px == 205, -1, np.round((255.0 - px) / 255.0 * 100))
    known = occ >= 0
    print("%-5s  %dx%d  free=%.2f%%  lethal=%.2f%%  unknown=%.2f%%"
          % (w, wd, ht,
             100.0 * ((occ == 0) & known).sum() / known.sum(),
             100.0 * (occ == 100).sum() / known.sum(),
             100.0 * (~known).sum() / occ.size))
    if expect_flat:
        assert (occ[known] == 0).all(), "PARK MUST BE ALL FREE -- inversion bug?"
print("inversion OK")
EOF
```

Expected: prints both rows and `inversion OK`. If park shows lethal cells, the value inversion is backwards — fix Task 4 before continuing.

- [ ] **Step 4: Commit the generated maps**

```bash
cd /home/thinh/Documents/Husky_viz
git add maps/lake_slope.npy maps/lake_slope.pgm maps/lake_slope.yaml \
        maps/park_slope.npy maps/park_slope.pgm maps/park_slope.yaml
git commit -m "feat(maps): generated slope costmaps for lake and park"
```

---

### Task 7: Wire the layer into move_base

**Files:**
- Modify: `config/costmap_global_gps_map.yaml`
- Modify: `launch/move_base_gps_map.launch`

**Interfaces:**
- Consumes: `maps/<world>_slope.yaml` from Task 6.
- Produces: a `/slope_map` topic and a `slope` layer in the global costmap.

- [ ] **Step 1: Add the slope layer to the global costmap config**

In `config/costmap_global_gps_map.yaml`, change the `plugins` list so `slope` sits between `static` and `obstacles`:

```yaml
plugins:
  - {name: static,     type: "costmap_2d::StaticLayer"}
  - {name: slope,      type: "costmap_2d::StaticLayer"}
  - {name: obstacles,  type: "costmap_2d::ObstacleLayer"}
  - {name: inflation,  type: "costmap_2d::InflationLayer"}
```

and append this block to the same file:

```yaml
# Terrain steepness, precomputed offline by map_tools/slope_costmap.py from the
# world's DTM. Free below 10 deg, a graded ramp 10-18 deg, lethal above 18 deg.
#
# TWO PARAMETERS HERE ARE LOAD-BEARING -- both defaults are wrong for this use:
#
#   trinary_costmap: false
#     StaticLayer::interpretValue() collapses EVERY non-lethal value to
#     FREE_SPACE when trinary is true (its default). Leaving it default would
#     silently flatten the entire 10-18 deg graded band to zero, leaving the
#     planner a cliff edge and nothing in between.
#
#   use_maximum: true
#     StaticLayer OVERWRITES by default (use_maximum defaults to false). Since
#     this layer runs AFTER `static`, the default would erase tree and bench
#     costs wherever the two overlap -- a tree on flat ground would be wiped to
#     free. With use_maximum the composite is max(tree, slope, lidar) per cell.
#
# track_unknown_space: false because ~31% of the lake grid is off-mesh NaN;
# treating that as unknown inside a static_map:true costmap would carve holes in
# a map the planner is told is fully known.
slope:
  map_topic: /slope_map
  subscribe_to_updates: false
  trinary_costmap: false
  lethal_cost_threshold: 100
  track_unknown_space: false
  unknown_cost_value: -1
  use_maximum: true
```

- [ ] **Step 2: Add the second map_server to the launch file**

In `launch/move_base_gps_map.launch`, alongside the existing `map_server`, add:

VERIFIED: this repo has **no `package.xml`**, so `$(find husky_viz)` will NOT
resolve. The existing `map_server` node at line 25 takes an absolute path via a
`map` arg (line 23). Match that convention exactly:

```xml
  <!-- Terrain slope layer. Served as a second map on /slope_map, consumed by
       the `slope` StaticLayer in the global costmap. Absolute path, matching
       the `map` arg above -- this repo is not a catkin package, so $(find ...)
       does not resolve here.
       Generate with: python3 -m map_tools.slope_costmap <world> -->
  <arg name="slope_map"
       default="/home/thinh/Documents/Husky_viz/maps/park_slope.yaml"/>
  <node name="slope_map_server" pkg="map_server" type="map_server"
        args="$(arg slope_map)">
    <remap from="map" to="/slope_map"/>
    <remap from="map_metadata" to="/slope_map_metadata"/>
  </node>
```

For the lake demo, override it the same way the existing header documents
overriding `map:=` (see line 18):

    roslaunch launch/move_base_gps_map.launch \
      map:=/home/thinh/Documents/Husky_viz/maps/lake_map.yaml \
      slope_map:=/home/thinh/Documents/Husky_viz/maps/lake_slope.yaml

Add that line to the launch file's header comment block alongside the existing
`map:=` example.

- [ ] **Step 3: Verify the launch file parses**

```bash
cd /home/thinh/Documents/Husky_viz
python3 -c "import xml.etree.ElementTree as ET; ET.parse('launch/move_base_gps_map.launch'); print('XML OK')"
```

Expected: `XML OK`

- [ ] **Step 4: Verify the YAML parses and the params are present**

```bash
cd /home/thinh/Documents/Husky_viz
python3 - <<'EOF'
import yaml
d = yaml.safe_load(open("config/costmap_global_gps_map.yaml"))
names = [p["name"] for p in d["plugins"]]
assert names == ["static", "slope", "obstacles", "inflation"], names
s = d["slope"]
assert s["trinary_costmap"] is False, "trinary MUST be false or the band flattens"
assert s["use_maximum"] is True, "use_maximum MUST be true or trees get erased"
assert s["unknown_cost_value"] == -1
assert s["map_topic"] == "/slope_map"
print("costmap config OK:", names)
EOF
```

Expected: `costmap config OK: ['static', 'slope', 'obstacles', 'inflation']`

- [ ] **Step 5: Commit**

```bash
cd /home/thinh/Documents/Husky_viz
git add config/costmap_global_gps_map.yaml launch/move_base_gps_map.launch
git commit -m "feat(nav): add terrain slope StaticLayer to the global costmap"
```

---

### Task 8: Document the runbook step

**Files:**
- Modify: `RUN-MAP-NAV.md`

**Interfaces:**
- Consumes: everything above.
- Produces: the documented procedure a future session follows.

- [ ] **Step 1: Add a slope-map section**

Add to `RUN-MAP-NAV.md`, in the map-preparation part of the document (match the surrounding heading level and prose style):

```markdown
### Terrain slope layer

The global costmap carries a `slope` layer so the planner routes around steep
uphill and downhill ground. It is precomputed from the world's DTM -- regenerate
it whenever the DTM changes or you want different thresholds:

    python3 -m map_tools.slope_costmap lake      # the sloped demo world
    python3 -m map_tools.slope_costmap park      # flat: produces an all-free layer

Writes `maps/<world>_slope.{npy,pgm,yaml}`. The `.pgm` is what move_base
consumes (via a second `map_server` on `/slope_map`); the `.npy` holds the slope
in DEGREES and is for inspection and future terrain work, not for the planner.

Thresholds are absolute degrees, defaulting to free below 10 deg, a graded
ramp 10-18 deg, and lethal above 18 deg. Retune with `--warn-deg` /
`--lethal-deg` and regenerate -- the PGM cannot reinterpret itself.

Expected output: lake ~0.8% lethal / ~14% graded; park 0% of both.
```

- [ ] **Step 2: Verify the commands in the doc actually run**

```bash
cd /home/thinh/Documents/Husky_viz
python3 -m map_tools.slope_costmap lake && python3 -m map_tools.slope_costmap park
```

Expected: both exit 0 with the stats from Task 6.

- [ ] **Step 3: Commit**

```bash
cd /home/thinh/Documents/Husky_viz
git add RUN-MAP-NAV.md
git commit -m "docs: document terrain slope layer generation in the nav runbook"
```

---

### Task 9: Live verification in the lake world

**Files:** none modified — this is a run, not an edit.

**Interfaces:**
- Consumes: everything above.
- Produces: a judgement on whether the planner actually avoids steep ground.

**This task is run by the main conversation, NOT by an implementing subagent** (standing project rule: subagents implement and test offline; the main session runs the sim). An implementing agent should stop at Task 8 and hand back.

**Follow `RUN-MAP-NAV.md` Steps 0-3 verbatim and in full.** Do not skip a step you judge irrelevant. Ensure `gzclient` is up on `:0` or `/os0_cloud_node/points` will have zero publishers and the costmaps stay empty.

- [ ] **Step 1: Confirm the slope map is actually being served**

```bash
rostopic echo -n1 /slope_map/info
```

Expected: resolution 0.15 and an origin matching `maps/lake_slope.yaml`. If this hangs, the second `map_server` did not start.

- [ ] **Step 2: Confirm the layer reached the costmap**

```bash
rosrun costmap_2d costmap_2d_markers &
rostopic echo -n1 /move_base/global_costmap/costmap/info
```

Then confirm in RViz (add a Map display on `/move_base/global_costmap/costmap`) that a dark ridge traces the lakebed banks. Compare against the preview: the steep structure should be **linear and connected along the shoreline**, not scattered speckle.

- [ ] **Step 3: Verify trees survived the merge**

This is the `use_maximum` check. In RViz, confirm known tree positions are STILL lethal in the global costmap after the slope layer loads. If trees vanished, `use_maximum` is not taking effect — re-check Task 7.

- [ ] **Step 4: Send a goal across a steep bank**

Send a goal through the operator (never raw `cmd_vel`, never a hand-driven pose) positioned so the straight line from the robot crosses the steep ridge.

Expected: the global path **bows around** the steep band instead of crossing it.

- [ ] **Step 5: Judge the result honestly**

Judge arrival and path shape by **the robot's actual position in Gazebo versus the goal marker** — never by move_base's `SUCCEEDED`, never by the fused pose, both of which drift or can be spoofed.

Record the outcome, including these known open questions:
- **Does the unknown water wedge become a planned-through shortcut?** With `track_unknown_space: false` those cells read as free. `maps/lake_water.npy` and the `static` layer are supposed to block the water. There is a **known contradiction in the notes**: `lake_dtm.yaml` says `lago.dae` IS the collision mesh, while an earlier note says lago has NO collision. Resolve it by observation here. If water is not blocked, move the demo goal to the land-side bank and note it.
- **Is 0.77% lethal a thick enough barrier?** If the path threads between lethal cells, regenerate with `--lethal-deg 15` (~2.5% lethal). Retune the threshold — do not fake the result.
- **Do two static layers on different topics coexist** without a grid-geometry complaint from `costmap_2d`?

- [ ] **Step 6: Record findings**

Append the outcome to the spec's testing section, or to a short run report under `docs/`, so the next session inherits the answers rather than rediscovering them.

---

## Self-Review

**1. Spec coverage**

| Spec requirement | Task |
|---|---|
| Universal generator, world as argument | 5 (CLI), 6 (both worlds run) |
| Absolute-degree thresholds, never percentile | 2 (`test_thresholds_are_honoured_not_hardcoded`), 6 (park gate) |
| Park all-free correctness case | 2, 5, 6 |
| slope in degrees via np.gradient | 1 |
| Cost mapping 0 / ramp / lethal / unknown | 2 |
| Resample DTM grid -> map grid | 3 |
| Inverted PGM + row flip | 4 |
| `.npy` in degrees as first-class output | 5 (`test_build_writes_degrees_not_cost_in_the_npy`) |
| YAML carries thresholds | 4 |
| Second map_server on /slope_map | 7 |
| StaticLayer with trinary_costmap:false | 7 (config + assertion) |
| use_maximum:true | 7 (config + assertion), 9 Step 3 (live) |
| track_unknown_space:false, unknown -1 | 7 |
| Layer order static→slope→obstacles→inflation | 7 (assertion) |
| Inflation seeds only from lethal | 9 (observed, not configured — no task needed) |
| Live lake demo, judged in Gazebo | 9 |
| Water-wedge open question | 9 Step 5 |
| 0.77% thin-barrier fallback | 9 Step 5 |

No gaps.

**2. Placeholder scan:** No TBD/TODO. Every code step carries real code; every test step carries real assertions. Task 9 has no code because it is a sim run, and its steps are concrete commands plus explicit pass criteria.

**3. Type consistency:** `slope_degrees` → float64 → `slope_to_occupancy` → int16 (0..100, -1) → `resample_nearest` (dtype-preserving, `fill=UNKNOWN_OCC`) → `occupancy_to_pixels` → uint8 → `write_pgm`. `GridSpec` constructed identically in Tasks 3 and 5. `UNKNOWN_OCC = -1` and `UNKNOWN_PIXEL = 205` defined once in Task 2 and imported by Task 4's tests. `build()` returns exactly the keys Task 6 prints. Consistent.

**Known ordering note:** Task 5 adds `import argparse/os/sys`, but Task 4's tests already `import os`. That is in the test file, not the module, so there is no ordering problem.
