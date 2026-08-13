# RANSAC Whole-Set Constellation Matcher Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax.

**Goal:** Replace the brittle `_grow` constellation matcher with a RANSAC whole-set matcher that identifies landmarks by drift-invariant shape, surviving the measured ~4m transient odom-prior drift (≈ same-type landmark spacing) that breaks prior-dependent association.

**Architecture:** `constellation.match` becomes: enumerate seed pairs (observed pair → catalog pair, matched by frame-invariant pairwise distance, no prior); for each, compute the implied rigid transform and count whole-set inliers (observations landing within a TIGHT 0.5m of a same-type catalog landmark); keep the max-inlier arrangement; require ≥3 inliers; a WIDE (~15m) prior sanity check only. `solve_pose` reverts to calling `constellation.match` and does the final pose refit + residual gate.

**Tech Stack:** Python 3, numpy, pytest. Files in `landmark_loc/`.

## Global Constraints

- Spec is authoritative: `docs/superpowers/specs/2026-08-13-ransac-constellation-matcher-design.md`.
- **Drift-invariance is the core property:** identification MUST NOT depend on the prior. The prior is used ONLY as a final WIDE (~15m) sanity check on the winning pose — never to select seeds or count inliers.
- Parameters: `seed_tol` = the existing `tol` arg (localizer passes `constellation_tol`); `inlier_tol` = 0.5m (TIGHT); min inliers = 3; prior sanity = 15m (WIDE).
- `match` keeps its exact signature `match(observations, gated_landmarks, prior_xyz, tol, max_prior_dist=5.0)` and return type `[(Observation, MapLandmark)]` (drop-in).
- Reuse existing helpers: `_dist`, `_obs_pair_dists`, `_cat_pair_index`, `_seed_orientations` in constellation.py.
- Keep the tree feature, center-offset fix, cloud-odom sync fix, crop — all untouched.
- No ground truth for pose in code (project rule).

---

### Task 1: Shared 2D rigid-transform helper (break the import cycle)

**Files:**
- Create: `landmark_loc/geom.py`
- Modify: `landmark_loc/solve.py` (import from geom)
- Test: `landmark_loc/tests/test_geom.py`

**Interfaces:**
- Produces: `geom.rigid_transform_2d(src_xy, dst_xy) -> (tx, ty, yaw, rms)` — identical to the current `solve.rigid_transform_2d`.

**Why:** `constellation.py` needs `rigid_transform_2d` for seed transforms, but `solve.py` imports `constellation` (cycle if constellation imports solve). Move the function to a dependency-free `geom.py` that both import.

- [ ] **Step 1: Write the failing test**
```python
# landmark_loc/tests/test_geom.py
import math
from landmark_loc.geom import rigid_transform_2d


def test_identity_transform():
    src = [[0, 0], [1, 0], [0, 1]]
    tx, ty, yaw, rms = rigid_transform_2d(src, src)
    assert abs(tx) < 1e-9 and abs(ty) < 1e-9 and abs(yaw) < 1e-9 and rms < 1e-9


def test_pure_translation():
    src = [[0, 0], [1, 0], [0, 1]]
    dst = [[2, 3], [3, 3], [2, 4]]
    tx, ty, yaw, rms = rigid_transform_2d(src, dst)
    assert abs(tx - 2) < 1e-6 and abs(ty - 3) < 1e-6 and abs(yaw) < 1e-6 and rms < 1e-6


def test_rotation_90():
    src = [[1, 0], [0, 1], [-1, 0]]
    dst = [[0, 1], [-1, 0], [0, -1]]   # rotate +90deg about origin
    tx, ty, yaw, rms = rigid_transform_2d(src, dst)
    assert abs(yaw - math.pi / 2) < 1e-6 and rms < 1e-6
```

- [ ] **Step 2: Run — expect fail** (`geom` module missing)
Run: `cd ~/Documents/Husky_viz/.worktrees/constellation-matcher && PYTHONPATH=. python3 -m pytest landmark_loc/tests/test_geom.py -v` → FAIL (no module).

- [ ] **Step 3: Create `landmark_loc/geom.py`** by moving `rigid_transform_2d` verbatim from `solve.py`:
```python
"""Dependency-free 2D geometry: rigid (Kabsch/Umeyama) transform.

Shared by solve.py (final N-point pose refit) and constellation.py (seed
transforms). Kept import-free so both can use it without a cycle.
"""
import math
import numpy as np


def rigid_transform_2d(src_xy, dst_xy):
    src = np.asarray(src_xy, float)
    dst = np.asarray(dst_xy, float)
    cs, cd = src.mean(axis=0), dst.mean(axis=0)
    s0, d0 = src - cs, dst - cd
    H = s0.T @ d0
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:      # reflection guard
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    yaw = math.atan2(R[1, 0], R[0, 0])
    t = cd - R @ cs
    resid = (R @ s0.T).T + cd - dst
    rms = float(np.sqrt(np.mean(np.sum(resid ** 2, axis=1)))) if len(dst) else 0.0
    return float(t[0]), float(t[1]), yaw, rms
```

- [ ] **Step 4: Update `solve.py`** — remove its `rigid_transform_2d` definition, add `from landmark_loc.geom import rigid_transform_2d`. All existing callers keep working (same name).

- [ ] **Step 5: Run tests**
Run: `PYTHONPATH=. python3 -m pytest landmark_loc/tests/test_geom.py landmark_loc/tests/test_solve.py -v` → all pass (the known `test_launch` failure is unrelated and not run here).

- [ ] **Step 6: Commit**
```bash
git add landmark_loc/geom.py landmark_loc/solve.py landmark_loc/tests/test_geom.py
git commit -m "refactor(geom): extract rigid_transform_2d to shared module (no import cycle)"
```

---

### Task 2: RANSAC matcher core in constellation.py

**Files:**
- Modify: `landmark_loc/constellation.py`
- Test: `landmark_loc/tests/test_constellation.py` (rewrite for RANSAC behavior)

**Interfaces:**
- Consumes: `geom.rigid_transform_2d`; existing `_dist`, `_obs_pair_dists`, `_cat_pair_index`, `_seed_orientations`.
- Produces: `match(observations, gated_landmarks, prior_xyz, tol, max_prior_dist=5.0) -> [(Observation, MapLandmark)]` — same signature; now RANSAC.

**Context:** Replace the `_grow`/size-selection/`_prior_dist`-filter body of `match` with RANSAC. `tol` is the seed pairwise-distance tolerance (`seed_tol`). Add module constants `_INLIER_TOL = 0.5`, `_MIN_INLIERS = 3`, `_PRIOR_SANITY = 15.0`.

- [ ] **Step 1: Write the failing tests** (rewrite the RANSAC-relevant tests in `test_constellation.py`; keep any that test the reused helpers `_dist`/`_obs_pair_dists`/`_cat_pair_index` unchanged). Add:

```python
import math
from landmark_loc.constellation import match
from landmark_loc.classify import Observation
from landmark_loc.catalog import MapLandmark


def _obs(ident, x, y):
    return Observation(ident, x, y)


def _lm(name, ident, x, y):
    return MapLandmark(name, ident, x, y)


# A map-frame catalog constellation (3 distinct-type landmarks) and the robot
# observing them from a KNOWN true pose. Observations are the catalog points
# expressed in the robot frame at that true pose.
def _scene(true_x, true_y, true_yaw):
    cat = [_lm("lampA", "lamp", 10.0, 0.0),
           _lm("benchB", "bench", 13.0, 4.0),
           _lm("binC", "trash_bin_1", 8.0, 5.0)]
    c, s = math.cos(-true_yaw), math.sin(-true_yaw)
    obs = []
    for lm in cat:
        dx, dy = lm.x - true_x, lm.y - true_y
        obs.append(_obs(lm.identity, c * dx - s * dy, s * dx + c * dy))
    return obs, cat


def test_clean_match_at_correct_prior():
    obs, cat = _scene(0.0, 0.0, 0.0)
    pairs = match(obs, cat, (0.0, 0.0, 0.0), 1.0)
    names = sorted(lm.name for _, lm in pairs)
    assert names == ["benchB", "binC", "lampA"]


def test_drift_immunity_prior_4m_off():
    # THE key test: the prior is 4m off, but identification must still be correct
    # because RANSAC uses the seed shape, not the prior.
    obs, cat = _scene(0.0, 0.0, 0.0)
    pairs = match(obs, cat, (4.0, -3.0, 0.0), 1.0)   # prior 5m off
    names = sorted(lm.name for _, lm in pairs)
    assert names == ["benchB", "binC", "lampA"]


def test_drift_immunity_prior_10m_off():
    obs, cat = _scene(0.0, 0.0, 0.0)
    pairs = match(obs, cat, (10.0, 0.0, 0.0), 1.0)   # 10m off, still under 15m sanity
    names = sorted(lm.name for _, lm in pairs)
    assert names == ["benchB", "binC", "lampA"]


def test_dense_same_type_picks_max_inlier_arrangement():
    # Two lamps + a bench + a bin. A tie on the lamp alone would defeat _grow;
    # RANSAC's whole-set inliers resolve it.
    cat = [_lm("lamp1", "lamp", 10.0, 0.0),
           _lm("lamp2", "lamp", 12.5, 0.0),
           _lm("benchB", "bench", 13.0, 4.0),
           _lm("binC", "trash_bin_1", 8.0, 5.0)]
    # robot at origin observing all four honestly
    obs = [_obs(lm.identity, lm.x, lm.y) for lm in cat]  # true_pose=(0,0,0)
    pairs = match(obs, cat, (0.0, 0.0, 0.0), 1.0)
    assert len(pairs) >= 3
    # every observation maps to the correctly-positioned catalog landmark
    for o, lm in pairs:
        assert abs(o.x - lm.x) < 0.6 and abs(o.y - lm.y) < 0.6


def test_too_few_consistent_returns_empty():
    # Only 2 landmarks visible -> cannot reach 3 inliers -> []
    cat = [_lm("lampA", "lamp", 10.0, 0.0), _lm("benchB", "bench", 13.0, 4.0)]
    obs = [_obs(lm.identity, lm.x, lm.y) for lm in cat]
    assert match(obs, cat, (0.0, 0.0, 0.0), 1.0) == []


def test_absurd_pose_rejected_by_sanity():
    # A self-consistent 3-landmark scene whose implied robot pose is >15m from the
    # prior must be rejected.
    obs, cat = _scene(0.0, 0.0, 0.0)
    assert match(obs, cat, (100.0, 0.0, 0.0), 1.0) == []
```

- [ ] **Step 2: Run — expect the new tests to fail** (old `_grow` behavior differs, esp. drift-immunity at 4-10m which the old `max_prior_dist=5` filter rejected).
Run: `PYTHONPATH=. python3 -m pytest landmark_loc/tests/test_constellation.py -v`

- [ ] **Step 3: Implement RANSAC `match`.** Replace the body of `match` (the part after the helpers) with the RANSAC loop. Add at top of the RANSAC section:
```python
from landmark_loc.geom import rigid_transform_2d

_INLIER_TOL = 0.5       # TIGHT: a correct transform lands obs within this of catalog
_MIN_INLIERS = 3        # 3 non-collinear correspondences pin pose (no reflection flip)
_PRIOR_SANITY = 15.0    # WIDE final-only guard; tolerates ~4m drift with margin


def _score_transform(tx, ty, yaw, observations, gated):
    """Project every observation through (tx,ty,yaw); assign it to the nearest
    same-type catalog landmark within _INLIER_TOL, one-to-one (a landmark is taken
    by its closest observation). Returns list[(Observation, MapLandmark)]."""
    c, s = math.cos(yaw), math.sin(yaw)
    # candidate (obs_index, landmark, dist) within tol
    cand = []
    for k, o in enumerate(observations):
        mx = tx + c * o.x - s * o.y
        my = ty + s * o.x + c * o.y
        best, bd = None, _INLIER_TOL
        for lm in gated:
            if lm.identity != o.identity:
                continue
            d = math.hypot(lm.x - mx, lm.y - my)
            if d <= bd:
                best, bd = lm, d
        if best is not None:
            cand.append((k, best, bd))
    # one-to-one: each landmark kept by its closest observation
    best_for_lm = {}
    for k, lm, d in cand:
        key = id(lm)
        if key not in best_for_lm or d < best_for_lm[key][2]:
            best_for_lm[key] = (k, lm, d)
    return [(observations[k], lm) for (k, lm, d) in best_for_lm.values()]


def match(observations, gated_landmarks, prior_xyz, tol, max_prior_dist=5.0):
    if len(observations) < _MIN_INLIERS or len(gated_landmarks) < _MIN_INLIERS:
        return []
    obs_d = _obs_pair_dists(observations)
    cat_idx = _cat_pair_index(gated_landmarks)
    n = len(observations)
    best_inliers = []
    for i in range(n):
        for j in range(i + 1, n):
            oi, oj = observations[i], observations[j]
            key = frozenset((oi.identity, oj.identity))
            for (a, b, dab) in cat_idx.get(key, ()):
                if abs(dab - obs_d[(i, j)]) > tol:       # seed_tol on pair distance
                    continue
                for cat_i, cat_j in _seed_orientations(oi, oj, a, b):
                    tx, ty, yaw, _ = rigid_transform_2d(
                        [[oi.x, oi.y], [oj.x, oj.y]],
                        [[cat_i.x, cat_i.y], [cat_j.x, cat_j.y]])
                    inliers = _score_transform(tx, ty, yaw,
                                               observations, gated_landmarks)
                    if len(inliers) > len(best_inliers):
                        best_inliers = inliers
    if len(best_inliers) < _MIN_INLIERS:
        return []
    # WIDE prior sanity on the refit pose
    src = [[o.x, o.y] for o, _ in best_inliers]
    dst = [[lm.x, lm.y] for _, lm in best_inliers]
    px, py, _, _ = rigid_transform_2d(src, dst)
    if math.hypot(px - prior_xyz[0], py - prior_xyz[1]) > _PRIOR_SANITY:
        return []
    return best_inliers
```
Remove `_grow`, `_centroid`, `_prior_dist` (now unused). Update the module docstring to describe RANSAC.

- [ ] **Step 4: Run tests — expect pass**
Run: `PYTHONPATH=. python3 -m pytest landmark_loc/tests/test_constellation.py -v` → all pass.

- [ ] **Step 5: Commit**
```bash
git add landmark_loc/constellation.py landmark_loc/tests/test_constellation.py
git commit -m "feat(constellation): RANSAC whole-set matcher (drift-invariant; replaces _grow)"
```

---

### Task 3: Point solve_pose back at constellation.match

**Files:**
- Modify: `landmark_loc/solve.py`
- Modify: `landmark_loc/localizer_node.py`
- Test: `landmark_loc/tests/test_solve.py`, `test_association_duplicate.py`

**Interfaces:**
- `solve_pose(observations, gated, prior, dist_gate, residual_gate, max_prior_dist=5.0)` — unchanged signature; internally calls `constellation.match` (RANSAC) instead of the NN `associate`+dedup experiment.

**Context:** The NN experiment (commits e1a8427/d7cd58f) made `solve_pose` use `associate` + `_dedupe_one_to_one`. Revert that so `solve_pose` calls `constellation.match`. Keep `associate` and `rigid_transform_2d` (imported from geom) for legacy/tests.

- [ ] **Step 1: Write/adjust the failing test** — `test_solve.py`: `solve_pose` with a 3-landmark drift-4m scene returns a fix with the correct pose (drift-immunity end-to-end). Reuse the `_scene` pattern from Task 2.
```python
def test_solve_pose_recovers_under_4m_drift():
    from landmark_loc.solve import solve_pose
    # 3 distinct-type landmarks, robot truly at origin, prior 4m off
    from landmark_loc.classify import Observation
    from landmark_loc.catalog import MapLandmark
    import math
    cat = [MapLandmark("lampA", "lamp", 10.0, 0.0),
           MapLandmark("benchB", "bench", 13.0, 4.0),
           MapLandmark("binC", "trash_bin_1", 8.0, 5.0)]
    obs = [Observation(lm.identity, lm.x, lm.y) for lm in cat]  # true pose (0,0,0)
    out = solve_pose(obs, cat, (4.0, -3.0, 0.0), 1.0, 1.0)
    assert out is not None
    x, y, yaw, rms, n = out
    assert abs(x) < 0.5 and abs(y) < 0.5 and n >= 3 and rms < 0.5
```

- [ ] **Step 2: Run — expect fail** (NN `associate` under a 4m-off prior misassociates → None or wrong).
Run: `PYTHONPATH=. python3 -m pytest landmark_loc/tests/test_solve.py -v`

- [ ] **Step 3: Revert solve_pose to constellation.match:**
```python
def solve_pose(observations, gated_landmarks, prior_xyz, dist_gate, residual_gate,
                max_prior_dist=5.0):
    pairs = constellation.match(observations, gated_landmarks, prior_xyz, dist_gate,
                                 max_prior_dist)
    if len(pairs) < 3:
        return None
    src = np.array([[o.x, o.y] for o, _ in pairs])
    dst = np.array([[lm.x, lm.y] for _, lm in pairs])
    x, y, yaw, rms = rigid_transform_2d(src, dst)
    if rms > residual_gate:
        return None
    return (x, y, yaw, rms, len(pairs))
```
(`dist_gate` is now the RANSAC `seed_tol`.) `_dedupe_one_to_one` is no longer used by solve_pose — remove it or leave it unused; `associate` stays for the characterization tests. Update `test_association_duplicate.py` so its solve_pose-level assertions reflect RANSAC one-to-one (a duplicate cannot pass; the matcher is one-to-one by construction).

- [ ] **Step 4: localizer_node** — the diag `_pairs` line and the `solve_pose` call should pass `constellation_tol` as the seed_tol (dist_gate slot). Set the diag line back to `_pairs = solve.constellation.match(obs, gated, prior, p["constellation_tol"], p["max_prior_dist"])` and the solve_pose call to `solve.solve_pose(obs, gated, prior, p["constellation_tol"], p["residual_gate"], p["max_prior_dist"])`. The `dist_gate=6.0` param is now unused by matching — leave the declaration (harmless) or remove; keep the change minimal. Keep center-offset, sync, crop, trees.

- [ ] **Step 5: Run full suite**
Run: `PYTHONPATH=. python3 -m pytest landmark_loc/tests/ -v` → all pass except the known unrelated `test_launch.py::test_runbook_offers_both_modes`.

- [ ] **Step 6: Commit**
```bash
git add landmark_loc/solve.py landmark_loc/localizer_node.py landmark_loc/tests/test_solve.py landmark_loc/tests/test_association_duplicate.py
git commit -m "feat(solve): route solve_pose through RANSAC constellation.match (revert NN experiment)"
```

---

### Task 4: In-sim acceptance (RUN BY MAIN, not a subagent)

**Files:** none (verification only).

- [ ] **Step 1:** Bring the sim up clean per RUN-MAP-NAV.md Steps 0-3 (localizer at the worktree, `_places_path` absolute worktree yaml, `python3 -u`).
- [ ] **Step 2:** Full demo — GPS goal → spoof → `mode landmark`. Watch the localizer diag.
- [ ] **Step 3:** Success criteria (judge by GAZEBO, not move_base SUCCEEDED/dist): the robot's actual position reaches the goal marker; the diag shows fixes staying alive during the drive/turns (no long assoc=0/STALE stretches on the drift spikes) and matched landmarks tracking the robot's region (not frozen at spawn).
- [ ] **Step 4:** Tear down clean (kill by exact PID, verify master down by reading).
- [ ] **Step 5:** Record the result (Gazebo-judged) in the SDD ledger.

---

## Self-Review

**Spec coverage:** RANSAC core (Task 2), drift-invariance tests (Task 2), solve_pose reroute + NN revert (Task 3), import-cycle fix via geom (Task 1), in-sim acceptance judged by Gazebo (Task 4). All covered.

**Placeholder scan:** all steps carry real code and concrete constants (inlier_tol 0.5, min 3, sanity 15, seed_tol via tol). No TBDs.

**Type consistency:** `match` signature/return unchanged; `rigid_transform_2d` moved to geom, same signature, imported by both solve and constellation; `solve_pose` unchanged signature. `_score_transform` returns `[(Observation, MapLandmark)]` matching `match`'s contract.

**Known conflict flagged for pre-flight:** Task 2 and Task 3 REWRITE existing tests (`test_constellation.py` for RANSAC behavior; `test_association_duplicate.py` for RANSAC one-to-one) — intended, since the matcher's behavior changes (drift-immune, one-to-one by construction). Not lost coverage; the new tests assert the stronger drift-immunity property.
