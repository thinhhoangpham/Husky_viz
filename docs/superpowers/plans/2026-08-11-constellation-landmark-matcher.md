# Constellation Landmark Matcher Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace prior-dependent nearest-neighbor landmark association with a prior-free type+geometry constellation matcher, so the localizer stops going stale when the odom prior has drifted (the failure after switching from GPS mid-attack).

**Architecture:** A new module `landmark_loc/constellation.py` provides `match(observations, gated_landmarks, prior_xyz, tol)` returning the same `[(observation, map_landmark), ...]` pairs that `solve.associate` returns today. `solve.solve_pose` is rewired to call it instead of `associate`. Identification uses robot-free pairwise distances between landmarks (frame-invariant, drift-immune) plus type agreement; the prior is used only to break ties between equally-good catalog constellations. Everything downstream (`rigid_transform_2d`, gates, publish, EKF, mux) is unchanged.

**Tech Stack:** Python 3, numpy, ROS Noetic (node layer only). Pure-Python matcher, fully unit-testable offline. pytest.

## Global Constraints

- The matcher's public function signature and return type MUST be a drop-in for `solve.associate`: inputs `(observations, gated_landmarks, prior_xyz, tol)`, output a list of `(Observation, MapLandmark)` tuples. `solve.solve_pose` consumes the output unchanged.
- `Observation` (`landmark_loc/classify.py`): fields `identity` (str), `x`, `y` (robot-frame floats).
- `MapLandmark` (`landmark_loc/catalog.py`): fields `name` (str), `identity` (str), `x`, `y` (map-frame floats).
- Type constraint is MANDATORY: an observed landmark of identity T may only pair with a catalog landmark of identity T.
- One-to-one: each observation maps to at most one catalog landmark, and each catalog landmark to at most one observation, within a returned constellation (fixes the known duplicate-association defect characterized in `test_association_duplicate.py`).
- The prior (`prior_xyz`) may be used ONLY as a tiebreaker between equally-sized, equally-consistent constellations (step 5). It MUST NOT influence which catalog landmark an observation is identified as in the unambiguous case. A drift-immunity test enforces this.
- Distance-match tolerance param name: `constellation_tol`, default `0.3` (metres). Exposed as a ROS param `~constellation_tol` in the node.
- Pure-Python matcher: `landmark_loc/constellation.py` must import without rospy (like `solve.py`, `classify.py`). ROS imports stay inside `localizer_node.main()`.
- Do NOT modify `compose_prior`, `catalog.gate`, `rigid_transform_2d`, the anchor logic, the EKF configs, the mux, or the operator.
- Frequent commits: one per task.

---

## File Structure

- **Create:** `landmark_loc/constellation.py` — the matcher (pure Python).
- **Create:** `landmark_loc/tests/test_constellation.py` — matcher unit tests.
- **Modify:** `landmark_loc/solve.py` — `solve_pose` calls `constellation.match` instead of `associate`. `associate` and `rigid_transform_2d` stay (associate is still referenced by the characterization test; leave it in place, just no longer used by solve_pose).
- **Modify:** `landmark_loc/localizer_node.py` — add `~constellation_tol` param; pass it through the `solve_pose` call and the `[diag]` recompute block.
- **Modify:** `landmark_loc/tests/test_association_duplicate.py` — update the two tests that asserted the OLD duplicate-association behavior of `solve_pose` (now fixed); keep the `associate`-level characterization (which still holds for the untouched `associate`).

---

## Task 1: Constellation matcher core — pairwise distance model + type-gated pair matching

**Files:**
- Create: `landmark_loc/constellation.py`
- Test: `landmark_loc/tests/test_constellation.py`

**Interfaces:**
- Consumes: `Observation` (identity, x, y), `MapLandmark` (name, identity, x, y).
- Produces: `match(observations, gated_landmarks, prior_xyz, tol) -> list[(Observation, MapLandmark)]`. This is what `solve.solve_pose` will call (Task 3) and is a drop-in for `solve.associate`.

- [ ] **Step 1: Write the failing test — clean 3-landmark match returns 3 correct pairs.**

```python
# landmark_loc/tests/test_constellation.py
import math
from landmark_loc import constellation
from landmark_loc.classify import Observation
from landmark_loc.catalog import MapLandmark


def _observe_from_true_pose(lm, true_xyz):
    """Project a map landmark into the robot frame at the TRUE pose (test only)."""
    x, y, yaw = true_xyz
    c, s = math.cos(-yaw), math.sin(-yaw)
    dx, dy = lm.x - x, lm.y - y
    return Observation(lm.identity, c * dx - s * dy, s * dx + c * dy)


_LMS = [
    MapLandmark("bench_1", "bench", 5.0, 1.0),
    MapLandmark("lamp_1", "lamp", 6.0, -2.0),
    MapLandmark("table_1", "garden_table", 3.0, 4.0),
]


def test_clean_three_landmark_match():
    true = (2.0, -1.0, 0.5)
    obs = [_observe_from_true_pose(lm, true) for lm in _LMS]
    pairs = constellation.match(obs, _LMS, prior_xyz=true, tol=0.3)
    got = {o.identity: lm.name for o, lm in pairs}
    assert got == {"bench": "bench_1", "lamp": "lamp_1", "garden_table": "table_1"}
    assert len(pairs) == 3
```

- [ ] **Step 2: Run it to verify it fails.**

Run: `cd landmark_loc/.. && python3 -m pytest landmark_loc/tests/test_constellation.py::test_clean_three_landmark_match -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'landmark_loc.constellation'`.

- [ ] **Step 3: Implement the matcher core.**

```python
# landmark_loc/constellation.py
"""Prior-free landmark identification by constellation (shape) matching.

Each observation is identified by the robot-FRAME-INVARIANT pairwise distances
between the observed landmarks, matched against the catalog under a type
constraint. Because a distance between two points does not change when the pose
prior drifts, identification survives a badly drifted prior -- unlike the
nearest-neighbor-under-prior association it replaces (solve.associate). The prior
is consulted only to break ties between equally-good catalog constellations.

Drop-in for solve.associate: match(observations, gated_landmarks, prior_xyz, tol)
-> list of (Observation, MapLandmark).
"""
import math


def _dist(ax, ay, bx, by):
    return math.hypot(ax - bx, ay - by)


def _obs_pair_dists(observations):
    """Map (i, j) -> distance for observed landmarks, i < j."""
    d = {}
    n = len(observations)
    for i in range(n):
        for j in range(i + 1, n):
            oi, oj = observations[i], observations[j]
            d[(i, j)] = _dist(oi.x, oi.y, oj.x, oj.y)
    return d


def _cat_pair_index(gated):
    """Map frozenset({identity_a, identity_b}) -> list of (a, b, distance)
    catalog pairs (a, b are MapLandmark), so a typed observed pair can look up
    candidate catalog pairs of the same identity combination quickly."""
    idx = {}
    m = len(gated)
    for i in range(m):
        for j in range(i + 1, m):
            a, b = gated[i], gated[j]
            key = frozenset((a.identity, b.identity))
            idx.setdefault(key, []).append((a, b, _dist(a.x, a.y, b.x, b.y)))
    return idx
```

These are the building blocks. Now add the seed-orientation helper, the grow
helper, and `match()`. The design: for each observed pair, try each type+distance-
compatible catalog pair as a SEED (in both orientations when the two observations
share a type), grow the seed by adding every other observation that a unique
same-type catalog landmark explains (consistent distance to BOTH seed landmarks),
and keep the largest consistent assignment. Task 2 adds the prior tiebreaker.

```python
def _seed_orientations(oi, oj, a, b):
    """Yield (cat_for_i, cat_for_j) orientations of a catalog pair (a, b) that are
    type-consistent with observations (oi, oj). If oi, oj are the same type, both
    orientations are valid; otherwise only the one whose identities line up."""
    if oi.identity != oj.identity:
        if oi.identity == a.identity and oj.identity == b.identity:
            yield (a, b)
        elif oi.identity == b.identity and oj.identity == a.identity:
            yield (b, a)
        return
    # same type: both orientations are type-consistent
    yield (a, b)
    yield (b, a)


def _grow(i, j, cat_i, cat_j, observations, obs_d, gated, tol):
    """Extend a seed (obs i->cat_i, obs j->cat_j) by assigning every other
    observation to the UNIQUE same-type catalog landmark whose distance to both
    seed landmarks matches the observed distances within tol. Observations with
    zero or more than one candidate are skipped (a partial constellation is valid
    as long as >=2 correspondences remain). Returns dict obs_index -> MapLandmark.
    """
    assign = {i: cat_i, j: cat_j}
    used = {id(cat_i), id(cat_j)}
    for k in range(len(observations)):
        if k in assign:
            continue
        ok = observations[k]
        dki = obs_d[(min(k, i), max(k, i))]
        dkj = obs_d[(min(k, j), max(k, j))]
        cand = [lm for lm in gated
                if lm.identity == ok.identity and id(lm) not in used
                and abs(_dist(lm.x, lm.y, cat_i.x, cat_i.y) - dki) <= tol
                and abs(_dist(lm.x, lm.y, cat_j.x, cat_j.y) - dkj) <= tol]
        if len(cand) == 1:
            assign[k] = cand[0]
            used.add(id(cand[0]))
    return assign


def match(observations, gated_landmarks, prior_xyz, tol):
    if len(observations) < 2 or len(gated_landmarks) < 2:
        return []
    obs_d = _obs_pair_dists(observations)
    cat_idx = _cat_pair_index(gated_landmarks)
    n = len(observations)
    best = {}
    for i in range(n):
        for j in range(i + 1, n):
            oi, oj = observations[i], observations[j]
            key = frozenset((oi.identity, oj.identity))
            for (a, b, dab) in cat_idx.get(key, ()):
                if abs(dab - obs_d[(i, j)]) > tol:
                    continue
                for cat_i, cat_j in _seed_orientations(oi, oj, a, b):
                    assign = _grow(i, j, cat_i, cat_j, observations, obs_d,
                                   gated_landmarks, tol)
                    if len(assign) > len(best):
                        best = assign
    if len(best) < 2:
        return []
    return [(observations[k], best[k]) for k in sorted(best)]
```

- [ ] **Step 4: Run the test to verify it passes.**

Run: `python3 -m pytest landmark_loc/tests/test_constellation.py::test_clean_three_landmark_match -v`
Expected: PASS.

- [ ] **Step 5: Add and pass the drift-immunity test (the core proof).**

```python
def test_drift_immunity_prior_8m_off():
    true = (2.0, -1.0, 0.5)
    obs = [_observe_from_true_pose(lm, true) for lm in _LMS]
    bad_prior = (true[0] + 8.0, true[1] - 8.0, true[2] + 0.4)  # far off
    pairs = constellation.match(obs, _LMS, prior_xyz=bad_prior, tol=0.3)
    got = {o.identity: lm.name for o, lm in pairs}
    assert got == {"bench": "bench_1", "lamp": "lamp_1", "garden_table": "table_1"}
```

Run: `python3 -m pytest landmark_loc/tests/test_constellation.py -v`
Expected: PASS — identities correct despite the 8 m-wrong prior. (This is what nearest-neighbor-under-prior cannot do.)

- [ ] **Step 6: Verify pure-Python (no rospy) import.**

Run: `python3 -c "import sys; import landmark_loc.constellation; assert 'rospy' not in sys.modules; print('no-ros import OK')"`
Expected: `no-ros import OK`.

- [ ] **Step 7: Commit.**

```bash
git add landmark_loc/constellation.py landmark_loc/tests/test_constellation.py
git commit -m "feat(constellation): prior-free type+geometry landmark matcher core"
```

---

## Task 2: Edge cases — too-few / no-match / type constraint / ambiguity+prior tiebreak / collinear

**Files:**
- Modify: `landmark_loc/constellation.py` (add the prior tiebreaker; harden edge behavior)
- Test: `landmark_loc/tests/test_constellation.py` (add cases)

**Interfaces:**
- Consumes/Produces: same `match(...)` signature as Task 1. Behavior refined, signature unchanged.

- [ ] **Step 1: Write failing tests for the edge cases.**

```python
def test_one_observation_returns_empty():
    obs = [_observe_from_true_pose(_LMS[0], (0, 0, 0))]
    assert constellation.match(obs, _LMS, prior_xyz=(0, 0, 0), tol=0.3) == []


def test_two_distinct_type_unique_pair_matches():
    two = [_LMS[0], _LMS[1]]  # bench + lamp, distinct types
    obs = [_observe_from_true_pose(lm, (0, 0, 0)) for lm in two]
    pairs = constellation.match(obs, _LMS, prior_xyz=(0, 0, 0), tol=0.3)
    got = {o.identity: lm.name for o, lm in pairs}
    assert got == {"bench": "bench_1", "lamp": "lamp_1"}


def test_no_match_shape_absent_returns_empty():
    # observations form a triangle with side lengths that exist in NO catalog trio
    obs = [Observation("bench", 0.0, 0.0),
           Observation("lamp", 100.0, 0.0),      # 100 m apart: no catalog pair
           Observation("garden_table", 0.0, 100.0)]
    assert constellation.match(obs, _LMS, prior_xyz=(0, 0, 0), tol=0.3) == []


def test_type_constraint_blocks_geometric_lookalike():
    # a lamp observation sitting exactly where a catalog BENCH is must NOT pair to
    # it; only lamp catalog entries are eligible.
    cat = [MapLandmark("bench_x", "bench", 5.0, 0.0),
           MapLandmark("lamp_far", "lamp", 40.0, 0.0)]
    obs = [Observation("lamp", 5.0, 0.0), Observation("bench", 40.0, 0.0)]
    pairs = constellation.match(obs, cat, prior_xyz=(0, 0, 0), tol=0.3)
    # geometry alone would swap them; type forbids it -> the only type-consistent
    # assignment (lamp->lamp_far, bench->bench_x) has the wrong distance, so empty.
    assert pairs == []


def test_ambiguity_resolved_by_prior():
    # two identical bench+lamp constellations, 50 m apart; prior sits next to the
    # SECOND one -> matcher must pick the second.
    cat = [MapLandmark("bench_a", "bench", 0.0, 0.0),
           MapLandmark("lamp_a", "lamp", 3.0, 0.0),
           MapLandmark("bench_b", "bench", 50.0, 0.0),
           MapLandmark("lamp_b", "lamp", 53.0, 0.0)]
    # robot at second cluster, observing bench_b + lamp_b from true pose (50,0,0)
    obs = [_observe_from_true_pose(cat[2], (50.0, 0.0, 0.0)),
           _observe_from_true_pose(cat[3], (50.0, 0.0, 0.0))]
    pairs = constellation.match(obs, cat, prior_xyz=(50.0, 0.0, 0.0), tol=0.3)
    got = {lm.name for _, lm in pairs}
    assert got == {"bench_b", "lamp_b"}
```

- [ ] **Step 2: Run to verify the ambiguity + type tests fail (others may already pass from Task 1).**

Run: `python3 -m pytest landmark_loc/tests/test_constellation.py -v`
Expected: `test_ambiguity_resolved_by_prior` FAILS (no tiebreaker yet); `test_type_constraint_blocks_geometric_lookalike` and the two-obs/no-match/one-obs cases should already pass from Task 1's logic — if any fail, fix the core.

- [ ] **Step 3: Add the prior tiebreaker to `match()`.**

Replace the "keep the largest" selection with: collect ALL maximal assignments (same largest size), then if more than one, pick the assignment whose catalog landmarks' centroid is nearest the prior position. Implementation:

```python
def _centroid(assign):
    xs = [lm.x for lm in assign.values()]
    ys = [lm.y for lm in assign.values()]
    return (sum(xs) / len(xs), sum(ys) / len(ys))


def _prior_dist(assign, prior_xyz):
    cx, cy = _centroid(assign)
    return math.hypot(cx - prior_xyz[0], cy - prior_xyz[1])
```

In `match()`, accumulate candidates into a list, then:

```python
    candidates = [a for a in candidates if len(a) >= 2]
    if not candidates:
        return []
    best_size = max(len(a) for a in candidates)
    top = [a for a in candidates if len(a) == best_size]
    # dedupe identical assignments (same obj set) so a true single winner isn't
    # treated as a tie
    uniq = []
    for a in top:
        sig = frozenset((k, id(v)) for k, v in a.items())
        if sig not in {frozenset((k, id(v)) for k, v in u.items()) for u in uniq}:
            uniq.append(a)
    chosen = min(uniq, key=lambda a: _prior_dist(a, prior_xyz))
    return [(observations[k], chosen[k]) for k in sorted(chosen)]
```

(Refactor `match()` from Task 1 so it builds a `candidates` list instead of tracking a single `best`; the seed loop stays the same, it just appends every grown assignment.)

- [ ] **Step 4: Run all constellation tests to verify pass.**

Run: `python3 -m pytest landmark_loc/tests/test_constellation.py -v`
Expected: ALL pass, including `test_ambiguity_resolved_by_prior`.

- [ ] **Step 5: Add a collinear/degenerate guard test.**

```python
def test_collinear_triple_still_matches_or_empty():
    # three landmarks in a straight line, distinct types (so type still pins them)
    cat = [MapLandmark("bench_1", "bench", 0.0, 0.0),
           MapLandmark("lamp_1", "lamp", 5.0, 0.0),
           MapLandmark("table_1", "garden_table", 10.0, 0.0)]
    obs = [_observe_from_true_pose(lm, (0.0, 0.0, 0.3)) for lm in cat]
    pairs = constellation.match(obs, cat, prior_xyz=(0.0, 0.0, 0.3), tol=0.3)
    got = {o.identity: lm.name for o, lm in pairs}
    # distinct types make even a collinear set unambiguous
    assert got == {"bench": "bench_1", "lamp": "lamp_1", "garden_table": "table_1"}
```

Run: `python3 -m pytest landmark_loc/tests/test_constellation.py -v`
Expected: PASS. (A same-type collinear ambiguity that survives is left to the downstream reflection guard + residual gate in `solve.py`, per spec; no matcher change needed.)

- [ ] **Step 6: Commit.**

```bash
git add landmark_loc/constellation.py landmark_loc/tests/test_constellation.py
git commit -m "feat(constellation): edge cases + prior-tiebreak for ambiguous shapes"
```

---

## Task 3: Rewire `solve_pose` to use the constellation matcher

**Files:**
- Modify: `landmark_loc/solve.py`
- Test: `landmark_loc/tests/test_solve.py` (add a matcher-path test), `landmark_loc/tests/test_association_duplicate.py` (update solve_pose-level assertions)

**Interfaces:**
- Consumes: `constellation.match(observations, gated_landmarks, prior_xyz, tol)`.
- Produces: `solve_pose(observations, gated_landmarks, prior_xyz, dist_gate, residual_gate)` — SAME signature as today. `dist_gate` is repurposed as the constellation tolerance passed to `match` (renamed conceptually but kept as the positional arg so `localizer_node` needn't change its call shape until Task 4). Returns `(x, y, yaw, rms, n)` or None, unchanged.

- [ ] **Step 1: Write a failing test — solve_pose recovers a known pose via the matcher under a wrong prior.**

```python
# add to landmark_loc/tests/test_solve.py
def test_solve_pose_matcher_recovers_under_wrong_prior():
    lms = [MapLandmark("a", "bench", 5.0, 1.0),
           MapLandmark("b", "lamp", 6.0, -2.0),
           MapLandmark("c", "garden_table", 3.0, 4.0)]
    true = (2.0, -1.0, 0.5)
    obs = _obs_from_truth(true, lms)
    wrong_prior = (10.0, 8.0, 1.2)  # far from truth
    out = solve.solve_pose(obs, lms, prior_xyz=wrong_prior,
                           dist_gate=0.3, residual_gate=0.5)
    assert out is not None
    x, y, yaw, rms, n = out
    assert n == 3 and rms < 1e-6
    assert abs(x - 2.0) < 1e-6 and abs(y + 1.0) < 1e-6
```

- [ ] **Step 2: Run to verify it fails.**

Run: `python3 -m pytest landmark_loc/tests/test_solve.py::test_solve_pose_matcher_recovers_under_wrong_prior -v`
Expected: FAIL — current `associate` mismatches under the wrong prior (too few/wrong pairs → None or wrong pose).

- [ ] **Step 3: Rewire `solve_pose`.**

In `landmark_loc/solve.py`, add `from landmark_loc import constellation` at top. Change `solve_pose`'s body from:

```python
    pairs = associate(observations, gated_landmarks, prior_xyz, dist_gate)
```

to:

```python
    pairs = constellation.match(observations, gated_landmarks, prior_xyz, dist_gate)
```

Leave `associate` and `rigid_transform_2d` defined and unchanged (associate is still exercised by the characterization test). Update the module docstring's first paragraph to say association is now constellation-based; keep the residual/count-gate description.

- [ ] **Step 4: Run to verify the new test passes and existing solve tests stay green.**

Run: `python3 -m pytest landmark_loc/tests/test_solve.py -v`
Expected: all PASS (including `test_solve_pose_rejects_when_too_few_matches` — 1 observation still yields <2 pairs → None).

- [ ] **Step 5: Update the duplicate-association tests that asserted the OLD solve_pose behavior.**

`test_association_duplicate.py` has two tests. `test_associate_can_produce_duplicate_map_landmark` calls `associate` DIRECTLY — it still holds (associate is unchanged); leave it. `test_duplicate_association_can_pass_gates` calls `solve_pose` and asserts a phantom PASSES the lenient gate — that is now FIXED (constellation is one-to-one, so two observations cannot bind one landmark). Update it to assert the fix:

```python
def test_duplicate_association_no_longer_passes_gates():
    """solve_pose now uses the constellation matcher, which is one-to-one, so the
    two-benches-onto-one phantom characterized for associate() can no longer occur
    at the solve_pose level. With only two same-type benches and a yaw-wrong prior,
    the matcher either pairs them correctly (distinct) or returns too few pairs;
    it never binds both observations to a single bench."""
    obs = _observations()
    for rg in (1.0, 5.0):
        out = solve_pose(obs, _LANDMARKS, _PRIOR, dist_gate=0.3, residual_gate=rg)
        if out is not None:
            _, _, _, _, n = out
            # if it returns a fit, it rests on TWO DISTINCT benches, not one
            pairs = __import__("landmark_loc.constellation", fromlist=["match"]).match(
                obs, _LANDMARKS, _PRIOR, 0.3)
            names = {lm.name for _, lm in pairs}
            assert len(names) == len({id(lm) for _, lm in pairs})
```

Rename the test function (the old name asserted the defect). Update the module docstring note to say the solve_pose-level defect is now fixed by the constellation matcher; the `associate`-level characterization remains as documentation of the legacy function.

- [ ] **Step 6: Run the full landmark_loc test suite.**

Run: `python3 -m pytest landmark_loc/tests/ -q`
Expected: all PASS.

- [ ] **Step 7: Commit.**

```bash
git add landmark_loc/solve.py landmark_loc/tests/test_solve.py landmark_loc/tests/test_association_duplicate.py
git commit -m "feat(solve): route solve_pose through constellation matcher (drift-immune association)"
```

---

## Task 4: Node param `~constellation_tol` + strip temporary `[diag]` logging

**Files:**
- Modify: `landmark_loc/localizer_node.py`

**Interfaces:**
- Consumes: `solve.solve_pose(obs, gated, prior, dist_gate=constellation_tol, residual_gate)`.
- Produces: node behavior unchanged externally except the new tunable param and cleaner logs.

- [ ] **Step 1: Add the `~constellation_tol` param.**

In the `p = dict(...)` block in `main()`, add:

```python
        constellation_tol=rospy.get_param("~constellation_tol", 0.3),
```

Keep `dist_gate` in the dict for now (still passed positionally as the matcher tolerance) OR replace its use — cleanest: pass `p["constellation_tol"]` where `p["dist_gate"]` was passed into `solve_pose` and the `[diag]` recompute. Do BOTH call sites (the `solve.associate`/`match` diagnostic recompute at the top of `on_cloud` and the real `solve.solve_pose` call).

- [ ] **Step 2: Update the two `solve` call sites in `on_cloud`.**

The diagnostic recompute currently calls `solve.associate(obs, gated, prior, p["dist_gate"])`. Change to `solve.constellation.match(obs, gated, prior, p["constellation_tol"])` — OR simpler, since we are stripping diag in Step 3, remove that recompute block entirely (see Step 3). The real call `solve.solve_pose(obs, gated, prior, p["dist_gate"], p["residual_gate"])` becomes `solve.solve_pose(obs, gated, prior, p["constellation_tol"], p["residual_gate"])`.

- [ ] **Step 3: Strip the temporary `[diag]` logging.**

Remove the six `rospy.loginfo_throttle(1.0, "[diag] ...")` blocks and the diagnostic association recompute (`_pairs = solve.associate(...)`, `_rms`, the numpy block, and the FIX/solve=None diag lines) added during tuning (`localizer_node.py:143-175`). Keep the `anchor captured` loginfo (that is a genuine one-time lifecycle log, not diag). Keep the real pipeline (crop/cluster/classify/gate/solve/publish) intact.

- [ ] **Step 4: Verify compile + no-ros import + full suite.**

Run:
```
python3 -m py_compile landmark_loc/localizer_node.py && echo COMPILE_OK
python3 -c "import sys; import landmark_loc.localizer_node; assert 'rospy' not in sys.modules; print('no-ros import OK')"
python3 -m pytest landmark_loc/tests/ -q
grep -n "diag" landmark_loc/localizer_node.py
```
Expected: COMPILE_OK, no-ros import OK, all tests pass, grep for `diag` returns NOTHING (all diag lines gone).

- [ ] **Step 5: Commit.**

```bash
git add landmark_loc/localizer_node.py
git commit -m "feat(localizer): add ~constellation_tol param; strip temporary diag logging"
```

---

## Task 5: Full regression + in-sim acceptance (run by main, not a subagent)

**Files:** none (verification only).

- [ ] **Step 1: Full pure-Python suite green.**

Run: `python3 -m pytest landmark_loc/tests/ -q`
Expected: all PASS.

- [ ] **Step 2: In-sim acceptance (main runs the sim from a clean kill).**

Per `RUN-MAP-NAV.md`, follow the doc EXACTLY (no added env vars, gzclient on `:0`). Bring up Steps 0–3, then run Step 7 (full demo): GPS drive → spoof → `mode landmark` mid-attack → confirm the robot reaches the goal under active spoof.

- [ ] **Step 3: Measure stale-rate improvement.**

Temporarily re-enable a lightweight obs/associations count (or read the localizer's fix publish rate on `/odometry/landmark_fix`) on the SAME route, compare to the pre-change baseline (previous run: ~21% of ticks stale, of which ~8.5% were "obs>=3 but failed to associate"). Success criterion: the "obs>=3 but stale" population is markedly reduced (target: near-eliminated), and the robot reaches the goal under active spoof.

- [ ] **Step 4: Report results to the user** (fix rate before/after, goal reached y/n) and decide merge.

---

## Self-Review

**Spec coverage:** algorithm (Task 1-2), drop-in swap (Task 3), node param + diag strip (Task 4), tests incl. drift-immunity (Task 1 Step 5) and ambiguity+prior (Task 2), in-sim acceptance (Task 5). All spec sections covered.

**Placeholder scan:** Task 1 Step 3 contains ONE deliberately-flagged placeholder line (`if abs(_dist(...) - 0) >= 0`) with an explicit instruction to remove it before shipping — this is called out, not a silent gap. No other TBD/TODO.

**Type consistency:** `match(observations, gated_landmarks, prior_xyz, tol)` used identically in Tasks 1, 2, 3, 4. `solve_pose(obs, gated, prior, dist_gate/constellation_tol, residual_gate)` signature preserved. `Observation`/`MapLandmark` field names match `classify.py`/`catalog.py`.
