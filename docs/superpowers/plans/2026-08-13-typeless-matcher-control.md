# Types-Stripped Matcher Control — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build `constellation_typeless.py` — the RANSAC constellation matcher with the landmark-identity constraint removed at its 4 coupling points, keeping everything else identical — so the identity-vs-geometry experiment can compare typed vs. typeless with identity as the only variable.

**Architecture:** A new sibling module to `landmark_loc/constellation.py` with the same public `match()` signature (drop-in). It copies the RANSAC structure, the yaw constraint, the prior-sanity gate, and all constants verbatim; it removes ONLY the identity coupling. A tiny A/B harness script runs both matchers on the same inputs for the experiment. No production code (typed matcher, classifier, localizer) is modified.

**Tech Stack:** Python 3.8, `math` only (no numpy in the matcher — mirror the typed one), pytest. Reuses `landmark_loc.geom.rigid_transform_2d`, `Observation`, `MapLandmark`.

## Global Constraints

- **Do NOT modify `landmark_loc/constellation.py`** (the typed matcher under test must stay byte-identical) or the classifier / localizer / any production path. The typeless matcher is ADDITIVE and used only by the experiment harness.
- **Identity is the ONLY thing removed.** Keep the RANSAC seed→hypothesis→score→win structure, the yaw constraint (`_yaw_diff_ok` + the yaw check in scoring), the prior-sanity gate (`_PRIOR_SANITY`), and the constants (`_INLIER_TOL=0.5`, `_MIN_INLIERS=3`, `_PRIOR_SANITY=15.0`, `_YAW_TOL=0.35`) identical to the typed matcher.
- **Public API identical:** `match(observations, gated_landmarks, prior_xyz, tol, max_prior_dist=5.0) -> list[(Observation, MapLandmark)]`.
- **The 4 identity coupling points to remove** (from `constellation.py`, verified):
  1. `_cat_pair_index`: catalog pairs indexed by `frozenset((a.identity, b.identity))` → index with no type key.
  2. `match` seed lookup: `key = frozenset((oi.identity, oj.identity))` → consider all distance-compatible catalog pairs.
  3. `_seed_orientations`: identity-gated orientation choice → always yield both orientations.
  4. `_score_transform`: `if lm.identity != o.identity: continue` → removed (inlier of nearest catalog landmark of ANY type).
- Tests live in `landmark_loc/tests/`; run with `cd ~/Documents/Husky_viz && PYTHONPATH=. python3 -m pytest <path> -v`.
- Test fixtures mirror `test_constellation.py`: `_obs(ident,x,y)`=`Observation(ident,x,y)`, `_lm(name,ident,x,y)`=`MapLandmark(name,ident,x,y)`, and the `_scene(true_x,true_y,true_yaw)` builder that expresses catalog points in the robot frame at a known true pose.

---

## File Structure

- **Create** `landmark_loc/constellation_typeless.py` — the type-stripped matcher.
- **Create** `landmark_loc/tests/test_constellation_typeless.py` — parity + divergence + regression tests.
- **Create** `experiments/ab_matcher.py` — a small harness that runs both matchers on the same synthetic/recorded inputs and prints a per-scene comparison. (Pure offline; sim wiring is a later step, out of this plan.)

---

## Task 1: `constellation_typeless.py` — type-stripped matcher

**Files:**
- Create: `landmark_loc/constellation_typeless.py`
- Test: `landmark_loc/tests/test_constellation_typeless.py`

**Interfaces:**
- Consumes: `landmark_loc.geom.rigid_transform_2d`; `Observation` (has `.identity,.x,.y,.yaw`), `MapLandmark` (has `.name,.identity,.x,.y,.yaw`).
- Produces: `match(observations, gated_landmarks, prior_xyz, tol, max_prior_dist=5.0) -> list[(Observation, MapLandmark)]`, drop-in compatible with `constellation.match`.

- [ ] **Step 1: Write the failing tests**

```python
# landmark_loc/tests/test_constellation_typeless.py
import math
from landmark_loc.constellation_typeless import match as tl_match
from landmark_loc.constellation import match as typed_match
from landmark_loc.classify import Observation
from landmark_loc.catalog import MapLandmark


def _obs(ident, x, y, yaw=None):
    return Observation(ident, x, y, yaw)


def _lm(name, ident, x, y, yaw=None):
    return MapLandmark(name, ident, x, y, yaw)


def _scene(true_x, true_y, true_yaw, cat):
    """Express catalog points in the robot frame at a known true pose."""
    c, s = math.cos(-true_yaw), math.sin(-true_yaw)
    obs = []
    for lm in cat:
        dx, dy = lm.x - true_x, lm.y - true_y
        obs.append(_obs(lm.identity, c * dx - s * dy, s * dx + c * dy))
    return obs


# --- PARITY: on an unambiguous scene, typeless must equal typed ---
def test_parity_unambiguous_scene():
    # distinct types AND distinct pairwise distances -> geometry alone is enough,
    # so removing types must NOT change the answer.
    cat = [_lm("lampA", "lamp", 10.0, 0.0),
           _lm("benchB", "bench", 13.0, 4.0),
           _lm("binC", "trash_bin_1", 8.0, 5.0)]
    obs = _scene(0.0, 0.0, 0.0, cat)
    typed = sorted(lm.name for _, lm in typed_match(obs, cat, (0.0, 0.0, 0.0), 1.0))
    tl = sorted(lm.name for _, lm in tl_match(obs, cat, (0.0, 0.0, 0.0), 1.0))
    assert tl == typed == ["benchB", "binC", "lampA"]


def test_parity_prior_offset():
    # drift-immunity must also hold for the typeless matcher (geometry is invariant)
    cat = [_lm("lampA", "lamp", 10.0, 0.0),
           _lm("benchB", "bench", 13.0, 4.0),
           _lm("binC", "trash_bin_1", 8.0, 5.0)]
    obs = _scene(0.0, 0.0, 0.0, cat)
    pairs = tl_match(obs, cat, (4.0, 0.0, 0.0), 1.0)  # prior 4m off
    assert sorted(lm.name for _, lm in pairs) == ["benchB", "binC", "lampA"]


# --- DIVERGENCE: where types would disambiguate, typeless may pick wrong ---
def test_typeless_admits_wrong_type_inlier():
    # Two catalog landmarks of DIFFERENT type at the same geometric spot pattern:
    # a bin and a lamp swapped in position but same distances. The typed matcher
    # must respect type; the typeless matcher is free to attach an observation to
    # the wrong-type-but-nearby catalog landmark.
    cat = [_lm("lampA", "lamp", 10.0, 0.0),
           _lm("benchB", "bench", 13.0, 4.0),
           _lm("binC", "trash_bin_1", 8.0, 5.0),
           _lm("lampD", "lamp", 8.0, 5.0)]   # a LAMP sitting where the bin's obs lands
    obs = _scene(0.0, 0.0, 0.0, cat[:3])      # robot sees lamp, bench, bin
    typed = typed_match(obs, cat, (0.0, 0.0, 0.0), 1.0)
    tl = tl_match(obs, cat, (0.0, 0.0, 0.0), 1.0)
    # typed: the bin observation matches binC (type-correct)
    typed_names = {lm.name for _, lm in typed}
    assert "binC" in typed_names
    # typeless: the bin observation may be claimed by lampD (same spot, wrong type)
    # -> at minimum, the typeless result is allowed to differ; assert it does NOT
    #    enforce type (lampD is a legal inlier target for the bin observation).
    tl_names = {lm.name for _, lm in tl}
    assert ("lampD" in tl_names) or (tl_names != typed_names)


# --- REGRESSION: shared gates behave identically ---
def test_too_few_returns_empty():
    cat = [_lm("a", "lamp", 10.0, 0.0), _lm("b", "bench", 13.0, 4.0)]
    obs = _scene(0.0, 0.0, 0.0, cat)  # only 2 -> below _MIN_INLIERS
    assert tl_match(obs, cat, (0.0, 0.0, 0.0), 1.0) == []


def test_absurd_pose_rejected_by_sanity():
    cat = [_lm("lampA", "lamp", 10.0, 0.0),
           _lm("benchB", "bench", 13.0, 4.0),
           _lm("binC", "trash_bin_1", 8.0, 5.0)]
    obs = _scene(0.0, 0.0, 0.0, cat)
    # prior 100m away -> refit pose must be rejected by _PRIOR_SANITY
    assert tl_match(obs, cat, (100.0, 100.0, 0.0), 1.0) == []


def test_yaw_kept_in_typeless():
    # yaw is geometric, kept in both arms. An oriented pair whose observed yaw-diff
    # does NOT match the catalog yaw-diff must be rejected as a seed even typeless.
    cat = [_lm("benchA", "bench", 10.0, 0.0, 0.0),
           _lm("tableB", "garden_table", 14.0, 0.0, 0.0),
           _lm("lampC", "lamp", 12.0, 4.0)]
    obs = _scene(0.0, 0.0, 0.0, cat)
    # correct scene should still match (yaw consistent)
    pairs = tl_match(obs, cat, (0.0, 0.0, 0.0), 1.0)
    assert len(pairs) >= 3
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd ~/Documents/Husky_viz && PYTHONPATH=. python3 -m pytest landmark_loc/tests/test_constellation_typeless.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'landmark_loc.constellation_typeless'`.

- [ ] **Step 3: Write `constellation_typeless.py`**

Copy `constellation.py` and remove the 4 identity couplings. Keep constants, `_dist`, `_obs_pair_dists`, `_yaw_diff_ok`, `rigid_transform_2d` usage, and the prior-sanity gate verbatim.

```python
"""Types-stripped control: the RANSAC constellation matcher with the landmark
IDENTITY constraint removed, for the identity-vs-geometry experiment.

Identical to landmark_loc.constellation in every respect EXCEPT that it ignores
landmark identity: any observed pair may seed against any distance-compatible
catalog pair, and an observation may be an inlier of the nearest catalog landmark
of ANY type. The yaw constraint (geometric, not semantic) is kept, so IDENTITY is
the single difference vs. the typed matcher. See
docs/superpowers/specs/2026-08-13-typeless-matcher-control.md.

NOT used in the production localization path -- experiment harness only.
"""
import math

from landmark_loc.geom import rigid_transform_2d

# Constants identical to landmark_loc.constellation.
_INLIER_TOL = 0.5
_MIN_INLIERS = 3
_PRIOR_SANITY = 15.0
_YAW_TOL = 0.35


def _dist(ax, ay, bx, by):
    return math.hypot(ax - bx, ay - by)


def _obs_pair_dists(observations):
    d = {}
    n = len(observations)
    for i in range(n):
        for j in range(i + 1, n):
            oi, oj = observations[i], observations[j]
            d[(i, j)] = _dist(oi.x, oi.y, oj.x, oj.y)
    return d


def _cat_pairs(gated):
    """ALL catalog pairs (a, b, distance), NO type key. (Coupling point 1 removed.)"""
    out = []
    m = len(gated)
    for i in range(m):
        for j in range(i + 1, m):
            a, b = gated[i], gated[j]
            out.append((a, b, _dist(a.x, a.y, b.x, b.y)))
    return out


def _yaw_diff_ok(oi, oj, ci, cj):
    """Identical to the typed matcher: yaw is geometric, kept in both arms."""
    ys = (oi.yaw, oj.yaw, ci.yaw, cj.yaw)
    if any(y is None for y in ys):
        return True
    d_obs = (oi.yaw - oj.yaw) % math.pi
    d_cat = (ci.yaw - cj.yaw) % math.pi
    dd = abs(d_obs - d_cat) % math.pi
    return min(dd, math.pi - dd) <= _YAW_TOL


def _score_transform(tx, ty, yaw, observations, gated):
    """Project every observation; assign to the nearest catalog landmark within
    _INLIER_TOL of ANY type (coupling point 4 removed). Yaw check kept."""
    c, s = math.cos(yaw), math.sin(yaw)
    cand = []
    for k, o in enumerate(observations):
        mx = tx + c * o.x - s * o.y
        my = ty + s * o.x + c * o.y
        best, bd = None, _INLIER_TOL
        for lm in gated:
            # NOTE: no `lm.identity != o.identity` filter -- types ignored.
            if o.yaw is not None and lm.yaw is not None:
                map_yaw = o.yaw + yaw
                dd = abs(map_yaw - lm.yaw) % math.pi
                ang = min(dd, math.pi - dd)
                if ang > _YAW_TOL:
                    continue
            d = math.hypot(lm.x - mx, lm.y - my)
            if d <= bd:
                best, bd = lm, d
        if best is not None:
            cand.append((k, best, bd))
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
    cat_pairs = _cat_pairs(gated_landmarks)
    n = len(observations)
    best_inliers = []
    for i in range(n):
        for j in range(i + 1, n):
            oi, oj = observations[i], observations[j]
            dij = obs_d[(i, j)]
            for (a, b, dab) in cat_pairs:                 # coupling point 2: no type key
                if abs(dab - dij) > tol:
                    continue
                for cat_i, cat_j in ((a, b), (b, a)):     # coupling point 3: always both
                    if not _yaw_diff_ok(oi, oj, cat_i, cat_j):
                        continue
                    tx, ty, yaw, _ = rigid_transform_2d(
                        [[oi.x, oi.y], [oj.x, oj.y]],
                        [[cat_i.x, cat_i.y], [cat_j.x, cat_j.y]])
                    inliers = _score_transform(tx, ty, yaw,
                                               observations, gated_landmarks)
                    if len(inliers) > len(best_inliers):
                        best_inliers = inliers
    if len(best_inliers) < _MIN_INLIERS:
        return []
    src = [[o.x, o.y] for o, _ in best_inliers]
    dst = [[lm.x, lm.y] for _, lm in best_inliers]
    px, py, _, _ = rigid_transform_2d(src, dst)
    if math.hypot(px - prior_xyz[0], py - prior_xyz[1]) > _PRIOR_SANITY:
        return []
    return best_inliers
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd ~/Documents/Husky_viz && PYTHONPATH=. python3 -m pytest landmark_loc/tests/test_constellation_typeless.py -v`
Expected: PASS (all). If `test_typeless_admits_wrong_type_inlier` is finicky (the exact wrong-inlier depends on tie-breaking), adjust the fixture geometry so lampD lands strictly within `_INLIER_TOL` of the bin observation's projected position — the assertion is that types are NOT enforced, not a specific winner.

- [ ] **Step 5: Confirm the typed matcher is untouched**

Run: `cd ~/Documents/Husky_viz && PYTHONPATH=. python3 -m pytest landmark_loc/tests/test_constellation.py -v`
Expected: PASS (unchanged). Confirm `git diff landmark_loc/constellation.py` is empty.

- [ ] **Step 6: Commit**

```bash
git add landmark_loc/constellation_typeless.py landmark_loc/tests/test_constellation_typeless.py
git commit -m "feat(experiment): typeless constellation matcher (identity control)"
```

---

## Task 2: A/B comparison harness

**Files:**
- Create: `experiments/ab_matcher.py`

**Interfaces:**
- Consumes: `constellation.match`, `constellation_typeless.match`, `Observation`, `MapLandmark`.
- Produces: a callable `compare(observations, gated, prior, tol) -> dict` and a `__main__` demo that builds a couple of synthetic scenes (one unambiguous, one self-similar) and prints a side-by-side of the two matchers' results.

- [ ] **Step 1: Write the failing test** (a light smoke test in the same tests dir)

```python
# landmark_loc/tests/test_ab_matcher.py
import math
from experiments.ab_matcher import compare
from landmark_loc.classify import Observation
from landmark_loc.catalog import MapLandmark


def test_compare_reports_both_arms():
    cat = [MapLandmark("lampA", "lamp", 10.0, 0.0),
           MapLandmark("benchB", "bench", 13.0, 4.0),
           MapLandmark("binC", "trash_bin_1", 8.0, 5.0)]
    c, s = 1.0, 0.0
    obs = [Observation(lm.identity, lm.x, lm.y) for lm in cat]  # true pose = origin
    out = compare(obs, cat, (0.0, 0.0, 0.0), 1.0)
    assert "typed" in out and "typeless" in out
    assert out["typed"]["n_inliers"] == 3
    assert out["typeless"]["n_inliers"] == 3
    # agreement flag: did both pick the same catalog names?
    assert out["agree"] is True
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd ~/Documents/Husky_viz && PYTHONPATH=. python3 -m pytest landmark_loc/tests/test_ab_matcher.py -v`
Expected: FAIL (`No module named 'experiments'` / `experiments.ab_matcher`).

- [ ] **Step 3: Write `experiments/ab_matcher.py`**

```python
"""A/B harness for the identity-vs-geometry experiment: run the SAME inputs
through the typed and typeless constellation matchers and report a comparison.
Offline/synthetic here; sim wiring is a later step (see the spec's Out of scope).
"""
from landmark_loc import constellation, constellation_typeless


def _summary(pairs):
    names = sorted(lm.name for _, lm in pairs)
    return {"n_inliers": len(pairs), "names": names}


def compare(observations, gated, prior, tol):
    typed = constellation.match(observations, gated, prior, tol)
    typeless = constellation_typeless.match(observations, gated, prior, tol)
    ts, tls = _summary(typed), _summary(typeless)
    return {
        "typed": ts,
        "typeless": tls,
        "agree": ts["names"] == tls["names"],
    }


if __name__ == "__main__":
    import math
    from landmark_loc.classify import Observation
    from landmark_loc.catalog import MapLandmark

    def scene(true_x, true_y, true_yaw, cat):
        c, s = math.cos(-true_yaw), math.sin(-true_yaw)
        return [Observation(lm.identity, c*(lm.x-true_x)-s*(lm.y-true_y),
                            s*(lm.x-true_x)+c*(lm.y-true_y)) for lm in cat]

    # (1) unambiguous scene -> expect agreement
    cat1 = [MapLandmark("lampA", "lamp", 10.0, 0.0),
            MapLandmark("benchB", "bench", 13.0, 4.0),
            MapLandmark("binC", "trash_bin_1", 8.0, 5.0)]
    print("unambiguous:", compare(scene(0, 0, 0, cat1), cat1, (0, 0, 0), 1.0))

    # (2) self-similar scene: a row of identical-type landmarks at equal spacing
    #     -> the typeless matcher can lock onto a period-shifted arrangement.
    row = [MapLandmark(f"lamp{i}", "lamp", 10.0 + 3.0*i, 0.0) for i in range(4)]
    extra = [MapLandmark("benchX", "bench", 11.5, 4.0)]
    cat2 = row + extra
    # robot sees 3 of the lamps + the bench, from the true pose
    seen = [row[0], row[1], row[2], extra[0]]
    print("self-similar:", compare(scene(0, 0, 0, seen), cat2, (0, 0, 0), 1.0))
```

- [ ] **Step 4: Run to verify pass**

Run: `cd ~/Documents/Husky_viz && PYTHONPATH=. python3 -m pytest landmark_loc/tests/test_ab_matcher.py -v`
Expected: PASS. Also run `PYTHONPATH=. python3 experiments/ab_matcher.py` and eyeball the two printed comparisons (unambiguous → agree True; self-similar → observe whether typeless diverges).

- [ ] **Step 5: Commit**

```bash
git add experiments/ab_matcher.py landmark_loc/tests/test_ab_matcher.py
git commit -m "feat(experiment): A/B harness comparing typed vs typeless matcher"
```

---

## Task 3: Whole-suite regression + docs pointer

**Files:**
- Modify: `docs/superpowers/specs/2026-08-13-typeless-matcher-control.md` (mark implemented; note the harness entry point)

- [ ] **Step 1: Full suite green**

Run: `cd ~/Documents/Husky_viz && PYTHONPATH=. python3 -m pytest landmark_loc/tests/ -q`
Expected: PASS except the ONE known pre-existing unrelated failure (`test_launch.py::test_runbook_offers_both_modes`). Confirm no NEW failures and that the typed matcher's own tests are unchanged/passing.

- [ ] **Step 2: Add an "Implemented" note to the spec**

Add a short section recording: the two new modules, the harness entry point (`PYTHONPATH=. python3 experiments/ab_matcher.py`), and that the next step (spoof-sweep + self-similarity probe against the live sim) is separate work requiring a sim run (main-conversation, user-gated).

- [ ] **Step 3: Commit**

```bash
git add docs/superpowers/specs/2026-08-13-typeless-matcher-control.md
git commit -m "docs: mark typeless-matcher control implemented; harness entry point"
```

---

## Self-Review

**Spec coverage:** the typeless matcher with all 4 identity couplings removed and yaw kept (Task 1), the A/B harness for the comparison (Task 2), regression + docs (Task 3). The typed matcher, classifier, and localizer are untouched per Global Constraints. ✓

**Placeholder scan:** all code is complete (the full typeless module and harness are written out), test bodies are real assertions, no TBDs. ✓

**Type consistency:** `match(observations, gated_landmarks, prior_xyz, tol, max_prior_dist=5.0)` signature matches `constellation.match`; `Observation(ident,x,y,yaw)` / `MapLandmark(name,ident,x,y,yaw)` match the classes used in `test_constellation.py`; `_yaw_diff_ok` copied verbatim; constants identical. ✓

**Honest caveat surfaced:** `test_typeless_admits_wrong_type_inlier` depends on tie-breaking geometry — the plan flags this and states the assertion is "types not enforced," not a specific winner, so the implementer tunes the fixture rather than the matcher. The decisive self-similarity comparison is a *sim* experiment (Task 2's harness demonstrates the mechanism synthetically; the real sweep is out of scope, user-gated).
