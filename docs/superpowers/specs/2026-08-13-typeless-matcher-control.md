# Types-Stripped Matcher — Control for the Identity-vs-Geometry Experiment

**Date:** 2026-08-13
**Branch:** `feat/typeless-matcher-control`
**Status:** design / experiment spec, pending review

## Purpose (the research question this serves)

The novelty claim of the landmark localizer is that **semantic landmark identity**
makes spoof-resistant localization more robust than **geometry alone**. To test
that claim we need a controlled comparison that isolates exactly one variable:
**identity, present vs. absent.**

This spec defines that control: a **types-stripped** version of the existing
RANSAC constellation matcher (`landmark_loc/constellation.py`) that matches on
the **same pairwise-distance geometry** but **ignores landmark identity**. It is
NOT ICP/NDT — it is the current matcher with the type constraint removed, so the
only difference between the two arms is whether types are used. Everything else
(seed→hypothesis→score→win RANSAC structure, the prior-sanity gate, the pose
solve) is identical.

**Why this control and not ICP/NDT first:** ICP/NDT differ from our matcher in
*two* ways at once — no identity AND local-refinement vs. global-vote AND a
different data primitive (raw points vs. named landmarks). A win over ICP/NDT
would be confounded. The types-stripped matcher differs in *exactly one* way
(identity), so it cleanly answers "does identity help?" ICP/NDT remain a useful
*deployed-standard* reference to add later, but this control comes first because
it is decisive on the identity question and far cheaper to build (a variant of
code we already have, no external map cloud, no new library, no fair-tuning
burden).

## Where identity lives in the current matcher (the exact coupling points)

Read from `landmark_loc/constellation.py` (current). The type constraint appears
in **four** places; the types-stripped variant removes all four:

1. **`_cat_pair_index` (lines 53-64):** catalog pairs are indexed by
   `frozenset((a.identity, b.identity))`. An observed pair only looks up catalog
   pairs of the **same type-combination**.
2. **`match` seed lookup (lines 139-140):** the observed pair's
   `frozenset((oi.identity, oj.identity))` keys into that index, so only
   same-type catalog pairs are candidate seeds.
3. **`_seed_orientations` (lines 67-79):** decides which orientation(s) of a
   catalog pair are type-consistent with the observed pair — pure identity logic.
4. **`_score_transform` inlier test (lines 106-108):** `if lm.identity !=
   o.identity: continue` — an observation can only be an inlier of a **same-type**
   catalog landmark.

(The yaw constraint, `_yaw_diff_ok` / the yaw check in `_score_transform`, is a
separate geometric feature, not identity. Keep it in BOTH arms so the only
difference is identity. See "Open decision" below.)

## Design — a parallel module, not an edit

**Do NOT modify `constellation.py`.** The typed matcher is the experimental arm
under test; it must stay byte-identical to what shipped. Create a sibling:

- **New:** `landmark_loc/constellation_typeless.py` — same public entry point
  `match(observations, gated_landmarks, prior_xyz, tol, max_prior_dist=5.0)`
  returning `list[(Observation, MapLandmark)]`, so it is a drop-in swap.

### What changes vs. `constellation.py`

The typeless variant keeps the RANSAC structure and constants (`_INLIER_TOL`,
`_MIN_INLIERS`, `_PRIOR_SANITY`, `_YAW_TOL`) identical, and changes only the four
coupling points:

1. **`_cat_pair_index_typeless`:** index catalog pairs by nothing (a single flat
   list of all `(a, b, distance)` pairs), OR by a coarse distance bucket for
   speed — but semantically, every observed pair may match **any** catalog pair
   of compatible distance, regardless of type.
2. **Seed lookup in `match`:** for each observed pair `(i, j)`, consider **all**
   catalog pairs `(a, b)` whose distance is within `tol` of `obs_d[(i, j)]` — no
   type key.
3. **Seed orientations:** since there is no identity to line up, **both**
   orientations `(a, b)` and `(b, a)` are always candidates (the typed
   `_seed_orientations` collapses to "always yield both").
4. **`_score_transform_typeless`:** an observation is an inlier of the nearest
   catalog landmark within `_INLIER_TOL` **of any type** (drop the
   `lm.identity != o.identity` filter; keep the one-to-one nearest assignment and
   the yaw check per the open decision).

Everything else — `rigid_transform_2d` seed solve, whole-set inlier scoring, the
`_MIN_INLIERS` floor, the final `_PRIOR_SANITY` refit gate — is copied verbatim.

### Consequence to expect (and measure)

Removing types makes the search **larger and more ambiguous**: many more candidate
seeds (every distance-compatible pair, not just same-type), and inliers can attach
to the wrong-type-but-nearby catalog landmark. In an environment with repeated
geometry this should produce **more wrong-but-self-consistent matches** — which is
exactly the failure mode the typed matcher is hypothesized to avoid. That
divergence, if it appears, is the result.

## Yaw constraint — DECIDED: keep in both arms (option A)

**The yaw constraint stays in BOTH arms.** The typed matcher uses pairwise
yaw-difference (`_yaw_diff_ok`) as an extra *geometric* filter — yaw is an
orientation feature, not a semantic label — so it is kept identical in the
typeless variant. This makes **identity the single variable that differs** between
the two arms, which is the whole point of the control.

Concretely, in `constellation_typeless.py`: copy `_yaw_diff_ok` verbatim and keep
the yaw check inside `_score_transform_typeless` exactly as in the typed matcher
(the `o.yaw is not None and lm.yaw is not None` guard, `_YAW_TOL`, all unchanged).
The ONLY thing removed is the identity coupling at the four points listed above.
(Round types still carry `yaw=None` and skip the yaw check in both arms — that
behavior is shared and unchanged.)

## The experiment (what the control is FOR)

With both matchers available as drop-in `match()` implementations:

1. **A/B harness.** Run the same recorded/live lidar stream through both matchers
   each tick (typed = `constellation.match`, typeless =
   `constellation_typeless.match`), with the **same** observations, gated
   landmarks, prior, and tol. Log for each: match found? #inliers, implied pose,
   and (in sim, for SCORING ONLY — not fed to either matcher) the pose error vs.
   ground truth.
2. **Spoof sweep.** Drive a route under the `attack_navsat.py` slow-drift spoof at
   several aggressiveness levels (drift-rate × max-offset). For each level and
   each arm, measure: **detection latency** (time from spoof onset to the arm's
   pose diverging from the spoofed GPS by a threshold) and **false-alarm rate**
   (divergence flagged with no spoof present).
3. **Self-similarity probe (the decisive test).** Construct or find a
   region with **repeated geometry** — e.g. a row of near-identical lamp posts, or
   a symmetric cluster of same-type landmarks. Hypothesis: the typeless matcher
   locks onto a **period-shifted wrong** pose (low inliers-error but wrong place),
   while the typed matcher rejects it because the *type sequence* doesn't match.
   This is where identity is predicted to win; if it doesn't show a difference
   here, the identity advantage is weak and the paper's framing must change.

**Primary outcome:** a plot of detection-latency vs. false-alarm-rate for typed
vs. typeless across the spoof sweep, plus a head-to-head on the self-similarity
probe. If typed dominates in some regime → identity is a real primitive. If flat →
report "identity and geometry are equivalent here" honestly and pivot the framing
(e.g. toward complementary coverage or the harder-adversary argument).

## Testing (unit, before any sim)

- **Parity on a non-degenerate scene:** on a synthetic constellation with
  distinct types AND distinct geometry (no repeated distances), typed and typeless
  must return the **same** match — removing types shouldn't change the answer when
  geometry alone is unambiguous. This proves the typeless variant is a correct
  geometric matcher, not a broken one.
- **Divergence on a repeated scene:** a synthetic scene with two same-distance
  pairs of different types placed so the typeless matcher can pick the wrong one —
  assert typed picks the type-correct match and typeless is free to pick either
  (documents the exact mechanism of the hypothesized advantage).
- **Prior-sanity + min-inliers behavior identical** to the typed matcher (same
  constants, same gates) — regression on the copied logic.
- Existing `test_constellation.py` for the typed matcher stays green (we didn't
  touch `constellation.py`).

## Out of scope (for THIS branch)

- ICP/NDT baseline arm (a later, separate reference — needs a map cloud + fair
  tuning; noted in the research plan, not built here).
- The `spoof_monitor` auto-detection/auto-switch node (separate feature; this
  branch only provides the control matcher + A/B harness for offline/sim
  comparison).
- Any change to the typed matcher, the classifier, or the localizer's shipped
  behavior. The typeless matcher is additive and used only by the experiment
  harness, never in the production localization path.

## Risks / honest caveats

1. **The comparison may come out flat.** If the park lacks self-similar structure,
   identity may never demonstrably beat geometry. Mitigation: the self-similarity
   probe deliberately constructs the regime; if even that is flat, that is itself
   the (honest, publishable-as-negative) finding.
2. **Typeless is slower / noisier** — more seeds, more ambiguous inliers. That is
   expected and is part of the result, not a bug to tune away.
3. **Fairness:** both arms must get identical inputs and identical non-identity
   parameters (tol, inlier-tol, prior-sanity, yaw per decision A). Any asymmetry
   invalidates the comparison — the A/B harness must enforce it.

---

## Implemented (2026-08-13)

Built on branch `feat/typeless-matcher-control`:

- **`landmark_loc/constellation_typeless.py`** (commit `dae615e`) — the type-stripped
  matcher. Reviewed function-by-function against the typed matcher: identity removed
  at exactly the 4 coupling points, everything non-identity (yaw, prior-sanity,
  constants, inlier scoring, seed solve) byte-identical. The typed matcher is
  untouched. 6/6 typeless tests + 9/9 typed tests pass.
- **`experiments/ab_matcher.py`** (commit `3737762`) — the A/B harness.
  `compare(observations, gated, prior, tol)` runs both matchers on identical inputs
  and returns `{typed, typeless, agree}`. Reviewed FAIR (same inputs, no mutation).
  Entry point: `PYTHONPATH=. python3 experiments/ab_matcher.py`.
- Full suite green (107 passed; the 1 failure is the pre-existing unrelated
  `test_launch.py::test_runbook_offers_both_modes`).

### Early result (honest, and it matters)

The first synthetic self-similarity probe in the harness `__main__` came out
**AGREE** — typed and typeless returned identical matches even on the "self-similar
row of lamps" scene. This is almost certainly a **weak-scene artifact**, not the
research answer: the demo scene includes a unique **bench** alongside the lamps, and
a single unique landmark anchors the whole constellation, so the lamps get pinned
correctly even WITHOUT types. A genuinely ambiguous scene must have the OBSERVED set
be all-same-type (no unique anchor) for identity to matter.

**Implication for the experiment:** the decisive divergence test needs a scene the
typeless matcher genuinely can't disambiguate — all-same-type observations in a
symmetric/repeated arrangement — before any conclusion about identity-vs-geometry
can be drawn. This is an experiment-DESIGN step, not a code fix.

### Next (separate work, user-gated)

- Construct a genuinely-ambiguous divergence scene (all-same-type observed set) and
  confirm typeless diverges where typed holds — the minimal proof identity matters.
- The real study: spoof-sweep (`attack_navsat.py`) + self-similarity probe against
  the live sim, logging detection-latency vs. false-alarm for both arms. Requires a
  sim run (main-conversation, user-gated).
