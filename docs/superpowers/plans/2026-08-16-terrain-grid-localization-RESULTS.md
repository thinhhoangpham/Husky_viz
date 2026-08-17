# SDD ledger — plan: docs/superpowers/plans/2026-08-16-terrain-grid-localization.md

## Pre-flight conflict scan (2026-08-16)

Shared-file / shared-interface rows:
| Tasks | producer → consumer | finding |
|---|---|---|
| 1 → 2 | derotate.{roll_pitch_from_quat,derotate_cloud} → localizer on_compass/on_cloud | OK — names match across both task texts |
| 2 → 5 → 7 | localizer_node.py modified by 3 tasks sequentially | OK — 2 (de-rotate), 5 (terrain cand), 7 (tracker) touch DIFFERENT regions of on_cloud, in order; each builds on prior. No overlap conflict; must run in order (enforced). |
| 3 → 5 | terrain_grid.{bin_min_z,morphological_ground} → localizer | OK — signatures consistent |
| 4 → 5 | terrain_match.{load_dtm,match_terrain} → localizer; Dtm mirrors DtmGrid fields | OK |
| 4 → 7 | match_terrain returns (x,y,score) → terrain_cand[0/1] | OK — consistent |
| 6 → 7 | HypothesisTracker.{predict,update,committed} → localizer | OK — signatures consistent |

Self-consistency rows:
| Task | finding |
|---|---|
| 1 | tests assert on funcs the impl defines; consistent |
| 2 | modifies files it names; wiring seam tested via helper (on_cloud is a closure — plan acknowledges, tests at fn level). OK |
| 3 | bin_min_z min-per-cell via argsort descending + last-write: subtle but correct; tests cover it |
| 4 | match_terrain center-of-placement math self-consistent with tests |
| 5 | no-DTM path = no-op (dtm is None); regression-safe. OK |
| 6 | tracker decay/commit logic matches its tests |
| 7 | reuses committed (x,y) through existing jump/median/cov machinery. OK |

SCAN RESULT: clean. No rulings needed pre-execution. Sequential ordering of
Tasks 2/5/7 on localizer_node.py is a dependency, not a conflict — enforced by
running in plan order.

## Pre-flight rulings
Ruling: committed this session's DTM tooling (map_tools/extract_dtm.py etc.) +
operator.rviz as the plan's prerequisite baseline (commit before Task 1); left
pre-existing M localizer_node.py (one-line stamp fix) + M CLAUDE.md uncommitted.
— why: DTM tooling is a Task-4 prerequisite and was done+tested but uncommitted;
the two pre-existing edits predate this work and aren't mine to bundle.
— cost if wrong: pre-existing stamp fix coexists in the working tree with
implementer edits to localizer_node.py; isolated line far from plan insertion
points, low collision risk.

## Task progress

Ruling: Task 1 implementer changed level_rotation pitch sign Ry(-pitch)->Ry(+pitch),
reporting the brief's verbatim code failed its own flatten test. Controller
INDEPENDENTLY verified: a body-frame cloud tilted by robot pitch flattens to
z-std 0.0 with +pitch. The brief (my spec) had the sign wrong; the fix is correct
and minimal (names/signatures unchanged, Task 2 imports safe).
— cost if wrong: a de-rotation sign error would inject rather than remove tilt,
corrupting terrain matching. Verified correct, so cost ~nil.

Task 1: complete (commits 4cc75ee..c5dda22, review clean — Spec ✅, quality Approved).
Task 1: minor (deferred): derotate_cloud empty-input accepts any (0,N) shape (conf ~40, non-blocking).

Task 2: complete (commits c5dda22..b517c54, review clean — Spec ✅, quality Approved, no findings).

## PHASE 1 GATE — pending decision (sim gate deferred, see ledger note below)

PHASE 1 GATE: PASS (lake world, run 2026-08-16).
- Localizer on_cloud runs every scan with de-rotation ACTIVE, no crash/traceback.
- obs=5 per tick: de-rotated cloud still crops/clusters/classifies into observations
  (de-rotation does not destroy clusters).
- Compass roll/pitch verified available (-3.86 deg / -0.67 deg) and feeding on_cloud.
- assoc=0 STALE at spawn standstill = EXPECTED (sparse lake catalog, robot idle);
  per memory rule "idle STALE localizer is expected, judge only during drive".
- The narrow gate question (does de-rotation break the real-lidar path?) = NO, clean.

Task 3: complete (commits b517c54..cb218b1, review clean — Spec ✅, quality Approved, no findings).

Task 4: review — Spec ✅, quality Approved, no Critical/Important. 3 Minor.
  Ruling: elevating Minor#1 (no offset-invariance test, conf 55) into ONE fix round.
  — why: offset-invariance is the load-bearing property of the whole terrain approach;
  it holds by construction but an untested invariant can be silently broken by a future
  edit. One-test insurance is cheap. Minor#2 = confirmed non-bug (ignored). Minor#3 =
  conf-30 stale-yaml edge (deferred, not in required signature).
  — cost if wrong: negligible; the test only pins an already-true property.
Task 4: minor (deferred): load_dtm ignores width/height yaml keys, no shape validation (conf 30).
Task 4: fix round 1/5 dispatched — add test_match_is_offset_invariant (resumed original implementer).

Task 4: fix round 1/5 (1 addressed, 0 open — offset-invariance test; commit 6ffd328..d3664c5).
  Scoped re-review: fix is test-only (17 insertions), terrain_match.py untouched, 6/6 green.
  Finding ADDRESSED, no new breakage.
Task 4: complete (commits cb218b1..d3664c5, review clean after 1 fix round; 1 minor deferred).

Task 5: complete (commits d3664c5..61299af, review clean — Spec ✅, quality Approved, no findings).
  Note: terrain block placed before the `result is None` early-return so terrain cue runs
  every tick even when object fix is STALE (correct for sparse-catalog lake). Reviewer confirmed.

## PHASE 2 GATE — running now (lake, terrain cue live via _dtm_path)

## PHASE 2 GATE: FAIL — plan defect found (terrain_cand always None on lake)
Ruling: match_terrain (Task 4 design) rejects any window not FULLY inside the DTM
(_score_at: `if r0<0 or c0<0 or r0+h>ph or c0+w>pw: return None`). Robot spawns at
the DTM's left edge (col 11 of 398), so its ~40m view hangs 17m off the left edge and
EVERY placement is rejected -> terrain cue never fires. Compounded by Task 5 building a
FULL-DTM-SIZED local grid (dtm.z.shape) instead of a small window around the prior, so
there is nothing to slide. Both trace to MY plan, not implementer error; unit tests
passed because they only placed the local patch safely inside the prior.
Fix (Task 4+5 fix round): (a) _score_at clips window to prior bounds and scores the
OVERLAP (keep >=25-cell floor); (b) Task 5 builds a windowed local grid (~40x40 m)
around the prior, not full-DTM-size. Add edge-placement unit tests. Re-run Phase 2 gate.
— cost if wrong: terrain cue stays dead near map edges; partial-overlap match could be
noisier with less overlap, mitigated by the 25-cell floor.

Task 6: complete (commit 61299af..8492dcb, review clean — Spec ✅, quality Approved, no findings).
  Reviewed independently of the in-flight terrain fix (different files, no overlap).

Terrain fix: commit 703b133 — _score_at clips to overlap; localizer builds windowed local grid (~terrain_window_m=20).
  9/9 test_terrain_match (3 new edge cases); full suite 221 pass, only test_launch pre-existing fail.
  OFFLINE VERIFIED by controller on REAL lake DTM at robot spawn edge (-47,-15):
    match_terrain now returns (-47.25,-15.0) score 1.0, 0.15m from truth, 6324/25600 window cells valid.
    (was None before fix). Offset-invariance survived (+5.0 constant cancelled via gradients).
  In review (dispatched a73c1c4a). Then re-run Phase 2 sim gate.

Terrain fix review: Spec/correctness ✅ Approved (index math hand-traced both edge cases, offset-invariance intact).
  1 Minor (conf 85): test_match_still_none_when_overlap_below_min hits the ZERO-overlap guard, not the
  <25-cell floor its comment claims -> the <25-but-nonzero path is untested. Elevated to a test-only fix
  round (cheap, and the floor matters for edge robustness). Resumed implementer a822f96.

Terrain fix round 2: commit 790d704 (test-only, 9/9). Amended test now hits <25-cell branch (4 valid cells).
  Scoped: production code untouched. Terrain fix COMPLETE + reviewed clean.
  Task 4/5 effectively amended (commits 703b133, 790d704).

## PHASE 2 GATE RE-RUN — starting (restart localizer with fixed code on the running lake world)

PHASE 2 GATE (re-run after terrain fix): ACCEPTED (conditional).
- Terrain matcher now RUNS live end-to-end with the fix: returns consistent (-42,-20) score ~0.41
  at the robot's standstill spawn (prior -47,-15). Before the fix it returned None every scan.
- The match is WEAK (0.41 < 0.5 gate) and ~7m off: robot is parked at the FLAT MAP CORNER seeing
  a small sparse ground patch — terrain matching's worst case, exactly the design §11 caveat.
- The 0.5 gate CORRECTLY REJECTS this weak/wrong match rather than publishing a 7m error. Safety
  mechanism verified working.
- Ruling (user-confirmed): code is correct; weak standstill-at-flat-corner match is EXPECTED, not a
  defect. Real terrain-tracking quality (driving across relief, fused via tracker) is judged in the
  PHASE 3 GATE. Proceed to Task 7.
  — cost if wrong: terrain cue may contribute little in this map's flat regions; caught in Phase 3 drive test.

Task 7: complete (commits 790d704..a5f087b, review clean — Spec PASS, quality Approved, no Critical/Important).
  Restructure verified: terrain-only fix now reaches the tracker (object-None no longer bails early).
  Double-smoothing (tracker 0.5-blend + fix_history median) assessed = LATENCY ONLY, cannot produce wrong
  pose (reviewer confidence high). WATCH in Phase 3 gate for sluggish re-convergence after jump/reset.
  covariance_for(n=committed.support) for terrain-only = numerically safe (max(n,1) guard).
  Added UNCOMMITTED hyps=N throttled diag (beyond brief) = reasonable visibility, approved.

ALL 7 TASKS COMPLETE + reviewed clean. Remaining: Phase 3 sim gate (drive test), final whole-branch review.

PHASE 3 GATE (drive + GPS-denied): system RUNS clean end-to-end, but landmark localizer
COMMITS 0 fixes -> cannot localize GPS-denied on the lake as-is. Honest outcome:
- Robot reached goal (-10,5) exactly, but ON GPS before the spoof bit. Landmark cue drove nothing.
- Object cue DEAD: classifier mislabels the 26 lake tree_8 trunks as 'bench'/'garden_table' (park
  types), never outputs 'tree', so despite 13 catalogued 'tree' entries -> assoc=0 ALL RUN. This is
  the ROOT-CAUSE BUG, and the biggest lever (fixing it likely makes the lake object-localizable).
- Terrain cue: cleared 0.5 gate 5x while driving (rose from 1x standstill), but never 3 consecutive
  -> tracker UNCOMMITTED 5x, 0 commits. Too intermittent to be load-bearing alone.
- Tracker behaved SAFELY: refused to commit on intermittent candidates (no wrong-lock).
The terrain-grid build (Tasks 1-7) is CORRECT and complete; the lake localization failure is due to
(a) the pre-existing object classifier misclassifying lake trees, and (b) terrain being intermittent.
NEXT (user-directed): investigate the tree-misclassification bug (separate from this plan's scope).

TREE MISCLASSIFICATION ROOT CAUSE (diagnosed live, sim up):
The object classifier uses ABSOLUTE sensor-frame height thresholds hand-tuned to the PARK
(flat ground at z~0): crop z_min=-0.5/z_max=7.0, _TREE_CANOPY_MIN_Z=2.5, _GROUND_Z_MAX=0.5.
On the LAKE the robot sits on ELEVATED terrain: ground bulk is at sensor-frame z ~ -0.8m
(14k pts at z=[-1.4,-0.2]), not ~0. So crop(z_min=-0.5) slices ABOVE most ground and clips
tree-trunk bases, leaving flat fragments (z_max ~1.6m, height <0.2m). Nothing reaches the
2.5m canopy band -> no cluster classifies as 'tree' -> they fall through to short-box classes
(bench/garden_table) -> assoc=0 vs the lake's 13 'tree' catalog entries.
This is the SAME "no local ground reference / define 0.0 on uneven terrain" problem from the
design discussion. The classifier needs a LOCAL ground datum (the morphological ground we built
for terrain, or a per-scan ground fit), not absolute park-calibrated z bands.
De-rotation is NOT the cause (raw crop is just as broken) but shares the root: absolute-z
assumptions don't transfer to elevated/sloped terrain.
SCOPE: this is a pre-existing OBJECT-CLASSIFIER defect, OUTSIDE the terrain-grid plan (Tasks 1-7,
which are correct + complete). Fixing it = separate work; needs user direction.

FINAL WHOLE-BRANCH REVIEW (opus, 10 commits 4cc75ee..a5f087b): CLEAN — mergeable.
No Critical, no Important, no blockers. Verified at feature level:
- frame discipline correct at every seam (object path sensor-frame; terrain path map-frame; no cross-contamination)
- no publish bypasses the tracker; terrain-only fixes DO reach it (the Task-7 restructure is correct)
- tracker_last_odom never double-applies/skips displacement across early returns
- de-rotation is EXACT identity on the flat park -> no park object-classification regression
- spec §3-8 all delivered
3 new Minors, ALL triaged "acceptable follow-on": dead empty-branch in derotate_cloud;
pure-Python morphological filters O(H·W·k²) (ran clean in gates, no timing recorded);
no diagnostic log for sub-gate terrain matches (observability gap).
3 ledger deferred-minors also triaged "acceptable follow-on" (load_dtm yaml keys;
covariance_for n=support is numerically safe via max(n,1); derotate empty shape).
Phase-3 lake failure confirmed NOT a finding against this diff (pre-existing classifier defect;
this diff does not worsen it).
BUILD COMPLETE.
