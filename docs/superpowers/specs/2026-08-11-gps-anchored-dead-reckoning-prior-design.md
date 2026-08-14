# GPS-Anchored Dead-Reckoning Prior for Landmark Localization — Design

**Date:** 2026-08-11
**Status:** Approved (design phase)

## Goal

Give the landmark localizer a **clean, attack-independent prior** so it can
localize in landmark mode even when GPS is being spoofed — the exact situation
in which an operator switches away from GPS. The prior is built from the
**initial (pre-attack) GPS anchor** propagated forward by **wheel/odom-frame
dead reckoning**, neither of which the GPS spoof can touch.

## Background — why the localizer fails under a GPS spoof today

The landmark localizer (`landmark_loc/localizer_node.py`) needs a
roughly-correct pose prior to work. Per scan it:

1. reads its prior from `/odometry/filtered_map` (the map-frame EKF pose),
2. `catalog.gate(landmarks, prior, max_range, fov)` — keeps only catalog
   landmarks near the prior,
3. `solve.associate(obs, gated, prior, dist_gate)` — matches observations to
   gated landmarks (nearest-neighbour, after transforming observations to map
   frame **using the prior**),
4. `rigid_transform_2d` (Umeyama) solves the pose; a fit with < 2 matches or
   RMS above `residual_gate` is rejected.

**Every step depends on the prior.** And the prior comes from
`/odometry/filtered_map`, which fuses `odometry/abs_fix` — the very topic a GPS
spoof corrupts. So when the operator switches to landmark mode *because GPS is
hacked*, the prior is already poisoned: the gate looks in the wrong place, the
observations and gated candidates are disjoint, association fails, and the
localizer produces no fix. Measured: with a spoof-dragged prior ~40 m off, the
localizer stayed `landmark:stale` and the robot aborted 51 m from goal.

**Note on the pre-existing "landmark mode":** on `main`,
`move_base_landmark.launch` never actually stopped navsat — GPS kept publishing
`abs_fix`, so the map-frame prior was GPS-anchored and correct, and the localizer
"worked" only because GPS was silently present. True GPS-free landmark
localization has never run; this design is the first thing that makes it
possible under attack.

## Non-goal (explicitly deferred)

Prior-free **global** localization (find the robot from landmarks with no prior
at all — e.g. type-keyed RANSAC over the catalog) is **out of scope** and
deferred to its own design. It is the only thing that recovers when GPS is
spoofed *from the very first second*. This design assumes **GPS is trustworthy
at startup**, before any attack, and captures the anchor then.

## Design

### The clean prior

Replace the localizer's poisonable prior source (`/odometry/filtered_map`) with
a prior computed from two attack-independent sources:

- **Initial anchor `A`** — the map-frame pose `(ax, ay, ayaw)` captured **once
  at startup**, while GPS is still clean. This is the robot's true pose at t0.
- **Odom-frame pose `O(t)`** — read from `/odometry/filtered_odom`, the
  odom-frame EKF (`localization.yaml`, `world_frame: odom`). Verified to fuse
  **only** `husky_velocity_controller/odom` + IMU — **no `abs_fix`, no GPS, no
  navsat** — so it is independent of the spoof. It drifts slowly but its
  *relative* motion is trustworthy.
- **Anchor-odom pose `O_A`** — the odom-frame pose captured at the same instant
  as `A`.

The clean prior at time `t` is the initial anchor composed with the odom-frame
displacement since the anchor:

    delta = O(t) ⊖ O_A            # relative motion in the odom frame since anchor
    prior(t) = A ⊕ delta          # startup pose advanced by clean dead reckoning

Composition is a standard 2D rigid transform (translation + yaw), the same math
already in `rigid_transform_2d` / `_to_map`. Concretely, with
`O_A = (ox0, oy0, oyaw0)` and `O(t) = (ox, oy, oyaw)`:

    d = oyaw - oyaw0
    # displacement of O(t) relative to O_A, expressed in the anchor's frame:
    dx_o = ox - ox0
    dy_o = oy - oy0
    c0, s0 = cos(-oyaw0), sin(-oyaw0)
    rx = c0*dx_o - s0*dy_o
    ry = s0*dx_o + c0*dy_o
    # apply that body-frame displacement from the map-frame anchor:
    ca, sa = cos(ayaw), sin(ayaw)
    prior_x = ax + ca*rx - sa*ry
    prior_y = ay + sa*rx + ca*ry
    prior_yaw = ayaw + d

This prior tracks the robot's true current pose using only pre-attack GPS + odom
encoders. It never reads the poisoned map pose.

### Capturing the anchor

Capture `A` and `O_A` **once**, at localizer startup, when all of:

- `/odometry/filtered_map` has produced a message (the GPS-anchored startup pose),
- `/odometry/filtered_odom` has produced a message (for `O_A`),
- `/navsat/fix` `status.status >= 0` (GPS fix valid — the "GPS is clean at start"
  precondition).

Until the anchor is captured, the localizer does not attempt matching (same
"no prior → return" guard as today). Once captured, log
`anchor captured: map=(ax,ay,ayaw) odom=(ox0,oy0,oyaw0)` so the assumption is
auditable. The **initial** anchor is GPS-derived and captured exactly once; it
is never updated from the raw map pose again (that is what would reintroduce the
spoof). It may only be advanced by the localizer's own accepted landmark fixes,
per the next section — never by `/odometry/filtered_map` after startup.

### Prior refinement from the localizer's own fixes (drift correction)

Odom-frame dead reckoning drifts. Left uncorrected, over a long run the prior
would eventually drift beyond `dist_gate` and matching would fail. The
localizer already produces absolute fixes; use them to re-anchor the dead
reckoning:

- On each **accepted** landmark fix `F = (fx, fy, fyaw)` (passes the count +
  residual gate), reset the anchor to that fix and re-capture `O_A` at the same
  instant: `A := F`, `O_A := O(t_fix)`.
- This keeps the dead-reckoning baseline fresh: between fixes the prior is
  `last good landmark fix ⊕ small odom delta`, so drift only ever accumulates
  over the gap between successful fixes, never over the whole run.

This is safe because an accepted fix has already passed the count + residual
gate — it is a *landmark-derived* absolute position, not GPS, so re-anchoring to
it does not reintroduce the spoof. (The very first anchor is GPS; every
subsequent re-anchor is landmark-derived.)

**Safeguard / risk:** the residual gate is now load-bearing for the anchor, not
just the published fix. A *wrong* landmark fix that slipped through the gate
would re-anchor the dead-reckoning baseline to the wrong place and corrupt every
subsequent prior. The existing `residual_gate` (0.4 m) and `≥ 2` correspondence
requirement are the defense; the deferred one-to-one association guard (noted in
the original landmark spec) would harden it further. Re-anchoring must happen
**only** on a fix that passed the full gate — never on a coasted/None result.

### What changes in the localizer node

`landmark_loc/localizer_node.py`:

- Add a subscriber to `/odometry/filtered_odom` caching the latest odom-frame
  pose `O(t)`.
- Add a subscriber to `/navsat/fix` (or read the existing map pose only once)
  to check the GPS-valid precondition at capture time.
- Replace the `on_prior` handler's role: instead of `state["prior"]` being the
  raw `/odometry/filtered_map` pose, `state["prior"]` becomes the **computed
  clean prior** `A ⊕ (O(t) ⊖ O_A)`, evaluated at match time in `on_cloud`.
- Keep `on_prior` on `/odometry/filtered_map` **only** for the one-time startup
  anchor capture; after capture, the map pose is no longer used as the prior.
- On accepted fix: re-anchor (`A := F`, `O_A := O(now)`).

The matching pipeline (`crop → cluster → classify → gate → associate →
solve_pose`) is **unchanged** — it simply receives a clean prior instead of a
poisonable one.

### Interaction with the selector / switch

No change to the selector or the switch. In GPS mode the map-EKF is GPS-anchored
as before. In landmark mode the localizer now feeds `landmark_fix` from a clean
prior, so the map-EKF (fed via the selector) re-anchors to landmark truth even
while GPS is spoofed. The operator can switch to landmark mode *during* an
attack and the localizer bootstraps from the pre-attack anchor + odom, not from
the corrupted map pose.

## Testing

### Unit (pytest, no ROS)

Extract the prior computation into a pure function
`compose_prior(anchor_map, anchor_odom, odom_now) -> (x, y, yaw)` and test:

- Zero motion (`odom_now == anchor_odom`) → prior == anchor.
- Pure translation in odom → same translation from the anchor (rotated by anchor
  yaw).
- Pure rotation → prior yaw = anchor yaw + odom yaw delta, position unchanged.
- Combined translation + rotation → matches a hand-computed rigid composition.
- Anchor at a non-zero map pose with non-zero yaw → composition respects the
  anchor frame.

### In-sim (main conversation, from a clean kill, gzclient on :0)

1. Bring up on GPS; start move_base, localizer, selector. Confirm the localizer
   logs `anchor captured`.
2. Drive toward a goal on GPS; mid-route, start a strong GPS spoof.
3. Switch to landmark mode **while the spoof is active** (`mode landmark`).
4. Confirm: `/odometry/landmark_fix` publishes (mode clears from `:stale`), the
   fused pose re-anchors toward truth (verified against `/navsat/fix` — the
   honest sensor — NOT the spoofed value), and the robot reaches the goal on
   landmark localization with the attack still running.
5. Compare to today's behaviour (aborts 51 m off) to confirm the fix.

## Assumptions and limits

- **GPS clean at startup** — the anchor is captured before any attack. If GPS is
  spoofed from t0, this design does not help (needs the deferred global matcher).
- **Odom drift bounded between fixes** — the prior must stay within `dist_gate`
  of truth long enough to get a landmark fix; the re-anchor-on-fix logic keeps
  the drift window short. If the robot crosses a long landmark-poor stretch,
  drift could exceed the gate before a fix re-anchors it; that is a known limit,
  acceptable for the park (landmark-dense).
- **No ground truth** used anywhere (standing project rule); the anchor is the
  robot's own GPS/odom/landmark estimates, never `/gazebo/*`.
