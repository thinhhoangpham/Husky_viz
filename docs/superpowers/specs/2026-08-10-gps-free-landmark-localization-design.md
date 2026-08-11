# GPS-Free Landmark Localization — Design

**Date:** 2026-08-10
**Status:** Design approved; ready for implementation plan.

## Problem

The robot currently localizes with **GPS + compass**: the map-frame EKF fuses an
absolute position fix from GNSS (`odometry/gps`, produced by `navsat_transform`) plus
absolute heading from `compass/data`. GPS is a radio signal an attacker can forge — the
existing GPS-spoof demo exploits exactly this, dragging the fused pose off-route while the
robot chases a phantom.

We want an **alternative navigation mode that removes GPS entirely** and instead figures
out where the robot is by **recognizing known park landmarks in the live lidar and
matching them against a map the robot already holds** — a dead-reckoning-plus-landmarks
localizer. Odom supplies motion, compass supplies heading, and lidar-vs-map supplies the
absolute position anchor that GPS used to provide.

This is a **full GPS replacement** (a standalone mode), not a fallback that runs alongside
GPS. GPS mode is kept intact as a separate launch so the spoof demo still works and the
two modes can be contrasted.

## Human analogy (the whole idea in one paragraph)

Blindfold someone in a park they know, spin them, set them down. They look around: "a
bench on my left, a lamp ahead, a table on my right — on my mental map, the only place
those three line up like *that* is the north entrance, facing east." That is exactly what
this localizer does: the lidar sees shapes, a classifier names each shape by its size and
height (bench / table / lamp / bin), and the arrangement of those named shapes is matched
against a stored list of where every such object is, yielding the robot's position and
heading.

## Approach

One new node recognizes typed landmarks in the live lidar, matches that typed
constellation against a known landmark catalog, solves the robot's absolute `(x, y, yaw)`,
and publishes it as `/odometry/landmark` — a `nav_msgs/Odometry` message in the **map
frame**, the same type and frame the GPS pipeline produced. That fix is fused into the
**existing, unchanged** map-frame EKF in place of the GPS fix. Everything downstream
(move_base, costmaps, planners, operator, goals) is untouched.

The pose comes from **geometry** (a rigid-transform fit of observed landmarks onto map
landmarks). The compass+odom estimate is used only as a **prior** that seeds and gates the
match — so a slightly-wrong compass is corrected by the fit, and a grossly-wrong prior
produces a bad fit that is **rejected**, not believed. No single channel is authoritative;
the fit residual is the arbiter.

### Data flow

```
/os0_cloud_node/points ─┐
                        ▼
              [landmark_localizer node]
   1. crop cloud by height + range (keep z)
   2. cluster → blobs
   3. classify each blob by footprint + vertical profile
        → bench | garden_table | lamp | trash_bin | tree | unknown
   4. drop tree/unknown; build typed constellation of the rest ({identity, x, y})
   5. hypothesis = last EKF pose (compass yaw + odom-propagated x,y)
   6. gate map candidates to plausible field-of-view given the hypothesis
   7. associate observed ↔ map by identity + proximity
   8. solve 2D rigid transform (Umeyama/Kabsch) → (x, y, yaw)
   9. gate on fit residual: good → publish; bad or <2 matches → publish nothing
                        │
                        ▼
              /odometry/landmark  (nav_msgs/Odometry, map frame, x,y + covariance)
                        │
                        ▼
   [map-frame EKF]  odom0 = wheel odom (velocity)
                    imu1  = compass/data (absolute yaw)   ← UNCHANGED
                    odom1 = /odometry/landmark (absolute x,y)   ← was odometry/gps
                        │
                        ▼
                  map→odom TF  →  move_base drives on it, GPS never consulted
```

## Components

### 1. Landmark catalog (the map the robot holds)

A static list of every identifiable landmark with its map-frame `(x, y)` and its **type**
(bench / garden_table / lamp / trash_bin). This is known survey data — legitimate under
the no-ground-truth rule: a real deployment surveys its environment; the robot's *pose*
still comes from its own lidar, never from Gazebo's live pose. Trees are **not** in the
catalog — they are near-identical and carry no identifying information, so they are
excluded from localization (they remain honest obstacles in the costmap).

### 2. Perception — segment + classify (the one new algorithm)

Turns `/os0_cloud_node/points` into a list of typed observations `{identity, x, y}` in the
robot frame.

**Crop.** Keep points in a height band above the robot's ground level (roughly
`0.1 m < z < 2.0 m`) to drop ground return and canopy, and within a range limit (roughly
`< 15 m`) where footprints are reliable. **Keep the z dimension** through this stage.

**Cluster.** Group nearby points into blobs (Euclidean clustering, ~`0.3 m` link
distance). Discard blobs with too few points (noise) or implausibly large extent (canopy
that slipped the crop).

**Classify — in 3D, using footprint *and* vertical profile.** For each blob compute its
oriented bounding-box dimensions `(major, minor)`, point count, and the height band it
occupies. A deterministic, rule-based decision tree assigns a type. Thresholds are seeded
from the actual `.dae` mesh geometry via `map_tools/mesh_bounds.py`, so the sim geometry
and the classifier agree by construction — not hand-tuned magic numbers.

| Type | Signature (shape + height) |
|---|---|
| **lamp** | thin, near-point footprint; tall vertical extent |
| **trash_bin** | small compact footprint; short |
| **bench** | elongated, high aspect ratio; low (returns only near seat height) |
| **garden_table** | large, low aspect ratio (square-ish); a slab at table height |
| **tree** | round trunk radius, tall — *excluded from identity*, obstacle only |
| **unknown** | ambiguous / near a threshold — **dropped** |

Vertical profile is what makes lamp-vs-tree and bench-vs-table reliable — a flattened
footprint alone cannot separate a lamp post from a thin trunk. The classifier is
deliberately **conservative**: a blob near two thresholds is labeled `unknown` and dropped,
because a *wrong* type label is far more damaging to constellation matching than a missing
one.

**Output.** A list of `{identity, x, y}` — flattened to 2D **only now**, for the matcher.
The pose the robot needs is 2D `(x, y, yaw)`, the map is 2D, so identity is judged in 3D
and geometry is matched in 2D.

An empty or near-empty list (too few identifiable landmarks this scan) is valid: the node
simply publishes no fix that tick.

### 3. Matching — typed constellation → pose

**The landmark descriptor (pluggable identity).** Every landmark, observed or from the
catalog, is a record `{ identity, x, y }`. Today `identity` is the shape-derived type. The
matcher reads `identity` only as an opaque label deciding "can these two match." This makes
the identity field **swappable**: in a future map where geometry is ambiguous (e.g.
identical houses on a grid), a camera- or fiducial-derived per-instance ID can populate
`identity` instead, with **no change to the matcher or the EKF wiring**. Not built now;
the descriptor is simply structured to allow it.

**Hypothesis.** Take the last fused EKF pose (compass yaw + odom-propagated x,y) as the
prior `H = (x₀, y₀, θ₀)`. This seed is what makes matching a local association rather than
a global place-recognition search.

**Gate candidates.** Using `H`, transform each catalog landmark into the robot's expected
frame and keep only those that *should* be visible now (within range, roughly in front).
This typically reduces the catalog to a handful of candidates and is what defuses the
"16 identical benches" ambiguity — you match against the 1–2 benches the prior says could
be in view, not all of them.

**Associate.** For each observed landmark, find the gated catalog candidate of the **same
identity** whose predicted position is nearest; reject associations beyond a distance gate.
Output: identity-consistent correspondences `observed_i ↔ map_j`.

**Solve.** Given ≥2 correspondences, solve the 2D rigid transform (rotation + translation)
that best maps observed positions onto matched map positions — the standard
Umeyama/Kabsch least-squares solution (centroid-align, then SVD for rotation). That
transform *is* the robot's map-frame pose `(x, y, yaw)`. With ≥3 correspondences it is
overdetermined and averages per-landmark noise.

**Residual gate.** Compute the RMS residual of the fit.
- Residual small → publish the pose as `/odometry/landmark`.
- Residual large, or fewer than 2 correspondences → **publish nothing this tick.** A bad
  fit means the associations were wrong (mislabeled cluster, or a drifted prior latched to
  the wrong constellation); refusing to publish is how a single bad scan cannot corrupt
  the EKF.

### 4. EKF wiring & the new launch file

The output contract is drop-in identical to the old GPS fix (`nav_msgs/Odometry`, map
frame), so the EKF is **not modified internally**.

New launch file `launch/move_base_landmark.launch`, a **sibling** of the existing GPS
launch, identical except for the localization source:

| Piece | GPS launch | Landmark launch |
|---|---|---|
| map_server, move_base, costmaps, planners | ✓ | ✓ (unchanged) |
| odom-frame EKF | ✓ | ✓ (unchanged) |
| `navsat_transform_node` | present | **removed** |
| map-frame EKF `odom1` | `odometry/gps` | **`odometry/landmark`** |
| `landmark_localizer` node | — | **added** |

**Yaw:** position-only fusion. The map-EKF keeps `imu1 = compass/data` for absolute yaw
exactly as today; only `odom1`'s topic changes. The solve's yaw is used *internally* to
validate the fit and gate associations (a fit disagreeing with compass raises the residual
and is rejected) but is **not** fused. Making heading fully geometry-derived
(compass-independent) is a documented follow-on, not v1.

**Covariance.** Each `/odometry/landmark` message carries a covariance that scales with
match quality — larger (less trusted) for a thin 2-landmark fix, smaller for a rich
multi-landmark fix — so the EKF weights fixes by how well-supported they are.

No change to `two_d_mode`, filter frequency, or the compass/wheel configuration.

### 5. Recovery behavior — coast-and-recover only

When a fix is bad or absent, the node publishes nothing and the EKF **coasts on odom**,
recovering automatically the moment landmarks line up again — identical to how the stack
already handles GPS dropouts. No global re-localization fallback in v1.

Accepted limitation, on the record: if the odom prior drifts *too* far (a long
landmark-sparse stretch, or the robot being physically relocated), the right catalog
landmarks never enter the gate and the robot can stay lost. The park is landmark-dense, so
this is unlikely; a global re-acquisition fallback is added **only if this is actually
observed in-sim.**

## Explicitly NOT touched

- Gazebo simulation, world, robot, lidar, compass — identical.
- move_base, global/local planners, costmaps — identical (the landmark catalog is a
  localization input only, never a costmap; obstacles still come from live lidar).
- The odom-frame EKF and the map-frame EKF *internals* — unchanged; only the map-EKF's
  `odom1` source topic differs between the two launch files.
- Operator, goal interface, RViz — unchanged.
- GPS mode (`move_base_gps_map.launch` and `navsat_transform`) — kept intact so the spoof
  demo still runs.

## Demo

`RUN-MAP-NAV.md` Step 2 offers **both** localization modes as a choice (not a
replacement); every other step (network, world+robot, operator, goals) is identical
between modes.

```
## Step 2 — Navigation + map  (choose ONE localization mode)

### Option A — GPS mode (spoofable; used by the attacker demo)
roslaunch launch/move_base_gps_map.launch

### Option B — Landmark mode (GPS-free; recognizes park landmarks)
roslaunch launch/move_base_landmark.launch
```

**Showing it works (Option B), judged by the robot's own sensors and the operator's view —
never ground truth:**
1. Send a normal goal (`goal garden_table`); success = the robot arrives and stops, same
   as GPS mode — proving the lidar+map pose is good enough to drive on.
2. RViz overlay: display `/odometry/landmark` with the static map; the estimated pose sits
   where live lidar clusters align with map landmarks — visibly snapped to real furniture.
3. Drive a long leg: in pure odom the pose would drift metres off; in landmark mode it
   stays pinned because each passed landmark re-anchors it. Watch it *not* drift.

**Security contrast (the payoff).** Run the existing GPS-spoof against both modes:
- GPS mode: the spoof drags the fused pose off; the robot thrashes chasing a phantom.
- Landmark mode: the same network attack has nothing to grab — no `navsat_transform`, no
  GPS topic in the loop — so it is **inert**. The robot keeps localizing off furniture it
  can see. Headline: unplug the spoofable signal and the spoof simply stops working.

## Testing / success criteria

Per the no-ground-truth rule — judged by the robot's own honest sensors and the operator's
Gazebo/RViz view, never against Gazebo ground truth:

1. **Classifier:** on live lidar, benches/tables/lamps/bins are labeled correctly and
   trees/ambiguous blobs are dropped, verified by overlaying labeled clusters on the map
   in RViz.
2. **Localizer nominal:** `/odometry/landmark` places the robot where the lidar clusters
   visibly align with the map landmarks; the robot drives to and stops at a named goal in
   landmark mode.
3. **No-drift:** over a long leg, the landmark-mode estimate stays anchored where pure odom
   would have drifted off.
4. **Spoof inert:** running the GPS-spoof in landmark mode has no effect on the robot's
   navigation.

## Known risks

- **Classification from a single sweep is the riskiest part** — a partially-occluded bench
  can look like a lamp; two close trees can look like a table. Mitigated by the
  height/range crop, the 3D (footprint + vertical-profile) features, the conservative
  `unknown` bias, and the matcher's residual gate rejecting fits built on a bad label.
- **Landmark-sparse stretches** → the robot coasts on odom (bounded drift) until the next
  sighting; only a problem if a leg is long and open (park is dense, so unlikely).
- **Prior drifting too far to gate** → coast-and-recover may not re-acquire; global
  fallback deferred until observed.
- **Compute** — clustering + classifying + matching at lidar rate alongside gzserver;
  verify it keeps up.
```
