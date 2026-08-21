# Tilt Disambiguation Demo

Shows why comparing the robot's roll/pitch against the terrain model eliminates a
duplicate landmark constellation that the shape matcher cannot separate.

**Finding it demonstrates.** `maps/lake_objects.yaml` contains exactly one pair of
fully-disjoint tree triples congruent within the matcher's `_INLIER_TOL` (0.5 m).
Enumerated globally they tie at 3/3 inliers — shape alone cannot choose. Their
centroids sit on ground differing by 5.45 deg in slope and 169 deg in aspect, so the
attitude the robot measures rules one out:

| robot standing at | correct hypothesis | wrong hypothesis |
|---|---|---|
| centroid A (−12.540, −20.623) | 0.28 deg | 8.90 deg |
| centroid B ( 46.078, −11.246) | 1.08 deg | 9.79 deg |

Predictor calibrated over 11 known-good poses: mean 0.253 deg, max 0.435 deg.

## Prerequisites

Bring up the lake world through **RUN-MAP-NAV.md Steps 0–3**, with `--world lake`
and the lake map/DTM/objects throughout. The demo needs only `/compass/data` and a
running master, but the RViz view comes from the operator container in Step 3.

## Step A — place the robot on one centroid

The measured tilt is only meaningful if the robot is actually standing at the
centroid under test. Place it ALIGNED to the local slope — dropping it flat onto a
slope makes it slide and tip (measured: a 6.3 deg site tipped it to 46.8 deg).

```bash
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311
cd ~/Documents/Husky_viz
source /opt/ros/noetic/setup.bash
PYTHONPATH=~/Documents/Husky_viz python3 scripts/place_on_centroid.py A   # or B
```

## Step B — publish the demo layer

```bash
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311
cd ~/Documents/Husky_viz
source /opt/ros/noetic/setup.bash
PYTHONPATH=~/Documents/Husky_viz python3 scripts/show_tilt_disambiguation.py
```

Logs the verdict for each hypothesis and latches 14 markers on
`/tilt_disambiguation`. Leave it running — a latched topic still needs its
publisher alive for new subscribers.

Optionally also show the duplicate pair on the map:

```bash
PYTHONPATH=~/Documents/Husky_viz python3 scripts/mark_duplicate_constellations.py
```

## Step C — view it

**http://localhost:6080/vnc.html?resize=scale** — displays **Tilt Disambiguation**
and **Duplicate Constellations** are already saved in `operator/operator.rviz`, so
they come back on their own after a restart.

At each centroid a stacked pair of tiles: the coloured one is the attitude the DTM
predicts for that hypothesis, the translucent white one is what the robot actually
measures. Flush = consistent. Scissoring apart = eliminated.

## Gotchas, all of them hit at least once

- **Restarting a marker node orphans RViz's subscription.** The topic briefly has no
  publisher and RViz does not re-subscribe — it keeps drawing the PREVIOUS run's
  latched markers, which looks exactly like "the sim didn't update". Toggle the
  display checkbox off/on in RViz to re-subscribe (faster than restarting RViz,
  which resets your camera).
- **The Gazebo camera does not follow a teleported robot.** After a 59 m jump the
  robot is simply off-screen. Right-click `husky` in the World tree → **Follow**.
- **Check `abs_fix` before believing the robot's drawn position.** If the selector is
  left in `landmark` mode while the localizer is silent, it reads `landmark:stale`
  and the EKF dead-reckons — measured 67.9 m of drift in a few minutes. `mode gps`
  at the operator prompt restores it (measured: 67.9 m → 0.12 m).
- **Offline mode.** With no sim, pass the recorded attitude instead of waiting on the
  compass: `_roll_deg:=0.143 _pitch_deg:=2.469 _publish_map_frame:=true`.

## What this is NOT

The localizer is **unmodified**. `constellation.match()` still returns a single
winner and still rejects the wrong hypothesis via its 15 m `_PRIOR_SANITY` check,
not by tilt. This layer VISUALIZES the evidence that would separate the candidates;
it does not show the matcher choosing. Making it a real defense means returning tied
hypotheses from the matcher and gating them on tilt residual — and using a RATIO
gate (3–5x between best and next-best), because both CORRECT hypotheses scored above
the absolute mean+3sd threshold when the hypothesis yaw differed from the robot's.
