#!/usr/bin/env python3
"""Plot the operator status log (continuous 2 Hz operator view of the fused pose)."""
import re
from datetime import datetime

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

IN = "/home/thinh/Documents/Husky_viz/operator_status.log"
OUT = "/home/thinh/Documents/Husky_viz/operator_status_plot.png"

GOAL = (1.0455350795894576, -2.349310460540776)

line_re = re.compile(
    r"^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}) "
    r"state=(\S+) .*? pos=\(([-\d.]+), ([-\d.]+)\) \| dist=(\S+)"
)

t, sx, sy, dist, state = [], [], [], [], []
t0 = None
with open(IN) as f:
    for ln in f:
        m = line_re.match(ln)
        if not m:
            continue
        ts = datetime.strptime(m.group(1), "%Y-%m-%d %H:%M:%S")
        if t0 is None:
            t0 = ts
        t.append((ts - t0).total_seconds())
        state.append(m.group(2))
        sx.append(float(m.group(3)))
        sy.append(float(m.group(4)))
        d = m.group(5)
        dist.append(float("nan") if d == "nan" else float(d))

# Detect drift window: contiguous span where pos_y > 3.0 m
drift_lo = drift_hi = None
for i, y in enumerate(sy):
    if y > 3.0:
        if drift_lo is None:
            drift_lo = t[i]
        drift_hi = t[i]

peak_y = max(sy)
print("parsed %d samples over %.1f s" % (len(t), t[-1]))
print("peak pos_y = %.2f m" % peak_y)
if drift_lo is not None:
    print("drift window (pos_y>3): %.1f s -> %.1f s (%.1f s)"
          % (drift_lo, drift_hi, drift_hi - drift_lo))

fig, (ax2, ax3) = plt.subplots(2, 1, figsize=(12, 11))
fig.suptitle("Operator status log — fused pose under GPS-drift attack", fontsize=14)

# Panel 1: dist to goal vs time
ax2.plot(t, dist, color="tab:green", label="dist to goal (m)")
if drift_lo is not None:
    ax2.axvspan(drift_lo, drift_hi, color="orange", alpha=0.2,
                label="GPS drift attack")
ax2.set_ylabel("distance (m)")
ax2.set_xlabel("time since start (s)")
ax2.set_title("Distance to sent goal  (rises during attack, then reaches goal)")
ax2.legend(loc="upper right")
ax2.grid(True, alpha=0.3)

# Panel 2: XY trajectory colored by time
sc = ax3.scatter(sx, sy, c=t, cmap="viridis", s=12)
ax3.plot(GOAL[0], GOAL[1], "r*", ms=18, label="goal")
ax3.plot(sx[0], sy[0], "ks", ms=8, label="start")
ax3.set_xlabel("pos_x  (down-range, m)")
ax3.set_ylabel("pos_y  (lateral / drift direction, m)")
ax3.set_title("Fused-pose trajectory in map plane (color = time)  — "
              "vertical spike = GPS drift")
ax3.set_aspect("equal", adjustable="datalim")
ax3.legend(loc="upper right")
ax3.grid(True, alpha=0.3)
cb = fig.colorbar(sc, ax=ax3)
cb.set_label("time (s)")

fig.tight_layout(rect=[0, 0, 1, 0.97])
fig.savefig(OUT, dpi=130)
print("wrote", OUT)
