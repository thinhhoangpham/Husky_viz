#!/usr/bin/env python3
"""Plot the GPS-spoof propagation chain: /navsat/fix -> /odometry/gps -> /odometry/filtered_map."""
import csv
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

IN = "/home/thinh/Documents/Husky_viz/gps_chain_record.csv"
OUT = "/home/thinh/Documents/Husky_viz/gps_chain_plot.png"

REF_LON = 8.9  # datum longitude (gps.urdf.xacro)


def f(v):
    return float("nan") if v == "nan" else float(v)


t, lon, gy, fy = [], [], [], []
with open(IN) as fh:
    for row in csv.DictReader(fh):
        t.append(f(row["elapsed_time"]))
        lon.append(f(row["navsat_lon"]))
        gy.append(f(row["gps_odom_y"]))
        fy.append(f(row["filtered_map_y"]))

# Longitude offset from datum, in micro-degrees, to make the tiny spoof visible.
lon_off = [(l - REF_LON) * 1e6 if l == l else float("nan") for l in lon]

fig, (a1, a2) = plt.subplots(2, 1, figsize=(12, 9), sharex=True)
fig.suptitle("GPS-spoof propagation: /navsat/fix -> navsat_transform -> map-EKF",
             fontsize=14)

# Top: the spoofed raw GPS longitude (the injection point)
a1.plot(t, lon_off, color="tab:purple", label="navsat_lon offset from datum (micro-deg)")
a1.set_ylabel("lon - 8.9  (1e-6 deg)")
a1.set_title("STAGE 1 — raw /navsat/fix longitude (attacker injects here)")
a1.legend(loc="upper left")
a1.grid(True, alpha=0.3)

# Bottom: the two downstream map-frame y estimates, overlaid (should coincide)
a2.plot(t, gy, color="tab:orange", lw=3, label="/odometry/gps  y  (navsat_transform output)")
a2.plot(t, fy, color="tab:blue", lw=1.3, label="/odometry/filtered_map  y  (map-EKF, operator sees)")
a2.set_ylabel("map-frame y (m)")
a2.set_xlabel("time since record start (s)")
a2.set_title("STAGE 2+3 — the lie carried downstream: gps_odom_y and filtered_map_y move together")
a2.legend(loc="upper left")
a2.grid(True, alpha=0.3)

fig.tight_layout(rect=[0, 0, 1, 0.96])
fig.savefig(OUT, dpi=130)
print("wrote", OUT)
peak = max(v for v in fy if v == v)
print("peak filtered_map_y = %.2f m" % peak)
