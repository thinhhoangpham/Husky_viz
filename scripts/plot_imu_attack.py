#!/usr/bin/env python3
"""Plot normal vs IMU attack: fused_yaw over time from the three run CSVs."""
import csv
import matplotlib.pyplot as plt


def load(path, col):
    t, y = [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            t.append(float(row["elapsed_time"]))
            y.append(float(row[col]))
    return t, y


files = [
    ("husky_auto_drive.csv", "normal (drive straight)", "tab:green"),
    ("attack_imu_faithful.csv", "faithful attack", "tab:orange"),
]

fig, ax1 = plt.subplots(1, 1, figsize=(11, 5))

for path, label, color in files:
    t, yaw = load(path, "fused_yaw")
    ax1.plot(t, yaw, label=label, color=color, marker=".")

ax1.set_ylabel("fused_yaw (rad)")
ax1.set_xlabel("time (s)")
ax1.set_title("Robot heading: normal vs IMU attack")
ax1.axhline(0, color="gray", lw=0.5)
ax1.legend()
ax1.grid(True, alpha=0.3)

fig.tight_layout()
fig.savefig("imu_attack_plot.png", dpi=120)
print("saved imu_attack_plot.png")
