#!/usr/bin/env python3
"""
plot_steps.py — Visualize step detection from a demo CSV.

Reads a CSV produced by step_demo.py and plots:
  1. Accel magnitude waveform with threshold line + detected steps
  2. Individual ax, ay, az components
  3. Cumulative step count over time

Usage:
    python3 plot_steps.py                          # plots most recent demo_*.csv
    python3 plot_steps.py path/to/demo_file.csv    # plots specific file
"""

import sys
import os
import glob
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

STEP_THRESHOLD = 11.0  # must match step_demo.py

# ── Find CSV file ─────────────────────────────────────────
if len(sys.argv) > 1:
    csv_path = sys.argv[1]
else:
    # Find the most recent demo_*.csv in trials/
    search_dirs = [
        os.path.expanduser("~/wyr/trials"),
        os.path.join(os.path.dirname(__file__), "..", "data"),
    ]
    candidates = []
    for d in search_dirs:
        candidates.extend(glob.glob(os.path.join(d, "demo_*.csv")))
        candidates.extend(glob.glob(os.path.join(d, "live_*.csv")))
    if not candidates:
        print("No demo_*.csv found. Run step_demo.py first, or pass a path.")
        sys.exit(1)
    csv_path = max(candidates, key=os.path.getmtime)

print("Plotting: {}".format(csv_path))

# ── Load data ─────────────────────────────────────────────
data = np.genfromtxt(csv_path, delimiter=",", names=True)
t = data["time_s"]
ax = data["ax"]
ay = data["ay"]
az = data["az"]
mag = data["mag"]
steps = data["step"]

step_mask = steps == 1
step_times = t[step_mask]
step_mags = mag[step_mask]
cumulative_steps = np.cumsum(steps)

# ── Plot ──────────────────────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True,
                         gridspec_kw={"height_ratios": [3, 2, 1.5]})

# --- Panel 1: Accel magnitude + threshold + steps ---
ax1 = axes[0]
ax1.plot(t, mag, color="#4A90D9", linewidth=0.6, alpha=0.9, label="Accel magnitude")
ax1.axhline(y=STEP_THRESHOLD, color="#E74C3C", linestyle="--", linewidth=1.2,
            label="Threshold ({} m/s²)".format(STEP_THRESHOLD))
ax1.scatter(step_times, np.full_like(step_times, STEP_THRESHOLD), color="#E74C3C",
            s=50, zorder=5, edgecolors="white", linewidths=0.5, label="Step detected")

# Shade above threshold
ax1.fill_between(t, STEP_THRESHOLD, mag,
                 where=(mag > STEP_THRESHOLD),
                 color="#E74C3C", alpha=0.1)

ax1.set_ylabel("Acceleration (m/s²)")
ax1.set_title("IMU Step Detection Demo — {} steps detected".format(int(steps.sum())),
              fontsize=13, fontweight="bold")
ax1.legend(loc="upper right", fontsize=9)
ax1.set_ylim(bottom=0)
ax1.grid(True, alpha=0.3)

# Annotate gravity baseline
ax1.axhline(y=9.81, color="gray", linestyle=":", linewidth=0.8, alpha=0.5)
ax1.annotate("gravity (9.81 m/s²)", xy=(t[-1], 9.81),
             fontsize=8, color="gray", ha="right", va="bottom")

# --- Panel 2: Individual axes ---
ax2 = axes[1]
ax2.plot(t, ax, linewidth=0.5, alpha=0.8, label="ax")
ax2.plot(t, ay, linewidth=0.5, alpha=0.8, label="ay")
ax2.plot(t, az, linewidth=0.5, alpha=0.8, label="az")
ax2.set_ylabel("Acceleration (m/s²)")
ax2.legend(loc="upper right", fontsize=9, ncol=3)
ax2.grid(True, alpha=0.3)

# --- Panel 3: Cumulative steps ---
ax3 = axes[2]
ax3.fill_between(t, 0, cumulative_steps, color="#2ECC71", alpha=0.3)
ax3.plot(t, cumulative_steps, color="#27AE60", linewidth=1.5)
ax3.set_ylabel("Steps")
ax3.set_xlabel("Time (s)")
ax3.grid(True, alpha=0.3)

# Add step rate annotation
duration = t[-1] - t[0]
total_steps = int(steps.sum())
if duration > 0 and total_steps > 0:
    rate = total_steps / duration * 60
    ax3.annotate("{} steps in {:.0f}s ({:.0f} steps/min)".format(
        total_steps, duration, rate),
        xy=(0.02, 0.85), xycoords="axes fraction", fontsize=9,
        bbox=dict(boxstyle="round,pad=0.3", facecolor="#2ECC71", alpha=0.2))

plt.tight_layout()

# Save next to the CSV
out_path = csv_path.replace(".csv", ".png")
plt.savefig(out_path, dpi=150, bbox_inches="tight")
print("Saved: {}".format(out_path))
plt.show()
