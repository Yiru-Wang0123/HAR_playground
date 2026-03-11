#!/usr/bin/env python3
"""
live_steps.py — Real-time step detection visualization.

Shows a scrolling accelerometer magnitude waveform with threshold line
and step detections highlighted. Designed for in-class demo.

Requires a display (run on Pi with monitor, or ssh -Y with XQuartz).
Stop hr_distance.service first:  sudo systemctl stop hr_distance.service

Usage:
    python3 live_steps.py             # default 10s window
    python3 live_steps.py --window 20 # 20s scrolling window
"""

import argparse
import math
import time
import os
import csv
import collections
from datetime import datetime

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from bmi270v2 import BMI270
from qwiic_oled import qwiic_oled_display

# ── Args ──────────────────────────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument("--window", type=float, default=10.0,
                    help="Scrolling window width in seconds (default 10)")
args = parser.parse_args()

# ── Configuration ─────────────────────────────────────────
IMU_ADDR       = 0x68
IMU_POLL_HZ    = 50
STEP_THRESHOLD = 11.0
STEP_DEBOUNCE  = 0.3
WINDOW_SEC     = args.window
BUF_SIZE       = int(WINDOW_SEC * IMU_POLL_HZ)
TRIALS_DIR     = os.path.expanduser("~/wyr/trials")

# ── Init CSV ──────────────────────────────────────────────
os.makedirs(TRIALS_DIR, exist_ok=True)
stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
csv_path = os.path.join(TRIALS_DIR, "live_{}.csv".format(stamp))
csv_file = open(csv_path, "w", newline="")
csv_writer = csv.writer(csv_file)
csv_writer.writerow(["timestamp", "time_s", "ax", "ay", "az", "mag", "step"])
print("[CSV] Logging to {}".format(csv_path))

# ── Init IMU ──────────────────────────────────────────────
imu = BMI270(IMU_ADDR)
imu.load_config_file()
imu.set_mode("performance")
imu.set_acc_range(0x00)  # 2G
print("[IMU] BMI270 ready")

# ── Init OLED ─────────────────────────────────────────────
oled = qwiic_oled_display.QwiicOledDisplay()
if oled.connected:
    oled.begin()
    oled.clear(oled.ALL)
    oled.set_font_type(1)
    oled.set_cursor(0, 0)
    oled.print("Step Demo")
    oled.display()
    print("[OLED] Initialized")
else:
    oled = None
    print("[OLED] Not connected, skipping")

OLED_UPDATE_SEC = 0.1
last_oled_update = 0.0

# ── Ring buffers ──────────────────────────────────────────
t_buf   = collections.deque(maxlen=BUF_SIZE)
mag_buf = collections.deque(maxlen=BUF_SIZE)
ax_buf  = collections.deque(maxlen=BUF_SIZE)
ay_buf  = collections.deque(maxlen=BUF_SIZE)
az_buf  = collections.deque(maxlen=BUF_SIZE)

# Step markers (time, magnitude) — only keep those in the visible window
step_t   = []
step_mag = []

# Step detection state
step_count     = 0
last_step_time = 0.0
was_above      = False
t_start        = time.time()

# ── Set up figure ─────────────────────────────────────────
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 6), sharex=True,
                                gridspec_kw={"height_ratios": [3, 2]})
fig.suptitle("Live Step Detection", fontsize=14, fontweight="bold")

# Panel 1: magnitude + threshold
line_mag, = ax1.plot([], [], color="#4A90D9", linewidth=0.8)
line_thresh = ax1.axhline(y=STEP_THRESHOLD, color="#E74C3C", linestyle="--",
                           linewidth=1.2, label="Threshold ({} m/s²)".format(STEP_THRESHOLD))
line_gravity = ax1.axhline(y=9.81, color="gray", linestyle=":", linewidth=0.8, alpha=0.5)
scatter_steps = ax1.scatter([], [], color="#E74C3C", s=60, zorder=5,
                             edgecolors="white", linewidths=0.5)
fill_above = None
step_text = ax1.text(0.02, 0.92, "Steps: 0", transform=ax1.transAxes,
                      fontsize=13, fontweight="bold", va="top",
                      bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8))
rate_text = ax1.text(0.98, 0.92, "", transform=ax1.transAxes,
                      fontsize=11, ha="right", va="top",
                      bbox=dict(boxstyle="round,pad=0.3", facecolor="#2ECC71", alpha=0.2))

ax1.set_ylabel("Acceleration (m/s²)")
ax1.set_ylim(0, 25)
ax1.legend(loc="upper center", fontsize=9)
ax1.grid(True, alpha=0.3)

# Panel 2: individual axes
line_ax, = ax2.plot([], [], linewidth=0.5, alpha=0.8, label="ax")
line_ay, = ax2.plot([], [], linewidth=0.5, alpha=0.8, label="ay")
line_az, = ax2.plot([], [], linewidth=0.5, alpha=0.8, label="az")
ax2.set_ylabel("Acceleration (m/s²)")
ax2.set_xlabel("Time (s)")
ax2.set_ylim(-20, 20)
ax2.legend(loc="upper right", fontsize=9, ncol=3)
ax2.grid(True, alpha=0.3)

plt.tight_layout()

# ── Animation update ──────────────────────────────────────
def update(frame):
    global was_above, last_step_time, step_count, fill_above

    # Read a batch of samples to keep up with real time
    for _ in range(3):
        t_now = time.time()
        elapsed = t_now - t_start

        try:
            acc = imu.get_acc_data()
        except (OSError, IOError):
            return []

        axv, ayv, azv = float(acc[0]), float(acc[1]), float(acc[2])
        mag = math.sqrt(axv**2 + ayv**2 + azv**2)

        t_buf.append(elapsed)
        mag_buf.append(mag)
        ax_buf.append(axv)
        ay_buf.append(ayv)
        az_buf.append(azv)

        # Step detection
        is_above = mag > STEP_THRESHOLD
        step_flag = 0
        if is_above and not was_above:
            if (t_now - last_step_time) >= STEP_DEBOUNCE:
                step_count += 1
                last_step_time = t_now
                step_t.append(elapsed)
                step_mag.append(mag)
                step_flag = 1
        was_above = is_above

        # Log to CSV
        csv_writer.writerow([
            datetime.now().isoformat(timespec="milliseconds"),
            round(elapsed, 4),
            round(axv, 3), round(ayv, 3), round(azv, 3),
            round(mag, 3), step_flag,
        ])

        time.sleep(1.0 / IMU_POLL_HZ)

    if len(t_buf) == 0:
        return []

    t_list   = list(t_buf)
    mag_list = list(mag_buf)

    # Scrolling x-axis
    t_max = t_list[-1]
    t_min = max(0, t_max - WINDOW_SEC)
    ax1.set_xlim(t_min, t_max)
    ax2.set_xlim(t_min, t_max)

    # Update magnitude line
    line_mag.set_data(t_list, mag_list)

    # Update fill above threshold
    if fill_above is not None:
        fill_above.remove()
    t_arr = np.array(t_list)
    mag_arr = np.array(mag_list)
    fill_above = ax1.fill_between(t_arr, STEP_THRESHOLD, mag_arr,
                                   where=(mag_arr > STEP_THRESHOLD),
                                   color="#E74C3C", alpha=0.1)

    # Prune old step markers outside window
    while step_t and step_t[0] < t_min:
        step_t.pop(0)
        step_mag.pop(0)

    if step_t:
        scatter_steps.set_offsets(list(zip(step_t, [STEP_THRESHOLD] * len(step_t))))
    else:
        scatter_steps.set_offsets(np.empty((0, 2)))

    # Update individual axes
    line_ax.set_data(t_list, list(ax_buf))
    line_ay.set_data(t_list, list(ay_buf))
    line_az.set_data(t_list, list(az_buf))

    # Update text
    step_text.set_text("Steps: {}".format(step_count))
    elapsed_total = t_list[-1]
    rate = 0
    if elapsed_total > 0 and step_count > 0:
        rate = step_count / elapsed_total * 60
        rate_text.set_text("{:.0f} steps/min".format(rate))

    # Update OLED
    global last_oled_update
    t_now_oled = time.time()
    if oled is not None and (t_now_oled - last_oled_update) >= OLED_UPDATE_SEC:
        last_oled_update = t_now_oled
        oled.clear(oled.PAGE)
        oled.set_cursor(0, 0)
        oled.print("Steps: {}".format(step_count))
        oled.set_cursor(0, 16)
        oled.print("{:.0f} stp/min".format(rate))
        oled.display()

    return [line_mag, scatter_steps, line_ax, line_ay, line_az,
            step_text, rate_text]


# ── Run ───────────────────────────────────────────────────
print("[LIVE] Window: {}s | Ctrl+C or close window to stop".format(WINDOW_SEC))
ani = animation.FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)

try:
    plt.show()
except KeyboardInterrupt:
    pass

csv_file.close()
if oled is not None:
    oled.clear(oled.PAGE)
    oled.set_cursor(0, 0)
    oled.print("Done!")
    oled.set_cursor(0, 16)
    oled.print("{} steps".format(step_count))
    oled.display()
print("[CSV] Saved to {}".format(csv_path))
print("[LIVE] Done — {} steps total".format(step_count))
