#!/usr/bin/env python3
"""
step_demo.py — Record raw accelerometer at 50 Hz for step detection demo.

Logs every IMU sample (not just step events) so the full waveform can be
visualized afterward with plot_steps.py.

Usage:
    python3 step_demo.py [seconds]    # default 30s
    python3 step_demo.py 60           # record for 60 seconds

Output: ~/wyr/trials/demo_YYYYMMDD_HHMMSS.csv
"""

import sys
import os
import csv
import time
import math
from datetime import datetime

from bmi270v2 import BMI270

# ── Configuration ─────────────────────────────────────────
IMU_ADDR       = 0x68
IMU_POLL_HZ    = 50
STEP_THRESHOLD = 11.0   # m/s² (~1.12g)
STEP_DEBOUNCE  = 0.3    # seconds
TRIALS_DIR     = os.path.expanduser("~/wyr/trials")
DURATION       = int(sys.argv[1]) if len(sys.argv) > 1 else 30

# ── Init IMU ──────────────────────────────────────────────
imu = BMI270(IMU_ADDR)
imu.load_config_file()
imu.set_mode("performance")
imu.set_acc_range(0x00)  # 2G
print("[IMU] BMI270 ready")

# ── Init CSV ──────────────────────────────────────────────
os.makedirs(TRIALS_DIR, exist_ok=True)
stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
csv_path = os.path.join(TRIALS_DIR, "demo_{}.csv".format(stamp))
f = open(csv_path, "w", newline="")
writer = csv.writer(f)
writer.writerow(["time_s", "ax", "ay", "az", "mag", "step"])
print("[REC] Recording {} seconds to {}".format(DURATION, csv_path))

# ── Record ────────────────────────────────────────────────
poll_interval = 1.0 / IMU_POLL_HZ
step_count = 0
last_step_time = 0.0
was_above = False
t_start = time.time()

try:
    while True:
        t_now = time.time()
        elapsed = t_now - t_start
        if elapsed >= DURATION:
            break

        acc = imu.get_acc_data()
        ax, ay, az = float(acc[0]), float(acc[1]), float(acc[2])
        mag = math.sqrt(ax**2 + ay**2 + az**2)

        # Step detection (same logic as hr_distance.py)
        is_above = mag > STEP_THRESHOLD
        step_flag = 0
        if is_above and not was_above:
            if (t_now - last_step_time) >= STEP_DEBOUNCE:
                step_count += 1
                last_step_time = t_now
                step_flag = 1
        was_above = is_above

        writer.writerow([
            round(elapsed, 4),
            round(ax, 3), round(ay, 3), round(az, 3),
            round(mag, 3), step_flag,
        ])

        # Maintain poll rate
        dt = time.time() - t_now
        sleep_time = poll_interval - dt
        if sleep_time > 0:
            time.sleep(sleep_time)

except KeyboardInterrupt:
    print("\n[REC] Stopped early")

f.close()
print("[REC] Done — {} steps in {:.1f}s".format(step_count, time.time() - t_start))
print("[REC] Saved to {}".format(csv_path))
