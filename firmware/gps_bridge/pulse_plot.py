#!/usr/bin/env python3
"""Live serial plot of PulseSensor BPM from Arduino bridge."""

import serial
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

PORT = "/dev/cu.usbmodem11301"
BAUD = 115200
WINDOW = 60  # seconds of data to show

bpm_times = deque(maxlen=300)
bpm_vals = deque(maxlen=300)
start_time = None

fig, ax = plt.subplots(figsize=(10, 4))
line, = ax.plot([], [], "r-o", markersize=3, linewidth=1.5)
ax.set_xlabel("Time (s)")
ax.set_ylabel("BPM")
ax.set_title("PulseSensor Live BPM")
ax.set_ylim(40, 180)
ax.grid(True, alpha=0.3)

ser = serial.Serial(PORT, BAUD, timeout=2)
time.sleep(2)
ser.reset_input_buffer()
print("Listening for $PULSE lines... place finger on sensor.")


def update(frame):
    global start_time

    # Read all available lines
    while ser.in_waiting:
        try:
            raw = ser.readline()
            line_str = raw.decode("ascii", errors="ignore").strip()

            if line_str.startswith("$PULSE,"):
                parts = line_str.split(",")
                bpm = int(parts[1])
                if 30 < bpm < 220:
                    if start_time is None:
                        start_time = time.time()
                    t = time.time() - start_time
                    bpm_times.append(t)
                    bpm_vals.append(bpm)
                    print("  BPM: {}".format(bpm))

            elif line_str.startswith("["):
                print(line_str)

        except Exception:
            pass

    if bpm_times:
        line.set_data(list(bpm_times), list(bpm_vals))
        ax.set_xlim(max(0, bpm_times[-1] - WINDOW), bpm_times[-1] + 2)
        if bpm_vals:
            lo = max(40, min(bpm_vals) - 10)
            hi = min(220, max(bpm_vals) + 10)
            ax.set_ylim(lo, hi)

    return (line,)


ani = animation.FuncAnimation(fig, update, interval=200, blit=False, cache_frame_data=False)

try:
    plt.tight_layout()
    plt.show()
except KeyboardInterrupt:
    pass
finally:
    ser.close()
    print("\nDone.")
