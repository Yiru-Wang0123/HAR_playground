#!/usr/bin/env python3
"""
hr_distance.py — Multi-sensor HAR data logger with OLED Display

Sensors:
    MAX32664 Biometric Sensor Hub (0x55) — HR/SpO2 via I2C on Pi
    BMI270 IMU (0x68) — step detection via I2C on Pi
    OLED display (0x3c) — status display via I2C on Pi
    Arduino bridge (USB serial) — GPS (MAX-M10S) + PulseSensor

Hardware (I2C bus 1):
    0x68  BMI270 6DoF IMU
    0x55  MAX32664 Biometric Sensor Hub (with MAX30101 inside)
    0x3c  OLED display (128x32)

Arduino bridge (/dev/ttyACM0 at 115200 baud):
    GPS NMEA sentences (position, speed, heading)
    $PULSE,bpm,ibi lines (PulseSensor analog HR)

GPIO (BCM numbering, via Qwiic HAT breakout):
    GPIO 16 (D16) → MAX32664 RST (green)
    GPIO 6  (D6)  → MAX32664 MFIO (blue)

Designed to run headless on boot via systemd.
"""

import threading
import time
import math
import sys
import os
import csv
from datetime import datetime
import serial
import RPi.GPIO as GPIO
from smbus2 import SMBus, i2c_msg

from bmi270v2 import BMI270
from qwiic_oled import qwiic_oled_display

# ── Configuration ──────────────────────────────────────────
IMU_ADDR        = 0x68
HR_ADDR         = 0x55   # MAX32664 Biometric Sensor Hub
RESET_PIN       = 16     # BCM GPIO 16 (HAT breakout D16)
MFIO_PIN        = 6      # BCM GPIO 6  (HAT breakout D6)
IMU_POLL_HZ     = 50     # accelerometer polling rate
STEP_THRESHOLD  = 11.0   # m/s² magnitude to register a step (~1.12g)
STEP_DEBOUNCE   = 0.3    # min seconds between steps
STRIDE_LENGTH   = 0.75   # meters per step
OLED_UPDATE_SEC = 0.5    # OLED refresh interval
TRIALS_DIR      = os.path.expanduser("~/wyr/trials")

# Arduino bridge (GPS + PulseSensor)
BRIDGE_PORT     = "/dev/ttyACM0"
BRIDGE_BAUD     = 115200

# MAX32664 timing
CMD_DELAY       = 0.006  # 6 ms — standard command delay
ENABLE_DELAY    = 0.045  # 45 ms — enable command delay

# ── Shared state between threads ───────────────────────────
lock = threading.Lock()
shared = {
    "hr": 0,
    "confidence": 0,
    "spo2": 0.0,
    "steps": 0,
    "distance_km": 0.0,
    "running": True,
    # GPS
    "lat": 0.0,
    "lon": 0.0,
    "alt_m": 0.0,
    "speed_ms": 0.0,
    "sats": 0,
    "gps_fix": False,
    "gps_distance_km": 0.0,
    # PulseSensor
    "pulse_bpm": 0,
    "pulse_ibi": 0,
}

# ── CSV logging ────────────────────────────────────────────
csv_lock = threading.Lock()
csv_writer = None
csv_file = None

def log_row(event, hr, confidence, spo2, steps, distance_km,
            ax=0, ay=0, az=0, lat=0, lon=0, alt_m=0, speed_ms=0,
            sats=0, pulse_bpm=0, pulse_ibi=0, gps_distance_km=0):
    """Write one row to the CSV immediately. Thread-safe."""
    with csv_lock:
        if csv_writer is not None:
            csv_writer.writerow([
                datetime.now().isoformat(timespec="milliseconds"),
                event, hr, confidence, spo2, steps,
                round(distance_km, 4), round(ax, 3), round(ay, 3), round(az, 3),
                round(lat, 7), round(lon, 7), round(alt_m, 1),
                round(speed_ms, 2), sats, pulse_bpm, pulse_ibi,
                round(gps_distance_km, 4),
            ])
            csv_file.flush()


# ── MAX32664 Biometric Sensor Hub Driver ───────────────────
class MAX32664:
    """
    Minimal driver for SparkFun Pulse Oximeter (MAX32664 + MAX30101).
    Protocol based on SparkFun_Bio_Sensor_Hub_Library (Arduino).

    The MAX32664 is a command-based I2C device — you send command bytes,
    wait for processing, then read a status byte + response data.
    It processes raw MAX30101 PPG data on-chip and returns HR/SpO2.
    """

    def __init__(self, address=0x55, reset_pin=16, mfio_pin=6):
        self.address = address
        self.reset_pin = reset_pin
        self.mfio_pin = mfio_pin
        self.bus = SMBus(1)

    def begin(self):
        """Reset sensor hub into application mode and configure for HR+SpO2."""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.reset_pin, GPIO.OUT)
        GPIO.setup(self.mfio_pin, GPIO.OUT)

        # Reset sequence: MFIO=HIGH during reset selects application mode
        GPIO.output(self.mfio_pin, GPIO.HIGH)
        GPIO.output(self.reset_pin, GPIO.LOW)
        time.sleep(0.01)
        GPIO.output(self.reset_pin, GPIO.HIGH)
        time.sleep(1.0)

        # Release MFIO — hub drives it as interrupt output
        GPIO.setup(self.mfio_pin, GPIO.IN)

        # Configure BPM mode (MODE_ONE: WHRM + SpO2)
        # 1. Output mode = algorithm data only
        s = self._write_cmd(0x10, 0x00, 0x02, CMD_DELAY)
        print("[MAX32664] setOutputMode: 0x{:02x}".format(s))

        # 2. FIFO interrupt threshold = 1 sample
        s = self._write_cmd(0x10, 0x01, 0x01, CMD_DELAY)
        print("[MAX32664] setFifoThresh: 0x{:02x}".format(s))

        # 3. Enable AGC (automatic gain control) algorithm
        s = self._write_cmd(0x52, 0x00, 0x01, ENABLE_DELAY)
        print("[MAX32664] enableAGC: 0x{:02x}".format(s))

        # 4. Enable MAX30101 sensor
        s = self._write_cmd(0x44, 0x03, 0x01, ENABLE_DELAY)
        print("[MAX32664] enableSensor: 0x{:02x}".format(s))

        # 5. Enable WHRM algorithm (mode 1 = continuous HR + SpO2)
        s = self._write_cmd(0x52, 0x02, 0x01, ENABLE_DELAY)
        print("[MAX32664] enableWHRM: 0x{:02x}".format(s))

        return s == 0x00

    def read_bpm(self):
        """
        Read processed HR/SpO2 from the sensor hub.
        Returns (hr, confidence, spo2, algo_status).
        HR/SpO2 are 0 if no data available.
        """
        # Ask how many samples are in the FIFO
        self._i2c_write([0x12, 0x00])
        time.sleep(CMD_DELAY)
        resp = self._i2c_read(2)  # [status, numSamples]
        num_samples = resp[1]

        if num_samples == 0:
            return 0, 0, 0.0, 0

        # Read algorithm data: 6 bytes for MODE_ONE
        self._i2c_write([0x12, 0x01])
        time.sleep(CMD_DELAY)
        resp = self._i2c_read(7)  # [status, hr_msb, hr_lsb, conf, spo2_msb, spo2_lsb, status]

        hr = ((resp[1] << 8) | resp[2]) / 10.0
        confidence = resp[3]
        spo2 = ((resp[4] << 8) | resp[5]) / 10.0
        algo_status = resp[6]

        return hr, confidence, spo2, algo_status

    def _write_cmd(self, family, index, data, delay):
        """Send a 3-byte command, wait, read 1-byte status."""
        self._i2c_write([family, index, data])
        time.sleep(delay)
        resp = self._i2c_read(1)
        return resp[0]

    def _i2c_write(self, data):
        """Raw I2C write (no SMBus register framing)."""
        msg = i2c_msg.write(self.address, data)
        self.bus.i2c_rdwr(msg)

    def _i2c_read(self, length):
        """Raw I2C read (no SMBus register framing)."""
        msg = i2c_msg.read(self.address, length)
        self.bus.i2c_rdwr(msg)
        return list(msg)


# ── HR Thread ──────────────────────────────────────────────
def hr_thread_func():
    """Background thread: init MAX32664 and poll HR every ~1 second."""
    try:
        sensor = MAX32664(address=HR_ADDR, reset_pin=RESET_PIN, mfio_pin=MFIO_PIN)
        ok = sensor.begin()
        if not ok:
            print("[HR] MAX32664 init returned non-zero status — continuing anyway")
        print("[HR] Sensor hub ready, waiting for finger...")
    except Exception as e:
        print("[HR] Failed to init MAX32664: {}".format(e))
        return

    while True:
        with lock:
            if not shared["running"]:
                break

        try:
            hr, confidence, spo2, status = sensor.read_bpm()

            with lock:
                if hr > 0:
                    shared["hr"] = int(hr)
                    shared["confidence"] = confidence
                if spo2 > 0:
                    shared["spo2"] = spo2
                snap_steps = shared["steps"]
                snap_dist = shared["distance_km"]
                snap_lat = shared["lat"]
                snap_lon = shared["lon"]
                snap_alt = shared["alt_m"]
                snap_spd = shared["speed_ms"]
                snap_sats = shared["sats"]
                snap_pulse = shared["pulse_bpm"]
                snap_ibi = shared["pulse_ibi"]
                snap_gps_dist = shared["gps_distance_km"]

            if hr > 0:
                print("[HR] {} BPM (conf:{}%) SpO2:{:.0f}%".format(
                    int(hr), confidence, spo2))
                log_row("hr", int(hr), confidence, spo2, snap_steps, snap_dist,
                        lat=snap_lat, lon=snap_lon, alt_m=snap_alt,
                        speed_ms=snap_spd, sats=snap_sats,
                        pulse_bpm=snap_pulse, pulse_ibi=snap_ibi,
                        gps_distance_km=snap_gps_dist)

        except Exception as e:
            print("[HR] Read error: {}".format(e))

        time.sleep(1.0)

    print("[HR] Thread stopped")


# ── Haversine ─────────────────────────────────────────────
def haversine_m(lat1, lon1, lat2, lon2):
    """Distance in meters between two lat/lon points."""
    R = 6371000.0
    rlat1, rlat2 = math.radians(lat1), math.radians(lat2)
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2 +
         math.cos(rlat1) * math.cos(rlat2) * math.sin(dlon / 2) ** 2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


# ── Arduino Bridge Thread (GPS + PulseSensor) ─────────────
def bridge_thread_func():
    """Background thread: read GPS NMEA + PulseSensor from Arduino USB serial."""
    try:
        ser = serial.Serial(BRIDGE_PORT, BRIDGE_BAUD, timeout=2)
        time.sleep(2)  # wait for Arduino reset
        ser.reset_input_buffer()
        print("[BRIDGE] Connected on {}".format(BRIDGE_PORT))
    except serial.SerialException as e:
        print("[BRIDGE] Cannot open {}: {} — GPS/Pulse disabled".format(BRIDGE_PORT, e))
        return

    # GPS distance accumulation state (local to this thread)
    prev_lat = None
    prev_lon = None
    gps_total_m = 0.0

    while True:
        with lock:
            if not shared["running"]:
                break

        try:
            raw = ser.readline()
            if not raw:
                continue

            line = raw.decode("ascii", errors="ignore").strip()
            if not line:
                continue

            # ── PulseSensor: $PULSE,bpm,ibi ──
            if line.startswith("$PULSE,"):
                parts = line.split(",")
                if len(parts) == 3:
                    try:
                        bpm = int(parts[1])
                        ibi = int(parts[2])
                        with lock:
                            shared["pulse_bpm"] = bpm
                            shared["pulse_ibi"] = ibi
                            snap_steps = shared["steps"]
                            snap_dist = shared["distance_km"]
                            snap_hr = shared["hr"]
                            snap_conf = shared["confidence"]
                            snap_spo2 = shared["spo2"]
                            snap_lat = shared["lat"]
                            snap_lon = shared["lon"]
                            snap_alt = shared["alt_m"]
                            snap_spd = shared["speed_ms"]
                            snap_sats = shared["sats"]
                            snap_gps_dist = shared["gps_distance_km"]
                        if bpm > 0 and bpm < 220:
                            print("[PULSE] {} BPM (IBI: {} ms)".format(bpm, ibi))
                            log_row("pulse", snap_hr, snap_conf, snap_spo2,
                                    snap_steps, snap_dist,
                                    lat=snap_lat, lon=snap_lon, alt_m=snap_alt,
                                    speed_ms=snap_spd, sats=snap_sats,
                                    pulse_bpm=bpm, pulse_ibi=ibi,
                                    gps_distance_km=snap_gps_dist)
                    except ValueError:
                        pass
                continue

            # ── GPS NMEA: validate checksum ──
            if not line.startswith("$") or "*" not in line:
                continue
            body = line[1:line.index("*")]
            expected = line[line.index("*") + 1:].strip()
            cs = 0
            for ch in body:
                cs ^= ord(ch)
            try:
                if cs != int(expected, 16):
                    continue
            except ValueError:
                continue

            fields = body.split(",")
            msg_type = fields[0]

            # ── GGA: position + altitude + satellites ──
            if msg_type in ("GNGGA", "GPGGA") and len(fields) >= 15:
                lat_raw = fields[2]
                lat_dir = fields[3]
                lon_raw = fields[4]
                lon_dir = fields[5]
                fix_q = fields[6]
                num_sats = fields[7]
                alt = fields[9]

                if lat_raw and lon_raw and fix_q != "0":
                    lat_deg = int(lat_raw[:2]) + float(lat_raw[2:]) / 60.0
                    if lat_dir == "S":
                        lat_deg = -lat_deg
                    lon_deg = int(lon_raw[:3]) + float(lon_raw[3:]) / 60.0
                    if lon_dir == "W":
                        lon_deg = -lon_deg
                    alt_m = float(alt) if alt else 0.0
                    sats = int(num_sats) if num_sats else 0

                    # Accumulate GPS distance
                    if prev_lat is not None:
                        d = haversine_m(prev_lat, prev_lon, lat_deg, lon_deg)
                        if d > 1.0:  # ignore GPS jitter < 1m
                            gps_total_m += d
                    prev_lat = lat_deg
                    prev_lon = lon_deg

                    with lock:
                        shared["lat"] = lat_deg
                        shared["lon"] = lon_deg
                        shared["alt_m"] = alt_m
                        shared["sats"] = sats
                        shared["gps_fix"] = True
                        shared["gps_distance_km"] = gps_total_m / 1000.0

            # ── RMC: speed + heading ──
            elif msg_type in ("GNRMC", "GPRMC") and len(fields) >= 9:
                if fields[2] == "A":
                    spd_knots = fields[7]
                    speed_ms = float(spd_knots) * 0.514444 if spd_knots else 0.0

                    with lock:
                        shared["speed_ms"] = speed_ms
                        snap_steps = shared["steps"]
                        snap_dist = shared["distance_km"]
                        snap_hr = shared["hr"]
                        snap_conf = shared["confidence"]
                        snap_spo2 = shared["spo2"]
                        snap_lat = shared["lat"]
                        snap_lon = shared["lon"]
                        snap_alt = shared["alt_m"]
                        snap_sats = shared["sats"]
                        snap_pulse = shared["pulse_bpm"]
                        snap_ibi = shared["pulse_ibi"]
                        snap_gps_dist = shared["gps_distance_km"]

                    print("[GPS] Sats:{} Lat:{:.5f} Lon:{:.5f} Spd:{:.1f}m/s".format(
                        snap_sats, snap_lat, snap_lon, speed_ms))
                    log_row("gps", snap_hr, snap_conf, snap_spo2,
                            snap_steps, snap_dist,
                            lat=snap_lat, lon=snap_lon, alt_m=snap_alt,
                            speed_ms=speed_ms, sats=snap_sats,
                            pulse_bpm=snap_pulse, pulse_ibi=snap_ibi,
                            gps_distance_km=snap_gps_dist)

        except Exception as e:
            print("[BRIDGE] Error: {}".format(e))

    ser.close()
    print("[BRIDGE] Thread stopped")


# ── Main: IMU + OLED ───────────────────────────────────────
def main():
    global csv_writer, csv_file

    # ── Init CSV ──
    os.makedirs(TRIALS_DIR, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(TRIALS_DIR, "trial_{}.csv".format(stamp))
    csv_file = open(csv_path, "w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow([
        "timestamp", "event", "hr_bpm", "confidence", "spo2",
        "steps", "distance_km", "ax", "ay", "az",
        "lat", "lon", "alt_m", "speed_ms", "sats",
        "pulse_bpm", "pulse_ibi", "gps_distance_km",
    ])
    csv_file.flush()
    print("[CSV] Logging to {}".format(csv_path))

    # ── Init OLED ──
    oled = qwiic_oled_display.QwiicOledDisplay()
    if not oled.connected:
        print("[OLED] Not connected!")
        sys.exit(1)
    oled.begin()
    oled.clear(oled.ALL)
    oled.set_font_type(1)  # 8x16 font — 16 chars x 2 lines on 128x32
    oled.set_cursor(0, 0)
    oled.print("Starting...")
    oled.display()
    print("[OLED] Initialized (128x32)")

    # ── Init IMU (retry up to 5 times) ──
    imu = None
    for attempt in range(5):
        try:
            imu = BMI270(IMU_ADDR)
            imu.load_config_file()
            imu.set_mode("performance")
            imu.set_acc_range(0x00)  # 2G range
            print("[IMU] BMI270 ready")
            break
        except Exception as e:
            print("[IMU] Init attempt {}/5 failed: {}".format(attempt + 1, e))
            time.sleep(2)
    if imu is None:
        print("[IMU] Failed after 5 attempts — running without step detection")

    # ── Start HR background thread ──
    hr_t = threading.Thread(target=hr_thread_func, daemon=True)
    hr_t.start()

    # ── Start Arduino bridge thread (GPS + PulseSensor) ──
    bridge_t = threading.Thread(target=bridge_thread_func, daemon=True)
    bridge_t.start()

    # ── Step detection state ──
    step_count = 0
    last_step_time = 0.0
    was_above = False
    last_oled_update = 0.0
    poll_interval = 1.0 / IMU_POLL_HZ

    print("[MAIN] Running — Ctrl+C to stop")

    try:
        while True:
            t_now = time.time()

            # ── Read accelerometer + step detection ──
            if imu is not None:
                try:
                    acc = imu.get_acc_data()  # [ax, ay, az] in m/s²
                    mag = math.sqrt(
                        float(acc[0]) ** 2 + float(acc[1]) ** 2 + float(acc[2]) ** 2
                    )

                    # ── Step detection: rising-edge threshold crossing with debounce ──
                    is_above = mag > STEP_THRESHOLD
                    if is_above and not was_above:
                        if (t_now - last_step_time) >= STEP_DEBOUNCE:
                            step_count += 1
                            last_step_time = t_now
                            dist_km = step_count * STRIDE_LENGTH / 1000.0
                            with lock:
                                shared["steps"] = step_count
                                shared["distance_km"] = dist_km
                                snap_hr = shared["hr"]
                                snap_conf = shared["confidence"]
                                snap_spo2 = shared["spo2"]
                                snap_lat = shared["lat"]
                                snap_lon = shared["lon"]
                                snap_alt = shared["alt_m"]
                                snap_spd = shared["speed_ms"]
                                snap_sats = shared["sats"]
                                snap_pulse = shared["pulse_bpm"]
                                snap_ibi = shared["pulse_ibi"]
                                snap_gps_dist = shared["gps_distance_km"]
                            log_row("step", snap_hr, snap_conf, snap_spo2,
                                    step_count, dist_km,
                                    float(acc[0]), float(acc[1]), float(acc[2]),
                                    lat=snap_lat, lon=snap_lon, alt_m=snap_alt,
                                    speed_ms=snap_spd, sats=snap_sats,
                                    pulse_bpm=snap_pulse, pulse_ibi=snap_ibi,
                                    gps_distance_km=snap_gps_dist)
                    was_above = is_above
                except (OSError, IOError) as e:
                    print("[IMU] Read error: {} — retrying".format(e))
                    time.sleep(0.1)
                    continue

            # ── Update OLED ──
            if (t_now - last_oled_update) >= OLED_UPDATE_SEC:
                last_oled_update = t_now

                with lock:
                    hr_val = shared["hr"]
                    pulse_val = shared["pulse_bpm"]
                    steps = shared["steps"]
                    step_dist = shared["distance_km"]
                    gps_dist = shared["gps_distance_km"]
                    gps_fix = shared["gps_fix"]

                oled.clear(oled.PAGE)

                # Line 1: both heart rates
                oled.set_cursor(0, 0)
                hr_str = str(hr_val) if hr_val > 0 else "---"
                ps_str = str(pulse_val) if pulse_val > 0 else "---"
                oled.print("H:{} P:{}".format(hr_str, ps_str))

                # Line 2: G=GPS dist, S=step dist, step count
                oled.set_cursor(0, 16)
                oled.print("G{:.1f} S{:.1f} {}".format(
                    gps_dist, step_dist, steps))

                oled.display()

            # ── Maintain poll rate ──
            elapsed = time.time() - t_now
            sleep_time = poll_interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n[MAIN] Shutting down...")
    except Exception as e:
        print("[MAIN] Unexpected error: {}".format(e))
    finally:
        # ── Cleanup ──
        with lock:
            shared["running"] = False

        with lock:
            final_steps = shared["steps"]
            final_dist = shared["distance_km"]
            final_hr = shared["hr"]

        oled.clear(oled.PAGE)
        oled.set_cursor(0, 0)
        oled.print("Done {} BPM".format(final_hr))
        oled.set_cursor(0, 16)
        oled.print("{}stp {:.2f}km".format(final_steps, final_dist))
        oled.display()

        # ── Close CSV ──
        with csv_lock:
            if csv_file is not None:
                csv_file.close()
                print("[CSV] File saved")

        GPIO.cleanup()
        hr_t.join(timeout=5)
        bridge_t.join(timeout=5)
        print("[MAIN] Final: {} steps, {:.2f} km".format(final_steps, final_dist))


if __name__ == "__main__":
    main()
