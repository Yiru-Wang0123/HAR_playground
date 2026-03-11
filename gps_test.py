#!/usr/bin/env python3
"""
gps_test.py — Read GPS + PulseSensor data from Arduino bridge over USB serial.

The Arduino reads the MAX-M10S over I2C and PulseSensor on A0,
forwarding raw NMEA + $PULSE lines over USB serial at 115200 baud.

Hardware: Arduino connected to Pi via USB cable (/dev/ttyACM0).
    GPS SDA → Arduino A4
    GPS SCL → Arduino A5
    GPS 3.3V → Arduino 3.3V
    GPS GND → Arduino GND
    PulseSensor Signal → Arduino A0
    PulseSensor VCC → Arduino 5V or 3.3V
    PulseSensor GND → Arduino GND
"""

import serial
import time

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
POLL_INTERVAL = 1.0


def validate_nmea(line):
    """Check that an NMEA sentence has a valid checksum."""
    if not line.startswith("$") or "*" not in line:
        return False
    body = line[1:line.index("*")]
    expected = line[line.index("*") + 1:].strip()
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    try:
        return cs == int(expected, 16)
    except ValueError:
        return False


def parse_gga(fields):
    """Parse $GNGGA or $GPGGA — position, altitude, satellite count."""
    if len(fields) < 15:
        return None

    lat_raw = fields[2]
    lat_dir = fields[3]
    lon_raw = fields[4]
    lon_dir = fields[5]
    fix_quality = fields[6]
    num_sats = fields[7]
    alt = fields[9]

    if not lat_raw or not lon_raw or fix_quality == "0":
        return {"fix": False, "sats": int(num_sats) if num_sats else 0}

    lat_deg = int(lat_raw[:2]) + float(lat_raw[2:]) / 60.0
    if lat_dir == "S":
        lat_deg = -lat_deg

    lon_deg = int(lon_raw[:3]) + float(lon_raw[3:]) / 60.0
    if lon_dir == "W":
        lon_deg = -lon_deg

    return {
        "fix": True,
        "sats": int(num_sats) if num_sats else 0,
        "lat": lat_deg,
        "lon": lon_deg,
        "alt_m": float(alt) if alt else 0.0,
    }


def parse_rmc(fields):
    """Parse $GNRMC or $GPRMC — speed and heading."""
    if len(fields) < 9:
        return None

    status = fields[2]
    speed_knots = fields[7]
    heading = fields[8]

    if status != "A":
        return None

    return {
        "speed_ms": float(speed_knots) * 0.514444 if speed_knots else 0.0,
        "heading": float(heading) if heading else 0.0,
    }


def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
    except serial.SerialException as e:
        print("[GPS] Cannot open {}: {}".format(SERIAL_PORT, e))
        print("[GPS] Is the Arduino plugged in? Try: ls /dev/ttyACM*")
        return

    print("[GPS] Listening on {} at {} baud".format(SERIAL_PORT, BAUD_RATE))
    print("[GPS] Waiting for data from Arduino bridge...\n")

    time.sleep(2)
    ser.reset_input_buffer()

    latest_gga = None
    latest_rmc = None
    last_print = 0

    while True:
        try:
            raw = ser.readline()
            if not raw:
                continue

            line = raw.decode("ascii", errors="ignore").strip()
            if not line:
                continue

            if line.startswith("[GPS]"):
                print(line)
                continue

            if not validate_nmea(line):
                continue

            fields = line[1:line.index("*")].split(",")
            msg_type = fields[0]

            if msg_type in ("GNGGA", "GPGGA"):
                latest_gga = parse_gga(fields)

            elif msg_type in ("GNRMC", "GPRMC"):
                latest_rmc = parse_rmc(fields)

            now = time.time()
            if now - last_print >= POLL_INTERVAL:
                last_print = now

                if latest_gga and latest_gga.get("fix"):
                    speed = latest_rmc["speed_ms"] if latest_rmc else 0.0
                    heading = latest_rmc["heading"] if latest_rmc else 0.0
                    print("  Sats: {}  Lat: {:.6f}  Lon: {:.6f}  "
                          "Alt: {:.1f}m  Speed: {:.1f} m/s  "
                          "Heading: {:.1f}°".format(
                              latest_gga["sats"],
                              latest_gga["lat"],
                              latest_gga["lon"],
                              latest_gga["alt_m"],
                              speed, heading))
                elif latest_gga:
                    print("  Searching... Sats: {}".format(
                        latest_gga.get("sats", 0)))
                else:
                    print("  No NMEA data yet...")

        except KeyboardInterrupt:
            print("\n[GPS] Stopped")
            break
        except Exception as e:
            print("[GPS] Error: {}".format(e))

    ser.close()


if __name__ == "__main__":
    main()
