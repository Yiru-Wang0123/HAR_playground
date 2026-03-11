#!/usr/bin/env python3
"""
pulse_test.py — Read PulseSensor BPM from Arduino bridge over USB serial.

The Arduino samples the PulseSensor on A0 at 500 Hz, detects beats,
and sends $PULSE,<bpm>,<ibi_ms> lines over USB at 115200 baud.

Hardware: Arduino connected to Pi via USB cable (/dev/ttyACM0).
    PulseSensor Signal → Arduino A0
    PulseSensor VCC → Arduino 5V or 3.3V
    PulseSensor GND → Arduino GND
"""

import serial
import time

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200


def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
    except serial.SerialException as e:
        print("[PULSE] Cannot open {}: {}".format(SERIAL_PORT, e))
        print("[PULSE] Is the Arduino plugged in? Try: ls /dev/ttyACM*")
        return

    print("[PULSE] Listening on {} at {} baud".format(SERIAL_PORT, BAUD_RATE))
    print("[PULSE] Waiting for heartbeat data...\n")

    time.sleep(2)
    ser.reset_input_buffer()

    while True:
        try:
            raw = ser.readline()
            if not raw:
                continue

            line = raw.decode("ascii", errors="ignore").strip()
            if not line:
                continue

            # Pass through status messages
            if line.startswith("[PULSE]"):
                print(line)
                continue

            # Parse $PULSE,bpm,ibi
            if not line.startswith("$PULSE,"):
                continue

            parts = line.split(",")
            if len(parts) != 3:
                continue

            try:
                bpm = int(parts[1])
                ibi = int(parts[2])
            except ValueError:
                continue

            print("  BPM: {}  IBI: {} ms".format(bpm, ibi))

        except KeyboardInterrupt:
            print("\n[PULSE] Stopped")
            break
        except Exception as e:
            print("[PULSE] Error: {}".format(e))

    ser.close()


if __name__ == "__main__":
    main()
