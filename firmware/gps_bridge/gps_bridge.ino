/*
 * gps_bridge.ino — Arduino sensor bridge for Raspberry Pi
 *
 * Reads two sensors and forwards data over USB serial at 115200 baud:
 *   1. u-blox MAX-M10S GNSS — raw NMEA sentences via I2C (0x42)
 *   2. PulseSensor — BPM via analog read on A0
 *
 * Output format:
 *   NMEA lines pass through as-is (start with '$')
 *   PulseSensor BPM: $PULSE,<bpm>,<ibi_ms>*\r\n
 *
 * Hardware:
 *   GPS SDA → Arduino A4
 *   GPS SCL → Arduino A5
 *   GPS 3.3V → Arduino 3.3V
 *   GPS GND → Arduino GND
 *   PulseSensor Signal → Arduino A0
 *   PulseSensor VCC → Arduino 5V (or 3.3V)
 *   PulseSensor GND → Arduino GND
 *   Arduino USB → Raspberry Pi
 */

#include <Wire.h>

// ── GPS config ──
#define GPS_ADDR   0x42
#define READ_CHUNK 32

// ── PulseSensor config ──
#define PULSE_PIN  A0
#define SAMPLE_INTERVAL_MS 2  // 500 Hz sampling

// ── PulseSensor state ──
volatile int bpm = 0;
volatile int ibi = 600;            // inter-beat interval (ms)
volatile bool pulse = false;        // true during a beat
volatile bool beatDetected = false; // flag for main loop

// Internal detection state
int rate[10];              // last 10 IBI values for averaging
unsigned long lastBeatTime = 0;
int peakVal = 512;
int troughVal = 512;
int thresh = 525;          // threshold for beat detection
int amp = 100;             // pulse waveform amplitude
bool firstBeat = true;
bool secondBeat = false;
unsigned long lastSampleTime = 0;

// ── BPM reporting ──
unsigned long lastBpmReport = 0;
#define BPM_REPORT_INTERVAL 1000  // report BPM every 1 second

void setup() {
    Serial.begin(115200);
    Wire.begin();
    analogReference(DEFAULT);

    // Check if GPS is present
    Wire.beginTransmission(GPS_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println("[GPS] Not found at 0x42 — check wiring");
    } else {
        Serial.println("[GPS] MAX-M10S ready");
    }

    Serial.println("[PULSE] PulseSensor on A0");

    // Init rate array
    for (int i = 0; i < 10; i++)
        rate[i] = 0;
}

void loop() {
    // ── Sample PulseSensor at 500 Hz ──
    unsigned long now = millis();
    if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
        lastSampleTime = now;
        samplePulse(now);
    }

    // ── Report BPM once per second ──
    if (now - lastBpmReport >= BPM_REPORT_INTERVAL) {
        lastBpmReport = now;
        if (bpm > 0 && bpm < 220) {
            Serial.print("$PULSE,");
            Serial.print(bpm);
            Serial.print(",");
            Serial.println(ibi);
        }
    }

    // ── Read GPS NMEA ──
    readGPS();
}

// ── PulseSensor beat detection ──
// Based on the PulseSensor Amped algorithm
void samplePulse(unsigned long sampleTime) {
    int signal = analogRead(PULSE_PIN);
    unsigned long elapsed = sampleTime - lastBeatTime;

    // Find trough of the waveform
    if (signal < thresh && elapsed > (unsigned long)(ibi / 5 * 3)) {
        if (signal < troughVal)
            troughVal = signal;
    }

    // Find peak of the waveform
    if (signal > thresh && signal > peakVal) {
        peakVal = signal;
    }

    // Look for a heartbeat — signal crosses threshold on the way up
    if (elapsed > 250) {  // avoid high frequency noise (max 240 BPM)
        if (signal > thresh && !pulse && elapsed > (unsigned long)(ibi / 5 * 3)) {
            pulse = true;
            ibi = sampleTime - lastBeatTime;
            lastBeatTime = sampleTime;

            if (secondBeat) {
                secondBeat = false;
                for (int i = 0; i < 10; i++)
                    rate[i] = ibi;
            }

            if (firstBeat) {
                firstBeat = false;
                secondBeat = true;
                return;  // skip first IBI — unreliable
            }

            // Running average of last 10 IBI values
            long runningTotal = 0;
            for (int i = 0; i < 9; i++) {
                rate[i] = rate[i + 1];
                runningTotal += rate[i];
            }
            rate[9] = ibi;
            runningTotal += ibi;
            runningTotal /= 10;
            bpm = 60000 / runningTotal;

            beatDetected = true;
        }
    }

    // Signal has dropped below threshold — reset pulse flag
    if (signal < thresh && pulse) {
        pulse = false;
        amp = peakVal - troughVal;
        thresh = amp / 2 + troughVal;
        peakVal = thresh;
        troughVal = thresh;
    }

    // No beat for 2.5 seconds — reset
    if (elapsed > 2500) {
        thresh = 512;
        peakVal = 512;
        troughVal = 512;
        lastBeatTime = sampleTime;
        firstBeat = true;
        secondBeat = false;
        bpm = 0;
    }
}

// ── GPS NMEA forwarding ──
void readGPS() {
    Wire.beginTransmission(GPS_ADDR);
    Wire.write(0xFD);
    Wire.endTransmission(false);
    Wire.requestFrom(GPS_ADDR, 2);

    if (Wire.available() < 2)
        return;

    uint16_t avail = (uint16_t)Wire.read() << 8;
    avail |= Wire.read();

    if (avail == 0 || avail == 0xFFFF)
        return;

    if (avail > 512)
        avail = 512;

    while (avail > 0) {
        uint8_t chunk = avail > READ_CHUNK ? READ_CHUNK : avail;

        Wire.beginTransmission(GPS_ADDR);
        Wire.write(0xFF);
        Wire.endTransmission(false);
        Wire.requestFrom(GPS_ADDR, (int)chunk);

        while (Wire.available()) {
            char c = Wire.read();
            if (c != 0xFF && c != '\0')
                Serial.print(c);
        }

        avail -= chunk;
    }
}
