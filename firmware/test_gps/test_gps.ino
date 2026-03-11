/*
 * Test Sketch: SparkFun GNSS Receiver Breakout (MAX-M10S)
 * Interface: I2C (Qwiic)
 *
 * Wiring: Qwiic Shield → Qwiic cable → GPS-18037 breakout
 *         Attach GPS antenna to the breakout board's antenna connector
 *
 * Library: SparkFun u-blox GNSS v3
 *   Install via Arduino IDE: Sketch → Include Library → Manage Libraries
 *   Search: "SparkFun u-blox GNSS v3"
 *
 * Open Serial Monitor at 115200 baud to see output.
 *
 * NOTE: GPS needs clear sky view. First fix may take 24+ seconds (cold start).
 *       Test outdoors for best results.
 */

#include <SparkFun_u-blox_GNSS_v3.h>
#include <Wire.h>

SFE_UBLOX_GNSS gps;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin();

  if (!gps.begin()) {
    Serial.println("GPS not detected. Check wiring and antenna.");
    while (1) {}
  }

  Serial.println("MAX-M10S GNSS initialized!");

  // Set measurement rate to 10 Hz (100ms interval)
  gps.setMeasurementRate(100);
  gps.setNavigationRate(1);

  // Use automatic polling
  gps.setAutoPVT(true);

  Serial.println("Waiting for GPS fix...");
  Serial.println("lat,lon,alt_m,speed_mps,heading,sats,fix_type");
}

void loop() {
  if (gps.getPVT()) {
    long lat = gps.getLatitude();       // degrees * 10^-7
    long lon = gps.getLongitude();       // degrees * 10^-7
    long alt = gps.getAltitudeMSL();    // mm
    long speed = gps.getGroundSpeed();   // mm/s
    long heading = gps.getHeading();     // degrees * 10^-5
    byte sats = gps.getSIV();
    byte fix = gps.getFixType();        // 0=no, 2=2D, 3=3D

    Serial.print(lat / 10000000.0, 7); Serial.print(",");
    Serial.print(lon / 10000000.0, 7); Serial.print(",");
    Serial.print(alt / 1000.0, 2); Serial.print(",");
    Serial.print(speed / 1000.0, 3); Serial.print(",");
    Serial.print(heading / 100000.0, 1); Serial.print(",");
    Serial.print(sats); Serial.print(",");
    Serial.println(fix);
  }

  delay(100);  // 10 Hz
}
