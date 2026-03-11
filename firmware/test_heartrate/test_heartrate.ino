/*
 * Test Sketch: SparkFun Pulse Oximeter & Heart Rate Sensor
 * Sensor: MAX30101 + MAX32664 (Biometric Hub)
 * Interface: I2C (Qwiic)
 *
 * Wiring: Qwiic Shield → Qwiic cable → SEN-15219 breakout
 *
 * Library: SparkFun Bio Sensor Hub
 *   Install via Arduino IDE: Sketch → Include Library → Manage Libraries
 *   Search: "SparkFun Bio Sensor Hub"
 *
 * Open Serial Monitor at 115200 baud to see output.
 *
 * NOTE: Place your finger on the sensor with steady, light pressure.
 *       It takes a few seconds to get a stable reading.
 */

#include <SparkFun_Bio_Sensor_Hub_Library.h>
#include <Wire.h>

// Reset pin and MFIO pin for the MAX32664 biometric hub
// If using Qwiic only (no extra wires), these may need to be connected:
//   RST  → Arduino D4
//   MFIO → Arduino D5
const int RESET_PIN = 4;
const int MFIO_PIN = 5;

SparkFun_Bio_Sensor_Hub bioHub(RESET_PIN, MFIO_PIN);
bioData body;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin();

  int result = bioHub.begin();
  if (!result) {
    Serial.println("Bio Sensor Hub initialized!");
  } else {
    Serial.println("Bio Sensor Hub failed to initialize. Check wiring.");
    Serial.print("Error code: ");
    Serial.println(result);
    while (1) {}
  }

  // Configure for heart rate + SpO2 mode
  int error = bioHub.configBpm(MODE_ONE);  // MODE_ONE = HR + SpO2
  if (!error) {
    Serial.println("Configured for HR + SpO2 (Mode 1)");
  } else {
    Serial.println("Configuration failed.");
    Serial.print("Error code: ");
    Serial.println(error);
    while (1) {}
  }

  // Give the sensor time to settle
  Serial.println("Stabilizing... place finger on sensor.");
  delay(4000);

  Serial.println("heartrate,confidence,spo2,status");
}

void loop() {
  body = bioHub.readBpm();

  Serial.print(body.heartRate);
  Serial.print(",");
  Serial.print(body.confidence);
  Serial.print(",");
  Serial.print(body.oxygen);
  Serial.print(",");
  Serial.println(body.status);

  delay(40);  // ~25 Hz
}
