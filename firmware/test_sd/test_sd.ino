/*
 * Test Sketch: MicroSD Card Module
 * Interface: SPI
 *
 * Wiring (via breadboard):
 *   CS   → D10
 *   MOSI → D11
 *   MISO → D12
 *   SCK  → D13
 *   VCC  → 5V
 *   GND  → GND
 *
 * Make sure the MicroSD card is formatted as FAT32.
 *
 * Open Serial Monitor at 115200 baud to see output.
 * This sketch writes a test file, reads it back, and prints the result.
 */

#include <SPI.h>
#include <SD.h>

const int CS_PIN = 10;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("Initializing SD card...");

  if (!SD.begin(CS_PIN)) {
    Serial.println("SD card initialization failed!");
    Serial.println("Check: wiring, card inserted, card formatted as FAT32");
    while (1) {}
  }

  Serial.println("SD card initialized.");

  // Write test
  Serial.println("Writing test file...");
  File testFile = SD.open("test.csv", FILE_WRITE);
  if (testFile) {
    testFile.println("timestamp,ax,ay,az,gx,gy,gz,hr,spo2,lat,lon");
    testFile.println("1000,0.01,-0.02,9.81,0.5,-0.3,0.1,72,98,43.65,-79.38");
    testFile.println("1010,0.03,-0.01,9.79,0.4,-0.2,0.2,73,98,43.65,-79.38");
    testFile.println("1020,0.02,-0.03,9.80,0.6,-0.1,0.0,71,97,43.65,-79.38");
    testFile.close();
    Serial.println("Write successful!");
  } else {
    Serial.println("Error opening file for writing.");
    while (1) {}
  }

  // Read test
  Serial.println("\nReading test file:");
  Serial.println("---");
  testFile = SD.open("test.csv");
  if (testFile) {
    while (testFile.available()) {
      Serial.write(testFile.read());
    }
    testFile.close();
    Serial.println("---");
    Serial.println("Read successful!");
  } else {
    Serial.println("Error opening file for reading.");
  }

  // File size
  testFile = SD.open("test.csv");
  if (testFile) {
    Serial.print("File size: ");
    Serial.print(testFile.size());
    Serial.println(" bytes");
    testFile.close();
  }

  Serial.println("\nSD card test complete!");
}

void loop() {
  // Nothing to do — test runs once in setup()
}
