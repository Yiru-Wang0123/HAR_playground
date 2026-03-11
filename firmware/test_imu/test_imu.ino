/*
 * Test Sketch: SparkFun ICM-20948 9DoF IMU
 * Interface: I2C (Qwiic)
 *
 * Wiring: Qwiic Shield → Qwiic cable → ICM-20948 breakout
 *
 * Library: SparkFun ICM-20948
 *   Install via Arduino IDE: Sketch → Include Library → Manage Libraries
 *   Search: "SparkFun ICM 20948"
 *
 * Open Serial Monitor at 115200 baud to see output.
 */

#include "ICM_20948.h"

ICM_20948_I2C imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin();
  Wire.setClock(400000);  // 400 kHz I2C

  bool initialized = false;
  while (!initialized) {
    imu.begin(Wire, 1);  // AD0 = 1 → I2C address 0x69 (SparkFun default)

    if (imu.status != ICM_20948_Stat_Ok) {
      Serial.println("IMU not detected. Check wiring. Retrying...");
      delay(1000);
    } else {
      initialized = true;
    }
  }

  Serial.println("ICM-20948 initialized!");
  Serial.println("ax,ay,az,gx,gy,gz,mx,my,mz");
}

void loop() {
  if (imu.dataReady()) {
    imu.getAGMT();  // Read accel, gyro, mag, temp

    Serial.print(imu.accX()); Serial.print(",");
    Serial.print(imu.accY()); Serial.print(",");
    Serial.print(imu.accZ()); Serial.print(",");
    Serial.print(imu.gyrX()); Serial.print(",");
    Serial.print(imu.gyrY()); Serial.print(",");
    Serial.print(imu.gyrZ()); Serial.print(",");
    Serial.print(imu.magX()); Serial.print(",");
    Serial.print(imu.magY()); Serial.print(",");
    Serial.println(imu.magZ());
  }

  delay(10);  // ~100 Hz
}
