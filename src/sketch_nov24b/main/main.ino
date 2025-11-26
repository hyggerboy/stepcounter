#include "imu.h"

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 2000) {}  // for USB boards

  Serial.println("Starting IMU...");

  if (!IMU.begin()) {
    Serial.println("IMU init failed!");
    while (1) {
      delay(1000);
    }
  }

  Serial.println("IMU ready");
}

void loop() {
  float x, y, z;

  // Only read when our time gate says "ready"
  if (IMU.accelerationAvailable()) {
    if (IMU.readAcceleration(x, y, z)) {
      Serial.print("ax: "); Serial.print(x, 4);
      Serial.print("\tay: "); Serial.print(y, 4);
      Serial.print("\taz: "); Serial.println(z, 4);
    }
  }

}


