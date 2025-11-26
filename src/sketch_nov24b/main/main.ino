#include <Arduino.h>
#include "imu.h"
#include <math.h>

// Forward declaration of the helper from imu.cpp
float sampleAverage(float newSample);

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

  // 1-pole low-pass: alpha = 1 - e^(-2π fc / fs)
  alpha = 1.0f - expf(-2.0f * PI * FC_HZ / FS_HZ);

  Serial.print("alpha = ");
  Serial.println(alpha, 6);

  Serial.println("IMU ready");
}

void loop() {
  float x, y, z;
  float steps;

  if (IMU.accelerationAvailable()) {
    if (IMU.readAcceleration(x, y, z)) {
      // Raw magnitude in g
      float m = sqrtf(x * x + y * y + z * z);

      // Optional moving average (from your previous code)
      float m_avg = sampleAverage(m);

      // Low-pass filter of the averaged magnitude
      // (this replaces your `mag_dc` expression)
      z_f += alpha * (m_avg - z_f);

      // Peak detection on the filtered signal
      bool isPeak = peakDet.update(z_f);

      Serial.print("m_raw: ");   Serial.print(m, 4);
      Serial.print("\tm_avg: "); Serial.print(m_avg, 4);
      Serial.print("\tz_f: ");   Serial.print(z_f, 4);

      if (isPeak) {
        steps+++;

        Serial.print(step);
      }
      Serial.println();
    }
  }
}

