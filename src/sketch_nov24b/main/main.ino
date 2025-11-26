#include <Arduino.h>
#include <math.h>
#include "imu.h"

// ---------- Peak detector ----------

struct PeakDetector {
  float thresh;
  size_t minDist;
  size_t sinceLast = 0;
  float prev2 = 0.0f, prev1 = 0.0f;

  PeakDetector(float thresh_, float fs, float minSeconds)
      : thresh(thresh_), minDist((size_t)(minSeconds * fs + 0.5f)) {}

  bool update(float x) {
    bool peak = false;

    if (sinceLast >= minDist) {
      if (prev1 > thresh && prev1 > prev2 && prev1 >= x) {
        peak = true;
        sinceLast = 0;
      }
    }

    prev2 = prev1;
    prev1 = x;

    if (sinceLast < 1000000) sinceLast++;

    return peak;
  }
};

// ----- signal processing config -----

// Approx sampling rate (matches IMU.accelerationAvailable target)
constexpr float FS_HZ = 100.0f;

// Low-pass cutoff for smoothing
constexpr float FC_HZ = 5.0f;

// Filter + step counter state
float alpha = 0.0f;      // low-pass coefficient (set in setup)
float z_f   = 0.0f;      // filtered magnitude
uint32_t steps = 0;      // step counter

// Threshold ~1.05 g and min 0.3 s between peaks
PeakDetector peakDet(1.05f, FS_HZ, 0.3f);

// sampleAverage is implemented in imu.cpp and declared in imu.h


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

  // 1-pole low-pass: alpha = 1 − e^(−2π fc / fs)
  alpha = 1.0f - expf(-2.0f * PI * FC_HZ / FS_HZ);

  Serial.print("alpha = ");
  Serial.println(alpha, 6);

  Serial.println("IMU ready");
}

void loop() {
  float x, y, z;

  if (IMU.accelerationAvailable()) {
    if (IMU.readAcceleration(x, y, z)) {
      // Raw magnitude in g
      float m = sqrtf(x * x + y * y + z * z);

      // 7-sample average
      float m_avg = sampleAverage(m);

      // Low-pass filter of the averaged magnitude
      z_f += alpha * (m_avg - z_f);

      // Peak detection on the filtered signal
      bool isPeak = peakDet.update(z_f);

      Serial.print("m_raw: ");   Serial.print(m, 4);
      Serial.print("\tm_avg: "); Serial.print(m_avg, 4);
      Serial.print("\tz_f: ");   Serial.print(z_f, 4);

      if (isPeak) {
        steps++;
        Serial.print("\tsteps: ");
        Serial.print(steps);
      }

      Serial.println();
    }
  }
}
