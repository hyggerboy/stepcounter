// ---------------------------------------------------------
// LIBRARIES
// ---------------------------------------------------------
#include <Wire.h>
#include <math.h>

// ---------------------------------------------------------
// I2C IMU SETTINGS (LSM9DS1 accel/gyro @ 0x6B)
// ---------------------------------------------------------
#define LSM9DS1_AG 0x6B

void writeRegister(uint8_t reg, uint8_t value) {
  Wire1.beginTransmission(LSM9DS1_AG);
  Wire1.write(reg);
  Wire1.write(value);
  Wire1.endTransmission();
}

void readRegisters(uint8_t reg, uint8_t* buffer, size_t len) {
  Wire1.beginTransmission(LSM9DS1_AG);
  Wire1.write(reg);
  Wire1.endTransmission(false);

  Wire1.requestFrom(LSM9DS1_AG, len);
  for (size_t i = 0; i < len; i++) {
    buffer[i] = Wire1.read();
  }
}

// ---------------------------------------------------------
// ACCELEROMETER VARIABLES
// ---------------------------------------------------------
float x = 0, y = 0, z = 0;
float mag = 0.0f;
float mag_bias = 0.0f, z_f = 0.0f, alpha = 0.2f;

const float FC_HZ = 5.0f;

// ---------------------------------------------------------
// HC-SR04 SONAR
// ---------------------------------------------------------
const int TRIG_PIN = 2;
const int ECHO_PIN = 4;


volatile bool measureState = false;
volatile unsigned long startTime = 0;
volatile unsigned long pulseWidth = 0;
float lastDistance = 0;

void ping(){
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN,LOW);
}


float readSonarCM() {
   if (!measureState) return lastDistance;
  
    noInterrupts();
    unsigned long duration = pulseWidth;
    measureState = false;
    interrupts();

    float distance = duration * 0.0343 / 2;
 
  // Accept only valid distances (avoid zeros/noise)
  if (duration > 100 && duration < 30000) {
    lastDistance = distance;
  }

  return lastDistance;
}

// ---------------------------------------------------------
// PEAK DETECTOR (Step detection)
// ---------------------------------------------------------
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

PeakDetector det(0.10f, 119.0f, 0.30f);
uint32_t stepCount = 0;

// ---------------------------------------------------------
// SETUP
// ---------------------------------------------------------
void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 2000);

  // I2C
  Wire1.begin();
  Wire1.setClock(400000);

  // Sonar pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Configure accelerometer: 119 Hz, ±2g, BW 50Hz
  writeRegister(0x10, 0x60);

  float fs = 119.0f;

  alpha = 1.0f - expf(-2.0f * PI * FC_HZ / fs);
  if (alpha < 0.01f) alpha = 0.01f;
  if (alpha > 0.99f) alpha = 0.99f;

  det = PeakDetector(0.10f, fs, 0.30f);

  // Bias calibration (1 second)
  unsigned long t_end = millis() + 1000;
  uint32_t n = 0;

  while (millis() < t_end) {
    uint8_t buffer[6];
    readRegisters(0x28, buffer, 6);

    int16_t rx = (int16_t)(buffer[1] << 8 | buffer[0]);
    int16_t ry = (int16_t)(buffer[3] << 8 | buffer[2]);
    int16_t rz = (int16_t)(buffer[5] << 8 | buffer[4]);

    const float scale = 0.000061f;
    float ax = rx * scale;
    float ay = ry * scale;
    float az = rz * scale;

    float m = sqrt(ax * ax + ay * ay + az * az);

    mag_bias += m;
    n++;
  }

  if (n > 0) mag_bias /= (float)n;

  z_f = 0.0f;

  attachInterrupt(digitalPinToInterrupt(ECHO_PIN),measurePulse, CHANGE);

}

// ---------------------------------------------------------
// Main LOOP
// ---------------------------------------------------------
void loop() {
  // ---- READ ACCELEROMETER USING I2C ----
  uint8_t buffer[6];
  readRegisters(0x28, buffer, 6);

  int16_t rx = (int16_t)(buffer[1] << 8 | buffer[0]);
  int16_t ry = (int16_t)(buffer[3] << 8 | buffer[2]);
  int16_t rz = (int16_t)(buffer[5] << 8 | buffer[4]);

  const float scale = 0.000061f;
  x = rx * scale;
  y = ry * scale;
  z = rz * scale;

  // ---- Magnitude, DC removal, filtering ----
  mag = sqrt(x * x + y * y + z * z);
  float mag_dc = mag - mag_bias;

  z_f += alpha * (mag_dc - z_f);

  // ---- Step detection ----
  if (det.update(z_f)) {
    stepCount++;
  }

  ping();

  // ---- Sonar measurement ----
  float distance = readSonarCM();

  // ---- Output ----
  Serial.print("mag_filt:");
  Serial.print(z_f, 4);
  Serial.print("\tsteps:");
  Serial.print(stepCount);
  Serial.print("\tdist:");
  Serial.println(distance);
}

void step_distans(){


}

void measurePulse(){
  if (digitalRead(ECHO_PIN) == HIGH) {
    startTime = micros();
  } else {
    pulseWidth = micros() - startTime;
    measureState = true;
  }
}