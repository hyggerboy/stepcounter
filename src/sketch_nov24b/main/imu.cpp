#include "imu.h"
#include <Wire.h>

#define IMU_ADDR 0x6B    // LSM9DS1 accel/gyro I2C address


IMUClass IMU;

// ---------- low-level helpers (only used in this file) ----------

static void writeRegister(uint8_t reg, uint8_t value) {
  Wire1.beginTransmission(IMU_ADDR);
  Wire1.write(reg);
  Wire1.write(value);
  Wire1.endTransmission();
}

static void readRegisters(uint8_t reg, uint8_t *buffer, size_t len) {
  Wire1.beginTransmission(IMU_ADDR);
  Wire1.write(reg);
  Wire1.endTransmission(false);      // repeated START

  Wire1.requestFrom(IMU_ADDR, (uint8_t)len);
  for (size_t i = 0; i < len && Wire1.available(); i++) {
    buffer[i] = Wire1.read();
  }
}

// ---------- IMUClass methods ----------

bool IMUClass::begin() {
  Wire1.begin();
  Wire1.setClock(400000);   // 400 kHz I2C

  // Check WHO_AM_I for LSM9DS1 accel/gyro (should be 0x68)
  uint8_t who = 0;
  readRegisters(0x0F, &who, 1);
  Serial.print("WHO_AM_I = 0x");
  Serial.println(who, HEX);

  // If this prints something weird (not 0x68), address/bus is wrong.
  if (who != 0x68) {
    Serial.println("Unexpected WHO_AM_I, IMU may not be LSM9DS1 or addr/bus wrong.");
    // you can either return false here, or just continue
    // return false;
  }

  // Turn on accelerometer:
  // CTRL_REG6_XL = 0x20
  // bits [7:5] = 0b011 -> ODR 119 Hz
  // bits [4:3] = 0b00  -> ±2 g
  // bits [2:0] = 0b000 -> BW auto
  writeRegister(0x20, 0b01100000);

  return true;
}

bool IMUClass::accelerationAvailable() {
  // pick the rate you want to sample at:
  const float targetHz = 100.0f;             // e.g. 100 Hz
  const uint32_t period_us = (uint32_t)(1000000.0f / targetHz + 0.5f);

  static uint32_t last_us = 0;
  uint32_t now = micros();

  if (now - last_us >= period_us) {
    last_us = now;
    return true;      // "ok, time to take one sample"
  }

  return false;
}


bool IMUClass::readAcceleration(float &ax, float &ay, float &az) {
  uint8_t buf[6];

  // OUT_X_L_XL = 0x28
  readRegisters(0x28, buf, 6);

  int16_t rawX = (int16_t)((buf[1] << 8) | buf[0]);
  int16_t rawY = (int16_t)((buf[3] << 8) | buf[2]);
  int16_t rawZ = (int16_t)((buf[5] << 8) | buf[4]);

  // ±2g -> 0.061 mg/LSB -> 0.000061 g/LSB
  const float scale = 0.000061f;
  ax = rawX * scale;
  ay = rawY * scale;
  az = rawZ * scale;

  return true;
}
 
void sampleavage(bool data_avilibal, float data){
  float sum_of_data;
  if (data_avilibal){
    sum_of_data = sum_of_data + data;
  }
  
}