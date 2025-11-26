#pragma once
#include <Arduino.h>

class IMUClass {
public:
  bool begin();                                      // init IMU
  bool accelerationAvailable();                      // time gate for reading
  bool readAcceleration(float &ax, float &ay, float &az);  // read accel
};

extern IMUClass IMU;
