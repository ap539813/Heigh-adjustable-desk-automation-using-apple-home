/*
 * Time-of-Flight (ToF) Sensor Module
 * Handles VL53L0X distance sensor for height calibration
 */

#ifndef TOF_SENSOR_H
#define TOF_SENSOR_H

#include <Arduino.h>
#include <VL53L0X.h>

class ToFSensor {
public:
  // Initialize the ToF sensor
  static bool init();

  // Measure height using ToF sensor (returns height in cm, or -1 on error)
  static float measureHeight();

  // Calibrate pulse count based on ToF measurement
  // Returns the new calibrated pulse count
  static int calibrate(int currentPulseCount);

private:
  static VL53L0X sensor;
  static bool initialized;
};

#endif
