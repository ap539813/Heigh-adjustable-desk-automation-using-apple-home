/*
 * ToF Sensor Module
 * VL53L0X distance sensor for height calibration
 */

#ifndef TOF_SENSOR_H
#define TOF_SENSOR_H

#include <Arduino.h>
#include <VL53L0X.h>

namespace ToFSensor {
  // Initialize ToF sensor
  bool init();

  // Measure height using ToF sensor
  float measureHeight();

  // Calibrate pulse count using ToF measurement
  int calibrate(int currentPulseCount);

  // Get ToF sensor instance
  VL53L0X& getSensor();
}

#endif
