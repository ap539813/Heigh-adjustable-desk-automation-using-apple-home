/*
 * Hall Sensor Module
 * Handles dual hall sensor pulse counting
 */

#ifndef HALL_SENSOR_H
#define HALL_SENSOR_H

#include <Arduino.h>
#include "config.h"

class HallSensor {
public:
  // Initialize hall sensors
  static void init();

  // Check hall sensors and update pulse count
  // Returns the current pulse count
  static int check(int currentPulseCount, MovementState state);

  // Get last debounce time
  static unsigned long getLastDebounceTime();

private:
  static int sensor1LastState;
  static int sensor2LastState;
  static unsigned long lastDebounceTime;
  static const unsigned long debounceDelay = 5;
};

#endif
