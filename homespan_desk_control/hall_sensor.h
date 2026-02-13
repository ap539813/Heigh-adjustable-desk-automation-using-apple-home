/*
 * Hall Sensor Module
 * Dual hall sensor pulse counting for position tracking
 */

#ifndef HALL_SENSOR_H
#define HALL_SENSOR_H

#include <Arduino.h>
#include "config.h"

namespace HallSensor {
  // Initialize hall sensors
  void init();

  // Check hall sensors and update pulse count directly
  void check(int& pulseCount, MovementState state);
}

#endif
