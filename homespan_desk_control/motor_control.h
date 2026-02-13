/*
 * Motor Control Module
 * BTS7960 motor driver control
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include "config.h"

namespace MotorControl {
  // Initialize motor control
  void init();

  // Start moving up
  void startMovingUp();

  // Start moving down
  void startMovingDown();

  // Smooth stop motor
  void smoothStop();

  // Get current direction
  Direction getCurrentDirection();
}

#endif
