/*
 * Motor Control Module
 * Handles BTS7960 motor driver control
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "config.h"

// Motor control namespace
class MotorControl {
public:
  // Initialize motor control pins and PWM
  static void init();

  // Start moving the desk upward
  static void startMovingUp();

  // Start moving the desk downward
  static void startMovingDown();

  // Stop the motor smoothly
  static void smoothStop();

  // Get current movement direction
  static Direction getCurrentDirection();

  // Set current movement direction
  static void setCurrentDirection(Direction dir);

private:
  // Smooth start motor on a specific PWM pin
  static void smoothStart(uint8_t pin);

  // Current movement direction
  static Direction currentDir;
};

#endif
