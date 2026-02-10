/*
 * Motor Control Module Implementation
 */

#include "motor_control.h"
#include <Arduino.h>

Direction MotorControl::currentDir = DIR_NONE;

void MotorControl::init() {
  pinMode(R_EN_PIN, OUTPUT);
  digitalWrite(R_EN_PIN, HIGH);
  pinMode(L_EN_PIN, OUTPUT);
  digitalWrite(L_EN_PIN, HIGH);

  ledcAttach(R_PWM_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(L_PWM_PIN, PWM_FREQ, PWM_RES);

  ledcWrite(R_PWM_PIN, 0);
  ledcWrite(L_PWM_PIN, 0);

  Serial.println("🔧 Motor control initialized");
}

void MotorControl::smoothStart(uint8_t pin) {
  for (uint8_t s = 0; s <= TARGET_SPEED; s += RAMP_STEP) {
    ledcWrite(pin, s);
    delay(RAMP_DELAY);
  }
}

void MotorControl::smoothStop() {
  uint8_t pin = (currentDir == DIR_UP) ? L_PWM_PIN
               : (currentDir == DIR_DOWN) ? R_PWM_PIN
               : 255;

  if (pin != 255) {
    Serial.println("🛑 Smooth stopping motor...");
    for (int s = TARGET_SPEED; s >= 0; s -= RAMP_STEP * 2) {
      ledcWrite(pin, s);
      delay(RAMP_DELAY);
    }
  }

  ledcWrite(R_PWM_PIN, 0);
  ledcWrite(L_PWM_PIN, 0);
  currentDir = DIR_NONE;
}

void MotorControl::startMovingUp() {
  Serial.println("⬆️ Starting upward movement (L_PWM)");
  smoothStop();
  delay(50);
  ledcWrite(R_PWM_PIN, 0);
  smoothStart(L_PWM_PIN);
  currentDir = DIR_UP;
}

void MotorControl::startMovingDown() {
  Serial.println("⬇️ Starting downward movement (R_PWM)");
  smoothStop();
  delay(50);
  ledcWrite(L_PWM_PIN, 0);
  smoothStart(R_PWM_PIN);
  currentDir = DIR_DOWN;
}

Direction MotorControl::getCurrentDirection() {
  return currentDir;
}

void MotorControl::setCurrentDirection(Direction dir) {
  currentDir = dir;
}
