/*
 * Hall Sensor Module Implementation
 */

#include "hall_sensor.h"
#include "utils.h"

int HallSensor::sensor1LastState = 0;
int HallSensor::sensor2LastState = 0;
unsigned long HallSensor::lastDebounceTime = 0;

void HallSensor::init() {
  pinMode(HALL_SENSOR1_PIN, INPUT);
  pinMode(HALL_SENSOR2_PIN, INPUT);
  sensor1LastState = digitalRead(HALL_SENSOR1_PIN);
  sensor2LastState = digitalRead(HALL_SENSOR2_PIN);
  Serial.println("🧲 Hall sensors initialized");
}

int HallSensor::check(int currentPulseCount, MovementState state) {
  int sensor1State = digitalRead(HALL_SENSOR1_PIN);
  int sensor2State = digitalRead(HALL_SENSOR2_PIN);
  int pulseCount = currentPulseCount;

  if (sensor1State == HIGH && sensor1LastState == LOW) {
    if (millis() - lastDebounceTime > debounceDelay) {
      if (sensor2State == LOW) {
        pulseCount++;
      } else {
        pulseCount--;
      }
      lastDebounceTime = millis();

      // Debug output during movement only
      if (state == MOVING_TO_TARGET) {
        static unsigned long lastHallPrint = 0;
        if (millis() - lastHallPrint > 200) { // Limit print frequency
          Serial.println("🧲 Hall: " + String(pulseCount) + " pulses, Height: " + String(pulsesToHeight(pulseCount), 1) + " cm");
          lastHallPrint = millis();
        }
      }
    }
  }

  sensor1LastState = sensor1State;
  sensor2LastState = sensor2State;

  return pulseCount;
}

unsigned long HallSensor::getLastDebounceTime() {
  return lastDebounceTime;
}
