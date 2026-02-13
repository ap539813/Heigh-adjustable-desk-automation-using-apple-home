/*
 * HomeKit Desk Service Implementation
 */

#include "desk_service.h"
#include "utils.h"
#include "motor_control.h"
#include "tof_sensor.h"
#include "storage.h"

SmartSliderDeskControl::SmartSliderDeskControl(
  int& pulseCountRef,
  MovementState& currentStateRef,
  int& targetPulsesRef,
  unsigned long& lastSliderChangeTimeRef,
  unsigned long& movementStartTimeRef,
  unsigned long& lastMotorStopTimeRef,
  bool& backlightOnRef,
  LiquidCrystal_I2C& lcdRef
) : Service::WindowCovering(),
    pulseCount(pulseCountRef),
    currentState(currentStateRef),
    targetPulses(targetPulsesRef),
    lastSliderChangeTime(lastSliderChangeTimeRef),
    movementStartTime(movementStartTimeRef),
    lastMotorStopTime(lastMotorStopTimeRef),
    backlightOn(backlightOnRef),
    lcd(lcdRef) {

  current = new Characteristic::CurrentPosition(0);
  target = new Characteristic::TargetPosition(0);
  positionState = new Characteristic::PositionState(2); // Stopped

  pendingTargetPercent = 0;
  hasPendingTarget = false;

  updateCurrentPosition();

  Serial.println("🎛️ Smart slider desk control service created");
}

boolean SmartSliderDeskControl::update() {
  int newTargetPercent = target->getNewVal();

  // Record the slider change but don't move yet
  pendingTargetPercent = newTargetPercent;
  hasPendingTarget = true;
  lastSliderChangeTime = millis();

  Serial.println("🎯 Slider changed to " + String(newTargetPercent) + "% - waiting for settle...");

  return true;
}

void SmartSliderDeskControl::loop() {
  // Handle the state machine
  handleMovementStateMachine();
}

void SmartSliderDeskControl::handleMovementStateMachine() {
  switch (currentState) {

    case IDLE:
      // Check if we have a pending target to process
      if (hasPendingTarget && millis() - lastSliderChangeTime >= SLIDER_DEBOUNCE_TIME) {
        // Slider has settled - start movement
        startMovementToTarget();
      }
      break;

    case WAITING_FOR_SETTLE:
      // This state is handled in IDLE now
      break;

    case MOVING_TO_TARGET:
      handleMovementProgress();
      break;

    case FINALIZING:
      finalizeMovement();
      break;
  }
}

void SmartSliderDeskControl::startMovementToTarget() {
  targetPulses = percentageToPulses(pendingTargetPercent);
  float targetHeight = pulsesToHeight(targetPulses);
  float currentHeight = pulsesToHeight(pulseCount);

  Serial.println("\n🚀 STARTING MOVEMENT:");
  Serial.println("   Target: " + String(pendingTargetPercent) + "% = " + String(targetHeight, 1) + " cm (" + String(targetPulses) + " pulses)");
  Serial.println("   Current: " + String(pulsesToPercentage(pulseCount), 1) + "% = " + String(currentHeight, 1) + " cm (" + String(pulseCount) + " pulses)");

  // Determine direction and start movement
  if (targetPulses > pulseCount) {
    MotorControl::startMovingUp();
    positionState->setVal(1); // Going to max
    Serial.println("   Direction: UP ⬆️");
  } else if (targetPulses < pulseCount) {
    MotorControl::startMovingDown();
    positionState->setVal(0); // Going to min
    Serial.println("   Direction: DOWN ⬇️");
  } else {
    Serial.println("   Already at target! ✅");
    hasPendingTarget = false;
    return;
  }

  // Update LCD
  if (!backlightOn) {
    lcd.backlight();
    backlightOn = true;
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Moving to:");
  lcd.setCursor(0, 1);
  lcd.print(String(targetHeight, 1) + " cm");

  // Transition to moving state
  currentState = MOVING_TO_TARGET;
  movementStartTime = millis();
  hasPendingTarget = false;
}

void SmartSliderDeskControl::handleMovementProgress() {
  // Update LCD periodically
  static unsigned long lastLcdUpdate = 0;
  if (millis() - lastLcdUpdate > 500) {
    float currentHeight = pulsesToHeight(pulseCount);
    float targetHeight = pulsesToHeight(targetPulses);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Moving:");
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f->%.1f", currentHeight, targetHeight);
    lcd.setCursor(0, 1);
    lcd.print(buf);

    lastLcdUpdate = millis();

    // Debug: Print to serial every LCD update
    Serial.println("📺 LCD Update: " + String(currentHeight, 1) + " cm -> " + String(targetHeight, 1) + " cm (Pulses: " + String(pulseCount) + "/" + String(targetPulses) + ")");
  }

  // Check if we've reached the target
  int pulseDifference = abs(pulseCount - targetPulses);
  bool timeoutReached = (millis() - movementStartTime) > MOVEMENT_TIMEOUT;

  // Debug: Print progress every 2 seconds
  static unsigned long lastProgressPrint = 0;
  if (millis() - lastProgressPrint > 2000) {
    Serial.println("⏱️ Progress: " + String(pulseCount) + "/" + String(targetPulses) + " pulses (diff: " + String(pulseDifference) + ", time: " + String((millis() - movementStartTime) / 1000) + "s)");
    lastProgressPrint = millis();
  }

  if (pulseDifference <= ERROR_THRESHOLD || timeoutReached) {
    if (timeoutReached) {
      Serial.println("⏰ Movement timeout reached");
      Serial.println("   Started at: " + String(pulseCount) + " pulses, Target: " + String(targetPulses) + " pulses");
    } else {
      Serial.println("✅ Target reached within " + String(pulseDifference) + " pulse tolerance");
    }

    currentState = FINALIZING;
  }
}

void SmartSliderDeskControl::finalizeMovement() {
  Serial.println("🏁 Finalizing movement...");

  // Stop the motor
  MotorControl::smoothStop();
  positionState->setVal(2); // Stopped
  lastMotorStopTime = millis();

  // Calibrate position
  delay(200);
  pulseCount = ToFSensor::calibrate(pulseCount);

  // Save position
  savePulseCount(pulseCount);

  // Update HomeKit position to actual position
  updateCurrentPosition();
  target->setVal(current->getVal()); // Sync target to actual position

  // Update LCD
  float finalHeight = pulsesToHeight(pulseCount);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Height:");
  lcd.setCursor(0, 1);
  lcd.print(String(finalHeight, 1) + " cm");

  Serial.println("🎉 Movement complete!");
  Serial.println("   Final: " + String(pulsesToPercentage(pulseCount), 1) + "% = " + String(finalHeight, 1) + " cm (" + String(pulseCount) + " pulses)");
  Serial.println("   HomeKit slider updated to match actual position\n");

  // Return to idle state
  currentState = IDLE;
}

void SmartSliderDeskControl::updateCurrentPosition() {
  float percentage = pulsesToPercentage(pulseCount);
  if (percentage < 0) percentage = 0;
  if (percentage > 100) percentage = 100;

  current->setVal((int)percentage);
}
