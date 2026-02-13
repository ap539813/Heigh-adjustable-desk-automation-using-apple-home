/*
 * HomeKit Desk Service
 * Smart slider-based desk control with debouncing
 */

#ifndef DESK_SERVICE_H
#define DESK_SERVICE_H

#include <HomeSpan.h>
#include <LiquidCrystal_I2C.h>
#include "config.h"

struct SmartSliderDeskControl : Service::WindowCovering {
  SpanCharacteristic *current;
  SpanCharacteristic *target;
  SpanCharacteristic *positionState;

  int pendingTargetPercent;
  bool hasPendingTarget;

  // References to global state
  int& pulseCount;
  MovementState& currentState;
  int& targetPulses;
  unsigned long& lastSliderChangeTime;
  unsigned long& movementStartTime;
  unsigned long& lastMotorStopTime;
  bool& backlightOn;
  LiquidCrystal_I2C& lcd;

  SmartSliderDeskControl(
    int& pulseCountRef,
    MovementState& currentStateRef,
    int& targetPulsesRef,
    unsigned long& lastSliderChangeTimeRef,
    unsigned long& movementStartTimeRef,
    unsigned long& lastMotorStopTimeRef,
    bool& backlightOnRef,
    LiquidCrystal_I2C& lcdRef
  );

  boolean update() override;
  void loop() override;

private:
  void handleMovementStateMachine();
  void startMovementToTarget();
  void handleMovementProgress();
  void finalizeMovement();
  void updateCurrentPosition();
};

#endif
