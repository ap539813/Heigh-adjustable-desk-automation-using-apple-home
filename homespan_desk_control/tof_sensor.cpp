/*
 * ToF Sensor Module Implementation
 */

#include "tof_sensor.h"
#include "config.h"
#include "utils.h"

namespace ToFSensor {
  static VL53L0X tof;

  bool init() {
    bool success = tof.init();
    if (success) {
      Serial.println("📡 VL53L0X sensor initialized");
      tof.setTimeout(500);
      tof.startContinuous();
    } else {
      Serial.println("❌ VL53L0X sensor failed to initialize");
    }
    return success;
  }

  float measureHeight() {
    float totalHeight = 0;
    int validReadings = 0;

    Serial.println("📏 Taking ToF measurements...");

    for (uint8_t s = 0; s < DISTANCE_CYCLE; s++) {
      uint16_t dist_mm = tof.readRangeContinuousMillimeters();

      if (!tof.timeoutOccurred()) {
        float height_cm = (dist_mm / 10.0);
        totalHeight += height_cm;
        validReadings++;
      }
      delay(5);
    }

    if (validReadings > 0) {
      float avgHeight = totalHeight / validReadings;
      Serial.println("📐 Average ToF height: " + String(avgHeight) + " cm (" + String(validReadings) + " readings)");
      return avgHeight;
    } else {
      Serial.println("❌ No valid ToF readings obtained");
      return -1;
    }
  }

  int calibrate(int currentPulseCount) {
    Serial.println("🎯 Starting ToF calibration...");
    float measuredHeight = measureHeight();

    if (measuredHeight > 0 && measuredHeight >= MIN_HEIGHT_CM && measuredHeight <= MAX_HEIGHT_CM) {
      int oldPulseCount = currentPulseCount;
      int newPulseCount = heightToPulses(measuredHeight);

      Serial.println("✅ ToF Calibration:");
      Serial.println("   Before: " + String(oldPulseCount) + " pulses (" + String(pulsesToHeight(oldPulseCount)) + " cm)");
      Serial.println("   After:  " + String(newPulseCount) + " pulses (" + String(measuredHeight) + " cm)");
      Serial.println("   Adjustment: " + String(newPulseCount - oldPulseCount) + " pulses");

      return newPulseCount;
    } else {
      Serial.println("⚠️ ToF reading out of range or invalid - keeping current position");
      return currentPulseCount;
    }
  }

  VL53L0X& getSensor() {
    return tof;
  }
}
