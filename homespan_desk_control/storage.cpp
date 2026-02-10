/*
 * EEPROM Storage Module Implementation
 */

#include "storage.h"
#include "config.h"
#include <EEPROM.h>

void initStorage() {
  EEPROM.begin(EEPROM_SIZE);
  Serial.println("📦 EEPROM storage initialized");
}

void savePulseCount(int pulseCount) {
  EEPROM.writeInt(EEPROM_PULSES_ADDR, pulseCount);
  EEPROM.commit();
  Serial.println("💾 Saved pulse count: " + String(pulseCount));
}

int loadPulseCount() {
  int magic = EEPROM.readInt(EEPROM_MAGIC_ADDR);
  int pulseCount = 0;

  if (magic != EEPROM_MAGIC) {
    Serial.println("🆕 First time setup - initializing EEPROM");
    EEPROM.writeInt(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
    EEPROM.writeInt(EEPROM_PULSES_ADDR, 0);
    EEPROM.commit();
    pulseCount = 0;
  } else {
    pulseCount = EEPROM.readInt(EEPROM_PULSES_ADDR);
    Serial.println("📂 Loaded pulse count: " + String(pulseCount));
  }

  // Clamp to valid range
  if (pulseCount < PULSES_AT_MIN_HEIGHT) pulseCount = PULSES_AT_MIN_HEIGHT;
  if (pulseCount > PULSES_AT_MAX_HEIGHT) pulseCount = PULSES_AT_MAX_HEIGHT;

  return pulseCount;
}
