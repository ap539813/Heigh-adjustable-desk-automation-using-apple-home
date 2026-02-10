/*
 * EEPROM Storage Module
 * Handles persistent storage of pulse count
 */

#ifndef STORAGE_H
#define STORAGE_H

#include <Arduino.h>

// Initialize EEPROM storage
void initStorage();

// Save current pulse count to EEPROM
void savePulseCount(int pulseCount);

// Load pulse count from EEPROM
int loadPulseCount();

#endif
