/*
 * Hardware Configuration and Constants
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ===== I2C PINS =====
const uint8_t SDA_PIN = 21;
const uint8_t SCL_PIN = 22;

// ===== BTS7960 MOTOR DRIVER PINS =====
const uint8_t R_EN_PIN = 25;
const uint8_t L_EN_PIN = 26;
const uint8_t R_PWM_PIN = 14;
const uint8_t L_PWM_PIN = 27;

// ===== HALL SENSOR PINS =====
const uint8_t HALL_SENSOR1_PIN = 32;
const uint8_t HALL_SENSOR2_PIN = 33;

// ===== HEIGHT AND PULSE CALIBRATION =====
const int PULSES_AT_MIN_HEIGHT = 0;
const int PULSES_AT_MAX_HEIGHT = 1840;
const float MIN_HEIGHT_CM = 69.0;
const float MAX_HEIGHT_CM = 107.0;
const float CM_PER_PULSE = (MAX_HEIGHT_CM - MIN_HEIGHT_CM) / (PULSES_AT_MAX_HEIGHT - PULSES_AT_MIN_HEIGHT);

// ===== EEPROM STORAGE =====
const int EEPROM_SIZE = 16;
const int EEPROM_MAGIC_ADDR = 0;
const int EEPROM_PULSES_ADDR = 4;
const int EEPROM_MAGIC = 0x4445534B;

// ===== MOTOR SETTINGS =====
const uint8_t TARGET_SPEED = 249;
const uint8_t DISTANCE_CYCLE = 15;
const uint8_t PWM_RES = 8;
const uint32_t PWM_FREQ = 1000;
const uint8_t RAMP_STEP = 10;
const unsigned RAMP_DELAY = 10;
const uint8_t ERROR_THRESHOLD = 10;

// ===== SYSTEM SETTINGS =====
const unsigned long BACKLIGHT_TIMEOUT = 10000;
const unsigned long MOVEMENT_TIMEOUT = 15000;
const unsigned long SLIDER_DEBOUNCE_TIME = 1000; // 1 second debounce

// ===== LCD CONFIGURATION =====
const uint8_t LCD_ADDRESS = 0x27;
const uint8_t LCD_COLS = 16;
const uint8_t LCD_ROWS = 2;

// ===== MOVEMENT STATES =====
enum MovementState {
  IDLE,
  WAITING_FOR_SETTLE,
  MOVING_TO_TARGET,
  FINALIZING
};

enum Direction {
  DIR_NONE = 0,
  DIR_UP = 1,
  DIR_DOWN = -1
};

#endif
