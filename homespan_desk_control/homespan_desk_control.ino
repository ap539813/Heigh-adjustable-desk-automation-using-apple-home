/*
 * Smart Debounced Slider Desk Control
 *
 * Key Features:
 * - Slider changes are captured but movement is delayed
 * - 1-second debounce period after last slider change
 * - Clean hall sensor operation during movement
 * - Slider position updates after movement completion
 * - Separate movement state machine
 */
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <VL53L0X.h>
#include <HomeSpan.h>

// ===== CONFIGURATION =====
const char* WIFI_SSID = "A-19_4G";
const char* WIFI_PASSWORD = "lebhikhari";

// I2C pins
const uint8_t SDA_PIN = 21;
const uint8_t SCL_PIN = 22;

// BTS7960 motor driver pins
const uint8_t R_EN_PIN = 25;
const uint8_t L_EN_PIN = 26;
const uint8_t R_PWM_PIN = 14;
const uint8_t L_PWM_PIN = 27;

// Hall sensor pins
const uint8_t HALL_SENSOR1_PIN = 32;
const uint8_t HALL_SENSOR2_PIN = 33;

// Height and pulse calibration
const int PULSES_AT_MIN_HEIGHT = 0;
const int PULSES_AT_MAX_HEIGHT = 1840;
const float MIN_HEIGHT_CM = 69.0;
const float MAX_HEIGHT_CM = 107.0;
const float CM_PER_PULSE = (MAX_HEIGHT_CM - MIN_HEIGHT_CM) / (PULSES_AT_MAX_HEIGHT - PULSES_AT_MIN_HEIGHT);

// EEPROM Storage
const int EEPROM_SIZE = 16;
const int EEPROM_MAGIC_ADDR = 0;
const int EEPROM_PULSES_ADDR = 4;
const int EEPROM_MAGIC = 0x4445534B;

// Motor settings
const uint8_t TARGET_SPEED = 249;
const uint8_t DISTANCE_CYCLE = 15;
const uint8_t PWM_RES = 8;
const uint32_t PWM_FREQ = 1000;
const uint8_t RAMP_STEP = 10;
const unsigned RAMP_DELAY = 10;
const uint8_t ERROR_THRESHOLD = 10;

// System settings
const unsigned long BACKLIGHT_TIMEOUT = 10000;
const unsigned long MOVEMENT_TIMEOUT = 15000;
const unsigned long SLIDER_DEBOUNCE_TIME = 1000; // 1 second debounce

// Movement states
enum MovementState {
  IDLE,
  WAITING_FOR_SETTLE,
  MOVING_TO_TARGET,
  FINALIZING
};

enum Direction { DIR_NONE = 0, DIR_UP = 1, DIR_DOWN = -1 };

// ===== GLOBAL VARIABLES =====
// Movement control
MovementState currentState = IDLE;
Direction currentDir = DIR_NONE;
int targetPulses = 0;
unsigned long lastSliderChangeTime = 0;
unsigned long movementStartTime = 0;
bool backlightOn = true;
unsigned long lastMotorStopTime = 0;

// Hall sensor variables
volatile int pulseCount = 0;
int sensor1LastState = 0;
int sensor2LastState = 0;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 5;

// I²C devices
VL53L0X tof;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Forward declarations
struct SmartSliderDeskControl;
SmartSliderDeskControl* deskControl = nullptr;

// ===== UTILITY FUNCTIONS =====
float pulsesToHeight(int pulses) {
  return MIN_HEIGHT_CM + (pulses * CM_PER_PULSE);
}

int heightToPulses(float height) {
  return (int)((height - MIN_HEIGHT_CM) / CM_PER_PULSE);
}

float pulsesToPercentage(int pulses) {
  return ((float)(pulses - PULSES_AT_MIN_HEIGHT) / (PULSES_AT_MAX_HEIGHT - PULSES_AT_MIN_HEIGHT)) * 100.0;
}

int percentageToPulses(float percentage) {
  return PULSES_AT_MIN_HEIGHT + (int)((percentage / 100.0) * (PULSES_AT_MAX_HEIGHT - PULSES_AT_MIN_HEIGHT));
}

// ===== EEPROM FUNCTIONS =====
void initStorage() {
  EEPROM.begin(EEPROM_SIZE);
  Serial.printf("[Init] EEPROM ready\n");
}

void savePulseCount() {
  EEPROM.writeInt(EEPROM_PULSES_ADDR, pulseCount);
  EEPROM.commit();
  Serial.printf("[EEPROM] Saved: %d pulses\n", pulseCount);
}

void loadPulseCount() {
  int magic = EEPROM.readInt(EEPROM_MAGIC_ADDR);
  if (magic != EEPROM_MAGIC) {
    Serial.printf("[EEPROM] First run, initializing\n");
    EEPROM.writeInt(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
    EEPROM.writeInt(EEPROM_PULSES_ADDR, 0);
    EEPROM.commit();
    pulseCount = 0;
  } else {
    pulseCount = EEPROM.readInt(EEPROM_PULSES_ADDR);
    Serial.printf("[EEPROM] Loaded: %d pulses\n", pulseCount);
  }

  // Clamp to valid range
  if (pulseCount < PULSES_AT_MIN_HEIGHT) pulseCount = PULSES_AT_MIN_HEIGHT;
  if (pulseCount > PULSES_AT_MAX_HEIGHT) pulseCount = PULSES_AT_MAX_HEIGHT;
}

// ===== TOF SENSOR FUNCTIONS =====
bool initToFSensor() {
  bool success = tof.init();
  if (success) {
    Serial.printf("[Init] ToF ready\n");
    tof.setTimeout(500);
    tof.startContinuous();
  } else {
    Serial.printf("[Init] ToF FAILED\n");
  }
  return success;
}

float measureHeightWithTof() {
  float totalHeight = 0;
  int validReadings = 0;

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
    return totalHeight / validReadings;
  } else {
    return -1;
  }
}

void calibrateWithTof() {
  float measuredHeight = measureHeightWithTof();

  // Accept readings within 5cm margin, then clamp to valid range
  if (measuredHeight > 0 && measuredHeight >= (MIN_HEIGHT_CM - 5.0) && measuredHeight <= (MAX_HEIGHT_CM + 5.0)) {
    float clamped = constrain(measuredHeight, MIN_HEIGHT_CM, MAX_HEIGHT_CM);
    int oldPulseCount = pulseCount;
    pulseCount = heightToPulses(clamped);
    pulseCount = constrain(pulseCount, PULSES_AT_MIN_HEIGHT, PULSES_AT_MAX_HEIGHT);

    Serial.printf("[ToF] Calibrated: %.1f cm (%d pulses, adj %+d)\n", clamped, pulseCount, pulseCount - oldPulseCount);
  } else {
    Serial.printf("[ToF] Out of range: %.1f cm, keeping %d pulses\n", measuredHeight, pulseCount);
  }
}

// ===== MOTOR CONTROL FUNCTIONS =====
void initMotorControl() {
  pinMode(R_EN_PIN, OUTPUT);
  digitalWrite(R_EN_PIN, HIGH);
  pinMode(L_EN_PIN, OUTPUT);
  digitalWrite(L_EN_PIN, HIGH);

  ledcAttach(R_PWM_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(L_PWM_PIN, PWM_FREQ, PWM_RES);

  ledcWrite(R_PWM_PIN, 0);
  ledcWrite(L_PWM_PIN, 0);

  Serial.printf("[Init] Motor ready\n");
}

void smoothStartMotor(uint8_t pin) {
  for (uint8_t s = 0; s <= TARGET_SPEED; s += RAMP_STEP) {
    ledcWrite(pin, s);
    delay(RAMP_DELAY);
  }
}

void smoothStopMotor() {
  uint8_t pin = (currentDir == DIR_UP) ? L_PWM_PIN
               : (currentDir == DIR_DOWN) ? R_PWM_PIN
               : 255;

  if (pin != 255) {
    for (int s = TARGET_SPEED; s >= 0; s -= RAMP_STEP * 2) {
      ledcWrite(pin, s);
      delay(RAMP_DELAY);
    }
  }

  ledcWrite(R_PWM_PIN, 0);
  ledcWrite(L_PWM_PIN, 0);
  currentDir = DIR_NONE;
}

void startMovingUp() {
  smoothStopMotor();
  delay(50);
  ledcWrite(R_PWM_PIN, 0);
  smoothStartMotor(L_PWM_PIN);
  currentDir = DIR_UP;
}

void startMovingDown() {
  smoothStopMotor();
  delay(50);
  ledcWrite(L_PWM_PIN, 0);
  smoothStartMotor(R_PWM_PIN);
  currentDir = DIR_DOWN;
}

// ===== HALL SENSOR FUNCTIONS =====
void initHallSensors() {
  pinMode(HALL_SENSOR1_PIN, INPUT);
  pinMode(HALL_SENSOR2_PIN, INPUT);
  sensor1LastState = digitalRead(HALL_SENSOR1_PIN);
  sensor2LastState = digitalRead(HALL_SENSOR2_PIN);
  Serial.printf("[Init] Hall sensors ready\n");
}

void checkHallSensors() {
  int sensor1State = digitalRead(HALL_SENSOR1_PIN);
  int sensor2State = digitalRead(HALL_SENSOR2_PIN);

  if (sensor1State == HIGH && sensor1LastState == LOW) {
    if (millis() - lastDebounceTime > debounceDelay) {
      // Use known motor direction instead of sensor2 inference.
      // This is always correct regardless of wiring/magnet orientation.
      if (currentDir == DIR_UP) {
        pulseCount++;
      } else if (currentDir == DIR_DOWN) {
        pulseCount--;
      } else {
        // Motor not running — fall back to sensor2
        // If still inverted for your hardware, swap +/- here
        if (sensor2State == LOW) {
          pulseCount++;
        } else {
          pulseCount--;
        }
      }

      // Clamp to valid range to prevent runaway
      pulseCount = constrain(pulseCount, PULSES_AT_MIN_HEIGHT, PULSES_AT_MAX_HEIGHT);
      lastDebounceTime = millis();
    }
  }

  sensor1LastState = sensor1State;
  sensor2LastState = sensor2State;
}

// Pad string to fixed width (prevents leftover characters on LCD)
String padRight(String s, int width) {
  while (s.length() < (unsigned)width) s += ' ';
  return s.substring(0, width);
}

// Write to LCD without clearing — no flicker
void lcdShowTwoLines(const String &line1, const String &line2) {
  lcd.setCursor(0, 0);
  lcd.print(padRight(line1, 16));
  lcd.setCursor(0, 1);
  lcd.print(padRight(line2, 16));
}

// ===== SMART SLIDER HOMEKIT SERVICE =====
struct SmartSliderDeskControl : Service::WindowCovering {
  SpanCharacteristic *current;
  SpanCharacteristic *target;
  SpanCharacteristic *positionState;

  int pendingTargetPercent;
  bool hasPendingTarget;

  SmartSliderDeskControl() : Service::WindowCovering() {
    current = new Characteristic::CurrentPosition(0);
    target = new Characteristic::TargetPosition(0);
    positionState = new Characteristic::PositionState(2); // Stopped

    pendingTargetPercent = 0;
    hasPendingTarget = false;

    deskControl = this;
    updateCurrentPosition();

    Serial.printf("[Init] HomeKit service ready\n");
  }

  boolean update() override {
    int newTargetPercent = target->getNewVal();

    // Record the slider change but don't move yet
    pendingTargetPercent = newTargetPercent;
    hasPendingTarget = true;
    lastSliderChangeTime = millis();


    return true;
  }

  void loop() override {
    // Handle the state machine
    handleMovementStateMachine();
  }

  void handleMovementStateMachine() {
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
        // CRITICAL: Finalize runs ONCE and immediately transitions to IDLE
        // Set to IDLE first to prevent re-entry
          currentState = IDLE;
        finalizeMovement();
        break;
    }
  }

  void startMovementToTarget() {
    targetPulses = percentageToPulses(pendingTargetPercent);
    float targetHeight = pulsesToHeight(targetPulses);
    float currentHeight = pulsesToHeight(pulseCount);

    // Determine direction and start movement
    if (targetPulses > pulseCount) {
      startMovingUp();
      positionState->setVal(1); // Going to max
    } else if (targetPulses < pulseCount) {
      startMovingDown();
      positionState->setVal(0); // Going to min
    } else {
      hasPendingTarget = false;
      return;
    }

    Serial.printf("[Move] %.1f -> %.1f cm\n", currentHeight, targetHeight);

    // Update LCD
    if (!backlightOn) {
      lcd.backlight();
      backlightOn = true;
    }

    lcdShowTwoLines("Moving to:", String(targetHeight, 1) + " cm");

    // Transition to moving state
    currentState = MOVING_TO_TARGET;
    movementStartTime = millis();
    hasPendingTarget = false;
  }

  void handleMovementProgress() {
    // Update LCD periodically
    static unsigned long lastLcdUpdate = 0;
    if (millis() - lastLcdUpdate > 300) {
      float currentHeight = pulsesToHeight(pulseCount);
      float targetHeight = pulsesToHeight(targetPulses);

      char buf[16];
      snprintf(buf, sizeof(buf), "%.1f->%.1f", currentHeight, targetHeight);
      lcdShowTwoLines("Moving:", String(buf));

      lastLcdUpdate = millis();
    }

    // Check if we've reached the target
    int pulseDifference = abs(pulseCount - targetPulses);
    bool timeoutReached = (millis() - movementStartTime) > MOVEMENT_TIMEOUT;

    if (pulseDifference <= ERROR_THRESHOLD || timeoutReached) {
      if (timeoutReached) {
        Serial.printf("[Timeout] %.1f cm (%d pulses)\n", pulsesToHeight(pulseCount), pulseCount);
      } else {
        Serial.printf("[Done] %.1f cm (%d pulses)\n", pulsesToHeight(pulseCount), pulseCount);
      }

      currentState = FINALIZING;
    }
  }

  void finalizeMovement() {
    // Stop the motor
    smoothStopMotor();
    positionState->setVal(2); // Stopped
    lastMotorStopTime = millis();

    // Calibrate position
    delay(500);
    calibrateWithTof();

    // Save position
    savePulseCount();

    // Update HomeKit position to actual position
    updateCurrentPosition();
    target->setVal(current->getVal()); // Sync target to actual position

    // Update LCD
    float finalHeight = pulsesToHeight(pulseCount);
    lcdShowTwoLines("Height:", String(finalHeight, 1) + " cm");

    // Note: currentState already set to IDLE before calling this function
  }

  void updateCurrentPosition() {
    float percentage = pulsesToPercentage(pulseCount);
    if (percentage < 0) percentage = 0;
    if (percentage > 100) percentage = 100;

    current->setVal((int)percentage);
  }
};

// ===== MAIN SETUP =====
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.printf("[Boot] Desk Controller v3.1\n");

  // Initialize storage
  initStorage();

  // Initialize I²C
  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcdShowTwoLines("Initializing...", "");

  // Initialize ToF sensor
  initToFSensor();

  // Load saved position
  loadPulseCount();

  // Fix negative pulse count
  if (pulseCount < 0) {
    pulseCount = 0;
    savePulseCount();
    Serial.printf("[Init] Fixed negative pulse count\n");
  }

  // Initialize motor control
  initMotorControl();

  // Initialize hall sensors
  initHallSensors();

  // Initial calibration
  delay(1000);
  calibrateWithTof();

  // Show initial height
  float initialHeight = pulsesToHeight(pulseCount);
  Serial.printf("[Init] Height: %.1f cm (%d pulses)\n", initialHeight, pulseCount);

  // Connect to WiFi
  delay(2000);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("[Init] Connecting to WiFi...\n");
  // More robust WiFi connection
  WiFi.persistent(true);
  WiFi.setAutoReconnect(true);
  delay(2000); // Give more time for WiFi to stabilize

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(100);
    Serial.print('.');
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[Init] WiFi connected\n");
  } else {
    Serial.printf("[Init] WiFi FAILED\n");

    // Show failure on LCD
    lcdShowTwoLines("WiFi FAILED!", "Check Router");

    // Keep showing error (don't continue setup)
    while(true) {
      delay(1000);
      // Blink the display to draw attention
      lcd.noBacklight();
      delay(500);
      lcd.backlight();
      delay(500);
    }
  }

  delay(2000);
  // Initialize HomeSpan
  Serial.printf("[Init] HomeSpan...\n");
  homeSpan.setLogLevel(0);
  homeSpan.enableOTA();
  homeSpan.setPairingCode("46637726");
  delay(2000);
  homeSpan.begin(Category::WindowCoverings, "Smart Desk");

  // Create HomeKit accessory
  new SpanAccessory();
    new Service::AccessoryInformation();
      new Characteristic::Identify();
      new Characteristic::Manufacturer("DIY");
      new Characteristic::Model("Smart Standing Desk");
      new Characteristic::SerialNumber("SMART-001");
      new Characteristic::FirmwareRevision("3.1");
    new SmartSliderDeskControl();

  // Final LCD update
  lcdShowTwoLines("HomeKit Ready", String(initialHeight, 1) + " cm");

  Serial.printf("[Boot] Ready. Pairing: 466-37-726\n");
}

// ===== MAIN LOOP =====
void loop() {
  homeSpan.poll();
  checkHallSensors();

  // LCD backlight timeout
  if (backlightOn && currentState == IDLE && lastMotorStopTime > 0) {
    if (millis() - lastMotorStopTime > BACKLIGHT_TIMEOUT) {
      lcd.noBacklight();
      backlightOn = false;
    }
  }

  // Status heartbeat every 30 seconds when idle
  static unsigned long lastHeartbeat = 0;
  if (currentState == IDLE && millis() - lastHeartbeat > 30000) {
    Serial.printf("[Idle] %.1f cm\n", pulsesToHeight(pulseCount));
    lastHeartbeat = millis();
  }
}
