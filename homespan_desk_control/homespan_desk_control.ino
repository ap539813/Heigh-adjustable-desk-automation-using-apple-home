/*
 * Smart Debounced Slider Desk Control
 *
 * Key Features:
 * - VL53L0X ToF is the sole position sensor — no hall sensor
 * - Slider changes are captured but movement is delayed
 * - 1-second debounce period after last slider change
 * - Slider position updates after movement completion
 */
#include <Arduino.h>
#include <EEPROM.h>
#include <HomeSpan.h>
#include <LiquidCrystal_I2C.h>
#include <VL53L0X.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>

// Web server for REST API (port 8080 — port 80 is used by HomeSpan HAP)
WebServer webServer(8080);

// ===== CONFIGURATION =====
const char *WIFI_SSID = "A-19_4G";
const char *WIFI_PASSWORD = "lebhikhari";

// I2C pins
const uint8_t SDA_PIN = 21;
const uint8_t SCL_PIN = 22;

// BTS7960 motor driver pins
const uint8_t R_EN_PIN = 25;
const uint8_t L_EN_PIN = 26;
const uint8_t R_PWM_PIN = 14;
const uint8_t L_PWM_PIN = 27;

// Physical button pins (active LOW — wire button between pin and GND)
const uint8_t BTN_UP_PIN   = 16;  // P16 — press-and-hold to go up
const uint8_t BTN_DOWN_PIN = 17;  // P17 — press-and-hold to go down
const unsigned long BTN_DEBOUNCE_MS = 50;

// Height range
const float MIN_HEIGHT_CM = 69.0;
const float MAX_HEIGHT_CM = 107.0;
const float HEIGHT_TOLERANCE_CM = 1.5; // stop within this of target
const int MAX_TOF_FAILURES = 5; // consecutive bad reads → emergency stop

// EEPROM storage
const int EEPROM_SIZE = 16;
const int EEPROM_MAGIC_ADDR = 0;
const int EEPROM_HEIGHT_ADDR = 4; // stores height × 10 as int
const int EEPROM_MAGIC = 0x4445534B;

// Motor settings
const uint8_t TARGET_SPEED = 249;
const uint8_t DISTANCE_CYCLE = 15;
const uint8_t PWM_RES = 8;
const uint32_t PWM_FREQ = 1000;
const uint8_t RAMP_STEP = 10;
const unsigned RAMP_DELAY = 10;

// System settings
const unsigned long BACKLIGHT_TIMEOUT = 10000;
const unsigned long MOVEMENT_TIMEOUT = 15000;
const unsigned long SLIDER_DEBOUNCE_TIME = 1000;

// Movement states
enum MovementState { IDLE, MOVING_TO_TARGET, MANUAL_MOVING };
enum Direction { DIR_NONE = 0, DIR_UP = 1, DIR_DOWN = -1 };

// ===== GLOBAL VARIABLES =====
MovementState currentState = IDLE;
Direction currentDir = DIR_NONE;
float currentHeightCm = MIN_HEIGHT_CM;
float targetHeightCm = MIN_HEIGHT_CM;
unsigned long lastSliderChangeTime = 0;
bool backlightOn = true;
unsigned long lastMotorStopTime = 0;

// Button state tracking
bool btnUpPrev  = false;   // previous debounced state
bool btnDownPrev = false;
unsigned long btnUpLastChange  = 0;
unsigned long btnDownLastChange = 0;
unsigned long lastManualLcdUpdate = 0;

// I²C devices
VL53L0X tof;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Forward declarations
struct SmartSliderDeskControl;
SmartSliderDeskControl *deskControl = nullptr;

// ===== UTILITY FUNCTIONS =====
float heightToPercentage(float height) {
  return ((height - MIN_HEIGHT_CM) / (MAX_HEIGHT_CM - MIN_HEIGHT_CM)) * 100.0;
}

float percentageToHeight(float percentage) {
  return MIN_HEIGHT_CM + (percentage / 100.0) * (MAX_HEIGHT_CM - MIN_HEIGHT_CM);
}

// ===== EEPROM FUNCTIONS =====
void initStorage() {
  EEPROM.begin(EEPROM_SIZE);
  Serial.printf("[Init] EEPROM ready\n");
}

void saveHeight() {
  EEPROM.writeInt(EEPROM_HEIGHT_ADDR, (int)(currentHeightCm * 10));
  EEPROM.commit();
  Serial.printf("[EEPROM] Saved: %.1f cm\n", currentHeightCm);
}

void loadHeight() {
  int magic = EEPROM.readInt(EEPROM_MAGIC_ADDR);
  if (magic != EEPROM_MAGIC) {
    Serial.printf("[EEPROM] First run, initializing\n");
    EEPROM.writeInt(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
    EEPROM.writeInt(EEPROM_HEIGHT_ADDR, (int)(MIN_HEIGHT_CM * 10));
    EEPROM.commit();
    currentHeightCm = MIN_HEIGHT_CM;
  } else {
    currentHeightCm = EEPROM.readInt(EEPROM_HEIGHT_ADDR) / 10.0f;
    Serial.printf("[EEPROM] Loaded: %.1f cm\n", currentHeightCm);
  }
  currentHeightCm = constrain(currentHeightCm, MIN_HEIGHT_CM, MAX_HEIGHT_CM);
}

// ===== TOF SENSOR FUNCTIONS =====
bool initToFSensor() {
  bool success = tof.init();
  if (success) {
    Serial.printf("[Init] ToF ready\n");
    tof.setTimeout(500);
    tof.setMeasurementTimingBudget(20000); // 20ms — fast reads during movement
    tof.startContinuous(20);               // new reading every 20ms
  } else {
    Serial.printf("[Init] ToF FAILED\n");
  }
  return success;
}

// Averaged fast read — 3 samples reduce vibration noise during movement
float readHeightNow() {
  float total = 0;
  int valid = 0;
  for (int i = 0; i < 3; i++) {
    uint16_t dist_mm = tof.readRangeContinuousMillimeters();
    if (!tof.timeoutOccurred()) {
      total += dist_mm / 10.0f;
      valid++;
    }
  }
  return (valid > 0) ? total / valid : -1;
}

// Averaged read — used for accurate post-move calibration
float measureHeightWithTof() {
  float total = 0;
  int valid = 0;
  for (uint8_t s = 0; s < DISTANCE_CYCLE; s++) {
    uint16_t dist_mm = tof.readRangeContinuousMillimeters();
    if (!tof.timeoutOccurred()) {
      total += dist_mm / 10.0f;
      valid++;
    }
    delay(5);
  }
  return (valid > 0) ? total / valid : -1;
}

void calibrateWithTof() {
  float measured = measureHeightWithTof();
  if (measured > 0 && measured >= (MIN_HEIGHT_CM - 5.0) &&
      measured <= (MAX_HEIGHT_CM + 5.0)) {
    float old = currentHeightCm;
    currentHeightCm = constrain(measured, MIN_HEIGHT_CM, MAX_HEIGHT_CM);
    Serial.printf("[ToF] Calibrated: %.1f cm (was %.1f cm)\n", currentHeightCm,
                  old);
  } else {
    Serial.printf("[ToF] Out of range: %.1f cm, keeping %.1f cm\n", measured,
                  currentHeightCm);
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
  uint8_t pin = (currentDir == DIR_UP)     ? L_PWM_PIN
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

// ===== LCD HELPERS =====

// Pad string to fixed width (prevents leftover characters on LCD)
String padRight(String s, int width) {
  while (s.length() < (unsigned)width)
    s += ' ';
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
    pendingTargetPercent = target->getNewVal();
    hasPendingTarget = true;
    lastSliderChangeTime = millis();
    return true;
  }

  void loop() override {
    if (currentState == IDLE && hasPendingTarget &&
        millis() - lastSliderChangeTime >= SLIDER_DEBOUNCE_TIME) {
      currentState = MOVING_TO_TARGET;
      startMovementToTarget();
      currentState = IDLE;
    }
  }

  void startMovementToTarget() {
    targetHeightCm = percentageToHeight(pendingTargetPercent);
    hasPendingTarget = false;

    if (abs(currentHeightCm - targetHeightCm) < HEIGHT_TOLERANCE_CM)
      return;

    if (targetHeightCm > currentHeightCm) {
      startMovingUp();
      positionState->setVal(1);
    } else {
      startMovingDown();
      positionState->setVal(0);
    }

    Serial.printf("[Move] %.1f -> %.1f cm\n", currentHeightCm, targetHeightCm);
    if (!backlightOn) {
      lcd.backlight();
      backlightOn = true;
    }
    lcdShowTwoLines("Moving to:", String(targetHeightCm, 1) + " cm");

    // ---- ToF MOVEMENT LOOP ----
    unsigned long startTime = millis();
    unsigned long lastLcdUpdate = 0;
    unsigned long lastSerialPrint = 0;
    int failCount = 0;
    int doneCount = 0; // consecutive readings within tolerance required to stop

    while (true) {
      unsigned long now = millis();

      // Read current height from ToF
      float h = readHeightNow();
      if (h > 0 && h >= (MIN_HEIGHT_CM - 5.0) && h <= (MAX_HEIGHT_CM + 5.0)) {
        currentHeightCm = constrain(h, MIN_HEIGHT_CM, MAX_HEIGHT_CM);
        failCount = 0;
      } else {
        failCount++;
        if (failCount >= MAX_TOF_FAILURES) {
          Serial.printf("[ToF] %d consecutive failures — stopping for safety\n",
                        failCount);
          break;
        }
      }

      // LCD every 300ms
      if (now - lastLcdUpdate > 300) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%.1f->%.1f", currentHeightCm,
                 targetHeightCm);
        lcdShowTwoLines("Moving:", String(buf));
        lastLcdUpdate = now;
      }

      // Serial every 1s
      if (now - lastSerialPrint > 1000) {
        Serial.printf("[Height] %.1f cm -> %.1f cm\n", currentHeightCm,
                      targetHeightCm);
        lastSerialPrint = now;
      }

      // Target reached? Require 3 consecutive readings within tolerance
      // so a single noise spike can never trigger an early stop.
      if (abs(currentHeightCm - targetHeightCm) <= HEIGHT_TOLERANCE_CM) {
        doneCount++;
        if (doneCount >= 3) {
          Serial.printf("[Done] %.1f cm\n", currentHeightCm);
          break;
        }
      } else {
        doneCount = 0;
      }

      // Timeout?
      if (now - startTime > MOVEMENT_TIMEOUT) {
        Serial.printf("[Timeout] %.1f cm\n", currentHeightCm);
        break;
      }
    }

    // Stop motor
    smoothStopMotor();
    positionState->setVal(2);
    lastMotorStopTime = millis();

    // Final accurate calibration (averaged readings, desk settled)
    delay(500);
    calibrateWithTof();

    // Save and sync HomeKit
    saveHeight();
    updateCurrentPosition();
    target->setVal(current->getVal());
    lcdShowTwoLines("Height:", String(currentHeightCm, 1) + " cm");
  }

  void updateCurrentPosition() {
    float pct = heightToPercentage(currentHeightCm);
    pct = constrain(pct, 0.0f, 100.0f);
    current->setVal((int)pct);
  }
};

// ===== WEB SERVER API =====
void sendCorsHeaders() {
  webServer.sendHeader("Access-Control-Allow-Origin", "*");
  webServer.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  webServer.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

void handleApiStatus() {
  sendCorsHeaders();
  float pct = heightToPercentage(currentHeightCm);
  float targetPct = heightToPercentage(targetHeightCm);
  const char *stateStr = (currentState == IDLE) ? "idle" : "moving";
  const char *dirStr = (currentDir == DIR_UP)     ? "up"
                       : (currentDir == DIR_DOWN) ? "down"
                                                  : "none";

  char json[256];
  snprintf(json, sizeof(json),
           "{\"height_cm\":%.1f,\"height_pct\":%d,"
           "\"target_cm\":%.1f,\"target_pct\":%d,"
           "\"state\":\"%s\",\"direction\":\"%s\","
           "\"wifi_rssi\":%d,\"uptime_s\":%lu}",
           currentHeightCm, (int)constrain(pct, 0, 100), targetHeightCm,
           (int)constrain(targetPct, 0, 100), stateStr, dirStr, WiFi.RSSI(),
           millis() / 1000);

  webServer.send(200, "application/json", json);
  Serial.printf("[API] GET /api/status\n");
}

void handleApiMove() {
  sendCorsHeaders();

  if (currentState != IDLE) {
    webServer.send(409, "application/json",
                   "{\"ok\":false,\"message\":\"Desk is already moving\"}");
    return;
  }

  float newTargetCm = -1;

  if (webServer.hasArg("height_cm")) {
    newTargetCm = webServer.arg("height_cm").toFloat();
  } else if (webServer.hasArg("height_pct")) {
    float pct = webServer.arg("height_pct").toFloat();
    newTargetCm = percentageToHeight(constrain(pct, 0, 100));
  } else {
    webServer.send(
        400, "application/json",
        "{\"ok\":false,\"message\":\"Provide height_cm or height_pct\"}");
    return;
  }

  newTargetCm = constrain(newTargetCm, MIN_HEIGHT_CM, MAX_HEIGHT_CM);
  int targetPct = (int)constrain(heightToPercentage(newTargetCm), 0, 100);

  // Trigger movement via HomeKit service (keeps everything in sync)
  if (deskControl) {
    deskControl->target->setVal(targetPct);
    deskControl->pendingTargetPercent = targetPct;
    deskControl->hasPendingTarget = true;
    lastSliderChangeTime = millis();
  }

  char json[128];
  snprintf(json, sizeof(json),
           "{\"ok\":true,\"message\":\"Moving to %.1f cm\","
           "\"target_cm\":%.1f,\"target_pct\":%d}",
           newTargetCm, newTargetCm, targetPct);

  webServer.send(200, "application/json", json);
  Serial.printf("[API] POST /api/move -> %.1f cm (%d%%)\n", newTargetCm,
                targetPct);
}

void handleApiStop() {
  sendCorsHeaders();

  smoothStopMotor();
  currentState = IDLE;
  if (deskControl) {
    deskControl->hasPendingTarget = false;
    deskControl->positionState->setVal(2); // Stopped
    deskControl->updateCurrentPosition();
    deskControl->target->setVal(deskControl->current->getVal());
  }

  char json[96];
  snprintf(json, sizeof(json),
           "{\"ok\":true,\"message\":\"Emergency stop executed\","
           "\"height_cm\":%.1f}",
           currentHeightCm);

  webServer.send(200, "application/json", json);
  Serial.printf("[API] POST /api/stop\n");
}

void handleApiConfig() {
  sendCorsHeaders();

  char json[192];
  snprintf(json, sizeof(json),
           "{\"min_height_cm\":%.1f,\"max_height_cm\":%.1f,"
           "\"tolerance_cm\":%.1f,\"firmware_version\":\"3.2\","
           "\"ip\":\"%s\",\"mac\":\"%s\"}",
           MIN_HEIGHT_CM, MAX_HEIGHT_CM, HEIGHT_TOLERANCE_CM,
           WiFi.localIP().toString().c_str(), WiFi.macAddress().c_str());

  webServer.send(200, "application/json", json);
  Serial.printf("[API] GET /api/config\n");
}

void handleCorsOptions() {
  sendCorsHeaders();
  webServer.send(204);
}

void setupWebServer() {
  webServer.on("/api/status", HTTP_GET, handleApiStatus);
  webServer.on("/api/move", HTTP_POST, handleApiMove);
  webServer.on("/api/stop", HTTP_POST, handleApiStop);
  webServer.on("/api/config", HTTP_GET, handleApiConfig);

  // Handle CORS preflight for all /api/ routes
  webServer.on("/api/status", HTTP_OPTIONS, handleCorsOptions);
  webServer.on("/api/move", HTTP_OPTIONS, handleCorsOptions);
  webServer.on("/api/stop", HTTP_OPTIONS, handleCorsOptions);
  webServer.on("/api/config", HTTP_OPTIONS, handleCorsOptions);

  webServer.begin();
  Serial.printf("[Init] Web API server started on port 8080\n");
  Serial.printf("[Init] API: http://%s:8080/api/status\n",
                WiFi.localIP().toString().c_str());
}

// ===== MAIN SETUP =====
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.printf("[Boot] Desk Controller v3.2\n");

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

  // Load last known height (overridden by live ToF below)
  loadHeight();

  // Initialize motor control
  initMotorControl();

  // Initialize physical buttons (internal pull-ups)
  pinMode(BTN_UP_PIN, INPUT_PULLUP);
  pinMode(BTN_DOWN_PIN, INPUT_PULLUP);
  Serial.printf("[Init] Buttons ready (UP=GPIO%d, DOWN=GPIO%d)\n", BTN_UP_PIN, BTN_DOWN_PIN);

  // Live calibration on boot
  delay(1000);
  calibrateWithTof();

  Serial.printf("[Init] Height: %.1f cm\n", currentHeightCm);

  // Connect to WiFi — retry up to MAX_WIFI_RETRIES times
  const int MAX_WIFI_RETRIES = 5;
  WiFi.persistent(true);
  WiFi.setAutoReconnect(true);

  bool wifiConnected = false;
  for (int retry = 1; retry <= MAX_WIFI_RETRIES; retry++) {
    Serial.printf("[WiFi] Attempt %d of %d...\n", retry, MAX_WIFI_RETRIES);
    char lcdBuf[17];
    snprintf(lcdBuf, sizeof(lcdBuf), "Attempt %d of %d", retry,
             MAX_WIFI_RETRIES);
    lcdShowTwoLines("WiFi Connect:", lcdBuf);

    WiFi.disconnect(true); // full disconnect before each attempt
    delay(1000);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // Wait up to 5 seconds per attempt (50 × 100ms)
    int polls = 0;
    while (WiFi.status() != WL_CONNECTED && polls < 50) {
      delay(100);
      Serial.print('.');
      polls++;
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
      wifiConnected = true;
      Serial.printf("[WiFi] Connected on attempt %d\n", retry);
      break;
    }

    Serial.printf("[WiFi] Attempt %d failed\n", retry);
    if (retry < MAX_WIFI_RETRIES) {
      lcdShowTwoLines("WiFi Failed", "Retrying...");
      delay(3000); // pause before next retry
    }
  }

  if (wifiConnected) {
    Serial.printf("[Init] WiFi connected: %s\n",
                  WiFi.localIP().toString().c_str());
    lcdShowTwoLines("WiFi Connected!", WiFi.localIP().toString().c_str());
    delay(1000);
  } else {
    Serial.printf("[Init] WiFi FAILED after %d attempts\n", MAX_WIFI_RETRIES);
    lcdShowTwoLines("WiFi FAILED!", "Check Router");
    while (true) {
      delay(1000);
      lcd.noBacklight();
      delay(500);
      lcd.backlight();
      delay(500);
    }
  }

  delay(2000);
  Serial.printf("[Init] HomeSpan...\n");
  homeSpan.setLogLevel(0);
  homeSpan.enableOTA();
  homeSpan.setPairingCode("46637726");
  delay(2000);
  homeSpan.begin(Category::WindowCoverings, "Smart Desk");

  new SpanAccessory();
  new Service::AccessoryInformation();
  new Characteristic::Identify();
  new Characteristic::Manufacturer("DIY");
  new Characteristic::Model("Smart Standing Desk");
  new Characteristic::SerialNumber("SMART-001");
  new Characteristic::FirmwareRevision("3.2");
  new SmartSliderDeskControl();

  // Start web API server (WiFi is already connected at this point)
  setupWebServer();

  lcdShowTwoLines("HomeKit Ready", String(currentHeightCm, 1) + " cm");

  // Start backlight timeout from boot
  lastMotorStopTime = millis();

  Serial.printf("[Boot] Ready. Pairing: 466-37-726\n");
}

// ===== BUTTON HELPERS =====

// Debounced read: returns true if button is pressed (active LOW)
bool readButtonDebounced(uint8_t pin, bool &prevState, unsigned long &lastChange) {
  bool raw = (digitalRead(pin) == LOW);
  unsigned long now = millis();
  if (raw != prevState && (now - lastChange) >= BTN_DEBOUNCE_MS) {
    prevState = raw;
    lastChange = now;
  }
  return prevState;
}

// Emergency-cancel any HomeKit-driven move in progress
void cancelHomeKitMove() {
  Serial.printf("[Btn] Overriding HomeKit move — stopping motor\n");
  smoothStopMotor();
  currentState = IDLE;
  if (deskControl) {
    deskControl->hasPendingTarget = false;
    deskControl->positionState->setVal(2); // Stopped
    calibrateWithTof();
    saveHeight();
    deskControl->updateCurrentPosition();
    deskControl->target->setVal(deskControl->current->getVal());
  }
  Serial.printf("[Btn] HomeKit move cancelled at %.1f cm\n", currentHeightCm);
}

// Sync HomeKit after a manual move ends
void syncHomeKitAfterManual() {
  delay(300);
  calibrateWithTof();
  saveHeight();
  if (deskControl) {
    deskControl->updateCurrentPosition();
    deskControl->target->setVal(deskControl->current->getVal());
  }
  lcdShowTwoLines("Height:", String(currentHeightCm, 1) + " cm");
  lastMotorStopTime = millis();
  Serial.printf("[Btn] HomeKit synced to %.1f cm\n", currentHeightCm);
}

// ===== MAIN LOOP =====
void loop() {
  homeSpan.poll();
  webServer.handleClient();

  // --- Physical button handling ---
  bool upNow   = readButtonDebounced(BTN_UP_PIN,   btnUpPrev,   btnUpLastChange);
  bool downNow = readButtonDebounced(BTN_DOWN_PIN, btnDownPrev, btnDownLastChange);

  // Both pressed = safety stop
  if (upNow && downNow) {
    if (currentState == MANUAL_MOVING) {
      Serial.printf("[Btn] BOTH pressed — emergency stop!\n");
      smoothStopMotor();
      currentState = IDLE;
      syncHomeKitAfterManual();
    }
  }
  // UP button held (and not both)
  else if (upNow && !downNow) {
    // If a HomeKit move is running, cancel it first
    if (currentState == MOVING_TO_TARGET) {
      cancelHomeKitMove();
    }

    if (currentState != MANUAL_MOVING) {
      // Check upper limit before starting
      if (currentHeightCm >= MAX_HEIGHT_CM) {
        Serial.printf("[Btn] UP pressed but already at max (%.1f cm)\n", currentHeightCm);
      } else {
        currentState = MANUAL_MOVING;
        startMovingUp();
        if (!backlightOn) { lcd.backlight(); backlightOn = true; }
        lcdShowTwoLines("Manual UP", String(currentHeightCm, 1) + " cm");
        Serial.printf("[Btn] UP pressed — moving up from %.1f cm\n", currentHeightCm);
      }
    } else if (currentDir == DIR_UP) {
      // Still held — read ToF and enforce upper limit
      float h = readHeightNow();
      if (h > 0 && h >= (MIN_HEIGHT_CM - 5.0) && h <= (MAX_HEIGHT_CM + 5.0)) {
        currentHeightCm = constrain(h, MIN_HEIGHT_CM, MAX_HEIGHT_CM);
      }
      if (currentHeightCm >= MAX_HEIGHT_CM) {
        Serial.printf("[Btn] Upper limit reached (%.1f cm) — stopping\n", currentHeightCm);
        smoothStopMotor();
        currentState = IDLE;
        syncHomeKitAfterManual();
      } else {
        // Update LCD periodically
        unsigned long now = millis();
        if (now - lastManualLcdUpdate > 300) {
          lcdShowTwoLines("Manual UP", String(currentHeightCm, 1) + " cm");
          Serial.printf("[Btn] UP holding — %.1f cm\n", currentHeightCm);
          lastManualLcdUpdate = now;
        }
      }
    }
  }
  // DOWN button held (and not both)
  else if (downNow && !upNow) {
    if (currentState == MOVING_TO_TARGET) {
      cancelHomeKitMove();
    }

    if (currentState != MANUAL_MOVING) {
      if (currentHeightCm <= MIN_HEIGHT_CM) {
        Serial.printf("[Btn] DOWN pressed but already at min (%.1f cm)\n", currentHeightCm);
      } else {
        currentState = MANUAL_MOVING;
        startMovingDown();
        if (!backlightOn) { lcd.backlight(); backlightOn = true; }
        lcdShowTwoLines("Manual DOWN", String(currentHeightCm, 1) + " cm");
        Serial.printf("[Btn] DOWN pressed — moving down from %.1f cm\n", currentHeightCm);
      }
    } else if (currentDir == DIR_DOWN) {
      float h = readHeightNow();
      if (h > 0 && h >= (MIN_HEIGHT_CM - 5.0) && h <= (MAX_HEIGHT_CM + 5.0)) {
        currentHeightCm = constrain(h, MIN_HEIGHT_CM, MAX_HEIGHT_CM);
      }
      if (currentHeightCm <= MIN_HEIGHT_CM) {
        Serial.printf("[Btn] Lower limit reached (%.1f cm) — stopping\n", currentHeightCm);
        smoothStopMotor();
        currentState = IDLE;
        syncHomeKitAfterManual();
      } else {
        unsigned long now = millis();
        if (now - lastManualLcdUpdate > 300) {
          lcdShowTwoLines("Manual DOWN", String(currentHeightCm, 1) + " cm");
          Serial.printf("[Btn] DOWN holding — %.1f cm\n", currentHeightCm);
          lastManualLcdUpdate = now;
        }
      }
    }
  }
  // No button pressed
  else {
    if (currentState == MANUAL_MOVING) {
      Serial.printf("[Btn] Released — stopping at %.1f cm\n", currentHeightCm);
      smoothStopMotor();
      currentState = IDLE;
      syncHomeKitAfterManual();
    }
  }

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
    Serial.printf("[Idle] %.1f cm\n", currentHeightCm);
    lastHeartbeat = millis();
  }
}
