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
const char* WIFI_SSID = "Shraddha_ 5G";
const char* WIFI_PASSWORD = "Password_leg@_bh!khari";

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
  Serial.println("📦 EEPROM storage initialized");
}

void savePulseCount() {
  EEPROM.writeInt(EEPROM_PULSES_ADDR, pulseCount);
  EEPROM.commit();
  Serial.println("💾 Saved pulse count: " + String(pulseCount));
}

void loadPulseCount() {
  int magic = EEPROM.readInt(EEPROM_MAGIC_ADDR);
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
}

// ===== TOF SENSOR FUNCTIONS =====
bool initToFSensor() {
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

float measureHeightWithTof() {
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

void calibrateWithTof() {
  Serial.println("🎯 Starting ToF calibration...");
  float measuredHeight = measureHeightWithTof();
  
  if (measuredHeight > 0 && measuredHeight >= MIN_HEIGHT_CM && measuredHeight <= MAX_HEIGHT_CM) {
    int oldPulseCount = pulseCount;
    pulseCount = heightToPulses(measuredHeight);
    
    Serial.println("✅ ToF Calibration:");
    Serial.println("   Before: " + String(oldPulseCount) + " pulses (" + String(pulsesToHeight(oldPulseCount)) + " cm)");
    Serial.println("   After:  " + String(pulseCount) + " pulses (" + String(measuredHeight) + " cm)");
    Serial.println("   Adjustment: " + String(pulseCount - oldPulseCount) + " pulses");
  } else {
    Serial.println("⚠️ ToF reading out of range or invalid - keeping current position");
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
  
  Serial.println("🔧 Motor control initialized");
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

void startMovingUp() {
  Serial.println("⬆️ Starting upward movement (L_PWM)");
  smoothStopMotor();
  delay(50);
  ledcWrite(R_PWM_PIN, 0);
  smoothStartMotor(L_PWM_PIN);
  currentDir = DIR_UP;
}

void startMovingDown() {
  Serial.println("⬇️ Starting downward movement (R_PWM)");
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
  Serial.println("🧲 Hall sensors initialized");
}

void checkHallSensors() {
  int sensor1State = digitalRead(HALL_SENSOR1_PIN);
  int sensor2State = digitalRead(HALL_SENSOR2_PIN);
  
  if (sensor1State == HIGH && sensor1LastState == LOW) {
    if (millis() - lastDebounceTime > debounceDelay) {
      if (sensor2State == LOW) {
        pulseCount++;
      } else {
        pulseCount--;
      }
      lastDebounceTime = millis();
      
      // Debug output during movement only
      if (currentState == MOVING_TO_TARGET) {
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
    
    Serial.println("🎛️ Smart slider desk control service created");
  }

  boolean update() override {
    int newTargetPercent = target->getNewVal();
    
    // Record the slider change but don't move yet
    pendingTargetPercent = newTargetPercent;
    hasPendingTarget = true;
    lastSliderChangeTime = millis();
    
    Serial.println("🎯 Slider changed to " + String(newTargetPercent) + "% - waiting for settle...");
    
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
        finalizeMovement();
        break;
    }
  }
  
  void startMovementToTarget() {
    targetPulses = percentageToPulses(pendingTargetPercent);
    float targetHeight = pulsesToHeight(targetPulses);
    float currentHeight = pulsesToHeight(pulseCount);
    
    Serial.println("\n🚀 STARTING MOVEMENT:");
    Serial.println("   Target: " + String(pendingTargetPercent) + "% = " + String(targetHeight, 1) + " cm (" + String(targetPulses) + " pulses)");
    Serial.println("   Current: " + String(pulsesToPercentage(pulseCount), 1) + "% = " + String(currentHeight, 1) + " cm (" + String(pulseCount) + " pulses)");
    
    // Determine direction and start movement
    if (targetPulses > pulseCount) {
      startMovingUp();
      positionState->setVal(1); // Going to max
      Serial.println("   Direction: UP ⬆️");
    } else if (targetPulses < pulseCount) {
      startMovingDown();
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
  
  void handleMovementProgress() {
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
    }
    
    // Check if we've reached the target
    int pulseDifference = abs(pulseCount - targetPulses);
    bool timeoutReached = (millis() - movementStartTime) > MOVEMENT_TIMEOUT;
    
    if (pulseDifference <= ERROR_THRESHOLD || timeoutReached) {
      if (timeoutReached) {
        Serial.println("⏰ Movement timeout reached");
      } else {
        Serial.println("✅ Target reached within " + String(pulseDifference) + " pulse tolerance");
      }
      
      currentState = FINALIZING;
    }
  }
  
  void finalizeMovement() {
    Serial.println("🏁 Finalizing movement...");
    
    // Stop the motor
    smoothStopMotor();
    positionState->setVal(2); // Stopped
    lastMotorStopTime = millis();
    
    // Calibrate position
    delay(200);
    calibrateWithTof();
    
    // Save position
    savePulseCount();
    
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
  
  Serial.println("\n\n🎛️ SMART DEBOUNCED SLIDER DESK CONTROL 🏠");
  Serial.println("============================================");
  
  // Initialize storage
  initStorage();
  
  // Initialize I²C
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");
  
  // Initialize ToF sensor
  initToFSensor();
  
  // Load saved position
  loadPulseCount();
  
  // Fix negative pulse count
  if (pulseCount < 0) {
    pulseCount = 0;
    savePulseCount();
    Serial.println("🔧 Fixed negative pulse count");
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
  Serial.println("📏 Initial height: " + String(initialHeight, 1) + " cm (" + String(pulseCount) + " pulses)");
  
  // Connect to WiFi
  delay(2000);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("📡 Connecting to WiFi");
  // More robust WiFi connection
  WiFi.persistent(true);
  WiFi.setAutoReconnect(true);
  delay(2000); // Give more time for WiFi to stabilize
  
  int attempts = 0;
  lcd.clear();
  lcd.setCursor(0, 0);
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(2000);
    Serial.print('.');
    lcd.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi connected! IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\n❌ WiFi connection failed!");
  
    // Show failure on LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi FAILED!");
    lcd.setCursor(0, 1);
    lcd.print("Check Router");
    
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
  Serial.println("🏠 Initializing HomeSpan...");
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
      new Characteristic::FirmwareRevision("3.0");
    new SmartSliderDeskControl();
  
  // Final LCD update
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("HomeKit Ready");
  lcd.setCursor(0, 1);
  lcd.print(String(initialHeight, 1) + " cm");
  
  Serial.println("\n🎉 SYSTEM READY!");
  Serial.println("📱 Add to Home app with code: 466-37-726");
  Serial.println("🎛️ Slider will wait 1 second after changes before moving");
  Serial.println("============================================\n");
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
    Serial.println("💓 System idle - Height: " + String(pulsesToHeight(pulseCount), 1) + " cm");
    lastHeartbeat = millis();
  }
}