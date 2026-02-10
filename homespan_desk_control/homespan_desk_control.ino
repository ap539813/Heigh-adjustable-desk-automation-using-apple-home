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
#include <HomeSpan.h>

// Include modular components
#include "credentials.h"
#include "config.h"
#include "utils.h"
#include "storage.h"
#include "motor_control.h"
#include "hall_sensor.h"
#include "tof_sensor.h"
#include "desk_service.h"

// ===== GLOBAL VARIABLES =====
// Movement control
MovementState currentState = IDLE;
int targetPulses = 0;
int pulseCount = 0;
unsigned long lastSliderChangeTime = 0;
unsigned long movementStartTime = 0;
bool backlightOn = true;
unsigned long lastMotorStopTime = 0;

// I²C LCD
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLS, LCD_ROWS);

// HomeKit desk service
SmartSliderDeskControl* deskControl = nullptr;

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
  ToFSensor::init();

  // Load saved position
  pulseCount = loadPulseCount();

  // Fix negative pulse count
  if (pulseCount < 0) {
    pulseCount = 0;
    savePulseCount(pulseCount);
    Serial.println("🔧 Fixed negative pulse count");
  }

  // Initialize motor control
  MotorControl::init();

  // Initialize hall sensors
  HallSensor::init();

  // Initial calibration
  delay(1000);
  pulseCount = ToFSensor::calibrate(pulseCount);

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
  homeSpan.setPairingCode(HOMEKIT_PAIRING_CODE);
  delay(2000);
  homeSpan.begin(Category::WindowCoverings, HOMEKIT_DEVICE_NAME);

  // Create HomeKit accessory
  new SpanAccessory();
    new Service::AccessoryInformation();
      new Characteristic::Identify();
      new Characteristic::Manufacturer("DIY");
      new Characteristic::Model("Smart Standing Desk");
      new Characteristic::SerialNumber("SMART-001");
      new Characteristic::FirmwareRevision("3.0");

    deskControl = new SmartSliderDeskControl(
      pulseCount,
      currentState,
      targetPulses,
      lastSliderChangeTime,
      movementStartTime,
      lastMotorStopTime,
      backlightOn,
      lcd
    );

  // Final LCD update
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("HomeKit Ready");
  lcd.setCursor(0, 1);
  lcd.print(String(initialHeight, 1) + " cm");

  Serial.println("\n🎉 SYSTEM READY!");
  Serial.println("📱 Add to Home app with code: " + String(HOMEKIT_PAIRING_CODE));
  Serial.println("🎛️ Slider will wait 1 second after changes before moving");
  Serial.println("============================================\n");
}

// ===== MAIN LOOP =====
void loop() {
  homeSpan.poll();

  // Check hall sensors and update pulse count
  pulseCount = HallSensor::check(pulseCount, currentState);

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
