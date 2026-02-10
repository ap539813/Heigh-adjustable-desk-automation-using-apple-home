# Modular Code Structure

This Arduino project has been restructured into modular components for better maintainability and organization.

## File Structure

```
homespan_desk_control/
├── homespan_desk_control.ino       # Original monolithic code (backup)
├── homespan_desk_control_new.ino   # New modular main file
├── credentials.h                    # WiFi and HomeKit credentials (DO NOT COMMIT)
├── config.h                         # Hardware configuration and constants
├── utils.h / utils.cpp             # Utility functions (conversions)
├── storage.h / storage.cpp         # EEPROM storage management
├── motor_control.h / motor_control.cpp  # BTS7960 motor driver control
├── hall_sensor.h / hall_sensor.cpp # Hall sensor pulse counting
├── tof_sensor.h / tof_sensor.cpp   # VL53L0X ToF distance sensor
├── desk_service.h / desk_service.cpp    # HomeKit service implementation
└── .gitignore                       # Prevents credentials from being committed
```

## Modules Overview

### credentials.h
**Purpose**: Stores sensitive WiFi and HomeKit credentials
- WiFi SSID and Password
- HomeKit pairing code
- Device name
- **IMPORTANT**: Add this file to .gitignore to keep credentials secure

### config.h
**Purpose**: Hardware configuration and system constants
- Pin definitions (I2C, motors, hall sensors)
- Height and pulse calibration constants
- EEPROM settings
- Motor settings (PWM, speed, ramp)
- System timing constants
- Enums for movement states and directions

### utils.h/cpp
**Purpose**: Utility functions for conversions
- `pulsesToHeight()` - Convert pulses to height in cm
- `heightToPulses()` - Convert height in cm to pulses
- `pulsesToPercentage()` - Convert pulses to percentage (0-100)
- `percentageToPulses()` - Convert percentage to pulses

### storage.h/cpp
**Purpose**: EEPROM persistence management
- `initStorage()` - Initialize EEPROM
- `savePulseCount()` - Save current position to EEPROM
- `loadPulseCount()` - Load saved position from EEPROM
- Handles first-time setup and magic number verification

### motor_control.h/cpp
**Purpose**: BTS7960 motor driver control
- `init()` - Initialize motor pins and PWM
- `startMovingUp()` - Start upward movement with smooth ramp
- `startMovingDown()` - Start downward movement with smooth ramp
- `smoothStop()` - Smoothly stop motor movement
- Direction tracking and management

### hall_sensor.h/cpp
**Purpose**: Dual hall sensor pulse counting
- `init()` - Initialize hall sensor pins
- `check()` - Read sensors and update pulse count with debouncing
- Directional pulse counting (up/down detection)
- Debug output during movement

### tof_sensor.h/cpp
**Purpose**: VL53L0X distance sensor for height calibration
- `init()` - Initialize ToF sensor
- `measureHeight()` - Take multiple readings and return average height
- `calibrate()` - Calibrate pulse count based on ToF measurement
- Automatic validation of readings

### desk_service.h/cpp
**Purpose**: HomeKit WindowCovering service implementation
- Smart slider with debouncing (1-second settle time)
- Movement state machine (IDLE → MOVING → FINALIZING)
- LCD display updates
- Position synchronization with HomeKit
- Automatic ToF calibration after movement

### homespan_desk_control_new.ino
**Purpose**: Main application entry point
- Hardware initialization sequence
- WiFi connection with retry logic
- HomeSpan setup and configuration
- Main loop with hall sensor checking and backlight management

## Migration Guide

### To switch to the modular version:

1. **Backup your current code** (already saved as `homespan_desk_control.ino`)

2. **Create credentials.h** with your actual credentials:
   ```cpp
   const char* WIFI_SSID = "YourWiFiName";
   const char* WIFI_PASSWORD = "YourWiFiPassword";
   const char* HOMEKIT_PAIRING_CODE = "46637726";
   const char* HOMEKIT_DEVICE_NAME = "Smart Desk";
   ```

3. **Rename files**:
   - Rename `homespan_desk_control.ino` to `homespan_desk_control_old.ino` (or delete)
   - Rename `homespan_desk_control_new.ino` to `homespan_desk_control.ino`

4. **Compile and upload** using Arduino IDE or PlatformIO

## Benefits of Modular Structure

✅ **Separation of Concerns**: Each module handles a specific responsibility
✅ **Easier Testing**: Individual modules can be tested independently
✅ **Better Maintainability**: Changes to one component don't affect others
✅ **Credential Security**: Sensitive data is isolated and can be excluded from version control
✅ **Reusability**: Modules can be reused in other projects
✅ **Cleaner Code**: Smaller, focused files are easier to understand
✅ **Scalability**: Easy to add new features without cluttering existing code

## Configuration

### To customize for your desk:

1. **Edit `config.h`** for hardware changes:
   - Pin assignments
   - Height calibration constants
   - Motor speed and timing

2. **Edit `credentials.h`** for network changes:
   - WiFi credentials
   - HomeKit pairing code

## Dependencies

This project requires the following Arduino libraries:
- HomeSpan
- VL53L0X
- LiquidCrystal_I2C
- Wire (built-in)
- WiFi (built-in)
- EEPROM (built-in)

## Notes

- The original monolithic code is preserved as `homespan_desk_control.ino`
- All functionality remains identical to the original implementation
- The modular structure makes future enhancements much easier
- Credentials are now properly separated and can be kept secure
