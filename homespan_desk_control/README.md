# Smart Standing Desk Controller

ESP32-based smart standing desk with HomeKit integration, ToF height sensing, and LCD display.

## 🎯 Features

- **HomeKit Integration** - Control via Apple Home app with debounced slider
- **Real-time Height Display** - Shows current height on 16x2 LCD during movement
- **ToF Calibration** - Automatic height correction using VL53L0X sensor
- **Position Memory** - EEPROM storage of desk position
- **Smooth Movement** - Ramped motor control for gentle starts/stops
- **1-Second Debounce** - Prevents jittery movement from rapid slider changes

## 📁 Project Structure

```
homespan_desk_control/
├── homespan_desk_control.ino  # Main entry point
├── credentials.h              # WiFi & HomeKit credentials (gitignored)
├── credentials.h.template     # Template for credentials
├── config.h                   # Hardware pins & constants
├── utils.h/cpp               # Height/pulse conversion
├── storage.h/cpp             # EEPROM persistence
├── tof_sensor.h/cpp          # VL53L0X ToF sensor
├── motor_control.h/cpp       # BTS7960 motor driver
├── hall_sensor.h/cpp         # Dual hall sensor tracking
├── desk_service.h/cpp        # HomeKit service
└── .gitignore                # Protects credentials.h
```

## 🚀 Setup Instructions

### 1. Hardware Requirements

- ESP32 development board
- BTS7960 motor driver
- Dual hall effect sensors (position tracking)
- VL53L0X ToF distance sensor
- 16x2 I2C LCD display
- Linear actuator motor

### 2. Software Setup

1. **Clone the repository**
   ```bash
   git clone <your-repo-url>
   cd homespan_desk_control
   ```

2. **Create credentials file**
   ```bash
   cp credentials.h.template credentials.h
   ```

3. **Edit credentials.h** with your WiFi details:
   ```cpp
   const char* WIFI_SSID = "YourWiFiName";
   const char* WIFI_PASSWORD = "YourWiFiPassword";
   const char* HOMEKIT_PAIRING_CODE = "46637726";  // Or your custom code
   ```

4. **Install required libraries** in Arduino IDE:
   - HomeSpan
   - VL53L0X
   - LiquidCrystal_I2C
   - Wire (built-in)
   - WiFi (built-in)
   - EEPROM (built-in)

5. **Upload to ESP32**
   - Select your ESP32 board in Arduino IDE
   - Upload the sketch

### 3. HomeKit Pairing

1. Open the **Home app** on your iPhone/iPad
2. Tap **+** → **Add Accessory**
3. Enter pairing code: **466-37-726** (or your custom code)
4. The desk will appear as "Smart Desk"

## 🔧 Hardware Configuration

Edit `config.h` to match your hardware setup:

### Pin Assignments
- **I2C**: SDA=21, SCL=22
- **Motor Driver**: R_PWM=14, L_PWM=27, R_EN=25, L_EN=26
- **Hall Sensors**: Pin 32, Pin 33

### Height Calibration
- Min height: 69.0 cm (0 pulses)
- Max height: 107.0 cm (1840 pulses)

Adjust these values in `config.h` to match your desk's range.

## 📱 Usage

### Via HomeKit
1. Open Apple Home app
2. Drag the slider to set target height
3. Wait 1 second (debounce period)
4. Desk moves to position automatically

### LCD Display States
- **Idle**: `Height: XX.X cm`
- **Moving**: `Moving: XX.X->YY.Y`
- **Startup**: `Initializing...` / `HomeKit Ready`

## 🔐 Security Notes

- `credentials.h` contains sensitive WiFi passwords
- This file is **gitignored** and won't be committed
- Never commit real credentials to version control
- Use `credentials.h.template` as a reference

## 🛠️ Troubleshooting

### WiFi Connection Failed
- Check SSID/password in `credentials.h`
- Ensure 2.4GHz WiFi (ESP32 doesn't support 5GHz)
- LCD will show "WiFi FAILED!" and blink

### Height Inaccurate
- Recalibrate ToF sensor mounting
- Check hall sensor connections
- Verify `PULSES_AT_MIN/MAX_HEIGHT` values

### Movement Timeout
- Default timeout is 15 seconds
- Check motor driver connections
- Ensure actuator isn't mechanically stuck

## 📄 License

MIT License - Feel free to modify and use for your own projects!

## 🙏 Credits

Built with:
- [HomeSpan](https://github.com/HomeSpan/HomeSpan) - ESP32 HomeKit library
- [VL53L0X](https://github.com/pololu/vl53l0x-arduino) - ToF sensor library
