# Smart Standing Desk — HomeKit + REST API

An ESP32-based controller for a height-adjustable desk with **Apple HomeKit** integration via [HomeSpan](https://github.com/HomeSpan/HomeSpan) and a **REST API** for external dashboards.

## Features

- 🏠 **Apple HomeKit** — control desk height from the Home app, Siri, and automations
- 🌐 **REST API** — JSON endpoints for monitoring and control from any HTTP client
- 📏 **VL53L0X ToF sensor** — real-time height measurement (no hall sensor needed)
- 🖥️ **LCD display** — shows current height, movement status, and WiFi info
- ⚡ **BTS7960 motor driver** — smooth ramp-up/down for quiet, safe movement
- 💾 **EEPROM persistence** — remembers last position across reboots
- 🔄 **OTA updates** — flash new firmware over WiFi

## Hardware

| Component | Purpose |
|---|---|
| ESP32 DevKit | Main controller |
| BTS7960 | Motor driver (R_EN=25, L_EN=26, R_PWM=14, L_PWM=27) |
| VL53L0X | Time-of-Flight distance sensor (I²C) |
| 16×2 LCD (I²C) | Status display (address 0x27) |
| Height-adjustable desk frame | The desk itself |

## Setup

1. Install [Arduino IDE](https://www.arduino.cc/en/software) with ESP32 board support
2. Install libraries: **HomeSpan**, **VL53L0X** (Pololu), **LiquidCrystal_I2C**
3. Update WiFi credentials in `homespan_desk_control.ino`:
   ```cpp
   const char *WIFI_SSID = "YOUR_SSID";
   const char *WIFI_PASSWORD = "YOUR_PASSWORD";
   ```
4. Flash to ESP32
5. Pair with Apple Home using code: **`466-37-726`**

## Architecture

```
┌──────────────────────────────────────────────┐
│              ESP32 (Smart Desk)               │
│                                               │
│   Port 80:  HomeSpan HAP Server (Apple Home)  │
│   Port 8080: WebServer REST API (Dashboard)   │
│                                               │
│   Both share the same desk control logic      │
│   and stay perfectly in sync.                 │
└──────────────────────────────────────────────┘
        ▲                       ▲
        │ HAP (encrypted)       │ HTTP (JSON)
        │                       │
   Apple Home              AI Dashboard
   (iPhone/Mac)           (Future Arduino)
```

---

## REST API Reference

**Base URL**: `http://<ESP32_IP>:8080`  
**Format**: JSON responses, query-parameter inputs  
**CORS**: Enabled for all origins

### `GET /api/status`

Returns the current state of the desk.

```bash
curl http://192.168.1.14:8080/api/status
```

```json
{
  "height_cm": 85.3,
  "height_pct": 42,
  "target_cm": 85.3,
  "target_pct": 42,
  "state": "idle",
  "direction": "none",
  "wifi_rssi": -45,
  "uptime_s": 3600
}
```

| Field | Type | Description |
|---|---|---|
| `height_cm` | float | Current desk height in centimeters |
| `height_pct` | int | Current height as percentage (0–100) |
| `target_cm` | float | Target height in centimeters |
| `target_pct` | int | Target height as percentage (0–100) |
| `state` | string | `"idle"` or `"moving"` |
| `direction` | string | `"none"`, `"up"`, or `"down"` |
| `wifi_rssi` | int | WiFi signal strength in dBm |
| `uptime_s` | int | Seconds since boot |

---

### `POST /api/move`

Move the desk to a target height. Accepts **either** `height_cm` or `height_pct`.

```bash
# By centimeters
curl -X POST "http://192.168.1.14:8080/api/move?height_cm=85"

# By percentage
curl -X POST "http://192.168.1.14:8080/api/move?height_pct=50"
```

**Success** `200 OK`
```json
{
  "ok": true,
  "message": "Moving to 85.0 cm",
  "target_cm": 85.0,
  "target_pct": 42
}
```

**Error** `409 Conflict` — desk already moving  
**Error** `400 Bad Request` — missing `height_cm` or `height_pct` parameter

| Parameter | Type | Description |
|---|---|---|
| `height_cm` | float | Target height in cm (clamped to 69.0–107.0) |
| `height_pct` | float | Target height as percentage (0–100) |

> **Note**: Movement routes through the HomeKit service internally, so Apple Home always stays in sync.

---

### `POST /api/stop`

Emergency stop — immediately kills the motor and cancels any pending movement.

```bash
curl -X POST http://192.168.1.14:8080/api/stop
```

```json
{
  "ok": true,
  "message": "Emergency stop executed",
  "height_cm": 85.3
}
```

---

### `GET /api/config`

Returns system configuration and network info.

```bash
curl http://192.168.1.14:8080/api/config
```

```json
{
  "min_height_cm": 69.0,
  "max_height_cm": 107.0,
  "tolerance_cm": 1.5,
  "firmware_version": "3.2",
  "ip": "192.168.1.14",
  "mac": "5C:01:3B:95:1F:6C"
}
```

---

## Using the API from Another ESP32

```cpp
#include <HTTPClient.h>

// Read desk status
HTTPClient http;
http.begin("http://192.168.1.14:8080/api/status");
int code = http.GET();
if (code == 200) {
  String payload = http.getString();
  // Parse JSON payload...
}
http.end();

// Move desk to 85 cm
http.begin("http://192.168.1.14:8080/api/move?height_cm=85");
code = http.POST("");
http.end();
```

---

## Height Range

| | Value |
|---|---|
| Minimum | 69.0 cm |
| Maximum | 107.0 cm |
| Tolerance | ±1.5 cm |

## Firmware Version

**Current**: 3.2  
**Last Updated**: June 9, 2026
