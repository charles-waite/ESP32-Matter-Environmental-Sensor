# ESP32 Matter Environmental Sensor

A compact, Matter-enabled environmental monitoring module built around the XIAO ESP32-C6 and Bosch BME680 sensor.  
This project demonstrates how to build a low-power, Wi-Fi or Thread-capable air quality sensor that integrates seamlessly into Matter-compatible ecosystems such as Apple Home, Google Home, and Home Assistant.

---

## 📦 Overview

The ESP32 Matter Environmental Sensor measures **temperature**, **humidity**, **pressure**, and **indoor air quality (IAQ)** using the Bosch BME680 environmental sensor and the BSEC2 library for accurate, compensated readings.  
An optional OLED display provides real-time readouts, while Matter support enables direct commissioning with any compatible ecosystem controller.

This project is designed for simple assembly, minimal wiring, and reliable 24/7 operation when powered from a compact USB charger.

---

## 🧰 Hardware

| Component | Description / Notes |
|------------|--------------------|
| **Seeed Studio XIAO ESP32-C6** | Main MCU; supports Wi-Fi, Thread, and BLE. Chosen for its small footprint and native Matter compatibility. |
| **Bosch BME680** | Environmental sensor measuring temperature, humidity, pressure, and gas/IAQ; connected via I²C. |
| **Apple 5W USB Charger** | Reliable, low-noise 5 V supply for continuous operation. |
| **USB-A Male Plug** | Interfaces the charger output to the device enclosure. |
| **USB-C Male Plug** | Connects to the XIAO board for power and flashing. |
| **Optional: SH1106 OLED Display (1.3")** | Displays temperature, humidity, pressure, and IAQ values locally. Connected via I²C (address `0x3C`). |

> ⚙️ *Future versions may include a VEML7700 ambient light sensor for automatic display dimming.*

---

## 🧩 Arduino Sketch & Libraries

The firmware is written in **Arduino**, using Espressif’s **Matter** support and Bosch’s **BSEC2** library for compensated BME680 readings.

**Core Features**
- Matter over Wi-Fi or Thread
- BME680 via BSEC2 for accurate IAQ, temperature, humidity, and pressure
- Periodic state saving (`setState()` / `getState()`) for stable IAQ tracking
- OLED visualization (optional)
- JSON-based serial diagnostics

**Required Libraries**
- `Arduino_BSEC2` (Bosch BSEC2)
- `Wire.h`
- `Adafruit_Sensor`
- `SH1106Wire` *(for OLED display)*
- `QRCodeOLED` *(for displaying the Matter QR code during commissioning)*
- `ESP_Matter` *(or Espressif’s Matter Arduino support package)*

> Example sketches:  
> - `/src/esp32_matter_env_sensor.ino`  
> - `/src/bme680_bsec2_helpers.cpp`

---

## 🧱 3D Printed Enclosure

A simple, vented enclosure designed for passive airflow and unobtrusive wall or outlet mounting.

**Files**
- `/3d_models/enclosure_top.stl`
- `/3d_models/enclosure_bottom.stl`
- `/3d_models/usb_adapter_bracket.stl`
- `/3d_models/oled_front_variant.stl` *(optional display version)*

**Notes**
- The case design accommodates the XIAO ESP32-C6, BME680 breakout, and optional SH1106 OLED.
- Optimized for front-facing airflow and USB-powered mounting directly on a 5 W charger.
- Printed in PLA or PETG recommended for indoor use.

---

## 🔌 Assembly & Wiring

Connect the BME680 (and optional OLED) to the XIAO ESP32-C6 over I²C:

| XIAO ESP32-C6 Pin | Signal | BME680 Pin | OLED Pin (optional) | Notes |
|--------------------|---------|-------------|----------------------|-------|
| **D4** | SDA | SDA | SDA | I²C data line |
| **D5** | SCL | SCL | SCL | I²C clock line |
| **3V3** | 3.3 V | VIN | VCC | Power supply |
| **GND** | Ground | GND | GND | Common ground |

**Steps**
1. Wire the BME680 (and OLED) to the XIAO as shown above.  
2. Flash the Arduino sketch via USB-C.  
3. Power the unit using the Apple 5 W charger with USB-A → USB-C adapter.  
4. On startup, scan the QR code shown on the OLED (or via serial output) to commission the device via a Matter controller.

---

## 🧠 Future Enhancements

- Thread support and border router discovery  
- OTA updates  
- Ambient light sensor integration for display dimming  
- Expanded Matter cluster support (e.g., VOC index, CO₂, etc.)

---

## 📜 License

MIT License © 2025 [CP Waite](https://github.com/charles-waite)

---

## 🖼️ Preview

*(Add photos or renders here)*  
- Device mounted on charger  
- OLED displaying environmental data  
- Matter commissioning QR code  
- Enclosure render or assembly diagram

---