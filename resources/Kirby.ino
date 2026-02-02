// XIAO ESP32C6 + SH1106 OLED (I2C) — draw Kirby bitmap
// Requires the ThingPulse "ESP8266 and ESP32 OLED driver for SSD1306 displays" library
// and that you select SH1106 via SH1106Wire.

#include <Wire.h>
#include <SH1106Wire.h>
#include "kirby.h"  // must contain: `static const unsigned char kirby_bits[] PROGMEM = { ... };`

// --- Adjust these if your wiring differs ---
constexpr uint8_t OLED_ADDR = 0x3C;
// You mentioned 22 (SDA) and 23 (SCL) work on your XIAO ESP32C6:
constexpr int OLED_SDA = 22;
constexpr int OLED_SCL = 23;
// -------------------------------------------

SH1106Wire display(OLED_ADDR, OLED_SDA, OLED_SCL);

// Kirby bitmap dimensions (from your kirby.h comment)
constexpr int KIRBY_W = 64;
constexpr int KIRBY_H = 64;

void setup() {
  // If you prefer to initialize Wire yourself, uncomment the next line:
  // Wire.begin(OLED_SDA, OLED_SCL);

  display.init();
  // Optional: flip if your screen is mounted upside-down
  // display.flipScreenVertically();

  display.clear();

  // Center Kirby on a 128x64 SH1106
  const int x = (128 - KIRBY_W) / 2;
  const int y = (64  - KIRBY_H) / 2;

  // Draw the XBM (monochrome) image
  display.drawXbm(x, y, KIRBY_W, KIRBY_H, kirby_bits);
  display.display();
}

void loop() {
  // Nothing to do—static image.
}