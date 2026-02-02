#include <Wire.h>
#include <WiFi.h>
#include <time.h>
#include <RTClib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_VEML7700.h>

// ==== I2C pins (XIAO ESP32-C6) ====
#define I2C_SDA   22
#define I2C_SCL   23
#define OLED_ADDR 0x3C

// ==== Wi-Fi ====
const char* WIFI_SSID = "MufflupagusAC";
const char* WIFI_PASS = "queenanne";

// ==== Display / RTC / ALS ====
// NOTE: Instantiate a concrete SH110X display class:
Adafruit_SH1106G display(128, 64, &Wire);
RTC_DS3231 rtc;
Adafruit_VEML7700 veml;

// ==== State ====
bool rtcPresent   = false;
bool timeSynced   = false;
bool vemlPresent  = false;

// ---------- SH1106 contrast control ----------
#ifndef SH1106_SETCONTRAST
#define SH1106_SETCONTRAST 0x81
#endif

// Contrast levels
const uint8_t OLED_CONTRAST_LOW  = 1;    // LOW
const uint8_t OLED_CONTRAST_MED  = 100;  // MED
const uint8_t OLED_CONTRAST_HIGH = 225;  // HIGH

// Track current step: 0=LOW, 1=MED, 2=HIGH, -1 unknown
int8_t currentContrastStep = -1;

void setContrast(uint8_t value) {
  // SH110X: use the public helper
  display.setContrast(value);
}

void applyOledContrastFromLux(float lux) {
  int8_t step;
  uint8_t value;
  if (lux >= 100.0f) {
    step = 2; value = OLED_CONTRAST_HIGH;
  } else if (lux >= 35.0f) {
    step = 1; value = OLED_CONTRAST_MED;
  } else {
    step = 0; value = OLED_CONTRAST_LOW;
  }

  if (step != currentContrastStep) {
    currentContrastStep = step;
    setContrast(value);
    const char* name = (step == 2) ? "HIGH" : (step == 1) ? "MED" : "LOW";
    Serial.printf("Lux=%.1f -> OLED contrast %s (%u)\n", lux, name, value);
  }
}

// ----------------- Helpers -----------------
void printWiFiStatus() {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("===== WiFi Status =====");
    Serial.printf("SSID: %s\n", WiFi.SSID().c_str());
    Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());
    Serial.println("========================\n");
  } else {
    Serial.println("WiFi not connected.\n");
  }
}

void i2cScan() {
  Serial.println(F("\nI2C scan:"));
  byte count = 0;
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  - Found device at 0x%02X\n", address);
      count++;
    }
  }
  if (!count) Serial.println(F("  (no I2C devices found)"));
  Serial.println();
}

bool connectWiFi() {
  if (!strlen(WIFI_SSID)) return false;
  Serial.printf("Connecting to WiFi SSID: %s\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < 15000) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    printWiFiStatus();
    return true;
  }
  Serial.println("WiFi not connected.");
  return false;
}

// Seattle (Pacific Time) with automatic DST via POSIX TZ
bool syncTimeFromNTP() {
  configTzTime("PST8PDT,M3.2.0,M11.1.0",
               "pool.ntp.org", "time.nist.gov", "time.google.com");

  Serial.println("Syncing time from NTP (Pacific Time via configTzTime)...");
  struct tm tinfo;
  for (int i = 0; i < 50; i++) {
    if (getLocalTime(&tinfo, 200)) {
      Serial.println("NTP time acquired.");

      // ---- Sanity check: print UTC vs Local once ----
      time_t raw = time(nullptr);

      struct tm gmt;
      gmtime_r(&raw, &gmt);
      Serial.printf("UTC:   %04d-%02d-%02d %02d:%02d:%02d\n",
                    gmt.tm_year + 1900, gmt.tm_mon + 1, gmt.tm_mday,
                    gmt.tm_hour, gmt.tm_min, gmt.tm_sec);

      struct tm lcl;
      localtime_r(&raw, &lcl);
      Serial.printf("Local: %04d-%02d-%02d %02d:%02d:%02d\n",
                    lcl.tm_year + 1900, lcl.tm_mon + 1, lcl.tm_mday,
                    lcl.tm_hour, lcl.tm_min, lcl.tm_sec);
      // -----------------------------------------------

      return true;
    }
    delay(100);
  }
  Serial.println("Failed to get NTP time.");
  return false;
}

String tmToString(const struct tm& t) {
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &t);
  return String(buf);
}

String dtToString(const DateTime& dt) {
  char buf[32];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
           dt.year(), dt.month(), dt.day(),
           dt.hour(), dt.minute(), dt.second());
  return String(buf);
}

void drawTwoLines(const String& top, const String& bottom) {
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);   // SH110X color constant
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(top);
  display.setCursor(0, 16);
  display.print(bottom);
  display.display();
}

// ----------------- Arduino setup/loop -----------------
void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(I2C_SDA, I2C_SCL);
  delay(50);
  i2cScan();

  // OLED init (no SSD1306_SWITCHCAPVCC on SH110X)
  if (!display.begin(OLED_ADDR, /*reset=*/true)) {
    Serial.println(F("SH1106 allocation/init failed"));
    for (;;);
  }
  display.clearDisplay();
  display.display();

  // Set a sane initial contrast (MED) until first lux reading
  setContrast(OLED_CONTRAST_MED);
  currentContrastStep = 1;

  // RTC init
  rtcPresent = rtc.begin();
  if (!rtcPresent) Serial.println("ERROR: DS3231 not found at 0x68.");
  else             Serial.println("DS3231 initialized.");

  // VEML7700 init
  vemlPresent = veml.begin();
  if (!vemlPresent) {
    Serial.println("VEML7700 not found");
  } else {
    Serial.println("VEML7700 found");
    // Auto lux uses readLux(VEML_LUX_AUTO)
  }

  if (connectWiFi()) {
    timeSynced = syncTimeFromNTP();
  }

  // If RTC is present and needs setting, set from NTP if available
  if (rtcPresent) {
    bool needsSet = false;
    if (rtc.lostPower()) {
      Serial.println("RTC lost power; will reset from NTP if available.");
      needsSet = true;
    } else {
      DateTime nowDT = rtc.now();
      if (nowDT.year() < 2020) needsSet = true;
    }
    if (needsSet && timeSynced) {
      struct tm tinfo;
      if (getLocalTime(&tinfo)) {
        DateTime nowDT(tinfo.tm_year + 1900, tinfo.tm_mon + 1, tinfo.tm_mday,
                       tinfo.tm_hour, tinfo.tm_min, tinfo.tm_sec);
        rtc.adjust(nowDT);
        Serial.print("RTC set from NTP: ");
        Serial.println(dtToString(nowDT));
      }
    } else if (needsSet && !timeSynced) {
      Serial.println("No NTP; cannot set RTC.");
    }
  }
}

void loop() {
  // Periodic Wi-Fi health check / auto-reconnect
  static unsigned long lastWiFiCheck = 0;
  if (millis() - lastWiFiCheck > 10000) {  // every 10s
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("⚠️  WiFi disconnected, attempting reconnect...");
      connectWiFi();
    }
    lastWiFiCheck = millis();
  }

  // Read lux and apply 3-step contrast
  float lux = NAN;
  if (vemlPresent) {
    lux = veml.readLux(VEML_LUX_AUTO);
    applyOledContrastFromLux(lux);
  }

  // NTP (local) time
  struct tm tinfo;
  String ntpLine = "(no NTP)";
  if (getLocalTime(&tinfo)) ntpLine = tmToString(tinfo);

  // RTC time
  String rtcLine = "(RTC not found)";
  if (rtcPresent) {
    DateTime nowDT = rtc.now();
    rtcLine = dtToString(nowDT);
  }

  // OLED: NTP top, RTC bottom
  drawTwoLines(ntpLine, rtcLine);

  // Serial mirror + lux
  if (!isnan(lux)) {
    Serial.printf("Lux: %.1f | NTP(Local): %s | RTC: %s\n",
                  lux, ntpLine.c_str(), rtcLine.c_str());
  } else {
    Serial.printf("NTP(Local): %s | RTC: %s\n", ntpLine.c_str(), rtcLine.c_str());
  }

  delay(1000);
}