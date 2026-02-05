// XIAO ESP32-C6 | BME680 (BSEC2) | SH1106 OLED | Arduino Matter (Wi-Fi)
// Endpoints: Temperature, Relative Humidity, Pressure
//
// Behavior:
//  • If NOT commissioned: draws a one-time, high-contrast QR ("MT:..." payload) -> no redraw flicker
//  • If commissioned: shows sensor screen, updates endpoints & UI every 3s
//  • Hold BOOT 3s: decommission & show QR again
//
// Libraries:
//  • BSEC2 + blob (bsec2.h)
//  • ThingPulse SH1106 (SH1106Wire.h)
//  • QRcodeOled (qrcodeoled.h)   <-- lowercase header
//  • Arduino-ESP32 core 3.3.x Matter (Matter.h, MatterEndPoint.h)
//------------------------------------------------------------------//
//    ---==== HUMAN WRITTEN NOTES ====---
//  This sketch is fully functional as a standalone module.
//  BME680 Sensor -> Temp, Humidity, Pressure, eCO2, IAQ as Matter Endpoints. 
//  SH1106-based 1.3" 128x64 OLED Display -> Shows QR Code for provisioning then basic sensor data on one screen.
//  Uses BSEC2 library to interpret raw sensor data.
//  BSEC calculated Temperature offset tested to be ~6.5°C in Solid lid housing

#include <Matter.h>
#include <MatterEndPoint.h>
#include <MatterAirQualitySensor.h>
#include "esp_pm.h"
#include "esp_openthread.h"
#include "esp_matter_attribute_utils.h"
#include <openthread/link.h>
#include <openthread/thread.h>
#include <openthread/thread_ftd.h>
#include <app-common/zap-generated/ids/Clusters.h>
#include <app-common/zap-generated/ids/Attributes.h>
#include <app/clusters/time-synchronization-server/time-synchronization-server.h>
#include <esp_matter_cluster.h>
#include <esp_matter_endpoint.h>
#include <system/SystemClock.h>
#include <Wire.h>
#include <bsec2.h>
#define SH1106_COL_OFFSET 2
#include <SH1106Wire.h>
#include <qrcodeoled.h>
#include "kirby.h"
#include <Preferences.h>
#include <math.h>
#include <string.h>

#if !CONFIG_ENABLE_CHIPOBLE
// if the device can be commissioned using BLE, WiFi is not used - save flash space
#include <WiFi.h>
#endif

/* =========== Debug Mode Switch ======== */
static constexpr bool DEBUG_SERIAL = false;

/* ========== OLED Present ============ */
#define USE_OLED 1   // set to 1 when the screen is installed

// ------------ Board Pin Defs -----------------
#define SDA_PIN 22
#define SCL_PIN 23
#define OLED_ADDR 0x3C
#define BME_ADDR 0x77
#define BOOT_BTN 9  // XIAO ESP32-C6 BOOT (active LOW)


/* =========== Button Defines ============ */
const uint8_t BUTTON_PIN = BOOT_BTN;
const uint32_t DECOMMISSION_HOLD_MS = 5000;
const uint32_t UI_INTERVAL_MS = 30000;
const uint32_t SCREEN_SWAP_MS = 30000;
const uint32_t BRIGHTNESS_CHECK_MS = 60000;

/* ======== Globals: display, QR ======== */
SH1106Wire display(OLED_ADDR, -1, -1);
QRcodeOled qrcode(&display);
bool showingQR = false;
uint32_t lastUi = 0;
uint32_t lastScreenSwap = 0;
uint32_t lastBrightnessCheck = 0;
bool showKirby = false;
static bool oledPresent = false;
static bool oledDimmed = false;
static bool timeSyncClusterReady = false;
static bool timeSyncLogged = false;

/* ============ BSEC2 ============= */
Bsec2 env;
bsec_virtual_sensor_t sensorList[] = {
  BSEC_OUTPUT_STATIC_IAQ,
  BSEC_OUTPUT_CO2_EQUIVALENT,
  BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
  BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
  BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  BSEC_OUTPUT_RAW_PRESSURE
};

Preferences prefs;
const char* NVS_NS = "bsec2";
const char* NVS_KEY = "state";
#define BSEC_STATE_SIZE 512

volatile float vTempC = NAN, vHum = NAN, vPres_hPa = NAN;
volatile float vIAQ = NAN, vCO2eq = NAN, vVOCeq = NAN;
volatile uint8_t vIAQacc = 0;
// Derived values
float vTempF = NAN, vPres_inHg = NAN, vPres_psi = NAN, vPres_hPa_sl = NAN, lastPres_inHg = NAN;


/* ===== Env / units ===== */
#define ALTITUDE_M 97.8
#define ALTITUDE_FT 321
#define TEMP_OFFSET_C 6.5  //Value is in Celsius
#define USE_SEA_LEVEL_P 1

/* ============ BSEC callback =========== */
static void onBsecOutputs(const bme68xData d, const bsecOutputs out, Bsec2 b) {
  for (uint8_t i = 0; i < out.nOutputs; i++) {
    const bsecData& o = out.output[i];
    switch (o.sensor_id) {
      case BSEC_OUTPUT_STATIC_IAQ:
        vIAQ = o.signal;
        vIAQacc = o.accuracy;
        break;
      case BSEC_OUTPUT_CO2_EQUIVALENT: vCO2eq = o.signal; break;
      case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT: vVOCeq = o.signal; break;
      case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE: vTempC = o.signal; break;
      case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY: vHum = o.signal; break;
      case BSEC_OUTPUT_RAW_PRESSURE: vPres_hPa = o.signal; break;
    }
  }
}

/* ============ BSEC state ============ */
void loadBsecState() {
  // Open RW so the namespace is created if missing.
  if (!prefs.begin(NVS_NS, false)) {
    Serial.println("[BSEC2] NVS open failed");
    return;
  }
  if (!prefs.isKey(NVS_KEY)) {
    prefs.end();
    return;
  }
  size_t n = prefs.getBytesLength(NVS_KEY);
  if (n == BSEC_STATE_SIZE) {
    uint8_t buf[BSEC_STATE_SIZE];
    prefs.getBytes(NVS_KEY, buf, BSEC_STATE_SIZE);
    if (env.setState(buf)) Serial.println("[BSEC2] state restored");
  } else if (n > 0) {
    Serial.printf("[BSEC2] NVS state size mismatch: %u\n", static_cast<unsigned>(n));
  }
  prefs.end();
}

void saveBsecStateIfReady(uint32_t nowMs) {
  static const uint32_t EVERY = 60UL * 60UL * 1000UL;
  static uint32_t last = 0;
  if (nowMs - last < EVERY) return;
  if (vIAQacc < 2) return;
  uint8_t st[BSEC_STATE_SIZE];
  if (env.getState(st) && prefs.begin(NVS_NS, false)) {
    prefs.putBytes(NVS_KEY, st, BSEC_STATE_SIZE);
    prefs.end();
    last = nowMs;
    Serial.println("[BSEC2] state saved");
  }
}
/* ----------- END BSEC2 -----------------*/

/* =========== Helper Functions =========== */
static inline void clearDisplayHard();
String extractMtPayload(const String& urlOrPayload);
static bool detectOled();
static void setDefaultNodeLabel();
static void showOledBootScreen();
static void showOledCalibration();    //-- FOR DEBUG ONLY --
static void drawKirbyScreen();
static void updateOledBrightness();
static void initTimeSyncCluster();

static void configureThreadRouterEligibility() {
#if CONFIG_ENABLE_MATTER_OVER_THREAD && CONFIG_OPENTHREAD_ENABLED
  otInstance* instance = esp_openthread_get_instance();
  if (!instance) return;

  otLinkModeConfig mode = otThreadGetLinkMode(instance);
  mode.mRxOnWhenIdle = true;
  mode.mDeviceType = true;
  mode.mNetworkData = true;
  otThreadSetLinkMode(instance, mode);
  otThreadSetRouterEligible(instance, true);
#endif
}

static void setDefaultNodeLabel() {
  static constexpr char kDefaultNodeLabel[] = "WALL-Env Sensor";
  char current[esp_matter::cluster::basic_information::k_max_node_label_length + 1] = {};
  esp_err_t err = esp_matter::attribute::get_val_raw(
      0, chip::app::Clusters::BasicInformation::Id,
      chip::app::Clusters::BasicInformation::Attributes::NodeLabel::Id,
      reinterpret_cast<uint8_t*>(current), sizeof(current));
  if (err == ESP_OK && current[0] != '\0') return;

  esp_matter_attr_val_t val =
      esp_matter_char_str(const_cast<char*>(kDefaultNodeLabel), strlen(kDefaultNodeLabel));
  esp_matter::attribute::update(
      0, chip::app::Clusters::BasicInformation::Id,
      chip::app::Clusters::BasicInformation::Attributes::NodeLabel::Id, &val);
}

void showQrOnceOnOLED() {
  if (!oledPresent) return;
  qrcode.init();
  display.setContrast(255);
  oledDimmed = false;
  clearDisplayHard();

  String url = Matter.getOnboardingQRCodeUrl();
  Serial.println("[Matter] QR URL: " + url);
  String payload = extractMtPayload(url);
  if (payload.length() == 0) payload = url;
  Serial.println("[Matter] QR payload: " + payload);

  if (payload.length() == 0) {
    display.drawString(0, 0, "QR unavailable");
    display.display();
    return;
  }
  qrcode.create(payload);  // draw once, no footer to avoid rolling bands
  display.display();
}

void handleBootLongPress() {
  static uint32_t t0 = 0;
  static bool was = false;
  bool p = (digitalRead(BOOT_BTN) == LOW);
  if (p && !was) t0 = millis();
  if (!p && was) t0 = 0;
  if (p && t0 > 0 && (millis() - t0) > DECOMMISSION_HOLD_MS) {
    Serial.println("[Matter] Decommission");
    Matter.decommission();
    delay(300);
    showQrOnceOnOLED();
    showingQR = oledPresent;
    delay(600);
  }
  was = p;
}

static inline void clearDisplayHard() {
  if (!oledPresent) return;
  for (int i = 0; i < 2; i++) {
    display.clear();
    display.display();
    delay(5);
  }
}
static inline float c_to_f(float c) {
  return c * 9.0f / 5.0f + 32.0f;
}
static inline float hPa_to_inHg(float h) {
  return h / 33.8638866667f;
}
static inline float hPa_to_psi(float h) {
  return h * 0.0145037738f;
}
static inline float seaLevelPressure_hPa(float p_hPa, float tC, float alt_m) {
  if (!isfinite(p_hPa) || !isfinite(tC)) return NAN;
  float T = tC + 273.15f;
  return p_hPa * powf(1.0f - (0.0065f * alt_m) / T, -5.257f);
}
static inline const char* iaqTrend(float nowVal, float prevVal, float th = 2.0f) {
  if (!isfinite(nowVal) || !isfinite(prevVal)) return " ";
  float d = nowVal - prevVal;
  if (d > th) return "↑";
  if (d < -th) return "↓";
  return "→";
}

static const char* sIaqLabel(float sIaq) {
  if (!isfinite(sIaq)) return "Unknown";
  if (sIaq <= 50.0f) return "Good";
  if (sIaq <= 100.0f) return "Fair";
  if (sIaq <= 150.0f) return "Moderate";
  if (sIaq <= 200.0f) return "Poor";
  if (sIaq <= 250.0f) return "Very Poor";
  return "Extremely Poor";
}

static inline int dayOfYear(int y, int m, int d) {
  static const int kDaysBeforeMonth[] = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };
  bool leap = (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0));
  return kDaysBeforeMonth[m - 1] + d + (leap && m > 2 ? 1 : 0);
}

static void civilFromDays(int64_t z, int &y, int &m, int &d) {
  z += 719468;
  int64_t era = (z >= 0 ? z : z - 146096) / 146097;
  unsigned doe = static_cast<unsigned>(z - era * 146097);
  unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  y = static_cast<int>(yoe) + static_cast<int>(era) * 400;
  unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  unsigned mp = (5 * doy + 2) / 153;
  d = static_cast<int>(doy - (153 * mp + 2) / 5 + 1);
  m = static_cast<int>(mp + (mp < 10 ? 3 : -9));
  y += (m <= 2);
}

static bool isDstPacific(int y, int m, int d, int hour) {
  if (m < 3 || m > 11) return false;
  if (m > 3 && m < 11) return true;

  int dow = 0; // 0=Sunday
  {
    int yy = y;
    int mm = m;
    int dd = d;
    if (mm < 3) {
      mm += 12;
      yy -= 1;
    }
    int k = yy % 100;
    int j = yy / 100;
    int h = (dd + (13 * (mm + 1)) / 5 + k + (k / 4) + (j / 4) + (5 * j)) % 7;
    dow = ((h + 6) % 7);
  }

  if (m == 3) {
    int firstSunday = (dow == 0) ? d : (d + (7 - dow));
    int secondSunday = firstSunday + 7;
    if (d > secondSunday) return true;
    if (d < secondSunday) return false;
    return hour >= 2;
  }

  int firstSunday = (dow == 0) ? d : (d + (7 - dow));
  if (d > firstSunday) return false;
  if (d < firstSunday) return true;
  return hour < 2;
}

static bool getUnixTimeSeconds(int64_t &outSec) {
  chip::System::Clock::Microseconds64 utc;
  if (chip::System::SystemClock().GetClock_RealTime(utc) != CHIP_NO_ERROR) return false;
  outSec = static_cast<int64_t>(utc.count() / 1000000);
  return true;
}

static bool isTimeSynced() {
  int64_t utcSec = 0;
  if (!getUnixTimeSeconds(utcSec)) return false;
  return utcSec >= 978307200; // 2001-01-01T00:00:00Z
}

static bool computeSunTimes(int y, int m, int d, float lat, float lon, float tzHours,
                            float &sunriseMin, float &sunsetMin) {
  const float zenith = 90.833f;
  int N = dayOfYear(y, m, d);
  float lngHour = lon / 15.0f;

  auto calcTime = [&](bool sunrise) -> float {
    float t = N + ((sunrise ? 6.0f : 18.0f) - lngHour) / 24.0f;
    float M = (0.9856f * t) - 3.289f;
    float L = M + (1.916f * sinf(M * DEG_TO_RAD)) + (0.020f * sinf(2 * M * DEG_TO_RAD)) + 282.634f;
    while (L < 0) L += 360.0f;
    while (L >= 360.0f) L -= 360.0f;
    float RA = atanf(0.91764f * tanf(L * DEG_TO_RAD)) * RAD_TO_DEG;
    while (RA < 0) RA += 360.0f;
    while (RA >= 360.0f) RA -= 360.0f;
    float Lquadrant = floorf(L / 90.0f) * 90.0f;
    float RAquadrant = floorf(RA / 90.0f) * 90.0f;
    RA = (RA + (Lquadrant - RAquadrant)) / 15.0f;

    float sinDec = 0.39782f * sinf(L * DEG_TO_RAD);
    float cosDec = cosf(asinf(sinDec));
    float cosH = (cosf(zenith * DEG_TO_RAD) - (sinDec * sinf(lat * DEG_TO_RAD))) /
                 (cosDec * cosf(lat * DEG_TO_RAD));
    if (cosH > 1.0f || cosH < -1.0f) return NAN;
    float H = sunrise ? (360.0f - acosf(cosH) * RAD_TO_DEG) : (acosf(cosH) * RAD_TO_DEG);
    H /= 15.0f;
    float T = H + RA - (0.06571f * t) - 6.622f;
    float UT = T - lngHour;
    while (UT < 0) UT += 24.0f;
    while (UT >= 24.0f) UT -= 24.0f;
    float localT = UT + tzHours;
    while (localT < 0) localT += 24.0f;
    while (localT >= 24.0f) localT -= 24.0f;
    return localT * 60.0f;
  };

  sunriseMin = calcTime(true);
  sunsetMin = calcTime(false);
  return isfinite(sunriseMin) && isfinite(sunsetMin);
}

static void printTimeDebug() {
  int64_t utcSec = 0;
  if (!getUnixTimeSeconds(utcSec)) {
    Serial.println(F("Local: --:--:-- PST"));
    Serial.println(F("UTC  : --:--:--"));
    Serial.println(F("Next : --"));
    return;
  }

  const float lat = 47.669f;
  const float lon = -122.347f;
  int64_t baseOffsetSec = -8 * 3600;
  int64_t localSec = utcSec + baseOffsetSec;
  int64_t days = localSec / 86400;
  int64_t rem = localSec % 86400;
  if (rem < 0) { rem += 86400; days -= 1; }
  int hour = static_cast<int>(rem / 3600);
  int minute = static_cast<int>((rem % 3600) / 60);
  int second = static_cast<int>(rem % 60);

  int y, m, d;
  civilFromDays(days, y, m, d);
  bool dst = isDstPacific(y, m, d, hour);
  if (dst) {
    localSec += 3600;
    days = localSec / 86400;
    rem = localSec % 86400;
    if (rem < 0) { rem += 86400; days -= 1; }
    hour = static_cast<int>(rem / 3600);
    minute = static_cast<int>((rem % 3600) / 60);
    second = static_cast<int>(rem % 60);
    civilFromDays(days, y, m, d);
  }

  char buf[48];
  snprintf(buf, sizeof(buf), "Local: %02d:%02d:%02d PST", hour, minute, second);
  Serial.println(buf);

  int64_t utcDay = utcSec / 86400;
  int64_t utcRem = utcSec % 86400;
  if (utcRem < 0) { utcRem += 86400; utcDay -= 1; }
  int uh = static_cast<int>(utcRem / 3600);
  int um = static_cast<int>((utcRem % 3600) / 60);
  int us = static_cast<int>(utcRem % 60);
  snprintf(buf, sizeof(buf), "UTC  : %02d:%02d:%02d", uh, um, us);
  Serial.println(buf);

  int uy, umon, uday;
  civilFromDays(utcDay, uy, umon, uday);
  snprintf(buf, sizeof(buf), "Date : %02d-%02d-%04d", umon, uday, uy);
  Serial.println(buf);

  float sunriseMin = NAN, sunsetMin = NAN;
  float tzHours = dst ? -7.0f : -8.0f;
  if (!computeSunTimes(y, m, d, lat, lon, tzHours, sunriseMin, sunsetMin)) {
    Serial.println(F("Next : --"));
    return;
  }

  float nowMin = hour * 60.0f + minute + (second / 60.0f);
  float targetMin = NAN;
  const char *label = "Sunrise";
  if (nowMin < sunriseMin) {
    targetMin = sunriseMin;
    label = "Sunrise";
  } else if (nowMin < sunsetMin) {
    targetMin = sunsetMin;
    label = "Sunset";
  } else {
    int y2, m2, d2;
    civilFromDays(days + 1, y2, m2, d2);
    bool dst2 = isDstPacific(y2, m2, d2, 12);
    float tz2 = dst2 ? -7.0f : -8.0f;
    float sr2 = NAN, ss2 = NAN;
    if (computeSunTimes(y2, m2, d2, lat, lon, tz2, sr2, ss2)) {
      targetMin = sr2 + 1440.0f;
      label = "Sunrise";
    }
  }

  if (!isfinite(targetMin)) {
    Serial.println(F("Next : --"));
    return;
  }

  float deltaMin = targetMin - nowMin;
  int deltaSec = static_cast<int>(deltaMin * 60.0f + 0.5f);
  int dh = deltaSec / 3600;
  int dm = (deltaSec % 3600) / 60;
  int ds = deltaSec % 60;
  snprintf(buf, sizeof(buf), "Next : %s in %02d:%02d:%02d", label, dh, dm, ds);
  Serial.println(buf);
}

static const char* threadRoleName(otDeviceRole role) {
  switch (role) {
    case OT_DEVICE_ROLE_DISABLED: return "disabled";
    case OT_DEVICE_ROLE_DETACHED: return "detached";
    case OT_DEVICE_ROLE_CHILD: return "child";
    case OT_DEVICE_ROLE_ROUTER: return "router";
    case OT_DEVICE_ROLE_LEADER: return "leader";
    default: return "unknown";
  }
}

// Matter AirQuality::AirQualityEnum (spec v1.2)
static constexpr uint8_t AIR_QUALITY_UNKNOWN = 0;
static constexpr uint8_t AIR_QUALITY_GOOD = 1;
static constexpr uint8_t AIR_QUALITY_FAIR = 2;
static constexpr uint8_t AIR_QUALITY_MODERATE = 3;
static constexpr uint8_t AIR_QUALITY_POOR = 4;
static constexpr uint8_t AIR_QUALITY_VERY_POOR = 5;
static constexpr uint8_t AIR_QUALITY_EXTREMELY_POOR = 6;

static uint8_t mapStaticIaqToAirQuality(float sIaq) {
  if (!isfinite(sIaq)) return AIR_QUALITY_UNKNOWN;
  if (sIaq <= 50.0f) return AIR_QUALITY_GOOD;            // Excellent
  if (sIaq <= 100.0f) return AIR_QUALITY_FAIR;           // Good
  if (sIaq <= 150.0f) return AIR_QUALITY_MODERATE;       // Lightly polluted
  if (sIaq <= 200.0f) return AIR_QUALITY_POOR;           // Moderately polluted
  if (sIaq <= 250.0f) return AIR_QUALITY_VERY_POOR;      // Heavily polluted
  return AIR_QUALITY_EXTREMELY_POOR;                    // Severely / Extremely polluted
}

/* ============== Matter ============ */
// Matter cluster / attribute IDs
static constexpr uint32_t CL_TEMPERATURE_MEASUREMENT = 0x0402;        // MeasuredValue = 0.01 °C (int16)
static constexpr uint32_t CL_RELATIVE_HUMIDITY_MEASUREMENT = 0x0405;  // MeasuredValue = 0.01 % (int16)
static constexpr uint32_t CL_PRESSURE_MEASUREMENT = 0x0403;           // MeasuredValue = 0.1 kPa (int16)
static constexpr uint32_t ATTR_MEASURED_VALUE = 0x0000;

// Matter Endpoints
MatterTemperatureSensor epTemp;
MatterHumiditySensor epRH;
MatterPressureSensor epPress;
MatterAirQualitySensor epAir;  // CO2 + AirQuality cluster

// ---------- Push sensor data into Matter endpoints ----------
void updateMatterEndpoints() {
  // ---- Temperature (C) ----
  if (isfinite(vTempC)) {
    epTemp.setTemperature((double)vTempC);
  }

  // ---- Relative Humidity (%) ----
  if (isfinite(vHum)) {
    epRH.setHumidity((double)vHum);
  }

  // ---- Pressure (hPa) ----
  if (isfinite(vPres_hPa)) {
    double p_hPa = vPres_hPa;
    if (USE_SEA_LEVEL_P && isfinite(vTempC)) {
      p_hPa = seaLevelPressure_hPa(vPres_hPa, vTempC, ALTITUDE_M);
    }
    epPress.setPressure(p_hPa);
  }

  // ---- Air Quality (CO2 ppm) ----
  if (isfinite(vCO2eq)) {
    epAir.setCO2((double)vCO2eq);
  }

  // ---- Air Quality (s-IAQ enum) ----
  uint8_t aq = (vIAQacc >= 2) ? mapStaticIaqToAirQuality(vIAQ) : AIR_QUALITY_UNKNOWN;
  epAir.setAirQualityEnum(aq);
}

/* =========== UI + Serial ==========*/
// ---------- UI ----------
void computeDerived() {
  float tC = isfinite(vTempC) ? (vTempC) : NAN;
  vTempF = isfinite(tC) ? c_to_f(tC) : NAN;
  float p = vPres_hPa;
  if (USE_SEA_LEVEL_P && isfinite(p) && isfinite(tC)) {
    p = seaLevelPressure_hPa(p, tC, ALTITUDE_M);
    vPres_hPa_sl = p;
  } else vPres_hPa_sl = NAN;
  vPres_inHg = isfinite(p) ? hPa_to_inHg(p) : NAN;
  vPres_psi = isfinite(p) ? hPa_to_psi(p) : NAN;
}

//--------- Serial Monitor --------
void handleSerialCommands() {
  static String cmd;

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      cmd.trim();
      cmd.toLowerCase();

      // ---- Decommission ----
      if (cmd == "decom") {
        Serial.println("\n[Matter] Decommission via serial");
        Matter.decommission();
        delay(300);

        // Print fresh commissioning info
        Serial.printf("Manual pairing code: %s\r\n",
                      Matter.getManualPairingCode().c_str());
        Serial.printf("QR code URL: %s\r\n",
                      Matter.getOnboardingQRCodeUrl().c_str());

        showQrOnceOnOLED();
        showingQR = true;
      }

      // ---- Status ----
      else if (cmd == "status") {
        Serial.println("\n[Matter] Status");
        Serial.printf("Commissioned: %s\r\n",
                      Matter.isDeviceCommissioned() ? "YES" : "NO");

        Serial.printf("Manual pairing code: %s\r\n",
                      Matter.getManualPairingCode().c_str());
        Serial.printf("QR code URL: %s\r\n",
                      Matter.getOnboardingQRCodeUrl().c_str());
      }

      // ---- Help ----
      else if (cmd == "help") {
        Serial.println("\nCommands:");
        Serial.println("  decom   - Decommission + show QR");
        Serial.println("  status  - Show Matter status + pairing info");
        Serial.println("  help    - This help text");
      }

      cmd = "";
    }
    else {
      if (cmd.length() < 24) {
        cmd += c;
      }
    }
  }
}

void printSerial() {
  computeDerived();
  Serial.println(F("---- BME680 (BSEC2, US Units) ----"));
  Serial.print(F("sIAQ: "));
  if (isnan(vIAQ)) Serial.println(F("--"));
  else {
    Serial.print(vIAQ, 0);
    Serial.print(F(" (acc="));
    Serial.print(vIAQacc);
    Serial.println(F(")"));
  }
  Serial.print(F("eCO2: "));
  if (isnan(vCO2eq)) Serial.println(F("-- ppm"));
  else {
    Serial.print(vCO2eq, 0);
    Serial.println(F(" ppm"));
  }
  Serial.print(F("VOC : "));
  if (isnan(vVOCeq)) Serial.println(F("-- ppm eq"));
  else {
    Serial.print(vVOCeq, 2);
    Serial.println(F(" ppm eq"));
  }
  Serial.print(F("Temp: "));
  if (isnan(vTempF)) Serial.println(F("-- °F"));
  else {
    Serial.print(vTempF, 1);
    Serial.println(F(" °F"));
  }
  Serial.print(F("RH  : "));
  if (isnan(vHum)) Serial.println(F("-- %"));
  else {
    Serial.print(vHum, 1);
    Serial.println(F(" %"));
  }
  Serial.print(USE_SEA_LEVEL_P ? F("SLP : ") : F("Pres: "));
  if (!isfinite(vPres_inHg)) Serial.println(F("--"));
  else {
    Serial.print(vPres_inHg, 3);
    Serial.print(F(" inHg  /  "));
    Serial.print(vPres_psi, 4);
    Serial.println(F(" psi"));
  }
  Serial.print(F("Alt : "));
  Serial.print(ALTITUDE_FT);
  Serial.println(F(" ft (97.8 m)"));
  printTimeDebug();
#if CONFIG_ENABLE_MATTER_OVER_THREAD && CONFIG_OPENTHREAD_ENABLED
  otInstance* instance = esp_openthread_get_instance();
  if (instance) {
    Serial.print(F("Thread: "));
    Serial.print(threadRoleName(otThreadGetDeviceRole(instance)));
    Serial.print(F(", routerEligible="));
    Serial.println(otThreadIsRouterEligible(instance) ? F("YES") : F("NO"));
  }
#endif
  Serial.println();
}

/* ========== OLED Screen ============*/
void drawSensorScreen() {
  if (!oledPresent) return;
  computeDerived();
  const char* pTrend = iaqTrend(vPres_inHg, lastPres_inHg, 0.02f);
  lastPres_inHg = vPres_inHg;

  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  String s = " ";
  s += isnan(vTempF) ? "--" : String(vTempF, 1);
  s += "F     ";
  s += isnan(vHum) ? "--" : String(vHum, 0);
  s += "% RH";
  display.drawString(0, 0, s);

  display.setFont(ArialMT_Plain_10);
  s = " IAQ: ";
  s += sIaqLabel(vIAQ);
  s += " (";
  s += isnan(vIAQ) ? "--" : String(vIAQ, 0);
  s += ",";
  s += String(vIAQacc);
  s += ")";
  display.drawString(0, 18, s);

  s = " CO2: ";
  s += isnan(vCO2eq) ? "--" : String(vCO2eq, 0);
  s += "ppm";
  display.drawString(0, 30, s);

  s = " Pressure: ";
  s += isnan(vPres_inHg) ? "--" : String(vPres_inHg, 2);
  s += "inHg ";
  s += pTrend;
  if (!isTimeSynced()) s += " CLK";
  display.drawString(0, 42, s);
  display.display();
}

static void drawKirbyScreen() {
  if (!oledPresent) return;
  display.clear();
  int x = (display.getWidth() - KIRBY_W) / 2;
  int y = (display.getHeight() - KIRBY_H) / 2;
  display.drawXbm(x, y, KIRBY_W, KIRBY_H, kirby_bits);
  display.display();
}

static void initTimeSyncCluster() {
  esp_matter::node_t *node = esp_matter::node::get();
  if (!node) return;
  esp_matter::endpoint_t *root = esp_matter::endpoint::get(node, 0);
  if (!root) return;

  esp_matter::cluster::time_synchronization::config_t cfg;
  esp_matter::cluster_t *cluster = esp_matter::cluster::time_synchronization::create(root, &cfg, CLUSTER_FLAG_SERVER);
  timeSyncClusterReady = (cluster != nullptr);
}

static void updateOledBrightness() {
  if (!oledPresent || !timeSyncClusterReady) return;
  int64_t utcSec = 0;
  if (!getUnixTimeSeconds(utcSec)) return;
  if (!timeSyncLogged) {
    Serial.println(F("[Time] Sync successful"));
    timeSyncLogged = true;
  }

  const float lat = 47.669f;
  const float lon = -122.347f;
  int64_t baseOffsetSec = -8 * 3600;
  int64_t localSec = utcSec + baseOffsetSec;
  int64_t days = localSec / 86400;
  int64_t rem = localSec % 86400;
  if (rem < 0) { rem += 86400; days -= 1; }
  int hour = static_cast<int>(rem / 3600);
  int minute = static_cast<int>((rem % 3600) / 60);

  int y, m, d;
  civilFromDays(days, y, m, d);
  bool dst = isDstPacific(y, m, d, hour);
  float tzHours = dst ? -7.0f : -8.0f;
  if (dst) {
    localSec += 3600;
    days = localSec / 86400;
    rem = localSec % 86400;
    if (rem < 0) { rem += 86400; days -= 1; }
    hour = static_cast<int>(rem / 3600);
    minute = static_cast<int>((rem % 3600) / 60);
    civilFromDays(days, y, m, d);
  }

  float sunriseMin = NAN, sunsetMin = NAN;
  if (!computeSunTimes(y, m, d, lat, lon, tzHours, sunriseMin, sunsetMin)) return;

  float nowMin = hour * 60.0f + minute;
  bool shouldDim = (nowMin >= sunsetMin || nowMin < sunriseMin);
  if (shouldDim != oledDimmed) {
    display.setContrast(shouldDim ? 1 : 255);
    oledDimmed = shouldDim;
  }
}

/* ============== QR helpers ============== */
String urlDecode(const String& s) {
  String o;
  o.reserve(s.length());
  for (int i = 0; i < (int)s.length(); i++) {
    char c = s[i];
    if (c == '%' && i + 2 < (int)s.length()) {
      auto hexVal = [](char h) -> int {
        if (h >= '0' && h <= '9') return h - '0';
        if (h >= 'A' && h <= 'F') return 10 + (h - 'A');
        if (h >= 'a' && h <= 'f') return 10 + (h - 'a');
        return -1;
      };
      int v1 = hexVal(s[i + 1]), v2 = hexVal(s[i + 2]);
      if (v1 >= 0 && v2 >= 0) {
        o += char((v1 << 4) | v2);
        i += 2;
        continue;
      }
    } else if (c == '+') {
      o += ' ';
      continue;
    }
    o += c;
  }
  return o;
}
String extractMtPayload(const String& urlOrPayload) {
  int i = urlOrPayload.indexOf("data=");
  if (i < 0) return urlOrPayload;  // maybe already "MT:..."
  String q = urlDecode(urlOrPayload.substring(i + 5));
  int amp = q.indexOf('&');
  if (amp > 0) q = q.substring(0, amp);
  return q;
}

static bool detectOled() {
  Wire.beginTransmission(OLED_ADDR);
  return Wire.endTransmission() == 0;
}

static void showOledBootScreen() {
  if (!oledPresent) return;
  const char* line1 = "WALL-Env Snsr";
  const char* line2 = "by Waite Design Labs";
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_16);
  int h1 = 16;
  display.setFont(ArialMT_Plain_10);
  int h2 = 10;
  int gap = 2;
  int total = h1 + gap + h2;
  int y0 = (display.getHeight() - total) / 2;
  display.setFont(ArialMT_Plain_16);
  display.drawString(display.getWidth() / 2, y0, line1);
  display.setFont(ArialMT_Plain_10);
  display.drawString(display.getWidth() / 2, y0 + h1 + gap, line2);
  display.display();
  delay(5000);
}

static void showOledCalibration() {
  if (!oledPresent) return;
  int w = display.getWidth();
  int h = display.getHeight();
  display.clear();
  display.drawRect(0, 0, w, h);
  display.drawLine(0, 0, 0, h - 1);
  display.drawLine(w - 1, 0, w - 1, h - 1);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "0");
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(w - 1, 0, String(w - 1));
  display.display();
  delay(3000);
}


/* ========================================== */
/* ================== SETUP ================= */
/* ========================================== */

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  delay(200);

  Serial.println("\nWALL-Env - BME680 Wall-Powered Sensor");
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (USE_OLED) {
    if (detectOled()) {
      oledPresent = true;
      display.init();
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.setFont(ArialMT_Plain_10);
      display.flipScreenVertically();
      clearDisplayHard();
      display.drawString(0, 0, "Init BSEC2 + OLED");
      display.display();
      //showOledCalibration();
      showOledBootScreen();
    } else {
      Serial.println("OLED display not detected. Disabling display output.");
      oledPresent = false;
    }
  }

 /* ====== BSEC2 Init ======= */
  env.begin(BME_ADDR, Wire, bme68xDelayUs);
  loadBsecState();
  env.updateSubscription(sensorList, sizeof(sensorList) / sizeof(sensorList[0]), BSEC_SAMPLE_RATE_LP);
  env.attachCallback(onBsecOutputs);
  env.setTemperatureOffset(TEMP_OFFSET_C);

  // --- Begin endpoints BEFORE Matter.begin() ---
  epTemp.begin(0.0);
  epRH.begin(0.0);
  epPress.begin(1013.25);
  epAir.begin(100.0);
  initTimeSyncCluster();

  Matter.begin();
  configureThreadRouterEligibility();
  setDefaultNodeLabel();
  
  // Commission if needed (info via Serial)
  if (!Matter.isDeviceCommissioned()) {
    String manualCode = Matter.getManualPairingCode().c_str();
    String qrUrl      = Matter.getOnboardingQRCodeUrl().c_str();

    Serial.println("--------------------------------------------------");
    Serial.println("Device not commissioned yet.");
    Serial.print("Manual pairing code: ");
    Serial.println(manualCode);
    Serial.print("QR code URL: ");
    Serial.println(qrUrl);
    Serial.println("Use your Matter controller to commission this device.");
    Serial.println("--------------------------------------------------");
    showQrOnceOnOLED();
    showingQR = oledPresent;
  } else {
    Serial.println("Device already commissioned.");
    showingQR = false;
    drawSensorScreen();
  }
 
  lastUi = millis();
}

void loop() {
  env.run();
  handleBootLongPress();
  handleSerialCommands();

  // auto-exit QR mode once commissioned
  if (showingQR && Matter.isDeviceCommissioned()) {
    showingQR = false;
    clearDisplayHard();
  }

  if (!showingQR && (millis() - lastUi > UI_INTERVAL_MS)) {
    lastUi = millis();
    if (millis() - lastScreenSwap > SCREEN_SWAP_MS) {
      lastScreenSwap = millis();
      showKirby = !showKirby;
    }
    printSerial();
    if (showKirby) drawKirbyScreen();
    else drawSensorScreen();
    updateMatterEndpoints();
  }
  if (!showingQR && (millis() - lastBrightnessCheck > BRIGHTNESS_CHECK_MS)) {
    lastBrightnessCheck = millis();
    updateOledBrightness();
  }
  saveBsecStateIfReady(millis());
  
  vTaskDelay(pdMS_TO_TICKS(500));

}
