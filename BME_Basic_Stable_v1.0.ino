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
//  BME680 Sensor -> Temp, Humidity, Pressure as Matter Endpoints. (no air quality yet)
//  SH1106-based 1.3" 128x64 OLED Display -> Shows QR Code for provisioning then basic sensor data on one screen.
//  Uses BSEC2 library to interpret raw sensor data. 
//  Basic Temperature offset I calculated with some quick testing -> -8.3°f

#include <Wire.h>
#include <bsec2.h>
#include <SH1106Wire.h>
#include <qrcodeoled.h>
#include <Preferences.h>
#include <math.h>

#include <Matter.h>
#include <MatterEndPoint.h>   // Temperature/Humidity/Pressure endpoints

// ===== Pins & addresses =====
#define SDA_PIN     22
#define SCL_PIN     23
#define OLED_ADDR   0x3C
#define BME_ADDR    0x77
#define BOOT_BTN    9         // XIAO ESP32-C6 BOOT (active LOW)

// ===== Env / units =====
#define ALTITUDE_M       97.8
#define ALTITUDE_FT      321
#define TEMP_OFFSET_C    4.9 //Value is in Celsius
#define USE_SEA_LEVEL_P  1

const uint32_t UI_INTERVAL_MS = 3000;

// ===== Globals: display, QR =====
SH1106Wire display(OLED_ADDR, SDA_PIN, SCL_PIN);
QRcodeOled  qrcode(&display);
// Matter cluster / attribute IDs
static constexpr uint32_t CL_TEMPERATURE_MEASUREMENT       = 0x0402; // MeasuredValue = 0.01 °C (int16)
static constexpr uint32_t CL_RELATIVE_HUMIDITY_MEASUREMENT = 0x0405; // MeasuredValue = 0.01 % (int16)
static constexpr uint32_t CL_PRESSURE_MEASUREMENT          = 0x0403; // MeasuredValue = 0.1 kPa (int16)
static constexpr uint32_t ATTR_MEASURED_VALUE              = 0x0000;

// ===== BSEC2 =====
Bsec2 env;
bsec_virtual_sensor_t sensorList[] = {
  BSEC_OUTPUT_IAQ,
  BSEC_OUTPUT_CO2_EQUIVALENT,
  BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
  BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
  BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  BSEC_OUTPUT_RAW_PRESSURE
};

Preferences prefs;
const char* NVS_NS  = "bsec2";
const char* NVS_KEY = "state";
#define BSEC_STATE_SIZE 512

// Live vals
volatile float   vTempC=NAN, vHum=NAN, vPres_hPa=NAN;
volatile float   vIAQ=NAN, vCO2eq=NAN, vVOCeq=NAN;
volatile uint8_t vIAQacc=0;

// Derived
float vTempF=NAN, vPres_inHg=NAN, vPres_psi=NAN, vPres_hPa_sl=NAN, lastIAQ=NAN;

// UI & state
bool showingQR=false;
uint32_t lastUi=0;

// ===== Matter endpoints (declare BEFORE Matter.begin()) =====
// If your core uses slightly different names, adjust here:
MatterTemperatureSensor epTemp;
MatterHumiditySensor    epRH;
MatterPressureSensor    epPress;

// ---------- tiny helpers ----------
static inline void clearDisplayHard(){ for(int i=0;i<2;i++){ display.clear(); display.display(); delay(5);} }
static inline float c_to_f(float c){ return c*9.0f/5.0f + 32.0f; }
static inline float hPa_to_inHg(float h){ return h/33.8638866667f; }
static inline float hPa_to_psi(float h){ return h*0.0145037738f; }
static inline float seaLevelPressure_hPa(float p_hPa,float tC,float alt_m){
  if(!isfinite(p_hPa)||!isfinite(tC)) return NAN; float T=tC+273.15f;
  return p_hPa * powf(1.0f - (0.0065f*alt_m)/T, -5.257f);
}
static inline const char* iaqTrend(float nowVal,float prevVal,float th=2.0f){
  if(!isfinite(nowVal)||!isfinite(prevVal)) return " ";
  float d=nowVal-prevVal; if(d>th) return "↑"; if(d<-th) return "↓"; return "→";
}

// ---------- BSEC callback ----------
static void onBsecOutputs(const bme68xData d,const bsecOutputs out,Bsec2 b){
  for(uint8_t i=0;i<out.nOutputs;i++){
    const bsecData&o = out.output[i];
    switch(o.sensor_id){
      case BSEC_OUTPUT_IAQ:                                 vIAQ=o.signal; vIAQacc=o.accuracy; break;
      case BSEC_OUTPUT_CO2_EQUIVALENT:                      vCO2eq=o.signal; break;
      case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:               vVOCeq=o.signal; break;
      case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE: vTempC=o.signal; break;
      case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:    vHum=o.signal; break;
      case BSEC_OUTPUT_RAW_PRESSURE:                        vPres_hPa=o.signal/100.0f; break;
    }
  }
}

// ---------- BSEC state ----------
void loadBsecState(){
  if(!prefs.begin(NVS_NS,true)) return;
  size_t n=prefs.getBytesLength(NVS_KEY);
  if(n==BSEC_STATE_SIZE){
    uint8_t buf[BSEC_STATE_SIZE]; prefs.getBytes(NVS_KEY,buf,BSEC_STATE_SIZE);
    if(env.setState(buf)) Serial.println("[BSEC2] state restored");
  }
  prefs.end();
}
void saveBsecStateIfReady(uint32_t nowMs){
  static const uint32_t EVERY=5UL*60UL*1000UL; static uint32_t last=0;
  if(nowMs-last<EVERY) return; if(vIAQacc<2) return;
  uint8_t st[BSEC_STATE_SIZE];
  if(env.getState(st) && prefs.begin(NVS_NS,false)){
    prefs.putBytes(NVS_KEY,st,BSEC_STATE_SIZE); prefs.end(); last=nowMs;
    Serial.println("[BSEC2] state saved");
  }
}

// ---------- UI ----------
void computeDerived(){
  float tC = isfinite(vTempC) ? (vTempC) : NAN;
  vTempF = isfinite(tC) ? c_to_f(tC) : NAN;
  float p = vPres_hPa;
  if(USE_SEA_LEVEL_P && isfinite(p) && isfinite(tC)){
    p = seaLevelPressure_hPa(p, tC, ALTITUDE_M);
    vPres_hPa_sl = p;
  } else vPres_hPa_sl = NAN;
  vPres_inHg = isfinite(p) ? hPa_to_inHg(p) : NAN;
  vPres_psi  = isfinite(p) ? hPa_to_psi(p)  : NAN;
}

void drawSensorScreen(){
  computeDerived();
  const char* trend = iaqTrend(vIAQ,lastIAQ); lastIAQ=vIAQ;

  display.clear();
  String s="IAQ "; s+=isnan(vIAQ)?"--":String(vIAQ,0);
  s+=" ("; s+=String(vIAQacc); s+=") "; s+=trend;
  display.drawString(0,0,s);

  display.drawString(88,0,"Alt:"+String(ALTITUDE_FT)+"ft");

  s="CO2 "; s+=isnan(vCO2eq)?"--":String(vCO2eq,0);
  s+="  VOC "; s+=isnan(vVOCeq)?"--":String(vVOCeq,2);
  display.drawString(0,12,s);

  s="T "; s+=isnan(vTempF)?"--":String(vTempF,1);
  s+="F  RH "; s+=isnan(vHum)?"--":String(vHum,1); s+="%";
  display.drawString(0,24,s);

  s=USE_SEA_LEVEL_P?"SLP ":"P ";
  s+=isnan(vPres_inHg)?"--":String(vPres_inHg,3); s+=" inHg";
  display.drawString(0,36,s);

  display.drawString(0,52,"Hold BOOT 3s: Reset");
  display.display();
}

void printSerial(){
  computeDerived();
  Serial.println(F("---- BME680 (BSEC2, US Units) ----"));
  Serial.print(F("IAQ : ")); if(isnan(vIAQ)) Serial.println(F("--")); else { Serial.print(vIAQ,0); Serial.print(F(" (acc=")); Serial.print(vIAQacc); Serial.println(F(")")); }
  Serial.print(F("eCO2: ")); if(isnan(vCO2eq)) Serial.println(F("-- ppm")); else { Serial.print(vCO2eq,0); Serial.println(F(" ppm")); }
  Serial.print(F("VOC : ")); if(isnan(vVOCeq)) Serial.println(F("-- ppm eq")); else { Serial.print(vVOCeq,2); Serial.println(F(" ppm eq")); }
  Serial.print(F("Temp: ")); if(isnan(vTempF)) Serial.println(F("-- °F")); else { Serial.print(vTempF,1); Serial.println(F(" °F")); }
  Serial.print(F("RH  : ")); if(isnan(vHum))  Serial.println(F("-- %")); else { Serial.print(vHum,1);  Serial.println(F(" %")); }
  Serial.print(USE_SEA_LEVEL_P?F("SLP : "):F("Pres: "));
  if(!isfinite(vPres_inHg)) Serial.println(F("--"));
  else { Serial.print(vPres_inHg,3); Serial.print(F(" inHg  /  ")); Serial.print(vPres_psi,4); Serial.println(F(" psi")); }
  Serial.print(F("Alt : ")); Serial.print(ALTITUDE_FT); Serial.println(F(" ft (97.8 m)"));
  Serial.println();
}

// ---------- QR helpers ----------
String urlDecode(const String &s){
  String o; o.reserve(s.length());
  for(int i=0;i<(int)s.length();i++){
    char c=s[i];
    if(c=='%' && i+2<(int)s.length()){
      auto hexVal=[](char h)->int{
        if(h>='0'&&h<='9')return h-'0';
        if(h>='A'&&h<='F')return 10+(h-'A');
        if(h>='a'&&h<='f')return 10+(h-'a');
        return -1;
      };
      int v1=hexVal(s[i+1]), v2=hexVal(s[i+2]);
      if(v1>=0 && v2>=0){ o+=char((v1<<4)|v2); i+=2; continue; }
    } else if (c=='+'){ o+=' '; continue; }
    o+=c;
  }
  return o;
}
String extractMtPayload(const String &urlOrPayload){
  int i=urlOrPayload.indexOf("data=");
  if(i<0) return urlOrPayload;   // maybe already "MT:..."
  String q=urlDecode(urlOrPayload.substring(i+5));
  int amp=q.indexOf('&'); if(amp>0) q=q.substring(0,amp);
  return q;
}
void showQrOnceOnOLED(){
  qrcode.init();
  display.setContrast(255);
  clearDisplayHard();

  String url = Matter.getOnboardingQRCodeUrl();
  Serial.println("[Matter] QR URL: " + url);
  String payload = extractMtPayload(url);
  if(payload.length()==0) payload=url;
  Serial.println("[Matter] QR payload: " + payload);

  if(payload.length()==0){
    display.drawString(0,0,"QR unavailable"); display.display(); return;
  }
  qrcode.create(payload);     // draw once, no footer to avoid rolling bands
  display.display();
}

// ---------- Decommission (BOOT 3s) ----------
void handleBootLongPress(){
  static uint32_t t0=0; static bool was=false;
  bool p=(digitalRead(BOOT_BTN)==LOW);
  if(p && !was) t0=millis();
  if(!p && was) t0=0;
  if(p && t0>0 && (millis()-t0)>3000){
    Serial.println("[Matter] Decommission");
    Matter.decommission();
    delay(300);
    showQrOnceOnOLED();
    showingQR=true;
    delay(600);
  }
  was=p;
}

// ---------- Push sensor data into Matter endpoints ----------
void updateMatterEndpoints() {
  using namespace esp_matter;

  // You'll need each endpoint's numeric ID. Most Arduino-Matter endpoint classes
  // expose one of these accessors — pick the one your class has:
  //   uint16_t eidTemp  = epTemp.getEndpointId();
  //   uint16_t eidTemp  = epTemp.getEndpointID();
  //   uint16_t eidTemp  = epTemp.endpoint_id();
  // If your class uses a different name, change it here (same for RH & Pressure).
  uint16_t eidTemp  = epTemp.getEndPointId();
  uint16_t eidRH    = epRH.getEndPointId();
  uint16_t eidPress = epPress.getEndPointId();

  // ---- Temperature (°C -> 0.01°C int16) ----
  if (isfinite(vTempC)) {
    float   tC = vTempC;
    int16_t tC_x100 = (int16_t) lroundf(tC * 100.0f);
    esp_matter_attr_val_t val = esp_matter_int16(tC_x100);
    attribute::update(eidTemp, CL_TEMPERATURE_MEASUREMENT, ATTR_MEASURED_VALUE, &val);
  }

  // ---- Relative Humidity (% -> 0.01% int16) ----
  if (isfinite(vHum)) {
    int16_t rh_x100 = (int16_t) lroundf(vHum * 100.0f);
    esp_matter_attr_val_t val = esp_matter_int16(rh_x100);
    attribute::update(eidRH, CL_RELATIVE_HUMIDITY_MEASUREMENT, ATTR_MEASURED_VALUE, &val);
  }

  // ---- Pressure (spec commonly uses 0.1 kPa int16) ----
  // Convert hPa to deci-kPa: 1 hPa == 0.1 kPa -> deci-kPa == hPa numerically.
  if (isfinite(vPres_hPa)) {
    float p_hPa = vPres_hPa;
    if (USE_SEA_LEVEL_P && isfinite(vTempC)) {
      p_hPa = seaLevelPressure_hPa(vPres_hPa, vTempC, ALTITUDE_M);
    }
    int16_t p_deci_kPa = (int16_t) lroundf(p_hPa);  // e.g. 1013.2 hPa -> 1013 (== 101.3 kPa)
    esp_matter_attr_val_t val = esp_matter_int16(p_deci_kPa);
    attribute::update(eidPress, CL_PRESSURE_MEASUREMENT, ATTR_MEASURED_VALUE, &val);
  }
}
// ================= setup / loop =================
void setup(){
  Serial.begin(115200); delay(100);
  pinMode(BOOT_BTN, INPUT_PULLUP);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  display.init();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.flipScreenVertically();
  clearDisplayHard();
  display.drawString(0,0,"Init BSEC2 + OLED");
  display.display();

  // BSEC2
  env.begin(BME_ADDR, Wire);
  loadBsecState();
  env.updateSubscription(sensorList, sizeof(sensorList)/sizeof(sensorList[0]), BSEC_SAMPLE_RATE_LP);
  env.attachCallback(onBsecOutputs);
  env.setTemperatureOffset(TEMP_OFFSET_C);
  // If your build DISABLES BLE commissioning, uncomment and set Wi-Fi:
  // #if !CONFIG_ENABLE_CHIPOBLE
  //   WiFi.mode(WIFI_STA);
  //   WiFi.begin("YourSSID","YourPass");
  //   while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  //   Serial.println("\nWiFi connected: " + WiFi.localIP().toString());
  // #endif

  // --- Begin endpoints BEFORE Matter.begin() ---
  epTemp.begin();   // TemperatureMeasurement
  epRH.begin();     // RelativeHumidityMeasurement
  epPress.begin();  // PressureMeasurement

  // --- Start Matter LAST ---
  Matter.begin();

  Serial.println("\nMatter commissioning:");
  Serial.printf("Manual pairing code: %s\r\n", Matter.getManualPairingCode().c_str());
  Serial.printf("QR code URL: %s\r\n",       Matter.getOnboardingQRCodeUrl().c_str());

  if (!Matter.isDeviceCommissioned()) {
    showQrOnceOnOLED();
    showingQR = true;
  } else {
    showingQR = false;
    drawSensorScreen();
  }

  lastUi = millis();
}

void loop(){
  env.run();
  handleBootLongPress();

  // If QR is showing, don't redraw OLED (prevents camera bands)
if (!showingQR && (millis() - lastUi > UI_INTERVAL_MS)) {
  lastUi = millis();
  printSerial();
  drawSensorScreen();
  updateMatterEndpoints();   // <-- pushes T / RH / P into Matter attributes
}
  saveBsecStateIfReady(millis());
}