/**
 * ════════════════════════════════════════════════════════════
 *  AchternSensorik  v2.0  –  SensESP Edition
 *  Marine Wellendrehzahl & Richtungserkennung für ESP32
 * ════════════════════════════════════════════════════════════
 *
 *  Framework: SensESP (https://github.com/SignalK/SensESP)
 *    → WiFi Access-Point + Captive-Portal (Konfig-UI Port 80)
 *    → Signal K Server Integration (über Web-UI konfigurierbar)
 *    → OTA Update über SensESP Web-UI oder mDNS
 *    → AsyncWebServer (ESPAsyncWebServer) Port 80
 *    → ReactESP Event-Loop (kein manuelles millis() nötig)
 *
 *  Zusätzlich parallel:
 *    → NMEA2000 CAN (PGN 127488 / 127245 / 127489)
 *    → SSD1306 OLED  3 Seiten, auto-Wechsel 5 s
 *    → Serial 115200 Bd, alle 1 s
 *
 *  Signal K Pfade (SI-Einheiten!):
 *    propulsion.0.revolutions          [Hz]  = RPM / 60
 *    propulsion.0.state                [str] "ahead|astern|stopped"
 *    propulsion.0.oilPressure          [Pa]
 *    propulsion.0.oilTemperature       [K]
 *    propulsion.0.coolantTemperature   [K]
 *    propulsion.0.exhaustTemperature   [K]   ← DS18B20 T3
 *    steering.rudderAngle              [rad]
 *    environment.outside.temperature   [K]
 *    environment.outside.humidity      [0-1]
 *    environment.outside.pressure      [Pa]
 *    environment.inside.engineRoom.temperature [K]  ← DS18B20 T2
 *
 *  WiFi AP (SensESP Konfig-Portal):
 *    SSID: AchternSensorik   Pass: siehe secrets.h
 *    IP:   192.168.4.1
 *    Konfigurationsseite:  http://192.168.4.1
 *    Echtzeit-Dashboard:   http://192.168.4.1/dash
 *    JSON-API:             http://192.168.4.1/api/data
 *
 *  OTA:
 *    Hostname:   AchternSensorik
 *    Passwort:   siehe secrets.h
 *    Upload:     pio run -t upload --upload-port AchternSensorik.local
 *
 *  Messprinzip Drehrichtung (Abstände 44 / 77 / 220 mm → Verhältnis 1.00 : 1.75 : 5.00):
 *    Umfang = 341 mm
 *    M1 = 0°,  M2 ≈ 46.5°,  M3 ≈ 127.7°
 *    CW  (Vorwärts) : kurz → mittel → lang  (1.00 : 1.75 : 5.00)
 *    CCW (Rückwärts): lang → mittel → kurz  (5.00 : 1.75 : 1.00)
 *    Toleranz 25% für Erkennung
 *
 * ════════════════════════════════════════════════════════════
 */

// ── CAN-Pins VOR NMEA2000_CAN.h definieren ──────────────────────────────
#define ESP32_CAN_TX_PIN GPIO_NUM_5
#define ESP32_CAN_RX_PIN GPIO_NUM_4

// ════════════════════════════════════════════════════════════
//  SENSESP – Framework Includes
// ════════════════════════════════════════════════════════════
#include "sensesp_app_builder.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/net/http_server.h"
#include "sensesp/net/ota.h"
#include "sensesp/system/saveable.h"
#include "sensesp/ui/config_item.h"
#include "sensesp/ui/status_page_item.h"

using namespace sensesp;

// ════════════════════════════════════════════════════════════
//  HARDWARE Includes
// ════════════════════════════════════════════════════════════
#include "secrets.h"
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <NMEA2000_esp32.h>
#include <N2kMessages.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BME680.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ════════════════════════════════════════════════════════════
//  KONFIGURATION
// ════════════════════════════════════════════════════════════

#define AP_SSID          "AchternSensorik"
#define DEVICE_HOSTNAME  "AchternSensorik"
// OTA_PASSWORD wird aus secrets.h geladen (gitignored, nicht im Repo).

// ── GPIO ─────────────────────────────────────────────────
#define ENGINE_RPM_PIN   27
#define ADC_RUDDER_PIN   34
#define ADC_OILPRESS_PIN 35
#define ONE_WIRE_BUS     13
#define BME_SDA          21
#define BME_SCL          22

// ── ADC Kalibrierung (100 kΩ / 27 kΩ Teiler) ────────────
// Teiler-Faktor (R1+R2)/R2 = 127/27 = 4.70, empirisch abgeglichen:
//   Ruder:    18.5 V Eingang / ~3.90 V ADC-Max = 4.74
//   Öldruck:  18.1 V Eingang / ~3.90 V ADC-Max = 4.64
#define ADC_CAL_RUDDER    4.74f
#define ADC_CAL_OILPRESS  4.64f
#define ADC_SAMPLES       16

// ── Ruder-Geber: Spannung → Winkel ──────────────────────
#define RUDDER_V_MIN    0.5f
#define RUDDER_V_MAX    9.5f
#define RUDDER_DEG_MIN -35.0f
#define RUDDER_DEG_MAX  35.0f

// ── Öldruckgeber VDO: 0.5–4.5 V = 0–10 bar ────────────
#define OIL_V_MIN       0.5f
#define OIL_V_MAX       4.5f
#define OIL_PA_MIN      0.0f
#define OIL_PA_MAX   1000000.0f

// ── Sensor-Intervalle [ms] ───────────────────────────────
#define INTERVAL_RPM_MS     500
#define INTERVAL_TEMP_MS   2000
#define INTERVAL_BME_MS    3000
#define INTERVAL_ADC_MS     200
#define INTERVAL_SERIAL_MS 1000
#define INTERVAL_OLED_MS    500
#define INTERVAL_N2K_MS      10
#define RPM_TIMEOUT_MS     3000

// ── OLED ─────────────────────────────────────────────────
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT    64
#define OLED_RESET       -1
#define OLED_ADDRESS   0x3C
#define DISPLAY_PAGES     3
#define PAGE_INTERVAL_MS 7000

// ── Drehrichtungs-Toleranz ───────────────────────────────
#define DIR_TOLERANCE   0.25f

// ── NMEA2000 ─────────────────────────────────────────────
#define N2K_ENGINE_INST 0
#define N2K_RUDDER_INST 0

// ════════════════════════════════════════════════════════════
//  GLOBALE VARIABLEN
// ════════════════════════════════════════════════════════════

// ── Hall-Sensor ISR Ring-Buffer (4 Zeitstempel) ──────────
#define PULSE_BUF 4
volatile uint32_t pulseBuf[PULSE_BUF] = {0};
volatile uint8_t  pulseBufHead        = 0;
volatile uint32_t pulseCount          = 0;
volatile uint32_t lastPulseMs         = 0;

void IRAM_ATTR hallISR() {
  pulseBuf[pulseBufHead] = micros();
  pulseBufHead = (pulseBufHead + 1) % PULSE_BUF;
  pulseCount++;
  lastPulseMs = millis();
}

// ── Geteilter Messwert-State ─────────────────────────────
struct SensorData {
  float   rpm        = 0.0f;
  int8_t  direction  = 0;       // +1 CW / -1 CCW / 0 Stillstand
  bool    shaftValid = false;
  float   temp[4]    = {NAN, NAN, NAN, NAN};
  int     tempCount  = 0;
  float   airTemp    = NAN;
  float   humidity   = NAN;
  float   pressure   = NAN;    // hPa
  float   gasRes     = NAN;    // kΩ
  bool    bmeOk      = false;
  float   rudderVolt  = 0.0f;
  float   rudderAngle = 0.0f;  // Grad
  float   oilVolt     = 0.0f;
  float   oilPressure = 0.0f;  // Pa
} sd;

// ── Objekte ──────────────────────────────────────────────
Adafruit_SSD1306  display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_BME680   bme;
OneWire           oneWire(ONE_WIRE_BUS);
DallasTemperature dallasSensors(&oneWire);

// ── OLED-State ───────────────────────────────────────────
uint8_t  displayPage    = 0;
uint32_t lastPageSwitch = 0;

// ── CAN-Bus Status & Zähler ──────────────────────────────
bool     canBusOk   = false;
uint32_t canTxPkts  = 0;
uint32_t canErrPkts = 0;
uint32_t canRxPkts  = 0;
uint32_t canRecoveries = 0;
uint32_t canLastRecoveryMs = 0;
uint32_t canTxErr = 0;   // SJA1000 TX error counter, cached from the event loop
uint32_t canRxErr = 0;   // SJA1000 RX error counter, cached from the event loop
uint8_t  canN2kAddr = 255; // live N2K source address after address claiming

// ── NMEA2000 ─────────────────────────────────────────────
tNMEA2000* nmea2000;
uint8_t    n2k_addr = 0;  // Abgeleitete N2K-Adresse

// ── SensESP Device-Status Items (in setup() instanziiert) ────
static StatusPageItem<String>*   g_st_can_state = nullptr;
static StatusPageItem<uint32_t>* g_st_can_tx    = nullptr;
static StatusPageItem<uint32_t>* g_st_can_err   = nullptr;
static StatusPageItem<uint32_t>* g_st_can_rx    = nullptr;
static StatusPageItem<uint32_t>* g_st_can_rxerr = nullptr;
static StatusPageItem<uint32_t>* g_st_can_recov = nullptr;
static StatusPageItem<uint8_t>*  g_st_n2k_addr  = nullptr;

// ── NMEA2000 PGN-Liste ───────────────────────────────────
const unsigned long TransmitMessages[] PROGMEM = {
  127488L, 127245L, 127489L, 0
};

// SensESP 3.x: sensesp_app wird von get_app() gesetzt (kein eigenes ReactESP nötig)

// ════════════════════════════════════════════════════════════
//  DREHRICHTUNGS-KONFIG (Vorwaerts/Rueckwaerts vertauschen)
//  Vor dem Algorithmus definiert, da calcRPMandDirection() es nutzt.
// ════════════════════════════════════════════════════════════

class DirConfig : public FileSystemSaveable {
 public:
  bool invert = false;   // true = CW/CCW (Vorwaerts/Rueckwaerts) vertauschen

  DirConfig() : FileSystemSaveable("/dir/config") { load(); }

  bool to_json(JsonObject& root) override {
    root["invert"] = invert;
    return true;
  }

  bool from_json(const JsonObject& cfg) override {
    if (cfg["invert"].is<bool>())
      invert = cfg["invert"].as<bool>();
    return true;
  }
};

inline const String ConfigSchema(const DirConfig&) {
  return R"###({"type":"object","properties":{
    "invert":{"title":"Richtung umdrehen","type":"boolean",
      "description":"Vertauscht Vorwaerts/Rueckwaerts (CW/CCW) der Wellendrehrichtung, falls der Hallsensor andersherum eingebaut ist."}
  }})###";
}

static std::shared_ptr<DirConfig> g_dir_cfg;

// ════════════════════════════════════════════════════════════
//  DREHRICHTUNGS- & RPM-ALGORITHMUS
// ════════════════════════════════════════════════════════════

static inline bool approx(float r, float target, float tol = DIR_TOLERANCE) {
  return fabsf(r - target) <= target * tol;
}

void calcRPMandDirection() {
  if (millis() - lastPulseMs > RPM_TIMEOUT_MS || pulseCount < 4) {
    sd.rpm       = 0.0f;
    sd.direction = 0;
    sd.shaftValid = false;
    return;
  }

  noInterrupts();
  uint8_t  head = pulseBufHead;
  uint32_t t[PULSE_BUF];
  for (int i = 0; i < PULSE_BUF; i++)
    t[i] = pulseBuf[(head - PULSE_BUF + i + PULSE_BUF) % PULSE_BUF];
  interrupts();

  float dt0 = (float)(t[1] - t[0]);
  float dt1 = (float)(t[2] - t[1]);
  float dt2 = (float)(t[3] - t[2]);
  if (dt0 <= 0 || dt1 <= 0 || dt2 <= 0) return;

  // RPM: eine vollständige Umdrehung = alle 3 Abstände
  sd.rpm = 60000000.0f / (dt0 + dt1 + dt2);

  // Normierung auf kleinsten Abstand → Verhältnisse
  float dtMin = min({dt0, dt1, dt2});
  float r0 = dt0 / dtMin, r1 = dt1 / dtMin, r2 = dt2 / dtMin;

  // Abstände 44 / 77 / 220 mm → normiert 1.00 / 1.75 / 5.00
  // CW: [1.00, 1.75, 5.00] und zyklische Rotationen
  bool isCW  = (approx(r0,1.00f) && approx(r1,1.75f) && approx(r2,5.00f)) ||
               (approx(r0,1.75f) && approx(r1,5.00f) && approx(r2,1.00f)) ||
               (approx(r0,5.00f) && approx(r1,1.00f) && approx(r2,1.75f));

  // CCW: [5.00, 1.75, 1.00] und zyklische Rotationen
  bool isCCW = (approx(r0,5.00f) && approx(r1,1.75f) && approx(r2,1.00f)) ||
               (approx(r0,1.75f) && approx(r1,1.00f) && approx(r2,5.00f)) ||
               (approx(r0,1.00f) && approx(r1,5.00f) && approx(r2,1.75f));

  if      (isCW)  { sd.direction = +1; sd.shaftValid = true; }
  else if (isCCW) { sd.direction = -1; sd.shaftValid = true; }
  else            { sd.shaftValid = true; }  // Richtung Hysterese

  // Optional: Vorwaerts/Rueckwaerts vertauschen (Web-UI Flag "Richtung umdrehen").
  if (g_dir_cfg && g_dir_cfg->invert) sd.direction = -sd.direction;
}

// ════════════════════════════════════════════════════════════
//  ADC KONFIGURATION (über SensESP Web-UI einstellbar)
// ════════════════════════════════════════════════════════════

class ADCConfig : public FileSystemSaveable {
 public:
  float div_rudder   = ADC_CAL_RUDDER;
  float div_oilpress = ADC_CAL_OILPRESS;

  ADCConfig() : FileSystemSaveable("/adc/config") { load(); }

  bool to_json(JsonObject& root) override {
    root["div_rudder"]   = div_rudder;
    root["div_oilpress"] = div_oilpress;
    return true;
  }

  bool from_json(const JsonObject& cfg) override {
    if (cfg["div_rudder"].is<float>())
      div_rudder   = cfg["div_rudder"].as<float>();
    if (cfg["div_oilpress"].is<float>())
      div_oilpress = cfg["div_oilpress"].as<float>();
    return true;
  }
};

inline const String ConfigSchema(const ADCConfig&) {
  return R"###({"type":"object","properties":{
    "div_rudder":{"title":"Ruder Teiler-Faktor","type":"number",
      "description":"(R1+R2)/R2 des Spannungsteilers, empirisch kalibriert (Standard: 4.74)"},
    "div_oilpress":{"title":"Öldruck Teiler-Faktor","type":"number",
      "description":"(R1+R2)/R2 des Spannungsteilers, empirisch kalibriert (Standard: 4.64)"}
  }})###";
}

static std::shared_ptr<ADCConfig> g_adc_cfg;

// ════════════════════════════════════════════════════════════
//  TEMPERATURSENSOR-KONFIG (Namen + Offset, je 4 Sensoren)
// ════════════════════════════════════════════════════════════

class TempConfig : public FileSystemSaveable {
 public:
  String names[4]   = {"Kuehlwasser", "Oel", "Maschinenraum", "Abgas"};
  float  offsets[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  TempConfig() : FileSystemSaveable("/temp/config") { load(); }

  bool to_json(JsonObject& root) override {
    for (int i = 0; i < 4; i++) {
      String kn = String("t") + i + "_name";
      String ko = String("t") + i + "_off";
      root[kn] = names[i];
      root[ko] = offsets[i];
    }
    return true;
  }

  bool from_json(const JsonObject& cfg) override {
    for (int i = 0; i < 4; i++) {
      String kn = String("t") + i + "_name";
      String ko = String("t") + i + "_off";
      if (cfg[kn].is<const char*>()) names[i]   = cfg[kn].as<const char*>();
      if (cfg[ko].is<float>())       offsets[i] = cfg[ko].as<float>();
    }
    return true;
  }
};

inline const String ConfigSchema(const TempConfig&) {
  return R"###({"type":"object","properties":{
    "t0_name":{"title":"T0 Name","type":"string","description":"Anzeigename Sensor 0 (Standard: Kuehlwasser)"},
    "t0_off":{"title":"T0 Offset (Grad C)","type":"number","description":"Korrekturwert wird auf gemessenen Wert addiert"},
    "t1_name":{"title":"T1 Name","type":"string","description":"Anzeigename Sensor 1 (Standard: Oel)"},
    "t1_off":{"title":"T1 Offset (Grad C)","type":"number"},
    "t2_name":{"title":"T2 Name","type":"string","description":"Anzeigename Sensor 2 (Standard: Maschinenraum)"},
    "t2_off":{"title":"T2 Offset (Grad C)","type":"number"},
    "t3_name":{"title":"T3 Name","type":"string","description":"Anzeigename Sensor 3 (Standard: Abgas)"},
    "t3_off":{"title":"T3 Offset (Grad C)","type":"number"}
  }})###";
}

static std::shared_ptr<TempConfig> g_temp_cfg;

// ════════════════════════════════════════════════════════════
//  AP-KONFIG (Auto-Abschaltung)
// ════════════════════════════════════════════════════════════

class APConfig : public FileSystemSaveable {
 public:
  uint32_t shutoff_min = 0;   // 0 = nie

  APConfig() : FileSystemSaveable("/ap/config") { load(); }

  bool to_json(JsonObject& root) override {
    root["shutoff_min"] = shutoff_min;
    return true;
  }

  bool from_json(const JsonObject& cfg) override {
    if (cfg["shutoff_min"].is<uint32_t>())
      shutoff_min = cfg["shutoff_min"].as<uint32_t>();
    return true;
  }
};

inline const String ConfigSchema(const APConfig&) {
  return R"###({"type":"object","properties":{
    "shutoff_min":{"title":"AP Auto-Abschaltung (Minuten)","type":"integer",
      "description":"Access-Point nach N Minuten Laufzeit ausschalten. 0 = nie."}
  }})###";
}

static std::shared_ptr<APConfig> g_ap_cfg;

// ════════════════════════════════════════════════════════════
//  ADC
// ════════════════════════════════════════════════════════════

float readADC_V(uint8_t pin, float divider) {
  uint32_t sum = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) {
    sum += analogReadMilliVolts(pin);
    delayMicroseconds(80);
  }
  return (float)(sum / ADC_SAMPLES) / 1000.0f * divider;
}

static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

float mapf(float x, float i0, float i1, float o0, float o1) {
  return o0 + (clampf(x,i0,i1) - i0) * (o1 - o0) / (i1 - i0);
}

void readADC() {
  sd.rudderVolt  = readADC_V(ADC_RUDDER_PIN,   g_adc_cfg->div_rudder);
  sd.rudderAngle = mapf(sd.rudderVolt,  RUDDER_V_MIN, RUDDER_V_MAX,
                        RUDDER_DEG_MIN, RUDDER_DEG_MAX);
  sd.oilVolt     = readADC_V(ADC_OILPRESS_PIN, g_adc_cfg->div_oilpress);
  sd.oilPressure = mapf(sd.oilVolt, OIL_V_MIN, OIL_V_MAX,
                        OIL_PA_MIN, OIL_PA_MAX);
}

// ════════════════════════════════════════════════════════════
//  TEMPERATURSENSOREN
// ════════════════════════════════════════════════════════════

void readTemperatures() {
  dallasSensors.requestTemperatures();
  sd.tempCount = min((int)dallasSensors.getDeviceCount(), 4);
  for (int i = 0; i < sd.tempCount; i++) {
    float t = dallasSensors.getTempCByIndex(i);
    if (t == DEVICE_DISCONNECTED_C) {
      sd.temp[i] = NAN;
    } else {
      sd.temp[i] = t + (g_temp_cfg ? g_temp_cfg->offsets[i] : 0.0f);
    }
  }
  for (int i = sd.tempCount; i < 4; i++) sd.temp[i] = NAN;
}

void readBME680() {
  if (!sd.bmeOk) return;
  if (bme.performReading()) {
    sd.airTemp  = bme.temperature;
    sd.humidity = bme.humidity;
    sd.pressure = bme.pressure / 100.0f;
    sd.gasRes   = bme.gas_resistance / 1000.0f;
  }
}

// ════════════════════════════════════════════════════════════
//  NMEA2000
// ════════════════════════════════════════════════════════════

static void onN2kRx(const tN2kMsg&) {
  canRxPkts++;
  if (g_st_can_rx) g_st_can_rx->set(canRxPkts);
}

void sendNMEA2000() {
  tN2kMsg msg;
  bool cycleOk = false;

  SetN2kEngineParamRapid(msg, N2K_ENGINE_INST, sd.rpm);
  if (nmea2000->SendMsg(msg)) { canTxPkts++; cycleOk = true; }
  else                        { canErrPkts++;               }

  SetN2kRudder(msg, (double)(sd.rudderAngle * DEG_TO_RAD), N2K_RUDDER_INST);
  if (nmea2000->SendMsg(msg)) { canTxPkts++; cycleOk = true; }
  else                        { canErrPkts++;               }

  double coolantK = isnan(sd.temp[0]) ? N2kDoubleNA : (double)(sd.temp[0] + 273.15f);
  double oilTempK = isnan(sd.temp[1]) ? N2kDoubleNA : (double)(sd.temp[1] + 273.15f);
  SetN2kEngineDynamicParam(msg, N2K_ENGINE_INST,
    (double)sd.oilPressure, oilTempK, coolantK,
    N2kDoubleNA, N2kDoubleNA,
    (double)millis() / 1000.0,
    N2kDoubleNA, N2kDoubleNA, N2kInt8NA, N2kInt8NA,
    tN2kEngineDiscreteStatus1(0), tN2kEngineDiscreteStatus2(0));
  if (nmea2000->SendMsg(msg)) { canTxPkts++; cycleOk = true; }
  else                        { canErrPkts++;               }

  canBusOk = cycleOk;

  // Cache the SJA1000 TX/RX error counters here (event-loop task) so the HTTP
  // handler can read them without concurrent CAN-peripheral access.
  canTxErr = MODULE_CAN->TXERR.B.TXERR;
  canRxErr = MODULE_CAN->RXERR.B.RXERR;

  canN2kAddr = nmea2000->GetN2kSource();

  if (g_st_can_state) g_st_can_state->set(canBusOk ? "OK (online)" : "Fehler");
  if (g_st_can_tx)    g_st_can_tx->set(canTxPkts);
  if (g_st_can_err)   g_st_can_err->set(canTxErr);
  if (g_st_can_rxerr) g_st_can_rxerr->set(canRxErr);
  if (g_st_can_recov) g_st_can_recov->set(canRecoveries);
  if (g_st_n2k_addr)  g_st_n2k_addr->set(canN2kAddr);
}

// ════════════════════════════════════════════════════════════
//  OLED DISPLAY
// ════════════════════════════════════════════════════════════

void displayPage0_Shaft() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);  display.print(F("-- WELLE --"));
  display.setCursor(90,0); display.print(F("1/3"));
  display.setTextSize(2);
  display.setCursor(0,14); display.print((int)sd.rpm);
  display.setTextSize(1);  display.print(F(" RPM"));
  display.setCursor(0,36); display.print(F("Richtung: "));
  if      (sd.direction > 0) display.print(F("CW  Vorw."));
  else if (sd.direction < 0) display.print(F("CCW Rueck."));
  else                        display.print(F("STILLSTAND"));
  int bar = constrain((int)(sd.rpm / 3000.0f * 126.0f), 0, 126);
  display.drawRect(0,52,128,10,SSD1306_WHITE);
  display.fillRect(1,53,bar,8,SSD1306_WHITE);
  display.display();
}

void displayPage1_Temps() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);  display.print(F("-- TEMP --"));
  display.setCursor(90,0); display.print(F("2/3"));
  for (int i = 0; i < 4; i++) {
    display.setCursor(0, 12 + i*12);
    String name = g_temp_cfg ? g_temp_cfg->names[i] : String("T") + i;
    if (name.length() > 7) name = name.substring(0, 7);
    display.print(name);
    display.print(F(":"));
    display.setCursor(78, 12 + i*12);
    if (isnan(sd.temp[i])) display.print(F("--"));
    else { display.print(sd.temp[i], 1); display.print(F("C")); }
  }
  display.display();
}

void displayPage2_Engine() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);  display.print(F("-- MOTOR/RUDER --"));
  display.setCursor(90,0); display.print(F("3/3"));
  display.setCursor(0,14); display.print(F("Oeldruck: "));
  display.print(sd.oilPressure/100000.0f, 2); display.print(F(" bar"));
  display.setCursor(0,26); display.print(F("Ruder:    "));
  if (sd.rudderAngle >= 0) display.print(F("+"));
  display.print(sd.rudderAngle, 1); display.print(F(" Grad"));
  display.setCursor(0,38);
  if (!isnan(sd.airTemp)) {
    display.print(F("Luft: "));
    display.print(sd.airTemp, 1); display.print(F("C "));
    display.print((int)sd.humidity); display.print(F("%"));
  }
  display.setCursor(0,46);
  if (!isnan(sd.gasRes)) {
    display.print(F("Gas:  "));
    display.print(sd.gasRes, 0); display.print(F(" kOhm"));
  }
  uint32_t us = millis() / 1000UL;
  display.setCursor(0,54);
  display.print(F("SK+N2K Up:")); display.print(us/3600);
  display.print(F("h ")); display.print((us%3600)/60); display.print(F("m"));
  display.display();
}

void updateDisplay() {
  uint32_t now = millis();
  if (now - lastPageSwitch >= PAGE_INTERVAL_MS) {
    displayPage = (displayPage + 1) % DISPLAY_PAGES;
    lastPageSwitch = now;
  }
  switch (displayPage) {
    case 0: displayPage0_Shaft();  break;
    case 1: displayPage1_Temps();  break;
    case 2: displayPage2_Engine(); break;
  }
}

// ════════════════════════════════════════════════════════════
//  SERIAL AUSGABE
// ════════════════════════════════════════════════════════════

void printSerial() {
  const char* d = (sd.direction>0) ? "CW/Vorwaerts" :
                  (sd.direction<0) ? "CCW/Rueckwaerts" : "STILLSTAND";
  Serial.printf(
    "RPM=%.1f  Dir=%s  Ruder=%.1f°  Oel=%.2fbar "
    "T0=%.1f T1=%.1f T2=%.1f T3=%.1f  "
    "Luft=%.1f°C  Up=%lus\n",
    sd.rpm, d, sd.rudderAngle, sd.oilPressure/100000.0f,
    isnan(sd.temp[0])?0.0f:sd.temp[0],
    isnan(sd.temp[1])?0.0f:sd.temp[1],
    isnan(sd.temp[2])?0.0f:sd.temp[2],
    isnan(sd.temp[3])?0.0f:sd.temp[3],
    isnan(sd.airTemp)?0.0f:sd.airTemp,
    millis()/1000UL
  );
}

// ════════════════════════════════════════════════════════════
//  JSON API + HTML DASHBOARD
// ════════════════════════════════════════════════════════════

// Escape double-quotes / backslashes for safe JSON string embedding
static String jsEsc(const String& s) {
  String o; o.reserve(s.length() + 4);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"' || c == '\\') o += '\\';
    o += c;
  }
  return o;
}

String buildJsonData() {
  char buf[1100], t[4][12];
  for (int i = 0; i < 4; i++) {
    if (isnan(sd.temp[i])) snprintf(t[i],12,"null");
    else                    snprintf(t[i],12,"%.2f",sd.temp[i]);
  }
  String n0 = g_temp_cfg ? jsEsc(g_temp_cfg->names[0]) : "T0";
  String n1 = g_temp_cfg ? jsEsc(g_temp_cfg->names[1]) : "T1";
  String n2 = g_temp_cfg ? jsEsc(g_temp_cfg->names[2]) : "T2";
  String n3 = g_temp_cfg ? jsEsc(g_temp_cfg->names[3]) : "T3";
  const char* dStr = sd.direction>0?"ahead":sd.direction<0?"astern":"stopped";
  String hn = jsEsc(SensESPBaseApp::get_hostname());
  snprintf(buf, sizeof(buf),
    "{\"hostname\":\"%s\",\"rpm\":%.2f,\"direction\":\"%s\",\"dirNum\":%d,"
    "\"shaftValid\":%s,\"rudder_deg\":%.2f,\"rudder_v\":%.3f,"
    "\"oil_pa\":%.0f,\"oil_bar\":%.3f,\"oil_v\":%.3f,"
    "\"temp0\":%s,\"temp1\":%s,\"temp2\":%s,\"temp3\":%s,"
    "\"t0_name\":\"%s\",\"t1_name\":\"%s\",\"t2_name\":\"%s\",\"t3_name\":\"%s\","
    "\"air_temp\":%.2f,\"humidity\":%.1f,\"pressure_hpa\":%.1f,"
    "\"gas_kohm\":%.1f,\"uptime_s\":%lu,\"sk_ready\":true,"
    "\"can_ok\":%s,\"n2k_addr\":%u,\"can_tx\":%lu,\"can_err\":%lu,\"can_rx\":%lu,"
    "\"can_txerr\":%lu,\"can_rxerr\":%lu,\"can_recoveries\":%lu,\"ip\":\"%s\",\"wifi\":%s,"
    "\"free_heap\":%lu}",
    hn.c_str(),
    sd.rpm, dStr, sd.direction,
    sd.shaftValid?"true":"false",
    sd.rudderAngle, sd.rudderVolt,
    sd.oilPressure, sd.oilPressure/100000.0f, sd.oilVolt,
    t[0], t[1], t[2], t[3],
    n0.c_str(), n1.c_str(), n2.c_str(), n3.c_str(),
    isnan(sd.airTemp) ?0.0f:sd.airTemp,
    isnan(sd.humidity)?0.0f:sd.humidity,
    isnan(sd.pressure)?0.0f:sd.pressure,
    isnan(sd.gasRes)  ?0.0f:sd.gasRes,
    millis()/1000UL,
    canBusOk?"true":"false", (unsigned)canN2kAddr, canTxPkts, canErrPkts, canRxPkts,
    (unsigned long)canTxErr, (unsigned long)canRxErr,
    (unsigned long)canRecoveries,
    WiFi.localIP().toString().c_str(),
    (WiFi.status() == WL_CONNECTED) ? "true" : "false",
    (unsigned long)ESP.getFreeHeap()
  );
  return String(buf);
}

// ════════════════════════════════════════════════════════════
//  CUSTOM HTTP-ROUTES auf SensESP's Port-80-Server
//  Wir holen den HTTPServer aus dem ConfigItem-Registry und
//  registrieren unsere Handler direkt dort. Kein 2. httpd!
// ════════════════════════════════════════════════════════════

// Trick um an die protected-Member von ConfigItemT<T> zu kommen:
// Subklasse mit identischem Layout, dann static_cast.
template<typename T>
struct ConfigItemPeek : public ConfigItemT<T> {
  static std::shared_ptr<T> grab(ConfigItemT<T>& it) {
    return static_cast<ConfigItemPeek<T>&>(it).config_object_;
  }
};

// Globale Handler – müssen leben solange Server lebt.
static std::shared_ptr<HTTPRequestHandler> g_h_data;
static std::shared_ptr<HTTPRequestHandler> g_h_diag;
static std::shared_ptr<HTTPRequestHandler> g_h_can;
static std::shared_ptr<HTTPRequestHandler> g_h_restart;
static std::shared_ptr<HTTPRequestHandler> g_h_dash;
static std::shared_ptr<HTTPRequestHandler> g_h_wifi;

// Forward decls (HTML-Strings stehen weiter unten)
extern const char DASH_HTML[];
extern const char WIFICONFIG_HTML[];

static void registerCustomRoutes() {
  auto base = ConfigItemBase::get_config_item("/system/httpserver");
  if (!base) {
    Serial.println(F("Custom routes: SensESP HTTPServer NICHT gefunden"));
    return;
  }
  auto cit = std::static_pointer_cast<ConfigItemT<HTTPServer>>(base);
  std::shared_ptr<HTTPServer> srv = ConfigItemPeek<HTTPServer>::grab(*cit);
  if (!srv) {
    Serial.println(F("Custom routes: HTTPServer nullptr"));
    return;
  }

  g_h_data = std::make_shared<HTTPRequestHandler>(
      1u << HTTP_GET, "/api/data",
      [](httpd_req_t* req) -> esp_err_t {
        String data = buildJsonData();
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
        httpd_resp_send(req, data.c_str(), data.length());
        return ESP_OK;
      });
  srv->add_handler(g_h_data);

  g_h_diag = std::make_shared<HTTPRequestHandler>(
      1u << HTTP_GET, "/api/diag",
      [](httpd_req_t* req) -> esp_err_t {
        size_t total = SPIFFS.totalBytes();
        size_t used  = SPIFFS.usedBytes();
        String body = "{\"spiffs_total\":" + String((unsigned long)total) +
                      ",\"spiffs_used\":"  + String((unsigned long)used)  +
                      ",\"spiffs_free\":"  + String((unsigned long)(total - used)) +
                      ",\"files\":[";
        File root = SPIFFS.open("/");
        bool first = true;
        if (root && root.isDirectory()) {
          File f = root.openNextFile();
          while (f) {
            if (!first) body += ",";
            first = false;
            body += "{\"name\":\"" + String(f.name()) +
                    "\",\"size\":" + String((unsigned long)f.size()) + "}";
            f = root.openNextFile();
          }
        }
        body += "]}";
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, body.c_str(), body.length());
        return ESP_OK;
      });
  srv->add_handler(g_h_diag);

  g_h_can = std::make_shared<HTTPRequestHandler>(
      1u << HTTP_GET, "/api/can",
      [](httpd_req_t* req) -> esp_err_t {
        // SJA1000-Register direkt lesen (NMEA2000_esp32 nutzt nicht den IDF-
        // TWAI-Treiber, daher kein twai_get_status_info verfuegbar).
        uint32_t sr_u   = MODULE_CAN->SR.U;
        uint32_t txerr  = MODULE_CAN->TXERR.B.TXERR;
        uint32_t rxerr  = MODULE_CAN->RXERR.B.RXERR;
        uint32_t ecc    = MODULE_CAN->ECC.B.ECC;
        uint32_t alc    = MODULE_CAN->ALC.B.ALC;
        bool bus_off    = (sr_u >> 7) & 1;
        bool err_status = (sr_u >> 6) & 1;
        bool tx_complete= (sr_u >> 3) & 1;
        bool tx_buf_idle= (sr_u >> 2) & 1;
        bool rx_buf_full= (sr_u >> 0) & 1;
        const char* state =
          bus_off    ? "BUS_OFF" :
          err_status ? "ERROR_PASSIVE_OR_WARN" :
                       "ACTIVE";
        char buf[420];
        snprintf(buf, sizeof(buf),
          "{\"state\":\"%s\",\"bus_off\":%s,\"error_status\":%s,"
          "\"tx_complete\":%s,\"tx_buf_idle\":%s,\"rx_buf_full\":%s,"
          "\"tx_err_cnt\":%lu,\"rx_err_cnt\":%lu,"
          "\"ecc\":%lu,\"alc\":%lu,"
          "\"sr_raw\":\"0x%02lx\","
          "\"recoveries\":%lu,"
          "\"app_tx\":%lu,\"app_err\":%lu,\"app_rx\":%lu}",
          state,
          bus_off?"true":"false", err_status?"true":"false",
          tx_complete?"true":"false", tx_buf_idle?"true":"false",
          rx_buf_full?"true":"false",
          (unsigned long)txerr, (unsigned long)rxerr,
          (unsigned long)ecc,   (unsigned long)alc,
          (unsigned long)(sr_u & 0xFF),
          canRecoveries,
          canTxPkts, canErrPkts, canRxPkts);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
        httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
      });
  srv->add_handler(g_h_can);

  g_h_restart = std::make_shared<HTTPRequestHandler>(
      1u << HTTP_POST, "/api/restart",
      [](httpd_req_t* req) -> esp_err_t {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, "{\"status\":\"restarting\"}", HTTPD_RESP_USE_STRLEN);
        event_loop()->onDelay(500, []() { ESP.restart(); });
        return ESP_OK;
      });
  srv->add_handler(g_h_restart);

  g_h_dash = std::make_shared<HTTPRequestHandler>(
      1u << HTTP_GET, "/dash",
      [](httpd_req_t* req) -> esp_err_t {
        httpd_resp_set_type(req, "text/html");
        httpd_resp_send(req, DASH_HTML, strlen(DASH_HTML));
        return ESP_OK;
      });
  srv->add_handler(g_h_dash);

  g_h_wifi = std::make_shared<HTTPRequestHandler>(
      1u << HTTP_GET, "/wificonfig",
      [](httpd_req_t* req) -> esp_err_t {
        httpd_resp_set_type(req, "text/html");
        httpd_resp_send(req, WIFICONFIG_HTML, strlen(WIFICONFIG_HTML));
        return ESP_OK;
      });
  srv->add_handler(g_h_wifi);

  Serial.println(F("Custom routes :80/{dash,wificonfig,api/data,api/diag,api/can,api/restart} OK"));
}

// HTML im Flash (PROGMEM) – spart Heap
const char DASH_HTML[] PROGMEM = R"rawhtml(<!DOCTYPE html>
<html lang="de"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>AchternSensorik</title><style>
:root{--bg:#0b1620;--card:#13202c;--fg:#e7eef5;--mut:#7f97a8;--acc:#00ff7f;--w:#fa0;--b:#f44}
*{box-sizing:border-box}body{margin:0;font-family:system-ui,Arial,sans-serif;background:var(--bg);color:var(--fg)}
header{padding:14px 18px;background:#0e1c28;display:flex;justify-content:space-between;align-items:center}
header h1{font-size:18px;margin:0;letter-spacing:.04em}#conn{font-size:13px;color:var(--mut)}
.brand{display:flex;align-items:center;gap:12px}
.seslogo{display:inline-flex;align-items:center;transition:opacity .15s}
.seslogo img{height:30px;width:auto;display:block}
.seslogo:hover{opacity:.75}
.grid{display:grid;gap:12px;grid-template-columns:repeat(auto-fill,minmax(240px,1fr));padding:14px}
.card{background:var(--card);border-radius:10px;padding:14px 16px}
.card h2{font-size:12px;text-transform:uppercase;letter-spacing:.06em;color:var(--mut);margin:0 0 10px}
.row{display:flex;justify-content:space-between;align-items:baseline;padding:4px 0}
.row .l{color:var(--mut);font-size:14px}
.row .v{font-variant-numeric:tabular-nums;font-weight:600;color:var(--acc)}
.v.w{color:var(--w)}.v.b{color:var(--b)}
.plain .v{color:var(--fg)}
.big{font-size:34px;font-weight:700;color:var(--acc)}.unit{font-size:14px;color:var(--mut);font-weight:400;margin-left:4px}
</style></head><body>
<header><div class="brand"><a class="seslogo" href="/" title="SensESP Konfiguration"><img src="/SensESP_logo_symbol.svg" alt="SensESP"></a><h1 id="hn">AchternSensorik</h1></div><span id="conn">verbinde…</span></header>
<div class="grid">
<div class="card"><h2>Welle</h2>
<div class="big"><span id="rpm">--</span><span class="unit">U/min</span></div>
<div class="row"><span class="l">Drehrichtung</span><span class="v" id="dir">--</span></div></div>
<div class="card"><h2>Motor &amp; Ruder</h2>
<div class="row"><span class="l">Öldruck</span><span class="v" id="oil">--</span></div>
<div class="row"><span class="l">Ruderwinkel</span><span class="v" id="rud">--</span></div></div>
<div class="card"><h2>Temperaturen</h2>
<div class="row"><span class="l" id="t0l">T0</span><span class="v" id="t0">--</span></div>
<div class="row"><span class="l" id="t1l">T1</span><span class="v" id="t1">--</span></div>
<div class="row"><span class="l" id="t2l">T2</span><span class="v" id="t2">--</span></div>
<div class="row"><span class="l" id="t3l">T3</span><span class="v" id="t3">--</span></div></div>
<div class="card"><h2>Umgebung</h2>
<div class="row"><span class="l">Lufttemperatur</span><span class="v" id="at">--</span></div>
<div class="row"><span class="l">Luftfeuchte</span><span class="v" id="hum">--</span></div>
<div class="row"><span class="l">Luftdruck</span><span class="v" id="pres">--</span></div>
<div class="row"><span class="l">Gas-Widerstand</span><span class="v" id="gas">--</span></div></div>
<div class="card plain"><h2>NMEA 2000</h2>
<div class="row"><span class="l">Status</span><span class="v" id="cbst">--</span></div>
<div class="row"><span class="l">Geräteadresse</span><span class="v" id="cbaddr">--</span></div>
<div class="row"><span class="l">TX / RX Pakete</span><span class="v"><span id="cbtx">--</span> / <span id="cbrx">--</span></span></div>
<div class="row"><span class="l">TX / RX Fehler</span><span class="v"><span id="cbte">--</span> / <span id="cbre">--</span></span></div>
<div class="row"><span class="l">Recoveries</span><span class="v" id="cbrec">--</span></div></div>
<div class="card plain"><h2>System</h2>
<div class="row"><span class="l">Hostname</span><span class="v" id="host">--</span></div>
<div class="row"><span class="l">IP</span><span class="v" id="ip">--</span></div>
<div class="row"><span class="l">WLAN</span><span class="v" id="wifi">--</span></div>
<div class="row"><span class="l">Laufzeit</span><span class="v" id="up">--</span></div>
<div class="row"><span class="l">Freier Speicher</span><span class="v" id="heap">--</span></div></div>
</div>
<script>
var $=function(i){return document.getElementById(i)};
function f(v,d){return (v==null)?'--':(typeof v==='number'?v.toFixed(d):v)}
function upt(s){if(s==null)return'--';var d=Math.floor(s/86400),h=Math.floor(s%86400/3600),m=Math.floor(s%3600/60);return (d?d+'d ':'')+h+'h '+m+'m'}
function cls(id,c){var e=$(id);if(e)e.className='v'+(c?' '+c:'')}
function setConn(ok){$('conn').textContent=ok?'● live':'○ offline';$('conn').style.color=ok?'#00ff7f':'#f44'}
function render(d){
if(d.hostname){document.title=d.hostname;$('hn').textContent=d.hostname}
$('rpm').textContent=f(d.rpm,0);
$('dir').textContent=d.dirNum>0?'Vorwärts':d.dirNum<0?'Rückwärts':'Stillstand';
$('oil').textContent=f(d.oil_bar,2)+' bar';cls('oil',(d.oil_bar!=null&&d.oil_bar<0.5)?'w':'');
$('rud').textContent=f(d.rudder_deg,1)+' °';
['t0','t1','t2','t3'].forEach(function(id,i){
var v=d['temp'+i];
$(id).textContent=(v==null?'--':v.toFixed(1)+' °C');cls(id,(v!=null&&v>95)?'b':'');
var l=$(id+'l');if(l)l.textContent=(d[id+'_name']||('T'+i))});
$('at').textContent=f(d.air_temp,1)+' °C';
$('hum').textContent=f(d.humidity,0)+' %';
$('pres').textContent=f(d.pressure_hpa,0)+' hPa';
$('gas').textContent=f(d.gas_kohm,0)+' kΩ';
$('cbst').textContent=d.can_ok?'Online':'Fehler';
$('cbaddr').textContent=d.n2k_addr!=null?(d.n2k_addr===255?'claiming…':String(d.n2k_addr)):'--';
$('cbtx').textContent=f(d.can_tx);$('cbrx').textContent=f(d.can_rx);
$('cbte').textContent=f(d.can_txerr);$('cbre').textContent=f(d.can_rxerr);
$('cbrec').textContent=f(d.can_recoveries);
$('host').textContent=f(d.hostname);
$('ip').textContent=f(d.ip);
$('wifi').textContent=d.wifi?'verbunden':'getrennt';
$('up').textContent=upt(d.uptime_s);
$('heap').textContent=d.free_heap!=null?Math.round(d.free_heap/1024)+' kB':'--'}
var fails=0;
function schedule(){setTimeout(tick,1000)}
function tick(){
var ac=new AbortController();var to=setTimeout(function(){ac.abort()},5000);
fetch('/api/data',{cache:'no-store',signal:ac.signal})
.then(function(r){if(!r.ok)throw 0;return r.json()})
.then(function(d){clearTimeout(to);fails=0;setConn(true);render(d);schedule()})
.catch(function(){clearTimeout(to);if(++fails>=3)setConn(false);schedule()})}
tick();
</script>
</body></html>)rawhtml";

// ── WLAN-Konfiguration: iframe-Wrapper mit Bootstrap-Spinner ────────────────
const char WIFICONFIG_HTML[] PROGMEM = R"rawhtml(<!DOCTYPE html>
<html lang="de">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Konfiguration</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
html,body{height:100%;background:#0a1520;font-family:monospace}
.topbar{display:flex;align-items:center;justify-content:space-between;
  padding:8px 14px;background:#0a1520;border-bottom:1px solid #1e2d45}
.logo{color:#00ff7f;text-decoration:none;font-size:.8rem;
  letter-spacing:.06em;font-weight:600}
.logo:hover{filter:brightness(1.2)}
.page{color:#555;font-size:.62rem;text-transform:uppercase;letter-spacing:.14em}
.frame-wrap{position:relative;height:calc(100vh - 37px)}
iframe{width:100%;height:100%;border:none}
.loading{position:absolute;inset:0;display:flex;align-items:center;
  justify-content:center;background:#212529;transition:opacity .3s}
.loading.done{opacity:0;pointer-events:none}
.spinner-border{display:inline-block;width:2rem;height:2rem;
  vertical-align:-.125em;border-radius:50%;color:#fff;
  border:.25em solid currentcolor;border-right-color:transparent;
  animation:spinner-border .75s linear infinite}
@keyframes spinner-border{to{transform:rotate(360deg)}}
</style>
</head>
<body>
<header class="topbar">
  <a class="logo" href="/dash" title="Zum Dashboard">⚓ <span id="hn">AchternSensorik</span></a>
  <span class="page" id="hdr">Konfiguration</span>
</header>
<script>
fetch('/api/data').then(r=>r.json()).then(d=>{
  if(d.hostname){
    document.title=d.hostname+' Konfiguration';
    var hn=document.getElementById('hn'); if(hn) hn.textContent=d.hostname;
  }
}).catch(function(){});
</script>
<div class="frame-wrap">
  <div class="loading" id="ld">
    <div class="spinner-border" role="status"></div>
  </div>
  <iframe src="/" onload="document.getElementById('ld').className='loading done'"></iframe>
</div>
<script>
// Wenn AP-Auto-Abschaltung gesetzt ist: nach Ablauf zur Dashboard-Seite springen.
Promise.all([
  fetch('/api/data').then(function(r){return r.json();}),
  fetch('/api/config/ap/config').then(function(r){return r.json();})
]).then(function(arr){
  var d = arr[0], cfg = arr[1].config || {};
  var shutoff_min = cfg.shutoff_min || 0;
  if (shutoff_min <= 0) return;
  var remain_s = (shutoff_min * 60) - (d.uptime_s || 0);
  var hdr = document.getElementById('hdr');
  if (remain_s <= 0) {
    if (hdr) hdr.textContent = 'Wechsel zum Dashboard...';
    location.replace('/dash');
  } else {
    var mins = Math.ceil(remain_s / 60);
    if (hdr) hdr.textContent = 'Konfig (Auto -> Dash in ' + mins + ' min)';
    setTimeout(function(){ location.replace('/dash'); }, remain_s * 1000);
  }
}).catch(function(){});
</script>
</body></html>)rawhtml";

// ════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println(F("\n=== AchternSensorik v2.0 – SensESP ==="));

  // ── I2C ─────────────────────────────────────────────────
  Wire.begin(BME_SDA, BME_SCL);

  // ── OLED ────────────────────────────────────────────────
  if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(14,18); display.print(F("AchternSensorik"));
    display.setCursor(22,30); display.print(F("SensESP  v2.0"));
    display.setCursor(10,44); display.print(F("AP: " AP_SSID));
    display.display();
    Serial.println(F("OLED: OK"));
  } else {
    Serial.println(F("OLED: nicht gefunden (optional)"));
  }

  // ── BME680 ──────────────────────────────────────────────
  if (bme.begin()) {
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150);
    sd.bmeOk = true;
    Serial.println(F("BME680: OK"));
  } else {
    Serial.println(F("BME680: nicht gefunden (optional)"));
  }

  // ── DS18B20 ─────────────────────────────────────────────
  dallasSensors.begin();
  sd.tempCount = min((int)dallasSensors.getDeviceCount(), 4);
  Serial.printf("DS18B20: %d Sensoren\n", sd.tempCount);

  // ── ADC ─────────────────────────────────────────────────
  analogReadResolution(12);
  analogSetPinAttenuation(ADC_RUDDER_PIN,   ADC_11db);
  analogSetPinAttenuation(ADC_OILPRESS_PIN, ADC_11db);

  // ── Hall-Sensor ─────────────────────────────────────────
  pinMode(ENGINE_RPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENGINE_RPM_PIN), hallISR, FALLING);
  Serial.printf("Hall: GPIO %d Interrupt aktiv\n", ENGINE_RPM_PIN);

  // ════════════════════════════════════════════════════════
  //  SENSESP APP
  //  Verwaltet: WiFi-AP, mDNS, OTA, AsyncWebServer, Signal K
  // ════════════════════════════════════════════════════════
  SensESPAppBuilder builder;
  // Wichtig: set_hostname() NICHT in der Builder-Chain aufrufen –
  // PersistingObservableValue::set() schreibt sofort in SPIFFS und
  // ueberschreibt jeden vom User geaenderten Hostnamen bei jedem Boot.
  sensesp_app = (&builder)
    ->set_wifi_access_point(AP_SSID, AP_PASS)
    ->get_app();

  // Erstboot-Default: nur setzen, wenn noch der SensESP-Default ("SensESP")
  // greift – dann wird unser Wunsch-Default einmalig persistiert.
  if (SensESPBaseApp::get_hostname() == "SensESP") {
    SensESPBaseApp::get()->get_hostname_observable()->set(DEVICE_HOSTNAME);
    Serial.println(F("Hostname Default gesetzt: " DEVICE_HOSTNAME));
  } else {
    Serial.printf("Hostname (gespeichert): %s\n",
                  SensESPBaseApp::get_hostname().c_str());
  }

  // SensESP::OTA ruft ArduinoOTA.begin() ohne setHostname() auf, daher
  // landet mDNS sonst auf "esp32-<MAC>". Hier den richtigen Namen vorgeben,
  // bevor der Event-Loop die OTA-Initialisierung anstoesst.
  ArduinoOTA.setHostname(SensESPBaseApp::get_hostname().c_str());

  // ── NMEA2000 Adresse von MAC ableiten ────────────────────
  uint8_t mac[6];
  WiFi.macAddress(mac);
  uint16_t mac_suffix = (mac[4] << 8) | mac[5];
  n2k_addr = mac_suffix % 128;  // 0-127 für Geräteadressen
  Serial.printf("MAC-Adresse: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.printf("Abgeleitete N2K-Adresse: %d\n", n2k_addr);

  // ── NMEA2000 ─────────────────────────────────────────────
  nmea2000 = new tNMEA2000_esp32(ESP32_CAN_TX_PIN, ESP32_CAN_RX_PIN);
  nmea2000->SetProductInformation("00000002", 2100,
    "AchternSensorik", "2.0.2", "2.0.3");
  nmea2000->SetDeviceInformation(100002, 130, 50, 2041);
  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly, n2k_addr);
  nmea2000->EnableForward(false);
  nmea2000->ExtendTransmitMessages(TransmitMessages);
  nmea2000->SetMsgHandler(onN2kRx);
  bool n2k_open = nmea2000->Open();
  Serial.printf("NMEA2000: CAN geöffnet TX=5 RX=4, Open()=%s\n", n2k_open ? "OK" : "FAIL");
  Serial.printf("NMEA2000: requested address=%u, current address=%u\n",
                n2k_addr, nmea2000->GetN2kSource());
  if (nmea2000->ReadResetAddressChanged()) {
    Serial.printf("NMEA2000: address changed after open -> %u\n",
                  nmea2000->GetN2kSource());
  }

  // ── N2K-Adresse Status Item ──────────────────────────────
  g_st_n2k_addr = new StatusPageItem<uint8_t>("N2K-Adresse", n2k_addr, "Canbus", 110);

  // ── ADC Konfiguration laden (SPIFFS jetzt bereit) ────────
  g_adc_cfg = std::make_shared<ADCConfig>();   // load() intern
  ConfigItem(g_adc_cfg)
    ->set_title("ADC Kalibrierung")
    ->set_description(
      "Spannungsteiler-Faktoren fuer Ruder- und Oeldruck-Geber.<br>"
      "Formel: V_in = V_adc &times; Faktor")
    ->set_sort_order(300);
  readADC();   // erste Messung mit geladenen Werten

  // ── Temperatursensor-Konfiguration (Namen + Offsets) ─────
  g_temp_cfg = std::make_shared<TempConfig>();
  ConfigItem(g_temp_cfg)
    ->set_title("Temperatursensoren")
    ->set_description(
      "Anzeigenamen und Offset-Korrektur fuer die 4 DS18B20-Sensoren.<br>"
      "Reihenfolge ergibt sich aus dem 1-Wire Bus.")
    ->set_sort_order(310);

  // ── AP-Konfiguration (Auto-Abschaltung) ──────────────────
  g_ap_cfg = std::make_shared<APConfig>();
  ConfigItem(g_ap_cfg)
    ->set_title("WLAN Access-Point")
    ->set_description(
      "Auto-Abschaltung des Access-Points nach N Minuten Laufzeit.<br>"
      "0 = nie abschalten. Aenderung wirkt nach Neustart.")
    ->set_sort_order(220);

  if (g_ap_cfg->shutoff_min > 0) {
    uint32_t ms = g_ap_cfg->shutoff_min * 60000UL;
    Serial.printf("AP Auto-Off in %lu min\n", g_ap_cfg->shutoff_min);
    event_loop()->onDelay(ms, []() {
      Serial.println(F("AP Auto-Off: stopping soft-AP"));
      WiFi.softAPdisconnect(true);
    });
  }

  // ── Drehrichtungs-Konfiguration (Vorwaerts/Rueckwaerts) ──
  g_dir_cfg = std::make_shared<DirConfig>();
  ConfigItem(g_dir_cfg)
    ->set_title("Wellendrehrichtung")
    ->set_description(
      "Vorwaerts/Rueckwaerts (CW/CCW) der Drehrichtungserkennung vertauschen, "
      "falls der Hallsensor andersherum eingebaut ist.")
    ->set_sort_order(210);

  // ── Device-Status Items: Canbus + Dashboard-Link ─────────
  g_st_can_state = new StatusPageItem<String>  ("Status",     "unbekannt", "Canbus", 100);
  g_st_can_tx    = new StatusPageItem<uint32_t>("TX Pakete",  0,           "Canbus", 120);
  g_st_can_rx    = new StatusPageItem<uint32_t>("RX Pakete",  0,           "Canbus", 130);
  g_st_can_err   = new StatusPageItem<uint32_t>("TX Fehler",  0,           "Canbus", 140);
  g_st_can_rxerr = new StatusPageItem<uint32_t>("RX Fehler",  0,           "Canbus", 150);
  g_st_can_recov = new StatusPageItem<uint32_t>("Recoveries", 0,           "Canbus", 160);

  // ── Sensor-Werte auf der Status-Seite (Gruppe "Sensoren") ──
  auto* st_rpm    = new StatusPageItem<String>("Drehzahl",  "--", "Sensoren", 10);
  auto* st_dir    = new StatusPageItem<String>("Richtung",  "--", "Sensoren", 20);
  auto* st_rudder = new StatusPageItem<String>("Ruder",     "--", "Sensoren", 30);
  auto* st_oil    = new StatusPageItem<String>("Oeldruck",  "--", "Sensoren", 40);
  auto* st_t0     = new StatusPageItem<String>(g_temp_cfg->names[0], "--", "Sensoren", 50);
  auto* st_t1     = new StatusPageItem<String>(g_temp_cfg->names[1], "--", "Sensoren", 55);
  auto* st_t2     = new StatusPageItem<String>(g_temp_cfg->names[2], "--", "Sensoren", 60);
  auto* st_t3     = new StatusPageItem<String>(g_temp_cfg->names[3], "--", "Sensoren", 65);
  auto* st_air    = new StatusPageItem<String>("Lufttemperatur", "--", "Sensoren", 70);
  auto* st_hum    = new StatusPageItem<String>("Luftfeuchte",    "--", "Sensoren", 75);
  auto* st_pres   = new StatusPageItem<String>("Luftdruck",      "--", "Sensoren", 80);
  auto* st_gas    = new StatusPageItem<String>("Gas-Widerstand", "--", "Sensoren", 85);

  event_loop()->onRepeat(1000, [st_rpm, st_dir, st_rudder, st_oil, st_t0, st_t1,
                                st_t2, st_t3, st_air, st_hum, st_pres, st_gas]() {
    char v[28];
    snprintf(v, sizeof(v), "%.0f RPM", sd.rpm);
    st_rpm->set(v);
    st_dir->set(sd.direction > 0 ? "Vorwärts"
              : sd.direction < 0 ? "Rückwärts" : "Stillstand");
    snprintf(v, sizeof(v), "%+.1f °", sd.rudderAngle);
    st_rudder->set(v);
    snprintf(v, sizeof(v), "%.2f bar", sd.oilPressure / 100000.0f);
    st_oil->set(v);
    StatusPageItem<String>* ts[4] = {st_t0, st_t1, st_t2, st_t3};
    for (int i = 0; i < 4; i++) {
      if (isnan(sd.temp[i])) {
        ts[i]->set("--");
      } else {
        snprintf(v, sizeof(v), "%.1f °C", sd.temp[i]);
        ts[i]->set(v);
      }
    }
    if (isnan(sd.airTemp)) st_air->set("--");
    else { snprintf(v, sizeof(v), "%.1f °C", sd.airTemp); st_air->set(v); }
    if (isnan(sd.humidity)) st_hum->set("--");
    else { snprintf(v, sizeof(v), "%.0f %%", sd.humidity); st_hum->set(v); }
    if (isnan(sd.pressure)) st_pres->set("--");
    else { snprintf(v, sizeof(v), "%.0f hPa", sd.pressure); st_pres->set(v); }
    if (isnan(sd.gasRes)) st_gas->set("--");
    else { snprintf(v, sizeof(v), "%.0f kOhm", sd.gasRes); st_gas->set(v); }
  });

  // ── OTA Passwort (SensESP-Wrapper) ───────────────────────
  new OTA(OTA_PASSWORD);

  // ════════════════════════════════════════════════════════
  //  REAKTIVE SENSOR-PIPELINE
  //  RepeatSensor<T> → [LambdaTransform] → SKOutput<T>
  //                  → LambdaConsumer (Nebenwirkungen)
  // ════════════════════════════════════════════════════════

  // ── RPM → Signal K (propulsion.0.revolutions in Hz) ─────
  auto* rpmSensor = new RepeatSensor<float>(INTERVAL_RPM_MS, []() -> float {
    calcRPMandDirection();   // aktualisiert sd.rpm + sd.direction
    return sd.rpm;
  });

  rpmSensor
    ->connect_to(new LambdaTransform<float,float>(
        [](float rpm) { return rpm / 60.0f; },   // RPM → Hz (SI)
        "/propulsion/rpm/hz_transform"
    ))
    ->connect_to(new SKOutput<float>(
        "propulsion.0.revolutions",
        "/propulsion/revolutions"
    ));

  // NMEA2000 parallel (alle PGNs auf einmal)
  rpmSensor->connect_to(new LambdaConsumer<float>([](float) {
    sendNMEA2000();
  }));

  // ── Drehrichtung → Signal K (propulsion.0.state) ─────────
  auto* dirSensor = new RepeatSensor<float>(INTERVAL_RPM_MS, []() -> float {
    return (float)sd.direction;   // sd.direction wurde bei rpm bereits gesetzt
  });

  dirSensor
    ->connect_to(new LambdaTransform<float,String>(
        [](float d) -> String {
          if (d > 0.5f) return "astern";
          if (d < -0.5) return "ahead";
          return "stopped";
        },
        "/propulsion/direction/transform"
    ))
    ->connect_to(new SKOutput<String>(
        "propulsion.0.state",
        "/propulsion/state"
    ));

  // ── Ruderwinkel → Signal K (steering.rudderAngle in rad) ──
  auto* rudSensor = new RepeatSensor<float>(INTERVAL_ADC_MS, []() -> float {
    return sd.rudderAngle * DEG_TO_RAD;
  });
  rudSensor->connect_to(new SKOutput<float>(
      "steering.rudderAngle", "/steering/rudderAngle"));

  // ── Öldruck → Signal K (Pa) ──────────────────────────────
  auto* oilSensor = new RepeatSensor<float>(INTERVAL_ADC_MS, []() -> float {
    return sd.oilPressure;
  });
  oilSensor->connect_to(new SKOutput<float>(
      "propulsion.0.oilPressure", "/propulsion/oilPressure"));

  // ── DS18B20 T0: Kühlwasser (K) ───────────────────────────
  auto* coolantSensor = new RepeatSensor<float>(INTERVAL_TEMP_MS, []() -> float {
    return isnan(sd.temp[0]) ? NAN : sd.temp[0] + 273.15f;
  });
  coolantSensor->connect_to(new SKOutput<float>(
      "propulsion.0.coolantTemperature", "/propulsion/coolantTemp"));

  // ── DS18B20 T1: Öltemperatur (K) ────────────────────────
  auto* oilTempSensor = new RepeatSensor<float>(INTERVAL_TEMP_MS, []() -> float {
    return isnan(sd.temp[1]) ? NAN : sd.temp[1] + 273.15f;
  });
  oilTempSensor->connect_to(new SKOutput<float>(
      "propulsion.0.oilTemperature", "/propulsion/oilTemp"));

  // ── DS18B20 T2: Maschinenraum (K) ───────────────────────
  auto* erTempSensor = new RepeatSensor<float>(INTERVAL_TEMP_MS, []() -> float {
    return isnan(sd.temp[2]) ? NAN : sd.temp[2] + 273.15f;
  });
  erTempSensor->connect_to(new SKOutput<float>(
      "environment.inside.engineRoom.temperature",
      "/environment/engineRoomTemp"));

  // ── DS18B20 T3: Abgastemperatur (K) ─────────────────────
  auto* exhSensor = new RepeatSensor<float>(INTERVAL_TEMP_MS, []() -> float {
    return isnan(sd.temp[3]) ? NAN : sd.temp[3] + 273.15f;
  });
  exhSensor->connect_to(new SKOutput<float>(
      "propulsion.0.exhaustTemperature", "/propulsion/exhaustTemp"));

  // ── BME680: Außenluft Temperatur (K) ─────────────────────
  auto* airTempSensor = new RepeatSensor<float>(INTERVAL_BME_MS, []() -> float {
    return isnan(sd.airTemp) ? NAN : sd.airTemp + 273.15f;
  });
  airTempSensor->connect_to(new SKOutput<float>(
      "environment.outside.temperature", "/environment/airTemp"));

  // ── BME680: Luftfeuchtigkeit (0.0–1.0) ───────────────────
  auto* humSensor = new RepeatSensor<float>(INTERVAL_BME_MS, []() -> float {
    return isnan(sd.humidity) ? NAN : sd.humidity / 100.0f;
  });
  humSensor->connect_to(new SKOutput<float>(
      "environment.outside.humidity", "/environment/humidity"));

  // ── BME680: Luftdruck (Pa) ───────────────────────────────
  auto* presSensor = new RepeatSensor<float>(INTERVAL_BME_MS, []() -> float {
    return isnan(sd.pressure) ? NAN : sd.pressure * 100.0f;  // hPa → Pa
  });
  presSensor->connect_to(new SKOutput<float>(
      "environment.outside.pressure", "/environment/pressure"));

  // ── BME680: Gas-Widerstand (Ω) ───────────────────────────
  auto* gasSensor = new RepeatSensor<float>(INTERVAL_BME_MS, []() -> float {
    return isnan(sd.gasRes) ? NAN : sd.gasRes * 1000.0f;  // kΩ → Ω
  });
  gasSensor->connect_to(new SKOutput<float>(
      "environment.outside.gasResistance", "/environment/gasResistance"));

  // ════════════════════════════════════════════════════════
  //  EVENT-LOOP TIMER (periodische Hintergrundaufgaben)
  //  event_loop()->onRepeat(ms, lambda) – SensESP Scheduler
  // ════════════════════════════════════════════════════════

  // ADC lesen (Ruder + Öldruck)
  event_loop()->onRepeat(INTERVAL_ADC_MS,  []() { readADC(); });

  // Temperatursensoren lesen
  event_loop()->onRepeat(INTERVAL_TEMP_MS, []() {
    readTemperatures();
    readBME680();
  });

  // OLED aktualisieren (3-Seiten-Rotation)
  event_loop()->onRepeat(INTERVAL_OLED_MS, []() { updateDisplay(); });

  // Serielle Ausgabe
  event_loop()->onRepeat(INTERVAL_SERIAL_MS, []() { printSerial(); });

  // NMEA2000 Bus-Kommunikation (ParseMessages muss regelmäßig aufgerufen werden)
  event_loop()->onRepeat(INTERVAL_N2K_MS, []() { nmea2000->ParseMessages(); });

  // BUS_OFF Auto-Recovery: SJA1000 setzt MOD.RM=1 automatisch bei BUS_OFF;
  // Software muss MOD.RM=0 schreiben, damit der Controller die Recovery-Sequenz
  // (128 * 11 rezessive Bits) startet und wieder am Bus teilnimmt.
  event_loop()->onRepeat(5000, []() {
    if (MODULE_CAN->SR.B.BS) {
      uint32_t now = millis();
      if (now - canLastRecoveryMs < 5000) return;
      canLastRecoveryMs = now;
      canRecoveries++;
      Serial.printf("CAN BUS_OFF erkannt (TEC=%lu REC=%lu) – Recovery #%lu\n",
                    (unsigned long)MODULE_CAN->TXERR.B.TXERR,
                    (unsigned long)MODULE_CAN->RXERR.B.RXERR,
                    (unsigned long)canRecoveries);
      MODULE_CAN->MOD.B.RM = 1;
      MODULE_CAN->TXERR.U  = 0;
      MODULE_CAN->RXERR.U  = 0;
      (void)MODULE_CAN->ECC;
      (void)MODULE_CAN->IR.U;
      MODULE_CAN->MOD.B.RM = 0;
    }
  });

  // ════════════════════════════════════════════════════════
  //  EIGENE WEB-ROUTES auf SensESP's Port-80-Server
  //    http://<ip>/dash         → Dashboard
  //    http://<ip>/wificonfig   → Konfig-Wrapper (iframe)
  //    http://<ip>/api/data     → JSON
  //    http://<ip>/api/diag     → SPIFFS Info
  //    http://<ip>/api/restart  → Geraet neu starten
  // ════════════════════════════════════════════════════════
  registerCustomRoutes();

  // ── SensESP starten ──────────────────────────────────────
  sensesp_app->start();

  // ── Startmeldung ─────────────────────────────────────────
  Serial.println(F("\n──── Bereit ────────────────────────────────"));
  Serial.printf("  WiFi-AP:    %-20s Pass: %s\n", AP_SSID, AP_PASS);
  Serial.println(F("  Konfig-UI:  http://192.168.4.1"));
  Serial.println(F("  Dashboard:  http://192.168.4.1/dash"));
  Serial.println(F("  JSON-API:   http://192.168.4.1/api/data"));
  Serial.printf("  mDNS:       http://%s.local\n", DEVICE_HOSTNAME);
  Serial.println(F("  Signal K:   über Konfig-UI eintragen (optional)"));
  Serial.println(F("  OTA:        " DEVICE_HOSTNAME ".local  Pass: " OTA_PASSWORD));
  Serial.println(F("──────────────────────────────────────────────\n"));
}

// ════════════════════════════════════════════════════════════
//  LOOP
//  SensESP / ReactESP übernimmt den kompletten Event-Loop.
//  Alle Timing-Logik läuft über event_loop()->onRepeat().
//  Hier nur app.tick() aufrufen – kein eigenes millis() nötig.
// ════════════════════════════════════════════════════════════

void loop() {
  event_loop()->tick();
}
