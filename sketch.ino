/*
 * ============================================================================
 *  EVAPOTRANSPIRATION CONTROLLER
 *  Automated micro-irrigation based on FAO-56 Penman-Monteith (hourly ET0)
 * ----------------------------------------------------------------------------
 *  Student : Toktorbaev Aitmyrza        Group : EEAIR-25
 *  Subject : Electronics Component and Circuit
 *  Board   : ESP32 DevKit V1 (Wokwi simulation)
 * ----------------------------------------------------------------------------
 *  Sensors / actuators
 *    DHT22            -> GPIO15  air temperature (degC) + relative humidity (%)
 *    LDR module (AO)  -> GPIO34  illuminance (lux) -> solar radiation (W/m2)
 *    Potentiometer 1  -> GPIO35  simulated cup anemometer, 0..15 m/s
 *    Potentiometer 2  -> GPIO32  simulated soil-moisture probe, 0..100 %
 *    Relay module IN  -> GPIO4   pump relay (LED on NO contact = pump)
 *    LCD 16x2 (I2C)   -> GPIO21 (SDA), GPIO22 (SCL), address 0x27
 * ============================================================================
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DHTesp.h"
#include <math.h>

// ----------------------------- Pin map --------------------------------------
const int PIN_DHT   = 15;
const int PIN_LDR   = 34;   // ADC1_CH6 (input only)
const int PIN_WIND  = 35;   // ADC1_CH7 (input only)
const int PIN_SOIL  = 32;   // ADC1_CH4
const int PIN_RELAY = 4;    // relay IN, active HIGH

// ------------------------- Site & model constants ---------------------------
const float ELEVATION_M    = 800.0;   // site elevation (Bishkek ~800 m a.s.l.)
const float ANEMO_HEIGHT_M = 2.0;     // anemometer mounting height
const float ALBEDO         = 0.23;    // FAO-56 reference grass albedo
const float SIGMA_H        = 2.043e-10; // Stefan-Boltzmann, MJ m-2 h-1 K-4
const float LUX_PER_WM2    = 120.0;   // daylight efficacy: ~120 lux = 1 W/m2
const float RSO_CLEAR_WM2  = 900.0;   // clear-sky noon radiation (approx.)
const float DAY_THRESHOLD  = 10.0;    // W/m2 above which it is "daytime"
const float WIND_MAX_MS    = 15.0;    // full-scale of anemometer pot

// --------------------------- Irrigation constants ---------------------------
const float KC             = 1.00;    // crop coefficient (ETc = Kc * ET0)
const float TRIGGER_MM     = 2.0;     // irrigate when water deficit >= 2 mm
const float REFILL_MM      = 3.0;     // dose when soil is too dry
const float SOIL_DRY_PCT   = 30.0;    // below -> emergency irrigation
const float SOIL_WET_PCT   = 70.0;    // above -> never irrigate, reset deficit
const float AREA_M2        = 1.0;     // irrigated area per valve
const float FLOW_LPM       = 0.10;    // drip flow (L/min); 1 L on 1 m2 = 1 mm
const float MAX_PUMP_MIN   = 60.0;    // safety limit per cycle (sim minutes)
const float SOAK_MIN       = 30.0;    // min. pause between cycles (sim minutes)

// Simulation time acceleration: 1 real second = TIME_SCALE simulated seconds.
// 360 -> 1 s = 6 min, so one simulated hour passes in 10 real seconds.
const float TIME_SCALE     = 360.0;

// ------------------------------ Timing --------------------------------------
const unsigned long CONTROL_PERIOD_MS = 1000;
const unsigned long DHT_PERIOD_MS     = 2000;   // DHT22 max rate 0.5 Hz
const unsigned long LCD_PAGE_MS       = 3000;

// ------------------------------ Objects -------------------------------------
LiquidCrystal_I2C lcd(0x27, 16, 2);
DHTesp dht;

// ------------------------------ State ---------------------------------------
struct Sensors {
  float tempC = 25.0, rh = 50.0, lux = 0, irrWm2 = 0, windMs = 0, soilPct = 50;
  bool dhtOk = false;
} s;

struct ETResult {
  float es, ea, vpd, delta, gamma, u2, Rs, Rns, Rnl, Rn, G, et0;
  bool day;
} et;

float lastRsRatio   = 0.5;   // Rs/Rso kept from daytime for night Rnl
float deficitMm     = 0.0;   // root-zone water deficit
float etcTodayMm    = 0.0;   // accumulated crop ET (simulated)
float waterUsedL    = 0.0;
float pumpRunMin    = 0.0;   // current cycle runtime (sim minutes)
float pumpOffMin    = 999;   // time since last pump stop (sim minutes)
float simHours      = 0.0;
bool  pumpOn        = false;
const char* reason  = "IDLE";

unsigned long tControl = 0, tDht = 0, tLcd = 0;
int lcdPage = 0;

// ============================================================================
//  Sensor conversions
// ============================================================================
float readLux() {
  // Wokwi LDR module: LDR in series with a 10 kOhm resistor, AO = divider
  // node. LDR model: R = RL10 * (10 / lux)^gamma  (RL10 = 50 k, gamma = 0.7)
  const float GAMMA = 0.7, RL10 = 50.0, VCC = 3.3;
  int raw = analogRead(PIN_LDR);
  float v = raw / 4095.0 * VCC;
  if (v >= VCC - 0.005) return 0.0;        // total darkness
  if (v <= 0.001) return 100000.0;         // saturated (direct sun)
  float r = 10000.0 * v / (VCC - v);       // LDR resistance (ohm)
  float lux = powf(RL10 * 1e3 * powf(10, GAMMA) / r, 1.0 / GAMMA);
  return constrain(lux, 0.0, 100000.0);
}

float readPotPercent(int pin) {
  return analogRead(pin) / 4095.0 * 100.0;
}

void readSensors(bool readDht) {
  if (readDht) {
    TempAndHumidity d = dht.getTempAndHumidity();
    if (dht.getStatus() == 0 && !isnan(d.temperature) && !isnan(d.humidity)) {
      s.tempC = d.temperature;
      s.rh    = constrain(d.humidity, 1.0, 100.0);
      s.dhtOk = true;
    } else {
      s.dhtOk = false;
    }
  }
  s.lux     = readLux();
  s.irrWm2  = s.lux / LUX_PER_WM2;
  s.windMs  = readPotPercent(PIN_WIND) / 100.0 * WIND_MAX_MS;
  s.soilPct = readPotPercent(PIN_SOIL);
}

// ============================================================================
//  FAO-56 Penman-Monteith, hourly time step (Allen et al., 1998, Eq. 53)
//
//          0.408*D*(Rn - G) + g*(37/(T+273))*u2*(es - ea)
//  ET0 = ---------------------------------------------------   [mm/h]
//                     D + g*(1 + Cd*u2)
// ============================================================================
ETResult computeET0(float T, float RH, float uz, float irrWm2) {
  ETResult r;
  // Wind speed at 2 m (FAO-56 Eq. 47)
  r.u2 = uz * 4.87 / logf(67.8 * ANEMO_HEIGHT_M - 5.42);
  if (r.u2 < 0.5) r.u2 = 0.5;           // FAO: calm-air lower limit

  // Vapour pressures (kPa) (Eq. 11, 19 for hourly)
  r.es  = 0.6108 * expf(17.27 * T / (T + 237.3));
  r.ea  = r.es * RH / 100.0;
  r.vpd = fmaxf(r.es - r.ea, 0.0);

  // Slope of saturation vapour-pressure curve (Eq. 13)
  r.delta = 4098.0 * r.es / ((T + 237.3) * (T + 237.3));

  // Atmospheric pressure and psychrometric constant (Eq. 7, 8)
  float P = 101.3 * powf((293.0 - 0.0065 * ELEVATION_M) / 293.0, 5.26);
  r.gamma = 0.000665 * P;

  // Radiation (MJ m-2 h-1): 1 W/m2 = 0.0036 MJ m-2 h-1
  r.Rs  = irrWm2 * 0.0036;
  r.day = irrWm2 > DAY_THRESHOLD;
  r.Rns = (1.0 - ALBEDO) * r.Rs;                         // Eq. 38

  float ratio;
  if (r.day) {
    ratio = constrain(irrWm2 / RSO_CLEAR_WM2, 0.25, 1.0);
    lastRsRatio = ratio;
  } else {
    ratio = lastRsRatio;                                  // FAO night rule
  }
  float TK = T + 273.16;
  r.Rnl = SIGMA_H * TK * TK * TK * TK
          * (0.34 - 0.14 * sqrtf(r.ea))
          * (1.35 * ratio - 0.35);                         // Eq. 39
  r.Rn  = r.Rns - r.Rnl;                                   // Eq. 40

  // Soil heat flux and Cd (Eq. 45, 46; FAO-56 Table for hourly step)
  float Cd;
  if (r.day) { r.G = 0.1 * r.Rn; Cd = 0.24; }
  else       { r.G = 0.5 * r.Rn; Cd = 0.96; }

  float num = 0.408 * r.delta * (r.Rn - r.G)
            + r.gamma * (37.0 / (T + 273.0)) * r.u2 * r.vpd;
  float den = r.delta + r.gamma * (1.0 + Cd * r.u2);
  r.et0 = fmaxf(num / den, 0.0);
  return r;
}

// ============================================================================
//  Irrigation control: soil water balance + soil-moisture safety limits
// ============================================================================
void setPump(bool on, const char* why) {
  if (on && !pumpOn) pumpRunMin = 0;
  if (!on && pumpOn) pumpOffMin = 0;
  pumpOn = on;
  reason = why;
  digitalWrite(PIN_RELAY, on ? HIGH : LOW);
}

void controlStep(float dtRealS) {
  float dtSimH = dtRealS * TIME_SCALE / 3600.0;   // simulated hours
  simHours += dtSimH;

  // Fail-safe: no valid climate data -> no irrigation
  if (!s.dhtOk) { setPump(false, "ERR"); return; }

  et = computeET0(s.tempC, s.rh, s.windMs, s.irrWm2);
  float etc = KC * et.et0;                        // mm/h
  deficitMm  += etc * dtSimH;
  etcTodayMm += etc * dtSimH;

  if (pumpOn) {
    float applied = (FLOW_LPM * 60.0 / AREA_M2) * dtSimH;  // mm
    deficitMm  -= applied;
    waterUsedL += applied * AREA_M2;
    pumpRunMin += dtSimH * 60.0;
  } else {
    pumpOffMin += dtSimH * 60.0;
  }

  if (s.soilPct >= SOIL_WET_PCT) {                // field capacity / rain
    deficitMm = 0;
    setPump(false, "WET");
    return;
  }
  if (!pumpOn) {
    if (pumpOffMin < SOAK_MIN) return;          // let water soak in
    if (s.soilPct < SOIL_DRY_PCT) {
      deficitMm = fmaxf(deficitMm, REFILL_MM);
      setPump(true, "DRY");
    } else if (deficitMm >= TRIGGER_MM) {
      setPump(true, "ET");
    }
  } else if (deficitMm <= 0 || pumpRunMin >= MAX_PUMP_MIN) {
    deficitMm = fmaxf(deficitMm, 0);
    setPump(false, pumpRunMin >= MAX_PUMP_MIN ? "MAX" : "DONE");
  }
}

// ============================================================================
//  Display & logging
// ============================================================================
void lcdLine(int row, const char* text) {
  char buf[17];
  snprintf(buf, sizeof(buf), "%-16s", text);
  lcd.setCursor(0, row);
  lcd.print(buf);
}

void updateLcd() {
  char l0[24], l1[24];
  if (!s.dhtOk) {
    lcdLine(0, "DHT22 ERROR!");
    lcdLine(1, "Pump locked OFF");
    return;
  }
  switch (lcdPage) {
    case 0:
      snprintf(l0, sizeof(l0), "T%5.1fC RH%3.0f%%", s.tempC, s.rh);
      snprintf(l1, sizeof(l1), "ET0 %5.3f mm/h", et.et0);
      break;
    case 1:
      snprintf(l0, sizeof(l0), "Wind %4.1f m/s", s.windMs);
      snprintf(l1, sizeof(l1), "Sun%4.0fW/m2 %s", s.irrWm2, et.day ? "DAY" : "NGT");
      break;
    case 2:
      snprintf(l0, sizeof(l0), "Soil%3.0f%% D%4.1fmm", s.soilPct, deficitMm);
      snprintf(l1, sizeof(l1), "Pump:%s %s", pumpOn ? "ON " : "OFF", reason);
      break;
    default:
      snprintf(l0, sizeof(l0), "ETc sum %5.1fmm", etcTodayMm);
      snprintf(l1, sizeof(l1), "Water %6.1f L", waterUsedL);
      break;
  }
  lcdLine(0, l0);
  lcdLine(1, l1);
}

void logSerial() {
  Serial.printf("t=%6.2fh | T=%5.1fC RH=%5.1f%% u=%4.1fm/s Rs=%6.1fW/m2 Soil=%5.1f%% | "
                "es=%.3f ea=%.3f D=%.4f g=%.4f Rn=%.3f G=%.3f | ET0=%.3fmm/h "
                "Def=%.2fmm Pump=%s(%s) Water=%.1fL\n",
                simHours, s.tempC, s.rh, s.windMs, s.irrWm2, s.soilPct,
                et.es, et.ea, et.delta, et.gamma, et.Rn, et.G, et.et0,
                deficitMm, pumpOn ? "ON" : "OFF", reason, waterUsedL);
}

// ============================================================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, LOW);
  analogReadResolution(12);

  dht.setup(PIN_DHT, DHTesp::DHT22);
  lcd.init();
  lcd.backlight();
  lcdLine(0, "ET0 Controller");
  lcdLine(1, "Penman-Monteith");

  Serial.println(F("=== Evapotranspiration Controller (FAO-56 PM) ==="));
  Serial.println(F("Student: Toktorbaev Aitmyrza, EEAIR-25"));
  Serial.printf("Elevation %.0f m, Kc %.2f, trigger %.1f mm, time x%.0f\n",
                ELEVATION_M, KC, TRIGGER_MM, TIME_SCALE);
  delay(2000);
  readSensors(true);
  tControl = tDht = tLcd = millis();
}

void loop() {
  unsigned long now = millis();

  if (now - tControl >= CONTROL_PERIOD_MS) {
    float dt = (now - tControl) / 1000.0;
    tControl = now;
    bool doDht = (now - tDht >= DHT_PERIOD_MS);
    if (doDht) tDht = now;
    readSensors(doDht);
    controlStep(dt);
    logSerial();
    updateLcd();
  }

  if (now - tLcd >= LCD_PAGE_MS) {
    tLcd = now;
    lcdPage = (lcdPage + 1) % 4;
    updateLcd();
  }
}