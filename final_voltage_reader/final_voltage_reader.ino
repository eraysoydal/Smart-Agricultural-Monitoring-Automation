#define BLYNK_TEMPLATE_ID           "TMPL6jYxEQC1m"
#define BLYNK_TEMPLATE_NAME         "Arazi Kontrol"
#define BLYNK_AUTH_TOKEN             //removed for security

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

char ssid[] = "ofis_verici_EXT";
char pass[] = "";

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
BlynkTimer timer;

// ---------- CONFIG ----------
const int pressurePin    = 36; // ADC input for pressure sensor
const int tempPin        = 32; // ADC input for LM35 (change if needed)
const int electricityPin = 39; // ADC input for electricity (boolean)
const int alarmPin       = 34; // ADC input for alarm (inverted boolean)
const int sondajPin      = 35; // ADC input for sondaj (boolean)

const int ADC_SAMPLES = 20;                 // samples to average
const unsigned long SAMPLE_INTERVAL_MS = 2000UL; // how often to sample (ms)

// Virtual pins (change to match your Blynk datastreams)
#define VP_PRESSURE V1
#define VP_TEMP     V2
#define VP_ELECTRICITY V3
#define VP_ALARM    V4
#define VP_SONDAJ   V5
#define VP_FORCE_REFRESH V10  // Virtual pin for the "Force Refresh" button

// Hysteresis / thresholds for sending updates
const float PRESSURE_DELTA = 0.1f;   // bar - minimum change to send (matches 1 decimal)
const float TEMP_DELTA     = 1.0f;   // °C  - minimum change to send (integer temp)
const float BOOLEAN_VOLT_THRESHOLD = 0.2f; // V - consider > this = present

// Force-send interval (ms) to ensure widgets occasionally get refreshed (overrides cooldown)
const unsigned long FORCE_SEND_INTERVAL_MS = 6UL * 60UL * 60UL * 1000UL; // 6 hours

// Per-parameter cooldowns (minimum time between consecutive sends of that parameter)
const unsigned long PRESSURE_COOLDOWN_MS    = 5UL * 1000UL;    // 5s
const unsigned long TEMP_COOLDOWN_MS        = 5UL * 60UL * 1000UL; // 5 minutes
const unsigned long ELECTRICITY_COOLDOWN_MS = 10UL * 1000UL;    // 10s
const unsigned long ALARM_COOLDOWN_MS       = 10UL * 1000UL;    // 10s
const unsigned long SONDAJ_COOLDOWN_MS      = 10UL * 1000UL;    // 10s

// ---------- STATE ----------
float lastPressure = NAN; // stores rounded-to-0.1 value
float lastTemp = NAN;     // stores integer-degree as float
bool lastElectricity = false;
bool lastAlarm = false;
bool lastSondaj = false;

// per-parameter last-sent timestamps
unsigned long lastPressureSentMs = 0;
unsigned long lastTempSentMs = 0;
unsigned long lastElectricitySentMs = 0;
unsigned long lastAlarmSentMs = 0;
unsigned long lastSondajSentMs = 0;

// last global force-send timestamp
unsigned long lastForceSendMs = 0;

// Manual force-send request flag (set by BLYNK_WRITE handler)
volatile bool manualForceSendRequest = false;

// ---------- UTIL ----------
float readAverageVoltage(int pin, int samples = ADC_SAMPLES) {
  long sum = 0;
  for (int i = 0; i < samples; ++i) {
    sum += analogRead(pin);
    delay(2); // tiny pause to allow ADC to settle
  }
  float avgRaw = sum / (float)samples;
  float voltage = avgRaw * (3.3f / 4095.0f);
  return voltage;
}

// Convert pressureVoltage to bar using your existing formula
float pressureFromVoltage(float v) {
  float pressure = 0.0f;
  if (v <= 0.39f) {
    pressure = 0.0f;
  } else {
    pressure = 10.0f * v - 3.7f;
  }
  return pressure;
}

// Convert LM35 voltage (V) to Celsius: °C = V * 100
float tempCFromVoltage(float v) {
  return v * 100.0f;
}

bool boolFromVoltage(float v, bool inverted = false) {
  bool val = (v > BOOLEAN_VOLT_THRESHOLD);
  return inverted ? (!val) : val;
}

// Generic: try to send a float parameter with per-parameter logic
void trySendFloatParam(float newVal, float &lastVal, unsigned long &lastSentMs,
                       unsigned long cooldownMs, float delta, int virtualPin,
                       const char* label, bool forceSend) {
  unsigned long now = millis();
  bool changed = false;
  if (isnan(lastVal)) changed = true;
  else if (fabs(newVal - lastVal) >= delta) changed = true;

  // Decide whether to send:
  bool cooldownOk = (now - lastSentMs) >= cooldownMs;
  if (forceSend) cooldownOk = true; // override cooldown on force-send

  if ((changed && cooldownOk) || (forceSend && isfinite(newVal))) {
    // Send
    Blynk.virtualWrite(virtualPin, newVal);
    Serial.print(label);
    Serial.print(": ");
    // format pressure with 1 decimal point if label contains "Pressure"
    if (virtualPin == VP_PRESSURE) Serial.print(newVal, 1);
    else if (virtualPin == VP_TEMP) Serial.print((int)newVal);
    else Serial.print(newVal);
    Serial.println("  -> SENT");
    lastVal = newVal;
    lastSentMs = now;
  } else {
    Serial.print(label);
    Serial.print(": ");
    if (virtualPin == VP_PRESSURE) Serial.print(newVal, 1);
    else if (virtualPin == VP_TEMP) Serial.print((int)newVal);
    else Serial.print(newVal);
    if (!changed) Serial.println("  (no significant change)");
    else Serial.println("  (cooldown active)");
  }
}

// Generic: try to send a boolean parameter with per-parameter logic
void trySendBoolParam(bool newVal, bool &lastVal, unsigned long &lastSentMs,
                      unsigned long cooldownMs, int virtualPin, const char* label,
                      bool forceSend) {
  unsigned long now = millis();
  bool changed = (newVal != lastVal);

  bool cooldownOk = (now - lastSentMs) >= cooldownMs;
  if (forceSend) cooldownOk = true; // override cooldown on force-send

  if ((changed && cooldownOk) || forceSend) {
    Blynk.virtualWrite(virtualPin, newVal ? 1 : 0);
    Serial.print(label);
    Serial.print(": ");
    Serial.print(newVal ? "TRUE" : "FALSE");
    Serial.println("  -> SENT");
    lastVal = newVal;
    lastSentMs = now;
  } else {
    Serial.print(label);
    Serial.print(": ");
    Serial.print(newVal ? "TRUE" : "FALSE");
    if (!changed) Serial.println("  (no change)");
    else Serial.println("  (cooldown active)");
  }
}

// ---------- MAIN READ & SEND ----------
void readAndMaybeSendAll() {
  unsigned long now = millis();
  bool periodicForceSendEvent = (now - lastForceSendMs) >= FORCE_SEND_INTERVAL_MS;

  // If manual force send was requested by app, honor it (override cooldowns)
  bool forceSendEvent = periodicForceSendEvent || manualForceSendRequest;

  // Read averaged voltages
  float vPressure = readAverageVoltage(pressurePin);
  float vTemp     = readAverageVoltage(tempPin);
  float vElectric = readAverageVoltage(electricityPin);
  float vAlarm    = readAverageVoltage(alarmPin);
  float vSondaj   = readAverageVoltage(sondajPin);

  // Convert
  float pressureBar = pressureFromVoltage(vPressure);
  // round pressure to 1 decimal point (e.g. 1.2)
  float pressureRounded = roundf(pressureBar * 10.0f) / 10.0f;

  float tempC = tempCFromVoltage(vTemp);
  // round temperature to nearest integer (no decimal required)
  float tempRounded = roundf(tempC);

  bool electricity = boolFromVoltage(vElectric, false); // true if voltage present
  bool alarm = boolFromVoltage(vAlarm, true);           // inverted logic: voltage absent => alarm true
  bool sondaj = boolFromVoltage(vSondaj, false);

  // Debug: print raw voltages
  Serial.print("Raw V - P:"); Serial.print(vPressure, 3);
  Serial.print(" T:"); Serial.print(vTemp, 3);
  Serial.print(" E:"); Serial.print(vElectric, 3);
  Serial.print(" A:"); Serial.print(vAlarm, 3);
  Serial.print(" S:"); Serial.println(vSondaj, 3);

  // Send only if changed, respecting per-parameter cooldowns.
  // If forceSendEvent is true, we override cooldowns and send current values.
  trySendFloatParam(pressureRounded, lastPressure, lastPressureSentMs, PRESSURE_COOLDOWN_MS, PRESSURE_DELTA, VP_PRESSURE, "Pressure(bar)", forceSendEvent);
  trySendFloatParam(tempRounded, lastTemp, lastTempSentMs, TEMP_COOLDOWN_MS, TEMP_DELTA, VP_TEMP, "Temperature(C)", forceSendEvent);
  
  trySendBoolParam(electricity, lastElectricity, lastElectricitySentMs, ELECTRICITY_COOLDOWN_MS, VP_ELECTRICITY, "Electricity", forceSendEvent);
  trySendBoolParam(alarm, lastAlarm, lastAlarmSentMs, ALARM_COOLDOWN_MS, VP_ALARM, "Alarm", forceSendEvent);
  trySendBoolParam(sondaj, lastSondaj, lastSondajSentMs, SONDAJ_COOLDOWN_MS, VP_SONDAJ, "Sondaj", forceSendEvent);

  if (forceSendEvent) {
    lastForceSendMs = now; // reset periodic force-send timer
    manualForceSendRequest = false; // clear manual request after it has been handled
    Serial.println("Force-send event executed (periodic/manual refresh).");
  }
}

// ---------- BLYNK HANDLERS ----------
// Manual Force Refresh button in Blynk should be set to virtual pin VP_FORCE_REFRESH (V10).
// Use "Push" mode so it sends 1 when pressed (momentary button).
BLYNK_WRITE(VP_FORCE_REFRESH) {
  int v = param.asInt();
  if (v == 1) {
    // set the flag — actual send happens in the timer callback (non-blocking)
    manualForceSendRequest = true;
    Serial.println("Manual force-refresh requested from app (V10).");
  }
}

// ---------- SETUP & LOOP ----------
void setup() {
  Serial.begin(115200);
  delay(100);

  // Configure ADC attenuation for full-scale ~0..3.3V on ADC pins
  analogSetPinAttenuation(pressurePin, ADC_11db);
  analogSetPinAttenuation(tempPin, ADC_11db);
  analogSetPinAttenuation(electricityPin, ADC_11db);
  analogSetPinAttenuation(alarmPin, ADC_11db);
  analogSetPinAttenuation(sondajPin, ADC_11db);

  // Start Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  // Sample every SAMPLE_INTERVAL_MS and conditionally send to Blynk
  timer.setInterval(SAMPLE_INTERVAL_MS, readAndMaybeSendAll);
}

void loop() {
  Blynk.run();
  timer.run();
}
