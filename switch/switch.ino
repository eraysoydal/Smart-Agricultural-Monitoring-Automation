// Full sketch with DS18B20 temp and inverted sondaj sensor boolean
#define BLYNK_TEMPLATE_ID           "TMPL6jYxEQC1m"
#define BLYNK_TEMPLATE_NAME         "Arazi Kontrol"
#define BLYNK_AUTH_TOKEN             //removed for security
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Preferences.h>
#include <OneWire.h>
#include <DallasTemperature.h>

BlynkTimer timer;
Preferences prefs;

// ---------- MULTI-WIFI CREDENTIALS ----------
struct WifiCred { const char* ssid; const char* pass; };
WifiCred wifiList[] = {
  {"TEST_ESP", "esp12345"},
  {"KARABUZA", "OSMAN1450"},
  {"CARDAK",   "OSMAN1450"},
  {"ofis_verici_EXT", "ss3264ss"},
};
const int WIFI_COUNT = sizeof(wifiList) / sizeof(wifiList[0]);
int currentWifiIndex = 0;

// ---------- CONFIG ----------
const int pressurePin    = 36; // ADC1_CH0
// DS18B20 1-Wire pin (choose a free digital pin). Example: GPIO21
const int TEMP_ONE_WIRE_PIN = 32;

const int electricityPin = 39;
const int alarmPin       = 34;
const int sondajPin      = 35;
const int SONDaj_CONTROL_PIN = 26;


#define VP_PRESSURE V1
#define VP_TEMP     V2
#define VP_ELECTRICITY V3
#define VP_ALARM    V4
#define VP_SONDAJ   V5
#define VP_FORCE_REFRESH V10
#define VP_SONDAJ_CONTROL V11
#define VP_PRESSURE_WINDOW V12


const bool SONDaj_RELAY_ACTIVE_LOW = false;

// Sampling
const unsigned long SAMPLE_INTERVAL_MS = 1000UL; // 1 s
const int ADC_SAMPLES = 128;

// Pressure windows & thresholds (defaults)
int pressureAvgWindowSec = 15;
const int PRESSURE_SHORT_WINDOW_SEC = 5;
const float PRESSURE_LARGE_DROP_THRESHOLD = 0.5f;
const float PRESSURE_DELTA = 0.1f;
const unsigned long PRESSURE_COOLDOWN_MS = 10UL * 1000UL;

// Other thresholds
const float TEMP_DELTA     = 1.0f;
const float BOOLEAN_VOLT_THRESHOLD = 0.2f;

// Force-send / other cooldowns
const unsigned long FORCE_SEND_INTERVAL_MS = 6UL * 60UL * 60UL * 1000UL;
const unsigned long TEMP_COOLDOWN_MS        = 60UL * 60UL * 1000UL;
const unsigned long ELECTRICITY_COOLDOWN_MS = 10UL * 1000UL;
const unsigned long ALARM_COOLDOWN_MS       = 10UL * 1000UL;
const unsigned long SONDAJ_COOLDOWN_MS      = 10UL * 1000UL;

// Connection/backoff
const unsigned long CONN_CHECK_INTERVAL_MS = 5000UL;
const unsigned long RECONNECT_INITIAL_BACKOFF_MS = 3000UL;
const unsigned long RECONNECT_MAX_BACKOFF_MS = 60000UL;
const unsigned long WIFI_TRY_TIMEOUT_MS = 8000UL;
const unsigned long RESTART_AFTER_MS = 15UL * 60UL * 1000UL;

// ---------- STATE ----------
float lastPressureSent = NAN;
unsigned long lastPressureSentMs = 0;

float lastTemp = NAN;
bool lastElectricity = false;
bool lastAlarm = false;
bool lastSondaj = false;

unsigned long lastTempSentMs = 0;
unsigned long lastElectricitySentMs = 0;
unsigned long lastAlarmSentMs = 0;
unsigned long lastSondajSentMs = 0;

unsigned long lastForceSendMs = 0;
volatile bool manualForceSendRequest = false;

// sondaj control (desired state commanded via app)
volatile bool sondajControlState = false;

// connection mgmt
unsigned long lastConnCheckMs = 0;
unsigned long lastReconnectAttemptMs = 0;
unsigned long reconnectBackoffMs = RECONNECT_INITIAL_BACKOFF_MS;
unsigned long disconnectedSinceMs = 0;
bool wasConnectedPreviously = true;
unsigned long lastWifiTryMs = 0;

// ---------- PRESSURE ROLLING BUFFER ----------
const int PRESSURE_BUF_MAX = 300;
float pressureBuf[PRESSURE_BUF_MAX];
int pressureBufLen = 0;
int pressureBufPos = 0;

// ---------- 1-Wire / DS18B20 ----------
OneWire oneWire(TEMP_ONE_WIRE_PIN);
DallasTemperature sensors(&oneWire);

// ---------- UTIL ----------
float readAverageVoltage(int pin, int samples = ADC_SAMPLES) {
  long sum = 0;
  for (int i = 0; i < samples; ++i) {
    sum += analogRead(pin);
    delay(1);
  }
  float avgRaw = sum / (float)samples;
  return avgRaw * (3.3f / 4095.0f);
}

// compensate measured voltage by +0.10 V before applying existing formula
float pressureFromVoltage(float v) {
  const float V_OFFSET = 0.11f;
  const float ZERO_THRESHOLD = 0.39f;

  float v_corr = v + V_OFFSET;
  if (v_corr <= ZERO_THRESHOLD) {
    return 0.0f;
  }
  float pressure = 10.0f * v_corr - 3.7f;
  if (pressure < 0.0f) pressure = 0.0f;
  return pressure;
}

// remove analog LM35 conversion - temperature will come from DS18B20
// float tempCFromVoltage(float v) { return (v + 0.140f) * 100.0f; }

bool boolFromVoltage(float v, bool inverted = false) {
  bool val = (v > BOOLEAN_VOLT_THRESHOLD);
  return inverted ? (!val) : val;
}

void trySendFloatParam(float newVal, float &lastVal, unsigned long &lastSentMs,
                       unsigned long cooldownMs, float delta, int virtualPin,
                       const char* label, bool forceSend) {
  unsigned long now = millis();
  bool changed = false;
  if (isnan(lastVal)) changed = true;
  else if (fabs(newVal - lastVal) >= delta) changed = true;

  bool cooldownOk = (now - lastSentMs) >= cooldownMs;
  if (forceSend) cooldownOk = true;

  if ((changed && cooldownOk) || (forceSend && isfinite(newVal))) {
    Blynk.virtualWrite(virtualPin, newVal);
    Serial.print(label); Serial.print(": ");
    if (virtualPin == VP_PRESSURE) Serial.print(newVal, 1);
    else if (virtualPin == VP_TEMP) Serial.print((int)newVal);
    else Serial.print(newVal);
    Serial.println("  -> SENT");
    lastVal = newVal;
    lastSentMs = now;
  }
}

void trySendBoolParam(bool newVal, bool &lastVal, unsigned long &lastSentMs,
                      unsigned long cooldownMs, int virtualPin, const char* label,
                      bool forceSend) {
  unsigned long now = millis();
  bool changed = (newVal != lastVal);
  bool cooldownOk = (now - lastSentMs) >= cooldownMs;
  if (forceSend) cooldownOk = true;

  if ((changed && cooldownOk) || forceSend) {
    Blynk.virtualWrite(virtualPin, newVal ? 1 : 0);
    Serial.print(label); Serial.print(": ");
    Serial.print(newVal ? "1" : "0");
    Serial.println("  -> SENT");
    lastVal = newVal;
    lastSentMs = now;
  }
}

void setSondajOutput(bool on) {
  if (SONDaj_RELAY_ACTIVE_LOW) digitalWrite(SONDaj_CONTROL_PIN, on ? LOW : HIGH);
  else digitalWrite(SONDaj_CONTROL_PIN, on ? HIGH : LOW);
  Serial.print("Sondaj control output set to ");
  Serial.println(on ? "ON" : "OFF");
}

// ---------- MULTI-WIFI: try next credential ----------
bool tryNextWiFi() {
  unsigned long start = millis();
  WiFi.disconnect(true);
  delay(100);

  const char* ssid = wifiList[currentWifiIndex].ssid;
  const char* pass = wifiList[currentWifiIndex].pass;
  Serial.print("Trying WiFi: "); Serial.println(ssid);

  WiFi.begin(ssid, pass);

  while ((WiFi.status() != WL_CONNECTED) && (millis() - start < WIFI_TRY_TIMEOUT_MS)) {
    delay(200);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Connected to WiFi: "); Serial.println(ssid);
    prefs.putInt("wifi_idx", currentWifiIndex);
    Serial.print("Saved wifi_idx="); Serial.println(currentWifiIndex);
    return true;
  } else {
    Serial.print("Failed to connect to: "); Serial.println(ssid);
    currentWifiIndex = (currentWifiIndex + 1) % WIFI_COUNT;
    return false;
  }
}

// ---------- CONNECTION MANAGEMENT ----------
void handleConnections() {
  unsigned long now = millis();
  if (now - lastConnCheckMs < CONN_CHECK_INTERVAL_MS) return;
  lastConnCheckMs = now;

  bool wifiOk = (WiFi.status() == WL_CONNECTED);
  bool blynkOk = Blynk.connected();

  if (wifiOk && blynkOk && !wasConnectedPreviously) {
    Serial.println("Connection restored: WiFi + Blynk connected.");
    reconnectBackoffMs = RECONNECT_INITIAL_BACKOFF_MS;
    disconnectedSinceMs = 0;
    wasConnectedPreviously = true;
    manualForceSendRequest = true;
    return;
  }

  if (!wifiOk || !blynkOk) {
    if (disconnectedSinceMs == 0) disconnectedSinceMs = now;

    if (!wifiOk) {
      if (now - lastWifiTryMs >= WIFI_TRY_TIMEOUT_MS) {
        lastWifiTryMs = now;
        Serial.println("WiFi not connected. Cycling credentials...");
        if (tryNextWiFi()) {
          Serial.println("Attempting Blynk.connect() after WiFi success...");
          Blynk.config(BLYNK_AUTH_TOKEN);
          Blynk.connect();
          reconnectBackoffMs = RECONNECT_INITIAL_BACKOFF_MS;
          Blynk.virtualWrite(VP_PRESSURE_WINDOW, pressureAvgWindowSec);
        }
      } else {
        Serial.println("Waiting before next WiFi credential attempt...");
      }
    }

    if (wifiOk && !blynkOk) {
      if (now - lastReconnectAttemptMs >= reconnectBackoffMs) {
        lastReconnectAttemptMs = now;
        Serial.print("Attempting Blynk.connect(), backoff=");
        Serial.println(reconnectBackoffMs);
        Blynk.connect();
        if (!Blynk.connected()) {
          reconnectBackoffMs = min(reconnectBackoffMs * 2, RECONNECT_MAX_BACKOFF_MS);
          Serial.print("Blynk still disconnected. New backoff=");
          Serial.println(reconnectBackoffMs);
        } else Serial.println("Blynk connected after attempt.");
      } else {
        Serial.println("Waiting before next Blynk reconnect attempt...");
      }
    }

    if ((now - disconnectedSinceMs) >= RESTART_AFTER_MS) {
      Serial.println("Disconnected for too long -> restarting device.");
      delay(200);
      ESP.restart();
    }

    wasConnectedPreviously = false;
    return;
  }

  wasConnectedPreviously = true;
}

// ---------- PRESSURE BUFFER ----------
void pressureBufferPush(float value) {
  if (pressureBufLen < PRESSURE_BUF_MAX) pressureBufLen++;
  pressureBuf[pressureBufPos] = value;
  pressureBufPos = (pressureBufPos + 1) % PRESSURE_BUF_MAX;
}

float pressureAvgLastN(int seconds) {
  if (pressureBufLen == 0) return NAN;
  int n = min(seconds, pressureBufLen);
  float sum = 0.0f;
  for (int i = 0; i < n; ++i) {
    int idx = (pressureBufPos - 1 - i + PRESSURE_BUF_MAX) % PRESSURE_BUF_MAX;
    sum += pressureBuf[idx];
  }
  return sum / (float)n;
}

// ---------- MAIN READ & SEND ----------
void readAndMaybeSendAll() {
  unsigned long now = millis();
  bool periodicForceSendEvent = (now - lastForceSendMs) >= FORCE_SEND_INTERVAL_MS;
  bool forceSendEvent = periodicForceSendEvent || manualForceSendRequest;

  float vPressure = readAverageVoltage(pressurePin);
  // DS18B20 temperature read (digital) - do not use analog tempPin anymore
  sensors.requestTemperatures(); // blocking; acceptable at 1s cadence
  float tempC = sensors.getTempCByIndex(0); // returns -127 if disconnected
  float vElectric = readAverageVoltage(electricityPin);
  float vAlarm    = readAverageVoltage(alarmPin);
  float vSondaj   = readAverageVoltage(sondajPin);

  float pressureVal = pressureFromVoltage(vPressure);

  bool electricity = boolFromVoltage(vElectric, false);
  bool alarm = boolFromVoltage(vAlarm, true);
  // SONDAJ SENSOR polarity inverted per user request
  bool sondaj = boolFromVoltage(vSondaj, true);

  pressureBufferPush(pressureVal);

  float avgShort = pressureAvgLastN(PRESSURE_SHORT_WINDOW_SEC);
  float avgWindow = pressureAvgLastN(pressureAvgWindowSec);

  Serial.print("Raw V - P:"); Serial.print(vPressure, 3);
  Serial.print("  Pval:"); Serial.print(pressureVal, 3);
  Serial.print("  AvgS:"); Serial.print(avgShort, 3);
  Serial.print("  AvgW:"); Serial.print(avgWindow, 3);
  Serial.print("  T:"); 
  if (tempC < -100.0f) Serial.print("ERR");
  else Serial.print(tempC, 2);
  Serial.print("  E:"); Serial.print(electricity ? 1 : 0);
  Serial.print("  A:"); Serial.print(alarm ? 1 : 0);
  Serial.print("  S(sensor):"); Serial.println(sondaj ? 1 : 0);

  bool sentPressure = false;
  if (forceSendEvent && !isnan(avgWindow)) {
    float pressureRounded = roundf(avgWindow * 10.0f) / 10.0f;
    Blynk.virtualWrite(VP_PRESSURE, pressureRounded);
    Serial.print("Pressure (force-send): "); Serial.println(pressureRounded, 1);
    lastPressureSent = pressureRounded;
    lastPressureSentMs = now;
    sentPressure = true;
  } else {
    if (!isnan(avgShort) && !isnan(lastPressureSent)) {
      if ((lastPressureSent - avgShort) >= PRESSURE_LARGE_DROP_THRESHOLD) {
        float pR = roundf(avgShort * 10.0f) / 10.0f;
        Blynk.virtualWrite(VP_PRESSURE, pR);
        Serial.print("Pressure (large short-window drop) -> SENT: "); Serial.println(pR, 1);
        lastPressureSent = pR;
        lastPressureSentMs = now;
        sentPressure = true;
      }
    }

    static float lastWindowAvg = NAN;
    if (!isnan(avgWindow)) {
      if (!isnan(lastWindowAvg)) {
        if ((lastWindowAvg - avgWindow) >= PRESSURE_LARGE_DROP_THRESHOLD) {
          float pR = roundf(avgWindow * 10.0f) / 10.0f;
          Blynk.virtualWrite(VP_PRESSURE, pR);
          Serial.print("Pressure (large window drop) -> SENT: "); Serial.println(pR, 1);
          lastPressureSent = pR;
          lastPressureSentMs = now;
          sentPressure = true;
        }
      }
      lastWindowAvg = avgWindow;
    }

    if (!sentPressure && !isnan(avgWindow)) {
      float pressureRounded = roundf(avgWindow * 10.0f) / 10.0f;
      if (isnan(lastPressureSent) || (fabs(pressureRounded - lastPressureSent) >= PRESSURE_DELTA)) {
        if ((now - lastPressureSentMs) >= PRESSURE_COOLDOWN_MS) {
          Blynk.virtualWrite(VP_PRESSURE, pressureRounded);
          Serial.print("Pressure (regular) -> SENT: "); Serial.println(pressureRounded, 1);
          lastPressureSent = pressureRounded;
          lastPressureSentMs = now;
          sentPressure = true;
        } else {
          Serial.println("Pressure change detected but cooldown active; suppressed.");
        }
      }
    }
  }

  // Temperature: only send if sensor valid
  if (tempC > -100.0f) {
    float tempRounded = roundf(tempC);
    trySendFloatParam(tempRounded, lastTemp, lastTempSentMs, TEMP_COOLDOWN_MS, TEMP_DELTA, VP_TEMP, "Temperature(C)", forceSendEvent);
  } else {
    Serial.println("Temperature sensor error - skipping send");
  }

  trySendBoolParam(electricity, lastElectricity, lastElectricitySentMs, ELECTRICITY_COOLDOWN_MS, VP_ELECTRICITY, "Electricity", forceSendEvent);
  trySendBoolParam(alarm, lastAlarm, lastAlarmSentMs, ALARM_COOLDOWN_MS, VP_ALARM, "Alarm", forceSendEvent);
  trySendBoolParam(sondaj, lastSondaj, lastSondajSentMs, SONDAJ_COOLDOWN_MS, VP_SONDAJ, "Sondaj", forceSendEvent);

  if (forceSendEvent) {
    lastForceSendMs = now;
    manualForceSendRequest = false;
    Serial.println("Force-send event executed (periodic/manual refresh).");
  }
}

// ---------- BLYNK HANDLERS ----------
BLYNK_WRITE(VP_FORCE_REFRESH) {
  int v = param.asInt();
  if (v == 1) {
    manualForceSendRequest = true;
    Serial.println("Manual force-refresh requested (V10).");
  }
}

// Sondaj control (user switch)
BLYNK_WRITE(VP_SONDAJ_CONTROL) {
  int v = param.asInt(); // 0 or 1
  bool newState = (v == 1);

  if (newState != sondajControlState) {
    sondajControlState = newState;
    setSondajOutput(sondajControlState);
    prefs.putBool("sondaj_state", sondajControlState);
    Serial.print("Saved sondaj_state to prefs: ");
    Serial.println(sondajControlState ? "ON" : "OFF");
  } else {
    Serial.println("Sondaj control command received but state unchanged; not writing prefs.");
  }

  manualForceSendRequest = true;
}

BLYNK_WRITE(VP_PRESSURE_WINDOW) {
  int v = param.asInt(); // seconds
  if (v < 5) v = 5;
  if (v > PRESSURE_BUF_MAX) v = PRESSURE_BUF_MAX;
  pressureAvgWindowSec = v;
  prefs.putInt("avg_win", pressureAvgWindowSec);
  Serial.print("Pressure average window set to ");
  Serial.print(pressureAvgWindowSec);
  Serial.println(" seconds (saved to prefs).");
  Blynk.virtualWrite(VP_PRESSURE_WINDOW, pressureAvgWindowSec);
}

BLYNK_CONNECTED() {
  Blynk.virtualWrite(VP_SONDAJ_CONTROL, sondajControlState ? 1 : 0);
  Serial.println("Blynk connected — synced sondajControlState to app UI.");
}

// ---------- SETUP & LOOP ----------
void setup() {
  Serial.begin(115200);
  delay(100);

  prefs.begin("settings", false);

  // Configure ADC attenuation for ADC pins (no temp analog pin anymore)
  analogSetPinAttenuation(pressurePin, ADC_11db);
  analogSetPinAttenuation(electricityPin, ADC_11db);
  analogSetPinAttenuation(alarmPin, ADC_11db);
  analogSetPinAttenuation(sondajPin, ADC_11db);

  // Configure sondaj control pin EARLY so we can safely restore the saved state
  pinMode(SONDaj_CONTROL_PIN, OUTPUT);

  // Initialize DS18B20
  sensors.begin();
  Serial.print("DS18B20 on pin "); Serial.println(TEMP_ONE_WIRE_PIN);

  // Load saved sondaj state and apply (relay pin already configured)
  bool savedSondaj = prefs.getBool("sondaj_state", false);
  sondajControlState = savedSondaj;
  setSondajOutput(sondajControlState);
  Serial.print("Restored sondajControlState from prefs and applied to hardware: ");
  Serial.println(sondajControlState ? "ON" : "OFF");

  // load persisted wifi index if exists
  int savedWifiIdx = prefs.getInt("wifi_idx", -1);
  if (savedWifiIdx >= 0 && savedWifiIdx < WIFI_COUNT) {
    currentWifiIndex = savedWifiIdx;
    Serial.print("Loaded saved wifi_idx: "); Serial.println(currentWifiIndex);
  } else {
    Serial.println("No saved wifi_idx (or out of range). Using default index 0.");
    currentWifiIndex = 0;
  }
  // load persisted avg window if exists
  int savedAvgWin = prefs.getInt("avg_win", -1);
  if (savedAvgWin >= 5 && savedAvgWin <= PRESSURE_BUF_MAX) {
    pressureAvgWindowSec = savedAvgWin;
    Serial.print("Loaded saved pressure avg window: "); Serial.println(pressureAvgWindowSec);
  } else {
    Serial.print("Using default pressure avg window: "); Serial.println(pressureAvgWindowSec);
  }

  // Multi-WiFi attempts at startup
  Serial.println("Starting multi-WiFi connection attempts...");
  unsigned long startSetup = millis();
  unsigned long overallTimeout = 30000UL;
  bool connectedAtSetup = false;
  while (!connectedAtSetup && (millis() - startSetup < overallTimeout)) {
    if (tryNextWiFi()) {
      connectedAtSetup = true;
      break;
    }
    delay(200);
  }

  Blynk.config(BLYNK_AUTH_TOKEN);
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected at setup, attempting Blynk.connect()...");
    Blynk.connect();
  } else {
    Serial.println("No WiFi at setup; Blynk will be attempted after WiFi is available.");
  }

  lastConnCheckMs = millis();
  wasConnectedPreviously = Blynk.connected() && (WiFi.status() == WL_CONNECTED);
  if (!wasConnectedPreviously) disconnectedSinceMs = millis();

  // write pressure window value (app will show it on connect)
  Blynk.virtualWrite(VP_PRESSURE_WINDOW, pressureAvgWindowSec);

  timer.setInterval(SAMPLE_INTERVAL_MS, readAndMaybeSendAll);
}

void loop() {
  Blynk.run();
  timer.run();
  handleConnections();
}
