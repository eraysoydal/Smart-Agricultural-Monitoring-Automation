// Full sketch — updated: noise-floor, killswitch (V14), unified send routine for pressure
#define BLYNK_TEMPLATE_ID           "TMPL6jYxEQC1m"
#define BLYNK_TEMPLATE_NAME         "Arazi Kontrol"
#define BLYNK_AUTH_TOKEN            //removed for security

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
  {"YEDEK_WIFI", "sifre123"},
};
const int WIFI_COUNT = sizeof(wifiList) / sizeof(wifiList[0]);
int currentWifiIndex = 0;

// ---------- CONFIG ----------
const int pressurePin    = 36; // ADC1_CH0
const int TEMP_ONE_WIRE_PIN = 32; // DS18B20 1-Wire pin
const int electricityPin = 39;
const int sondajPin      = 34;
const int SONDaj_CONTROL_PIN = 33;

#define VP_PRESSURE V1
#define VP_TEMP     V2
#define VP_ELECTRICITY V3
#define VP_SONDAJ   V5
#define VP_FORCE_REFRESH V10
#define VP_SONDAJ_CONTROL V11
#define VP_NOTIFY_KILL V14 // NEW killswitch virtual pin (0 = notifications OFF)

// relay polarity
const bool SONDaj_RELAY_ACTIVE_LOW = false;

// Sampling
const unsigned long SAMPLE_INTERVAL_MS = 1000UL; // 1 s
const int ADC_SAMPLES = 128;

// Pressure logic (new)
const int PRESSURE_COMPARE_SECONDS = 5;        // compare last 5s vs previous 5s
float pressurePercentThreshold = 0.10f;        // default 10% (runtime adjustable previously in other flows)
const float PRESSURE_ZERO_TO_NONZERO_MIN = 0.20f; // if prev ~=0 and now >= 0.2 bar -> send
const float PRESSURE_DELTA = 0.2f;             // rounding / regular delta (1 decimal)
const unsigned long PRESSURE_COOLDOWN_MS = 60UL * 1000UL; // cooldown for regular sends

// Confirmation (new)
const int PRESSURE_CONFIRM_COUNT = 3; // require 3 consecutive samples meeting percent rule

// Noise-floor: treat tiny pressure readings as zero to avoid chatter
const float PRESSURE_NOISE_FLOOR = 0.3f; // bar

// Other thresholds
const float TEMP_DELTA     = 1.0f;
const float BOOLEAN_VOLT_THRESHOLD = 0.2f;

// Force-send / other cooldowns
const unsigned long FORCE_SEND_INTERVAL_MS = 6UL * 60UL * 60UL * 1000UL;
const unsigned long TEMP_COOLDOWN_MS        = 30UL * 60UL * 1000UL; // 30 Minutes
const unsigned long ELECTRICITY_COOLDOWN_MS = 10UL * 1000UL;
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
bool lastSondaj = false;

unsigned long lastTempSentMs = 0;
unsigned long lastElectricitySentMs = 0;
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

// confirmation runtime counter (new)
int pressureRelConsec = 0;

// ---------- 1-Wire / DS18B20 ----------
OneWire oneWire(TEMP_ONE_WIRE_PIN);
DallasTemperature sensors(&oneWire);

// ---------- KILLSWITCH ----------
bool notificationsEnabled = true; // persisted in prefs, false = block automatic notifications

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

// compensate measured voltage by +0.055 V before applying existing formula (your chosen offset)
float pressureFromVoltage(float v) {
  const float V_OFFSET = 0.055f;
  const float ZERO_THRESHOLD = 0.39f;

  float v_corr = v + V_OFFSET;
  if (v_corr <= ZERO_THRESHOLD) {
    return 0.0f;
  }
  float pressure = 10.0f * v_corr - 3.7f;
  if (pressure < 0.0f) pressure = 0.0f;
  return pressure;
}

bool boolFromVoltage(float v, bool inverted = false) {
  bool val = (v > BOOLEAN_VOLT_THRESHOLD);
  return inverted ? (!val) : val;
}

// TRY-SEND helpers now return bool (true if sent). They respect killswitch:
// - if notificationsEnabled==false and forceSend==false => skip send and return false
bool trySendFloatParam(float newVal, float &lastVal, unsigned long &lastSentMs,
                       unsigned long cooldownMs, float delta, int virtualPin,
                       const char* label, bool forceSend) {
  unsigned long now = millis();

  // enforce killswitch: block automatic sends but allow manual/force sends
  if (!notificationsEnabled && !forceSend) {
    Serial.print(label); Serial.println(": notifications disabled -> SKIP send");
    return false;
  }

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
    return true;
  } else {
    Serial.print(label); Serial.print(": ");
    if (virtualPin == VP_PRESSURE) Serial.print(newVal, 1);
    else if (virtualPin == VP_TEMP) Serial.print((int)newVal);
    else Serial.print(newVal);
    if (!changed) Serial.println("  (no significant change)");
    else Serial.println("  (cooldown active)");
    return false;
  }
}

bool trySendBoolParam(bool newVal, bool &lastVal, unsigned long &lastSentMs,
                      unsigned long cooldownMs, int virtualPin, const char* label,
                      bool forceSend) {
  unsigned long now = millis();

  // killswitch
  if (!notificationsEnabled && !forceSend) {
    Serial.print(label); Serial.println(": notifications disabled -> SKIP send");
    return false;
  }

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
    return true;
  } else {
    Serial.print(label); Serial.print(": ");
    Serial.print(newVal ? "1" : "0");
    if (!changed) Serial.println("  (no change)");
    else Serial.println("  (cooldown active)");
    return false;
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

// average of last N values (N seconds)
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

// average of a range: offsetStart=0 means last sample; offsetStart=5 means skip last 5 samples
float pressureAvgRange(int offsetStart, int length) {
  if (pressureBufLen == 0) return NAN;
  if (length <= 0) return NAN;
  if (offsetStart < 0) offsetStart = 0;
  if (offsetStart + 1 > pressureBufLen) return NAN;
  int availableAfterOffset = pressureBufLen - offsetStart;
  int n = min(length, availableAfterOffset);
  if (n <= 0) return NAN;
  float sum = 0.0f;
  for (int i = 0; i < n; ++i) {
    int idx = (pressureBufPos - 1 - offsetStart - i + PRESSURE_BUF_MAX) % PRESSURE_BUF_MAX;
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
  // DS18B20 temperature read (digital)
  sensors.requestTemperatures(); // blocking; acceptable at 1s cadence
  float tempC = sensors.getTempCByIndex(0); // returns -127 if disconnected
  float vElectric = readAverageVoltage(electricityPin);
  float vSondaj   = readAverageVoltage(sondajPin);

  float pressureVal = pressureFromVoltage(vPressure);

  // Apply noise-floor: small values considered zero to avoid chatter
  if (pressureVal > 0.0f && pressureVal < PRESSURE_NOISE_FLOOR) {
    pressureVal = 0.0f;
  }

  bool electricity = boolFromVoltage(vElectric, true);
  bool sondaj = boolFromVoltage(vSondaj, true);

  pressureBufferPush(pressureVal);

  // compute last-5s and previous-5s averages
  float avgNow = pressureAvgRange(0, PRESSURE_COMPARE_SECONDS);     // last 0..4
  float avgPrev = pressureAvgRange(PRESSURE_COMPARE_SECONDS, PRESSURE_COMPARE_SECONDS); // 5..9

  Serial.print("Raw V - P:"); Serial.print(vPressure, 3);
  Serial.print("  Pval:"); Serial.print(pressureVal, 3);
  Serial.print("  AvgNow:"); Serial.print(avgNow, 3);
  Serial.print("  AvgPrev:"); Serial.print(avgPrev, 3);
  Serial.print("  T:");
  if (tempC < -100.0f) Serial.print("ERR");
  else Serial.print(tempC, 2);
  Serial.print("  E:"); Serial.print(electricity ? 1 : 0);
  Serial.print("  S(sensor):"); Serial.println(sondaj ? 1 : 0);

  bool sentPressure = false;

  // Force-send: always push the current avgNow if available (forceSend bypasses killswitch)
  if (forceSendEvent && !isnan(avgNow)) {
    float pressureRounded = roundf(avgNow * 10.0f) / 10.0f;
    trySendFloatParam(pressureRounded, lastPressureSent, lastPressureSentMs, PRESSURE_COOLDOWN_MS, PRESSURE_DELTA, VP_PRESSURE, "Pressure(bar)", true);
    lastPressureSent = pressureRounded;
    lastPressureSentMs = now;
    sentPressure = true;
    // reset confirmation counter after manual/periodic force send
    pressureRelConsec = 0;
  } else {
    // 1) If we have both windows (avgPrev valid), compare percent difference
    bool immediateSend = false;
    if (!isnan(avgNow) && !isnan(avgPrev)) {
      // If both are near zero (below noise-floor) treat as no-change
      if (fabs(avgNow) < PRESSURE_NOISE_FLOOR && fabs(avgPrev) < PRESSURE_NOISE_FLOOR) {
        // treat as stable zero -> do nothing
        pressureRelConsec = 0;
      } else {
        float denom = (fabs(avgPrev) < 0.0001f) ? 0.0001f : fabs(avgPrev);
        float rel = fabs(avgNow - avgPrev) / denom;
        if (rel >= pressurePercentThreshold) {
          // increment confirmation counter; require PRESSURE_CONFIRM_COUNT consecutive samples
          pressureRelConsec++;
          Serial.print("Pressure: relative change "); Serial.print(rel * 100.0f);
          Serial.print("% >= threshold "); Serial.print(pressurePercentThreshold * 100.0f);
          Serial.print("%  (consec=");
          Serial.print(pressureRelConsec);
          Serial.print("/");
          Serial.print(PRESSURE_CONFIRM_COUNT);
          Serial.println(")");
          if (pressureRelConsec >= PRESSURE_CONFIRM_COUNT) {
            immediateSend = true;
            pressureRelConsec = 0; // reset counter after confirmed send
          }
        } else {
          // reset counter when rule not met
          if (pressureRelConsec != 0) {
            Serial.println("Pressure: percent rule failed this sample -> reset confirmation counter");
          }
          pressureRelConsec = 0;
        }
      }
    } else {
      // If avgPrev not available but we have avgNow and no lastPressureSent, send initial value
      if (!isnan(avgNow) && isnan(lastPressureSent)) {
        immediateSend = true;
        Serial.println("Pressure: initial send (no lastPressureSent).");
      }
    }

    // Handle zero->nonzero special case when avgPrev is near zero
    if (!isnan(avgNow) && !isnan(avgPrev) && fabs(avgPrev) < 0.01f && avgNow >= PRESSURE_ZERO_TO_NONZERO_MIN) {
      immediateSend = true;
      Serial.println("Pressure: prev≈0 and now >= threshold -> immediate send");
      // also reset confirmation counter
      pressureRelConsec = 0;
    }

    if (immediateSend) {
      float pressureRounded = roundf(avgNow * 10.0f) / 10.0f;
      // immediate sends should still respect killswitch; use forceSend = false? We treat "immediateSend" as automatic
      // but it's an important automatic alert — we still must respect notificationsEnabled.
      // We call trySendFloatParam with forceSend = false (will be blocked if killswitch OFF).
      if (trySendFloatParam(pressureRounded, lastPressureSent, lastPressureSentMs, PRESSURE_COOLDOWN_MS, PRESSURE_DELTA, VP_PRESSURE, "Pressure(bar)", false)) {
        lastPressureSent = pressureRounded;
        lastPressureSentMs = now;
        sentPressure = true;
      } else {
        Serial.println("Pressure immediate send suppressed (killswitch/cooldown).");
      }
    } else {
      // Regular send logic (delta + cooldown)
      if (!isnan(avgNow)) {
        float pressureRounded = roundf(avgNow * 10.0f) / 10.0f;
        // Use trySendFloatParam to respect killswitch and cooldown
        if (trySendFloatParam(pressureRounded, lastPressureSent, lastPressureSentMs, PRESSURE_COOLDOWN_MS, PRESSURE_DELTA, VP_PRESSURE, "Pressure(bar)", false)) {
          sentPressure = true;
        } else {
          // either no meaningful change, cooldown, or killswitch suppressed
        }
      }
    }
  }

  // Temperature: only send if sensor valid
  if (tempC > -100.0f) {
    float tempRounded = roundf(tempC);
    trySendFloatParam(tempRounded, lastTemp, lastTempSentMs, TEMP_COOLDOWN_MS, TEMP_DELTA, VP_TEMP, "Temperature(C)", forceSendEvent);
  } else {
    // Serial.println("Temperature sensor error - skipping send");
  }

  trySendBoolParam(electricity, lastElectricity, lastElectricitySentMs, ELECTRICITY_COOLDOWN_MS, VP_ELECTRICITY, "Electricity", forceSendEvent);
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

// NEW: notification killswitch (V14). 1 = ON (notifications allowed), 0 = OFF (block automatic notifications)
BLYNK_WRITE(VP_NOTIFY_KILL) {
  int v = param.asInt();
  bool newState = (v != 0);
  notificationsEnabled = newState;
  prefs.putBool("notify_on", notificationsEnabled);
  Serial.print("Notifications enabled state updated: ");
  Serial.println(notificationsEnabled ? "ON" : "OFF");
  // On change, optionally trigger a force refresh to update app widgets (we allow this)
  manualForceSendRequest = true;
}

BLYNK_CONNECTED() {
  // sync sondaj control state
  Blynk.virtualWrite(VP_SONDAJ_CONTROL, sondajControlState ? 1 : 0);
  // sync killswitch state to app
  Blynk.virtualWrite(VP_NOTIFY_KILL, notificationsEnabled ? 1 : 0);
  Serial.println("Blynk connected — synced sondajControlState and killswitch to app UI.");
}

// ---------- SETUP & LOOP ----------
void setup() {
  Serial.begin(115200);
  delay(100);

  prefs.begin("settings", false);

  // load killswitch state (default true)
  notificationsEnabled = prefs.getBool("notify_on", true);
  Serial.print("Notifications enabled (loaded from prefs): ");
  Serial.println(notificationsEnabled ? "ON" : "OFF");

  // Configure ADC attenuation for ADC pins
  analogSetPinAttenuation(pressurePin, ADC_11db);
  analogSetPinAttenuation(electricityPin, ADC_11db);
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

  timer.setInterval(SAMPLE_INTERVAL_MS, readAndMaybeSendAll);
}

void loop() {
  Blynk.run();
  timer.run();
  handleConnections();
}
