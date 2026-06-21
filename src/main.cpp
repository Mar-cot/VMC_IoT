#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Relay.h>
#include <MySensor.h>
#include <time.h>
#include <string.h>
#include "config.h"

// --- NTP ---
static const char* NTP_SERVER          = "pool.ntp.org";
static const long  GMT_OFFSET_SEC      = 3600;
static const int   DAYLIGHT_OFFSET_SEC = 3600;

// =============================================================
//  VMC IoT — Deep Sleep + MQTT + Dual-Core Architecture
//
//  Flow:  Wake → Sense (Core 1) + Connect (Core 0) → Publish
//         → Listen for commands → Act → Sleep
//
//  Core 1 (APP_CPU): setup/loop — sensor reads, VMC logic, actuator control
//  Core 0 (PRO_CPU): WiFi connection + MQTT publish + command listen
// =============================================================

// --- Actuator index mapping ---
enum ActuatorIdx { ACT_VALVE_EXT = 0, ACT_VALVE_DEUM = 1, ACT_FAN = 2, ACT_DEHUMIDIFIER = 3, ACT_COUNT = 4 };

// --- RTC Memory: persists across deep sleep ---
// First-boot defaults: ext valve open, deum valve closed, fan off, dehumidifier off
RTC_DATA_ATTR bool     rtc_valve_ext_open   = true;
RTC_DATA_ATTR bool     rtc_valve_deum_open  = false;
RTC_DATA_ATTR bool     rtc_fan_on           = false;
RTC_DATA_ATTR bool     rtc_dehumidifier_on  = false;
RTC_DATA_ATTR uint32_t rtc_boot_count       = 0;

// RTC-persisted thresholds (defaults from config.h, updatable via MQTT)
RTC_DATA_ATTR float    rtc_soglia_umidita   = SOGLIA_UMIDITA;
RTC_DATA_ATTR float    rtc_temp_min_ext     = TEMP_MIN_ESTERNO;
RTC_DATA_ATTR float    rtc_temp_max_ext     = TEMP_MAX_ESTERNO;

// RTC-persisted actuator locks
// lock_remaining_s > 0 means the actuator is locked in forced_state
RTC_DATA_ATTR int32_t  rtc_lock_remaining_s[ACT_COUNT] = {0, 0, 0, 0};
RTC_DATA_ATTR bool     rtc_forced_state[ACT_COUNT]     = {false, false, false, false};

// --- Sensor readings (shared between cores) ---
volatile float g_temp_ext  = CANC_NUM;
volatile float g_humid_ext = CANC_NUM;
volatile float g_temp_int  = CANC_NUM;
volatile float g_humid_int = CANC_NUM;

// --- Actuator desired state (set by Core 1, read by Core 0 for publishing) ---
volatile bool g_valve_ext_open  = false;
volatile bool g_valve_deum_open = false;
volatile bool g_fan_on          = false;
volatile bool g_dehumidifier_on = false;

// --- Synchronisation ---
SemaphoreHandle_t sem_sensors_done   = NULL;  // Core 1 signals sensor data ready
SemaphoreHandle_t sem_wifi_ready     = NULL;  // Core 0 signals WiFi+MQTT connected
SemaphoreHandle_t sem_actuation_done = NULL;  // Core 1 signals actuation complete
SemaphoreHandle_t sem_publish_done   = NULL;  // Core 0 signals MQTT publish complete

// --- MQTT ---
WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);

// --- Hardware objects (created in setup on Core 1) ---
Sensor*       sensoreEsterno       = NULL;
Sensor*       sensoreInterno       = NULL;
RelayMotore*  rValvEsterno         = NULL;
RelayMotore*  rValvDeumidificatore = NULL;
SingleRelay*  rVentolaDeum         = NULL;
SingleRelay*  rDeumidificatore     = NULL;

// =============================================================
//  Simple JSON helpers (no ArduinoJson dependency)
// =============================================================

// Extract a quoted string value for "key":"value"
bool jsonGetString(const char* json, const char* key, char* out, size_t outSize) {
    char search[64];
    snprintf(search, sizeof(search), "\"%s\":\"", key);
    const char* start = strstr(json, search);
    if (!start) return false;
    start += strlen(search);
    const char* end = strchr(start, '"');
    if (!end) return false;
    size_t len = end - start;
    if (len >= outSize) len = outSize - 1;
    memcpy(out, start, len);
    out[len] = '\0';
    return true;
}

// Extract a numeric float value for "key":123.45
bool jsonGetFloat(const char* json, const char* key, float* out) {
    char search[64];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char* start = strstr(json, search);
    if (!start) return false;
    start += strlen(search);
    // Skip whitespace
    while (*start == ' ') start++;
    if (*start == '"' || *start == '{' || *start == '[') return false;  // Not a number
    *out = atof(start);
    return true;
}

// Extract an integer value for "key":123
bool jsonGetInt(const char* json, const char* key, int* out) {
    char search[64];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char* start = strstr(json, search);
    if (!start) return false;
    start += strlen(search);
    while (*start == ' ') start++;
    if (*start == '"' || *start == '{' || *start == '[') return false;
    *out = atoi(start);
    return true;
}

// Map actuator name string to index, returns -1 if unknown
int actuatorNameToIdx(const char* name) {
    if (strcmp(name, "valve_ext")      == 0) return ACT_VALVE_EXT;
    if (strcmp(name, "valve_deum")     == 0) return ACT_VALVE_DEUM;
    if (strcmp(name, "fan")            == 0) return ACT_FAN;
    if (strcmp(name, "dehumidifier")   == 0) return ACT_DEHUMIDIFIER;
    return -1;
}

// Apply a forced state to an actuator by index (bypasses lock, uses direct methods)
void forceActuatorState(int idx, bool state) {
    switch (idx) {
        case ACT_VALVE_EXT:
            rValvEsterno->is_open = !state;  // Set opposite so open/close will act
            if (state) rValvEsterno->openValve(); else rValvEsterno->closeValve();
            rtc_valve_ext_open = state;
            g_valve_ext_open = state;
            break;
        case ACT_VALVE_DEUM:
            rValvDeumidificatore->is_open = !state;
            if (state) rValvDeumidificatore->openValve(); else rValvDeumidificatore->closeValve();
            rtc_valve_deum_open = state;
            g_valve_deum_open = state;
            break;
        case ACT_FAN:
            rVentolaDeum->is_on = !state;
            if (state) rVentolaDeum->turnOn(); else rVentolaDeum->turnOff();
            rtc_fan_on = state;
            g_fan_on = state;
            break;
        case ACT_DEHUMIDIFIER:
            rDeumidificatore->is_on = !state;
            if (state) rDeumidificatore->turnOn(); else rDeumidificatore->turnOff();
            rtc_dehumidifier_on = state;
            g_dehumidifier_on = state;
            break;
    }
}

// ======================= CORE 0 TASK ==========================
// Runs WiFi connection + MQTT publish, then signals done.
// ==============================================================

bool connectWiFi() {
    Serial.print("[Core0] Connecting WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - start > WIFI_TIMEOUT_MS) {
            Serial.println(" TIMEOUT");
            return false;
        }
        delay(100);
    }
    Serial.printf(" OK (%s)\n", WiFi.localIP().toString().c_str());
    return true;
}

bool connectMQTT() {
    mqttClient.setBufferSize(1024);
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    Serial.print("[Core0] Connecting MQTT...");

    for (int attempt = 0; attempt < 3; attempt++) {
        bool connected;
        if (strlen(MQTT_USER) > 0) {
            connected = mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD);
        } else {
            connected = mqttClient.connect(MQTT_CLIENT_ID);
        }
        if (connected) {
            Serial.println(" OK");
            return true;
        }
        Serial.printf(" attempt %d failed (rc=%d)\n", attempt + 1, mqttClient.state());
        delay(500);
    }
    Serial.println(" FAILED after 3 attempts");
    return false;
}

void syncNTP() {
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
    struct tm timeinfo;
    // Wait up to 5s for NTP sync
    for (int i = 0; i < 10; i++) {
        if (getLocalTime(&timeinfo, 500)) {
            Serial.println("[Core0] NTP synced.");
            return;
        }
    }
    Serial.println("[Core0] NTP sync failed, timestamp will be empty.");
}

void publishJSON() {
    char topic[64];
    snprintf(topic, sizeof(topic), "%s/status", MQTT_TOPIC_PREFIX);

    // Get NTP timestamp
    char timestamp[24] = "";
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 0)) {
        strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", &timeinfo);
    }

    // Build JSON payload with lock status per actuator
    char payload[1024];
    snprintf(payload, sizeof(payload),
        "{\"timestamp\":\"%s\",\"boot\":%u,"
        "\"sensors\":{\"temp_ext\":%.2f,\"humid_ext\":%.2f,\"temp_int\":%.2f,\"humid_int\":%.2f},"
        "\"actuators\":{"
            "\"valve_ext\":{\"state\":\"%s\",\"locked\":%s,\"lock_s\":%d},"
            "\"valve_deum\":{\"state\":\"%s\",\"locked\":%s,\"lock_s\":%d},"
            "\"dehumidifier\":{\"state\":\"%s\",\"locked\":%s,\"lock_s\":%d},"
            "\"fan\":{\"state\":\"%s\",\"locked\":%s,\"lock_s\":%d}"
        "},"
        "\"thresholds\":{\"soglia_umidita\":%.2f,\"temp_min_ext\":%.2f,\"temp_max_ext\":%.2f}}",
        timestamp,
        rtc_boot_count,
        g_temp_ext, g_humid_ext, g_temp_int, g_humid_int,
        // valve_ext
        g_valve_ext_open  ? "APERTA" : "CHIUSA",
        rtc_lock_remaining_s[ACT_VALVE_EXT] > 0 ? "true" : "false",
        rtc_lock_remaining_s[ACT_VALVE_EXT] > 0 ? rtc_lock_remaining_s[ACT_VALVE_EXT] : 0,
        // valve_deum
        g_valve_deum_open ? "APERTA" : "CHIUSA",
        rtc_lock_remaining_s[ACT_VALVE_DEUM] > 0 ? "true" : "false",
        rtc_lock_remaining_s[ACT_VALVE_DEUM] > 0 ? rtc_lock_remaining_s[ACT_VALVE_DEUM] : 0,
        // dehumidifier
        g_dehumidifier_on ? "ON" : "OFF",
        rtc_lock_remaining_s[ACT_DEHUMIDIFIER] > 0 ? "true" : "false",
        rtc_lock_remaining_s[ACT_DEHUMIDIFIER] > 0 ? rtc_lock_remaining_s[ACT_DEHUMIDIFIER] : 0,
        // fan
        g_fan_on ? "ON" : "OFF",
        rtc_lock_remaining_s[ACT_FAN] > 0 ? "true" : "false",
        rtc_lock_remaining_s[ACT_FAN] > 0 ? rtc_lock_remaining_s[ACT_FAN] : 0,
        // thresholds (runtime values from RTC)
        rtc_soglia_umidita, rtc_temp_min_ext, rtc_temp_max_ext
    );

    mqttClient.publish(topic, payload, true);  // retained
    Serial.printf("[Core0] Published: %s\n", payload);
}

// =============================================================
//  MQTT Command Handler
//
//  Commands (JSON on MQTT_TOPIC_PREFIX/commands):
//
//  Set actuator (force state + lock):
//    {"cmd":"set_actuator","target":"valve_ext","state":"open"}
//    {"cmd":"set_actuator","target":"fan","state":"on","lock_s":3600}
//    state: "open"/"close" for valves, "on"/"off" for relays
//    lock_s: optional, defaults to ACTUATOR_LOCK_S
//
//  Unlock actuator (resume automatic logic):
//    {"cmd":"unlock","target":"valve_ext"}
//    {"cmd":"unlock_all"}
//
//  Update thresholds:
//    {"cmd":"set_thresholds","soglia_umidita":55.0,"temp_min_ext":10.0,"temp_max_ext":30.0}
//    (any subset of fields accepted)
//
//  Reset thresholds to compile-time defaults:
//    {"cmd":"reset_thresholds"}
//
//  Reboot:
//    {"cmd":"reboot"}
// =============================================================

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    char msg[512];
    unsigned int copyLen = (length < sizeof(msg) - 1) ? length : sizeof(msg) - 1;
    memcpy(msg, payload, copyLen);
    msg[copyLen] = '\0';

    Serial.printf("[Core0] Command received: %s\n", msg);

    char cmd[32] = "";
    if (!jsonGetString(msg, "cmd", cmd, sizeof(cmd))) {
        Serial.println("[Core0] No 'cmd' field in command, ignoring.");
        return;
    }

    // --- set_actuator ---
    if (strcmp(cmd, "set_actuator") == 0) {
        char target[20] = "";
        char state[10] = "";
        if (!jsonGetString(msg, "target", target, sizeof(target))) {
            Serial.println("[Core0] set_actuator: missing 'target'");
            return;
        }
        if (!jsonGetString(msg, "state", state, sizeof(state))) {
            Serial.println("[Core0] set_actuator: missing 'state'");
            return;
        }

        int idx = actuatorNameToIdx(target);
        if (idx < 0) {
            Serial.printf("[Core0] Unknown actuator: %s\n", target);
            return;
        }

        // Determine desired boolean state
        bool desiredState = (strcmp(state, "open") == 0 || strcmp(state, "on") == 0);

        // Get lock duration (optional, defaults to ACTUATOR_LOCK_S)
        int lock_s = ACTUATOR_LOCK_S;
        jsonGetInt(msg, "lock_s", &lock_s);

        // Force the actuator
        forceActuatorState(idx, desiredState);

        // Set lock
        rtc_lock_remaining_s[idx] = lock_s;
        rtc_forced_state[idx] = desiredState;

        Serial.printf("[Core0] Actuator %s forced to %s, locked for %ds\n",
                      target, state, lock_s);
    }
    // --- unlock ---
    else if (strcmp(cmd, "unlock") == 0) {
        char target[20] = "";
        if (!jsonGetString(msg, "target", target, sizeof(target))) {
            Serial.println("[Core0] unlock: missing 'target'");
            return;
        }
        int idx = actuatorNameToIdx(target);
        if (idx < 0) {
            Serial.printf("[Core0] Unknown actuator: %s\n", target);
            return;
        }
        rtc_lock_remaining_s[idx] = 0;
        Serial.printf("[Core0] Actuator %s unlocked\n", target);
    }
    // --- unlock_all ---
    else if (strcmp(cmd, "unlock_all") == 0) {
        for (int i = 0; i < ACT_COUNT; i++) {
            rtc_lock_remaining_s[i] = 0;
        }
        Serial.println("[Core0] All actuators unlocked");
    }
    // --- set_thresholds ---
    else if (strcmp(cmd, "set_thresholds") == 0) {
        float val;
        if (jsonGetFloat(msg, "soglia_umidita", &val)) {
            rtc_soglia_umidita = val;
            Serial.printf("[Core0] soglia_umidita -> %.2f\n", val);
        }
        if (jsonGetFloat(msg, "temp_min_ext", &val)) {
            rtc_temp_min_ext = val;
            Serial.printf("[Core0] temp_min_ext -> %.2f\n", val);
        }
        if (jsonGetFloat(msg, "temp_max_ext", &val)) {
            rtc_temp_max_ext = val;
            Serial.printf("[Core0] temp_max_ext -> %.2f\n", val);
        }
        Serial.println("[Core0] Thresholds updated (persisted in RTC)");
    }
    // --- reset_thresholds ---
    else if (strcmp(cmd, "reset_thresholds") == 0) {
        rtc_soglia_umidita = SOGLIA_UMIDITA;
        rtc_temp_min_ext   = TEMP_MIN_ESTERNO;
        rtc_temp_max_ext   = TEMP_MAX_ESTERNO;
        Serial.println("[Core0] Thresholds reset to compile-time defaults");
    }
    // --- reboot ---
    else if (strcmp(cmd, "reboot") == 0) {
        Serial.println("[Core0] Reboot command received!");
        mqttClient.disconnect();
        delay(200);
        ESP.restart();
    }
    else {
        Serial.printf("[Core0] Unknown command: %s\n", cmd);
    }
}

void networkTask(void* param) {
    // PHASE 1: Connect WiFi + MQTT while Core 1 reads sensors
    bool network_ok = false;
    if (connectWiFi()) {
        syncNTP();
        if (connectMQTT()) {
            network_ok = true;
        }
    }
    // Signal Core 1 that WiFi/MQTT is ready (or failed)
    xSemaphoreGive(sem_wifi_ready);

    if (!network_ok) {
        // Signal publish done (nothing to publish) so Core 1 can proceed to sleep
        xSemaphoreGive(sem_publish_done);
        vTaskDelete(NULL);
        return;
    }

    // PHASE 2: Wait for sensor data + actuation, then publish single JSON
    if (xSemaphoreTake(sem_sensors_done, pdMS_TO_TICKS(15000)) == pdTRUE) {
        // Wait for actuation to finish so JSON has final actuator states
        xSemaphoreTake(sem_actuation_done, pdMS_TO_TICKS(60000));

        publishJSON();

        // Ensure message is flushed
        mqttClient.loop();
        delay(100);
        mqttClient.loop();

        Serial.println("[Core0] Publish complete.");

        // PHASE 3: Listen for commands
        char cmdTopic[64];
        snprintf(cmdTopic, sizeof(cmdTopic), "%s/commands", MQTT_TOPIC_PREFIX);
        mqttClient.setCallback(mqttCallback);
        mqttClient.subscribe(cmdTopic);
        Serial.printf("[Core0] Listening for commands on %s for %ds...\n", cmdTopic, COMMAND_LISTEN_S);

        unsigned long listenStart = millis();
        while (millis() - listenStart < (unsigned long)COMMAND_LISTEN_S * 1000UL) {
            mqttClient.loop();
            delay(50);
        }
        Serial.println("[Core0] Listen window closed.");
    } else {
        Serial.println("[Core0] Timeout waiting for sensor data!");
    }

    xSemaphoreGive(sem_publish_done);

    // Clean disconnect
    mqttClient.disconnect();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);

    vTaskDelete(NULL);
}

// ======================= CORE 1 (setup/loop) ==================
// Reads sensors, runs VMC logic, drives actuators, then sleeps.
// ==============================================================

void readSensors() {
    Serial.println("[Core1] Reading sensors...");

    // DHT sensors need begin() called every boot (no state across deep sleep)
    sensoreEsterno->sns->begin();
    sensoreInterno->sns->begin();

    // Small delay for DHT22 to stabilize after power-on
    delay(2000);

    // Read external sensor (retry up to 3 times)
    for (int i = 0; i < 3; i++) {
        float h = sensoreEsterno->getHumidity();
        if (!isnan(h) && h >= 0 && h <= 100) {
            g_humid_ext = h;
            g_temp_ext  = sensoreEsterno->getTemperature();
            Serial.printf("[Core1] Esterno: T=%.1f H=%.1f\n", g_temp_ext, g_humid_ext);
            break;
        }
        Serial.println("[Core1] Ext sensor retry...");
        delay(sensoreEsterno->delayus / 1000);
    }

    // Read internal sensor (retry up to 3 times)
    for (int i = 0; i < 3; i++) {
        float h = sensoreInterno->getHumidity();
        if (!isnan(h) && h >= 0 && h <= 100) {
            g_humid_int = h;
            g_temp_int  = sensoreInterno->getTemperature();
            Serial.printf("[Core1] Interno: T=%.1f H=%.1f\n", g_temp_int, g_humid_int);
            break;
        }
        Serial.println("[Core1] Int sensor retry...");
        delay(sensoreInterno->delayus / 1000);
    }
}

// Decrement lock timers and apply lock flags to relay objects
void processLockTimers() {
    for (int i = 0; i < ACT_COUNT; i++) {
        if (rtc_lock_remaining_s[i] > 0) {
            rtc_lock_remaining_s[i] -= DEEP_SLEEP_TIME_S;
            if (rtc_lock_remaining_s[i] <= 0) {
                rtc_lock_remaining_s[i] = 0;
                Serial.printf("[Core1] Actuator %d lock expired\n", i);
            }
        }
    }

    // Set locked flags on relay objects
    rValvEsterno->locked         = (rtc_lock_remaining_s[ACT_VALVE_EXT]      > 0);
    rValvDeumidificatore->locked = (rtc_lock_remaining_s[ACT_VALVE_DEUM]     > 0);
    rVentolaDeum->locked         = (rtc_lock_remaining_s[ACT_FAN]            > 0);
    rDeumidificatore->locked     = (rtc_lock_remaining_s[ACT_DEHUMIDIFIER]   > 0);
}

void runVMCLogic() {
    Serial.println("[Core1] Running VMC logic...");

    // Decrement lock timers and set locked flags
    processLockTimers();

    // Default: preserve previous state from RTC memory
    bool want_valve_ext_open  = rtc_valve_ext_open;
    bool want_valve_deum_open = rtc_valve_deum_open;
    bool want_fan_on          = rtc_fan_on;
    bool want_dehumidifier_on = rtc_dehumidifier_on;

    // If external sensor failed, don't change anything — keep previous state
    if (g_temp_ext == CANC_NUM || isnan(g_temp_ext)) {
        Serial.println("[Core1] External sensor invalid, keeping previous state.");
    } else {
        // External conditions OK for ventilation?
        // Uses RTC thresholds (modifiable via MQTT)
        bool ext_ok = (g_humid_ext <= rtc_soglia_umidita) &&
                      (g_temp_ext  >= rtc_temp_min_ext) &&
                      (g_temp_ext  <= rtc_temp_max_ext);

        if (ext_ok) {
            want_valve_ext_open = true;
            if (g_humid_int >= (rtc_soglia_umidita - 10)) {
                want_valve_deum_open = true;
                want_fan_on          = true;
                want_dehumidifier_on = true;
            } else {
                want_valve_deum_open = false;
            }
        } else {
            want_valve_ext_open  = false;
            want_valve_deum_open = false;
            if (g_humid_int != CANC_NUM && g_humid_int >= (rtc_soglia_umidita - 10)) {
                want_fan_on          = true;
                want_dehumidifier_on = true;
            } else {
                want_dehumidifier_on = false;
            }
        }
    }

    // --- Apply actuator changes (using lock-aware tryXxx methods) ---
    // Locked actuators stay in their forced state; unlocked ones follow VMC logic.

    // External valve
    if (want_valve_ext_open && !rtc_valve_ext_open) {
        Serial.println("[Core1] Opening external valve...");
        rValvEsterno->is_open = false;
        rValvEsterno->tryOpenValve();
    } else if (!want_valve_ext_open && rtc_valve_ext_open) {
        Serial.println("[Core1] Closing external valve...");
        rValvEsterno->is_open = true;
        rValvEsterno->tryCloseValve();
    }

    // Dehumidifier valve
    if (want_valve_deum_open && !rtc_valve_deum_open) {
        Serial.println("[Core1] Opening deum valve...");
        rValvDeumidificatore->is_open = false;
        rValvDeumidificatore->tryOpenValve();
    } else if (!want_valve_deum_open && rtc_valve_deum_open) {
        Serial.println("[Core1] Closing deum valve...");
        rValvDeumidificatore->is_open = true;
        rValvDeumidificatore->tryCloseValve();
    }

    // Fan
    if (want_fan_on && !rtc_fan_on) {
        rVentolaDeum->is_on = false;
        rVentolaDeum->tryTurnOn();
    } else if (!want_fan_on && rtc_fan_on) {
        rVentolaDeum->is_on = true;
        rVentolaDeum->tryTurnOff();
    }

    // Dehumidifier
    if (want_dehumidifier_on && !rtc_dehumidifier_on) {
        rDeumidificatore->is_on = false;
        rDeumidificatore->tryTurnOn();
    } else if (!want_dehumidifier_on && rtc_dehumidifier_on) {
        rDeumidificatore->is_on = true;
        rDeumidificatore->tryTurnOff();
    }

    // For locked actuators, preserve forced state; for unlocked, use computed state
    g_valve_ext_open  = rValvEsterno->locked         ? rtc_valve_ext_open  : want_valve_ext_open;
    g_valve_deum_open = rValvDeumidificatore->locked  ? rtc_valve_deum_open : want_valve_deum_open;
    g_fan_on          = rVentolaDeum->locked          ? rtc_fan_on          : want_fan_on;
    g_dehumidifier_on = rDeumidificatore->locked      ? rtc_dehumidifier_on : want_dehumidifier_on;

    // Persist to RTC (locked actuators keep forced state)
    rtc_valve_ext_open  = g_valve_ext_open;
    rtc_valve_deum_open = g_valve_deum_open;
    rtc_fan_on          = g_fan_on;
    rtc_dehumidifier_on = g_dehumidifier_on;

    Serial.printf("[Core1] State: ValvExt=%s%s ValvDeum=%s%s Fan=%s%s Deum=%s%s\n",
                  g_valve_ext_open  ? "OPEN"  : "CLOSED", rValvEsterno->locked         ? "(L)" : "",
                  g_valve_deum_open ? "OPEN"  : "CLOSED", rValvDeumidificatore->locked  ? "(L)" : "",
                  g_fan_on          ? "ON"    : "OFF",    rVentolaDeum->locked          ? "(L)" : "",
                  g_dehumidifier_on ? "ON"    : "OFF",    rDeumidificatore->locked      ? "(L)" : "");
}

void enterDeepSleep() {
    Serial.printf("[Core1] Entering deep sleep for %d seconds...\n", DEEP_SLEEP_TIME_S);
    Serial.flush();

    esp_sleep_enable_timer_wakeup((uint64_t)DEEP_SLEEP_TIME_S * 1000000ULL);
    esp_deep_sleep_start();
}

// ========================= setup() ============================
void setup() {
    Serial.begin(115200);
    rtc_boot_count++;
    Serial.printf("\n\n=== VMC IoT Wake #%u ===\n", rtc_boot_count);

    // --- Create semaphores ---
    sem_sensors_done   = xSemaphoreCreateBinary();
    sem_wifi_ready     = xSemaphoreCreateBinary();
    sem_actuation_done = xSemaphoreCreateBinary();
    sem_publish_done   = xSemaphoreCreateBinary();

    // --- Initialise hardware (Core 1) ---
    sensoreEsterno = new Sensor(SENSORE_ESTERNO_PIN);
    sensoreInterno = new Sensor(SENSORE_INTERNO_PIN);

    // Construct relays with skip_init=true to avoid auto-actuation on every wake.
    // The real state is tracked in RTC memory across deep sleep cycles.
    rValvEsterno         = new RelayMotore(EXTERN_VALVE_OPEN_PIN, EXTERN_VALVE_CLOSE_PIN, MOTOR_TIME, false, true);
    rValvDeumidificatore = new RelayMotore(DEUM_VALVE_OPEN_PIN, DEUM_VALVE_CLOSE_PIN, MOTOR_TIME, false, true);
    rVentolaDeum         = new SingleRelay(FAN_PIN, 0, true);
    rDeumidificatore     = new SingleRelay(DEHUMIDIFIER_PIN, 0, true);

    // Restore relay pin states from RTC memory (active-low)
    digitalWrite(FAN_PIN, rtc_fan_on ? LOW : HIGH);
    digitalWrite(DEHUMIDIFIER_PIN, rtc_dehumidifier_on ? LOW : HIGH);

    // Set internal object state from RTC
    rValvEsterno->is_open         = rtc_valve_ext_open;
    rValvDeumidificatore->is_open = rtc_valve_deum_open;
    rVentolaDeum->is_on           = rtc_fan_on;
    rDeumidificatore->is_on       = rtc_dehumidifier_on;

    // --- Launch Core 0 network task (runs in parallel with sensor read) ---
    xTaskCreatePinnedToCore(
        networkTask,
        "Network",
        8192,       // Stack size
        NULL,       // Parameter
        1,          // Priority
        NULL,       // Task handle
        0           // Core 0 (PRO_CPU)
    );

    // --- PHASE 1: Read sensors on Core 1 while Core 0 connects WiFi ---
    readSensors();
    xSemaphoreGive(sem_sensors_done);

    // --- PHASE 2: Run VMC logic and actuate on Core 1 ---
    runVMCLogic();
    xSemaphoreGive(sem_actuation_done);

    // --- Wait for Core 0 to finish publishing + listening ---
    Serial.println("[Core1] Waiting for MQTT publish to complete...");
    if (xSemaphoreTake(sem_publish_done, pdMS_TO_TICKS((COMMAND_LISTEN_S + 30) * 1000)) == pdTRUE) {
        Serial.println("[Core1] MQTT cycle complete.");
    } else {
        Serial.println("[Core1] Timeout waiting for MQTT, proceeding to sleep.");
        // Force WiFi cleanup
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
    }

    // --- PHASE 3: Deep sleep ---
    delay(100);
    enterDeepSleep();
}

void loop() {
    // Never reached — deep sleep restarts from setup()
}
