#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Relay.h>
#include <MySensor.h>
#include <time.h>
#include <string.h>
#include <math.h>
#include "config.h"

// --- NTP ---
static const char* NTP_SERVER          = "pool.ntp.org";
static const long  GMT_OFFSET_SEC      = 3600;
static const int   DAYLIGHT_OFFSET_SEC = 3600;

// =============================================================
//  VMC IoT — Deep Sleep + MQTT + Dual-Core + Rule Engine
//
//  Flow:  Wake → Sense (Core 1) + Connect (Core 0)
//         → Evaluate rules → Actuate → Publish → Listen → Sleep
//
//  Core 1 (APP_CPU): sensors, rule evaluation, actuation
//  Core 0 (PRO_CPU): WiFi, MQTT publish, command listen
// =============================================================

// =============================================================
//  Rule Engine Types
// =============================================================

// Sensor variables that can appear in rule conditions
enum SensorVar : uint8_t {
    VAR_TEMP_EXT  = 0,
    VAR_HUMID_EXT = 1,
    VAR_TEMP_INT  = 2,
    VAR_HUMID_INT = 3,
    VAR_NONE      = 0xFF
};

// Comparison operators for conditions
enum CompOp : uint8_t {
    OP_NONE = 0,  // Unused term slot
    OP_GT   = 1,  // >
    OP_GTE  = 2,  // >=
    OP_LT   = 3,  // <
    OP_LTE  = 4,  // <=
    OP_EQ   = 5   // ==
};

// Effect action for actuators
enum EffectAction : uint8_t {
    EFF_NOOP = 0,  // Unused effect slot
    EFF_ON   = 1,  // Open valve / Turn on relay
    EFF_OFF  = 2   // Close valve / Turn off relay
};

// Actuator indices
enum ActuatorIdx : uint8_t {
    ACT_VALVE_EXT    = 0,
    ACT_VALVE_DEUM   = 1,
    ACT_FAN          = 2,
    ACT_DEHUMIDIFIER = 3,
    ACT_COUNT        = 4,
    ACT_NONE         = 0xFF
};

// A single condition: "sensor_var op value"
struct ConditionTerm {
    uint8_t var;    // SensorVar
    uint8_t op;     // CompOp (OP_NONE = unused)
    float   value;
};

// A single effect: "set actuator to on/off"
struct RuleEffect {
    uint8_t actuator;  // ActuatorIdx (ACT_NONE = unused)
    uint8_t action;    // EffectAction
};

#define MAX_TERMS   5
#define MAX_EFFECTS 4
#define MAX_RULES   12

// A logic rule: if ALL terms are true → apply effects
struct LogicRule {
    bool          active;
    ConditionTerm terms[MAX_TERMS];
    RuleEffect    effects[MAX_EFFECTS];
};

// =============================================================
//  RTC Memory — persists across deep sleep
// =============================================================

RTC_DATA_ATTR bool     rtc_valve_ext_open   = true;   // First-boot: ext valve open
RTC_DATA_ATTR bool     rtc_valve_deum_open  = false;
RTC_DATA_ATTR bool     rtc_fan_on           = false;
RTC_DATA_ATTR bool     rtc_dehumidifier_on  = false;
RTC_DATA_ATTR uint32_t rtc_boot_count       = 0;

// Thresholds (used to build default rules, updatable via set_thresholds)
RTC_DATA_ATTR float    rtc_soglia_umidita   = SOGLIA_UMIDITA;
RTC_DATA_ATTR float    rtc_temp_min_ext     = TEMP_MIN_ESTERNO;
RTC_DATA_ATTR float    rtc_temp_max_ext     = TEMP_MAX_ESTERNO;

// Actuator locks
RTC_DATA_ATTR int32_t  rtc_lock_remaining_s[ACT_COUNT] = {0, 0, 0, 0};
RTC_DATA_ATTR bool     rtc_forced_state[ACT_COUNT]     = {false, false, false, false};

// Logic rule table
RTC_DATA_ATTR bool      rtc_rules_initialized = false;
RTC_DATA_ATTR LogicRule rtc_rules[MAX_RULES];

// =============================================================
//  Shared volatile state (between cores)
// =============================================================

volatile float g_temp_ext  = CANC_NUM;
volatile float g_humid_ext = CANC_NUM;
volatile float g_temp_int  = CANC_NUM;
volatile float g_humid_int = CANC_NUM;

volatile bool g_valve_ext_open  = false;
volatile bool g_valve_deum_open = false;
volatile bool g_fan_on          = false;
volatile bool g_dehumidifier_on = false;

// =============================================================
//  Synchronisation & globals
// =============================================================

SemaphoreHandle_t sem_sensors_done   = NULL;
SemaphoreHandle_t sem_wifi_ready     = NULL;
SemaphoreHandle_t sem_actuation_done = NULL;
SemaphoreHandle_t sem_publish_done   = NULL;

WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);

Sensor*       sensoreEsterno       = NULL;
Sensor*       sensoreInterno       = NULL;
RelayMotore*  rValvEsterno         = NULL;
RelayMotore*  rValvDeumidificatore = NULL;
SingleRelay*  rVentolaDeum         = NULL;
SingleRelay*  rDeumidificatore     = NULL;

// =============================================================
//  Name ↔ Enum conversion tables
// =============================================================

const char* varNames[]      = {"temp_ext", "humid_ext", "temp_int", "humid_int"};
const char* opNames[]       = {"", ">", ">=", "<", "<=", "=="};
const char* actuatorNames[] = {"valve_ext", "valve_deum", "fan", "dehumidifier"};

uint8_t nameToVar(const char* s) {
    for (int i = 0; i < 4; i++) if (strcmp(s, varNames[i]) == 0) return i;
    return VAR_NONE;
}

uint8_t nameToOp(const char* s) {
    if (strcmp(s, ">")  == 0) return OP_GT;
    if (strcmp(s, ">=") == 0) return OP_GTE;
    if (strcmp(s, "<")  == 0) return OP_LT;
    if (strcmp(s, "<=") == 0) return OP_LTE;
    if (strcmp(s, "==") == 0) return OP_EQ;
    return OP_NONE;
}

uint8_t nameToActuator(const char* s) {
    for (int i = 0; i < ACT_COUNT; i++) if (strcmp(s, actuatorNames[i]) == 0) return i;
    return ACT_NONE;
}

uint8_t nameToAction(const char* s) {
    if (strcmp(s, "on")    == 0 || strcmp(s, "open")  == 0) return EFF_ON;
    if (strcmp(s, "off")   == 0 || strcmp(s, "close") == 0) return EFF_OFF;
    return EFF_NOOP;
}

const char* varToName(uint8_t v)   { return (v < 4) ? varNames[v] : "?"; }
const char* opToName(uint8_t o)    { return (o > 0 && o <= 5) ? opNames[o] : "?"; }
const char* actToName(uint8_t a)   { return (a < ACT_COUNT) ? actuatorNames[a] : "?"; }
const char* actionToName(uint8_t a){ return (a == EFF_ON) ? "on" : (a == EFF_OFF) ? "off" : "noop"; }

// =============================================================
//  Default rule loader
//  Encodes the original VMC logic using current RTC thresholds.
//  Called on first boot and on reset_rules command.
//
//  Original logic decomposed into 8 AND-only rules:
//    R0: ext_ok + int_humid → all ON
//    R1: ext_ok + int_dry   → ext_open, deum_close
//    R2-R7: three "ext bad" conditions × two humidity states
// =============================================================

void loadDefaultRules() {
    memset(rtc_rules, 0, sizeof(rtc_rules));

    float S    = rtc_soglia_umidita;
    float Tmin = rtc_temp_min_ext;
    float Tmax = rtc_temp_max_ext;
    float Si   = S - 10.0f;  // internal humidity threshold

    // Helper macro to set a term
    #define T(idx, v, o, val) do { r->terms[idx] = {(uint8_t)(v), (uint8_t)(o), (val)}; } while(0)
    #define E(idx, a, act)    do { r->effects[idx] = {(uint8_t)(a), (uint8_t)(act)}; } while(0)

    LogicRule* r;

    // R0: ext OK + int humid → valve_ext=OPEN, valve_deum=OPEN, fan=ON, deum=ON
    r = &rtc_rules[0]; r->active = true;
    T(0, VAR_HUMID_EXT, OP_LTE, S);
    T(1, VAR_TEMP_EXT,  OP_GTE, Tmin);
    T(2, VAR_TEMP_EXT,  OP_LTE, Tmax);
    T(3, VAR_HUMID_INT, OP_GTE, Si);
    E(0, ACT_VALVE_EXT, EFF_ON);  E(1, ACT_VALVE_DEUM, EFF_ON);
    E(2, ACT_FAN, EFF_ON);        E(3, ACT_DEHUMIDIFIER, EFF_ON);

    // R1: ext OK + int dry → valve_ext=OPEN, valve_deum=CLOSE
    r = &rtc_rules[1]; r->active = true;
    T(0, VAR_HUMID_EXT, OP_LTE, S);
    T(1, VAR_TEMP_EXT,  OP_GTE, Tmin);
    T(2, VAR_TEMP_EXT,  OP_LTE, Tmax);
    T(3, VAR_HUMID_INT, OP_LT,  Si);
    E(0, ACT_VALVE_EXT, EFF_ON);  E(1, ACT_VALVE_DEUM, EFF_OFF);

    // R2: ext too humid + int humid → close ext/deum, fan+deum ON
    r = &rtc_rules[2]; r->active = true;
    T(0, VAR_HUMID_EXT, OP_GT,  S);
    T(1, VAR_HUMID_INT, OP_GTE, Si);
    E(0, ACT_VALVE_EXT, EFF_OFF); E(1, ACT_VALVE_DEUM, EFF_OFF);
    E(2, ACT_FAN, EFF_ON);        E(3, ACT_DEHUMIDIFIER, EFF_ON);

    // R3: ext too humid + int dry → close ext/deum, deum OFF
    r = &rtc_rules[3]; r->active = true;
    T(0, VAR_HUMID_EXT, OP_GT,  S);
    T(1, VAR_HUMID_INT, OP_LT,  Si);
    E(0, ACT_VALVE_EXT, EFF_OFF); E(1, ACT_VALVE_DEUM, EFF_OFF);
    E(2, ACT_DEHUMIDIFIER, EFF_OFF);

    // R4: ext too cold + int humid → close ext/deum, fan+deum ON
    r = &rtc_rules[4]; r->active = true;
    T(0, VAR_TEMP_EXT,  OP_LT,  Tmin);
    T(1, VAR_HUMID_INT, OP_GTE, Si);
    E(0, ACT_VALVE_EXT, EFF_OFF); E(1, ACT_VALVE_DEUM, EFF_OFF);
    E(2, ACT_FAN, EFF_ON);        E(3, ACT_DEHUMIDIFIER, EFF_ON);

    // R5: ext too cold + int dry → close ext/deum, deum OFF
    r = &rtc_rules[5]; r->active = true;
    T(0, VAR_TEMP_EXT,  OP_LT,  Tmin);
    T(1, VAR_HUMID_INT, OP_LT,  Si);
    E(0, ACT_VALVE_EXT, EFF_OFF); E(1, ACT_VALVE_DEUM, EFF_OFF);
    E(2, ACT_DEHUMIDIFIER, EFF_OFF);

    // R6: ext too hot + int humid → close ext/deum, fan+deum ON
    r = &rtc_rules[6]; r->active = true;
    T(0, VAR_TEMP_EXT,  OP_GT,  Tmax);
    T(1, VAR_HUMID_INT, OP_GTE, Si);
    E(0, ACT_VALVE_EXT, EFF_OFF); E(1, ACT_VALVE_DEUM, EFF_OFF);
    E(2, ACT_FAN, EFF_ON);        E(3, ACT_DEHUMIDIFIER, EFF_ON);

    // R7: ext too hot + int dry → close ext/deum, deum OFF
    r = &rtc_rules[7]; r->active = true;
    T(0, VAR_TEMP_EXT,  OP_GT,  Tmax);
    T(1, VAR_HUMID_INT, OP_LT,  Si);
    E(0, ACT_VALVE_EXT, EFF_OFF); E(1, ACT_VALVE_DEUM, EFF_OFF);
    E(2, ACT_DEHUMIDIFIER, EFF_OFF);

    #undef T
    #undef E

    rtc_rules_initialized = true;
    Serial.println("[Rules] Default rules loaded");
}

// =============================================================
//  Rule evaluation
// =============================================================

float getSensorValue(uint8_t var) {
    switch (var) {
        case VAR_TEMP_EXT:  return g_temp_ext;
        case VAR_HUMID_EXT: return g_humid_ext;
        case VAR_TEMP_INT:  return g_temp_int;
        case VAR_HUMID_INT: return g_humid_int;
        default:            return CANC_NUM;
    }
}

bool evaluateTerm(const ConditionTerm& t) {
    if (t.op == OP_NONE) return true;  // Unused slot = vacuously true
    float val = getSensorValue(t.var);
    if (val == CANC_NUM || isnan(val)) return false;  // Invalid sensor → term fails
    switch (t.op) {
        case OP_GT:  return val >  t.value;
        case OP_GTE: return val >= t.value;
        case OP_LT:  return val <  t.value;
        case OP_LTE: return val <= t.value;
        case OP_EQ:  return fabsf(val - t.value) < 0.01f;
        default:     return false;
    }
}

bool evaluateRule(const LogicRule& rule) {
    if (!rule.active) return false;
    for (int i = 0; i < MAX_TERMS; i++) {
        if (!evaluateTerm(rule.terms[i])) return false;
    }
    return true;
}

// Evaluate all rules. desired[i] = -1 (no change), 0 (OFF), 1 (ON)
// Later matching rules overwrite earlier ones per actuator.
void evaluateAllRules(int8_t* desired) {
    for (int i = 0; i < ACT_COUNT; i++) desired[i] = -1;

    for (int r = 0; r < MAX_RULES; r++) {
        if (!evaluateRule(rtc_rules[r])) continue;
        Serial.printf("[Rules] Rule %d matched\n", r);
        for (int e = 0; e < MAX_EFFECTS; e++) {
            uint8_t act = rtc_rules[r].effects[e].actuator;
            uint8_t eff = rtc_rules[r].effects[e].action;
            if (act < ACT_COUNT && eff != EFF_NOOP) {
                desired[act] = (eff == EFF_ON) ? 1 : 0;
            }
        }
    }
}

// =============================================================
//  Actuator helpers
// =============================================================

// Force an actuator (bypasses locks — used by MQTT set_actuator)
void forceActuatorState(int idx, bool state) {
    switch (idx) {
        case ACT_VALVE_EXT:
            rValvEsterno->is_open = !state;
            if (state) rValvEsterno->openValve(); else rValvEsterno->closeValve();
            rtc_valve_ext_open = state; g_valve_ext_open = state;
            break;
        case ACT_VALVE_DEUM:
            rValvDeumidificatore->is_open = !state;
            if (state) rValvDeumidificatore->openValve(); else rValvDeumidificatore->closeValve();
            rtc_valve_deum_open = state; g_valve_deum_open = state;
            break;
        case ACT_FAN:
            rVentolaDeum->is_on = !state;
            if (state) rVentolaDeum->turnOn(); else rVentolaDeum->turnOff();
            rtc_fan_on = state; g_fan_on = state;
            break;
        case ACT_DEHUMIDIFIER:
            rDeumidificatore->is_on = !state;
            if (state) rDeumidificatore->turnOn(); else rDeumidificatore->turnOff();
            rtc_dehumidifier_on = state; g_dehumidifier_on = state;
            break;
    }
}

// Try to set actuator via VMC logic (respects locks). Returns true if changed.
bool trySetActuatorState(int idx, bool state) {
    bool changed = false;
    switch (idx) {
        case ACT_VALVE_EXT:
            if (state && !rtc_valve_ext_open) {
                rValvEsterno->is_open = false;
                changed = rValvEsterno->tryOpenValve();
            } else if (!state && rtc_valve_ext_open) {
                rValvEsterno->is_open = true;
                changed = rValvEsterno->tryCloseValve();
            }
            if (changed) { rtc_valve_ext_open = state; }
            g_valve_ext_open = rtc_valve_ext_open;
            break;
        case ACT_VALVE_DEUM:
            if (state && !rtc_valve_deum_open) {
                rValvDeumidificatore->is_open = false;
                changed = rValvDeumidificatore->tryOpenValve();
            } else if (!state && rtc_valve_deum_open) {
                rValvDeumidificatore->is_open = true;
                changed = rValvDeumidificatore->tryCloseValve();
            }
            if (changed) { rtc_valve_deum_open = state; }
            g_valve_deum_open = rtc_valve_deum_open;
            break;
        case ACT_FAN:
            if (state && !rtc_fan_on) {
                rVentolaDeum->is_on = false;
                changed = rVentolaDeum->tryTurnOn();
            } else if (!state && rtc_fan_on) {
                rVentolaDeum->is_on = true;
                changed = rVentolaDeum->tryTurnOff();
            }
            if (changed) { rtc_fan_on = state; }
            g_fan_on = rtc_fan_on;
            break;
        case ACT_DEHUMIDIFIER:
            if (state && !rtc_dehumidifier_on) {
                rDeumidificatore->is_on = false;
                changed = rDeumidificatore->tryTurnOn();
            } else if (!state && rtc_dehumidifier_on) {
                rDeumidificatore->is_on = true;
                changed = rDeumidificatore->tryTurnOff();
            }
            if (changed) { rtc_dehumidifier_on = state; }
            g_dehumidifier_on = rtc_dehumidifier_on;
            break;
    }
    return changed;
}

// =============================================================
//  Network: WiFi, MQTT, NTP
// =============================================================

bool connectWiFi() {
    Serial.print("[Core0] WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - start > WIFI_TIMEOUT_MS) { Serial.println(" TIMEOUT"); return false; }
        delay(100);
    }
    Serial.printf(" OK (%s)\n", WiFi.localIP().toString().c_str());
    return true;
}

bool connectMQTT() {
    mqttClient.setBufferSize(1024);
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    Serial.print("[Core0] MQTT...");
    for (int a = 0; a < 3; a++) {
        bool ok = (strlen(MQTT_USER) > 0)
            ? mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)
            : mqttClient.connect(MQTT_CLIENT_ID);
        if (ok) { Serial.println(" OK"); return true; }
        Serial.printf(" attempt %d failed\n", a + 1);
        delay(500);
    }
    Serial.println(" FAILED");
    return false;
}

void syncNTP() {
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
    struct tm ti;
    for (int i = 0; i < 10; i++) {
        if (getLocalTime(&ti, 500)) { Serial.println("[Core0] NTP synced."); return; }
    }
    Serial.println("[Core0] NTP sync failed.");
}

// =============================================================
//  MQTT Publish
// =============================================================

void publishJSON() {
    JsonDocument doc;

    // Timestamp
    char ts[24] = "";
    struct tm ti;
    if (getLocalTime(&ti, 0)) strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &ti);
    doc["timestamp"] = ts;
    doc["boot"] = rtc_boot_count;

    // Sensors
    JsonObject sens = doc["sensors"].to<JsonObject>();
    sens["temp_ext"]  = serialized(String(g_temp_ext, 2));
    sens["humid_ext"] = serialized(String(g_humid_ext, 2));
    sens["temp_int"]  = serialized(String(g_temp_int, 2));
    sens["humid_int"] = serialized(String(g_humid_int, 2));

    // Actuators with lock info
    JsonObject acts = doc["actuators"].to<JsonObject>();
    const char* actStates[] = {
        g_valve_ext_open ? "APERTA" : "CHIUSA",
        g_valve_deum_open ? "APERTA" : "CHIUSA",
        g_fan_on ? "ON" : "OFF",
        g_dehumidifier_on ? "ON" : "OFF"
    };
    for (int i = 0; i < ACT_COUNT; i++) {
        JsonObject a = acts[actuatorNames[i]].to<JsonObject>();
        a["state"]  = actStates[i];
        a["locked"] = (rtc_lock_remaining_s[i] > 0);
        a["lock_s"] = max(0, (int)rtc_lock_remaining_s[i]);
    }

    // Thresholds
    JsonObject thr = doc["thresholds"].to<JsonObject>();
    thr["soglia_umidita"] = rtc_soglia_umidita;
    thr["temp_min_ext"]   = rtc_temp_min_ext;
    thr["temp_max_ext"]   = rtc_temp_max_ext;

    // Active rules count
    int activeCount = 0;
    for (int i = 0; i < MAX_RULES; i++) if (rtc_rules[i].active) activeCount++;
    doc["active_rules"] = activeCount;

    char topic[64], payload[1024];
    snprintf(topic, sizeof(topic), "%s/status", MQTT_TOPIC_PREFIX);
    serializeJson(doc, payload, sizeof(payload));
    mqttClient.publish(topic, payload, true);
    Serial.printf("[Core0] Published: %s\n", payload);
}

// Publish a single rule to MQTT_TOPIC_PREFIX/rules/N
void publishRule(int idx) {
    if (idx < 0 || idx >= MAX_RULES) return;
    const LogicRule& rule = rtc_rules[idx];

    JsonDocument doc;
    doc["idx"] = idx;
    doc["active"] = rule.active;

    JsonArray terms = doc["terms"].to<JsonArray>();
    for (int t = 0; t < MAX_TERMS; t++) {
        if (rule.terms[t].op == OP_NONE) continue;
        JsonObject term = terms.add<JsonObject>();
        term["var"] = varToName(rule.terms[t].var);
        term["op"]  = opToName(rule.terms[t].op);
        term["val"] = rule.terms[t].value;
    }

    JsonArray effs = doc["effects"].to<JsonArray>();
    for (int e = 0; e < MAX_EFFECTS; e++) {
        if (rule.effects[e].action == EFF_NOOP) continue;
        JsonObject eff = effs.add<JsonObject>();
        eff["act"] = actToName(rule.effects[e].actuator);
        eff["do"]  = actionToName(rule.effects[e].action);
    }

    char topic[64], payload[512];
    snprintf(topic, sizeof(topic), "%s/rules/%d", MQTT_TOPIC_PREFIX, idx);
    serializeJson(doc, payload, sizeof(payload));
    mqttClient.publish(topic, payload, true);
}

// =============================================================
//  MQTT Command Handler
//
//  Commands (JSON on MQTT_TOPIC_PREFIX/commands):
//
//  --- Actuator control ---
//  {"cmd":"set_actuator","target":"valve_ext","state":"open","lock_s":1800}
//  {"cmd":"unlock","target":"valve_ext"}
//  {"cmd":"unlock_all"}
//
//  --- Rule management ---
//  {"cmd":"set_rule","idx":0,"terms":[
//      {"var":"humid_ext","op":"<=","val":59.0}
//    ],"effects":[
//      {"act":"valve_ext","do":"open"},
//      {"act":"fan","do":"on"}
//  ]}
//  {"cmd":"clear_rule","idx":3}
//  {"cmd":"reset_rules"}
//  {"cmd":"get_rules"}
//
//  --- Thresholds (convenience — reloads default rules) ---
//  {"cmd":"set_thresholds","soglia_umidita":55.0,"temp_min_ext":10.0}
//  {"cmd":"reset_thresholds"}
//
//  --- System ---
//  {"cmd":"reboot"}
// =============================================================

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    char msg[768];
    unsigned int copyLen = min(length, (unsigned int)(sizeof(msg) - 1));
    memcpy(msg, payload, copyLen);
    msg[copyLen] = '\0';
    Serial.printf("[Cmd] %s\n", msg);

    JsonDocument doc;
    if (deserializeJson(doc, msg)) {
        Serial.println("[Cmd] JSON parse error");
        return;
    }

    const char* cmd = doc["cmd"];
    if (!cmd) { Serial.println("[Cmd] No 'cmd' field"); return; }

    // --- set_actuator ---
    if (strcmp(cmd, "set_actuator") == 0) {
        const char* target = doc["target"];
        const char* state  = doc["state"];
        if (!target || !state) { Serial.println("[Cmd] Missing target/state"); return; }

        int idx = nameToActuator(target);
        if (idx >= ACT_COUNT) { Serial.printf("[Cmd] Unknown: %s\n", target); return; }

        bool desiredState = (strcmp(state, "open") == 0 || strcmp(state, "on") == 0);
        int lock_s = doc["lock_s"] | (int)ACTUATOR_LOCK_S;

        forceActuatorState(idx, desiredState);
        rtc_lock_remaining_s[idx] = lock_s;
        rtc_forced_state[idx] = desiredState;
        Serial.printf("[Cmd] %s → %s (locked %ds)\n", target, state, lock_s);
    }
    // --- unlock ---
    else if (strcmp(cmd, "unlock") == 0) {
        const char* target = doc["target"];
        if (!target) return;
        int idx = nameToActuator(target);
        if (idx < ACT_COUNT) { rtc_lock_remaining_s[idx] = 0; Serial.printf("[Cmd] %s unlocked\n", target); }
    }
    // --- unlock_all ---
    else if (strcmp(cmd, "unlock_all") == 0) {
        for (int i = 0; i < ACT_COUNT; i++) rtc_lock_remaining_s[i] = 0;
        Serial.println("[Cmd] All unlocked");
    }
    // --- set_rule ---
    else if (strcmp(cmd, "set_rule") == 0) {
        int idx = doc["idx"] | -1;
        if (idx < 0 || idx >= MAX_RULES) { Serial.println("[Cmd] Invalid rule idx"); return; }

        memset(&rtc_rules[idx], 0, sizeof(LogicRule));
        rtc_rules[idx].active = true;

        // Parse terms array
        JsonArray terms = doc["terms"];
        int t = 0;
        for (JsonObject term : terms) {
            if (t >= MAX_TERMS) break;
            const char* v = term["var"];
            const char* o = term["op"];
            if (v && o) {
                rtc_rules[idx].terms[t].var   = nameToVar(v);
                rtc_rules[idx].terms[t].op    = nameToOp(o);
                rtc_rules[idx].terms[t].value = term["val"] | 0.0f;
                t++;
            }
        }

        // Parse effects array
        JsonArray effects = doc["effects"];
        int e = 0;
        for (JsonObject eff : effects) {
            if (e >= MAX_EFFECTS) break;
            const char* a = eff["act"];
            const char* d = eff["do"];
            if (a && d) {
                rtc_rules[idx].effects[e].actuator = nameToActuator(a);
                rtc_rules[idx].effects[e].action   = nameToAction(d);
                e++;
            }
        }
        Serial.printf("[Cmd] Rule %d set (%d terms, %d effects)\n", idx, t, e);
    }
    // --- clear_rule ---
    else if (strcmp(cmd, "clear_rule") == 0) {
        int idx = doc["idx"] | -1;
        if (idx >= 0 && idx < MAX_RULES) {
            memset(&rtc_rules[idx], 0, sizeof(LogicRule));
            Serial.printf("[Cmd] Rule %d cleared\n", idx);
        }
    }
    // --- reset_rules ---
    else if (strcmp(cmd, "reset_rules") == 0) {
        loadDefaultRules();
    }
    // --- get_rules ---
    else if (strcmp(cmd, "get_rules") == 0) {
        for (int i = 0; i < MAX_RULES; i++) publishRule(i);
        mqttClient.loop();
        Serial.println("[Cmd] Rules published");
    }
    // --- set_thresholds (convenience: update thresholds + reload default rules) ---
    else if (strcmp(cmd, "set_thresholds") == 0) {
        if (doc["soglia_umidita"].is<float>()) rtc_soglia_umidita = doc["soglia_umidita"];
        if (doc["temp_min_ext"].is<float>())   rtc_temp_min_ext   = doc["temp_min_ext"];
        if (doc["temp_max_ext"].is<float>())   rtc_temp_max_ext   = doc["temp_max_ext"];
        loadDefaultRules();  // Rebuild default rules with new thresholds
        Serial.printf("[Cmd] Thresholds: S=%.1f Tmin=%.1f Tmax=%.1f (rules reloaded)\n",
                      rtc_soglia_umidita, rtc_temp_min_ext, rtc_temp_max_ext);
    }
    // --- reset_thresholds ---
    else if (strcmp(cmd, "reset_thresholds") == 0) {
        rtc_soglia_umidita = SOGLIA_UMIDITA;
        rtc_temp_min_ext   = TEMP_MIN_ESTERNO;
        rtc_temp_max_ext   = TEMP_MAX_ESTERNO;
        loadDefaultRules();
        Serial.println("[Cmd] Thresholds + rules reset to defaults");
    }
    // --- reboot ---
    else if (strcmp(cmd, "reboot") == 0) {
        Serial.println("[Cmd] Rebooting!");
        mqttClient.disconnect();
        delay(200);
        ESP.restart();
    }
    else {
        Serial.printf("[Cmd] Unknown: %s\n", cmd);
    }
}

// =============================================================
//  Network Task (Core 0)
// =============================================================

void networkTask(void* param) {
    bool network_ok = false;
    if (connectWiFi()) {
        syncNTP();
        if (connectMQTT()) network_ok = true;
    }
    xSemaphoreGive(sem_wifi_ready);

    if (!network_ok) {
        xSemaphoreGive(sem_publish_done);
        vTaskDelete(NULL);
        return;
    }

    // Wait for sensor data + actuation, then publish
    if (xSemaphoreTake(sem_sensors_done, pdMS_TO_TICKS(15000)) == pdTRUE) {
        xSemaphoreTake(sem_actuation_done, pdMS_TO_TICKS(60000));
        publishJSON();
        mqttClient.loop(); delay(100); mqttClient.loop();

        // Listen for commands
        char cmdTopic[64];
        snprintf(cmdTopic, sizeof(cmdTopic), "%s/commands", MQTT_TOPIC_PREFIX);
        mqttClient.setCallback(mqttCallback);
        mqttClient.subscribe(cmdTopic);
        Serial.printf("[Core0] Listening %ds on %s\n", COMMAND_LISTEN_S, cmdTopic);

        unsigned long start = millis();
        while (millis() - start < (unsigned long)COMMAND_LISTEN_S * 1000UL) {
            mqttClient.loop();
            delay(50);
        }
        Serial.println("[Core0] Listen window closed.");
    } else {
        Serial.println("[Core0] Timeout waiting for sensor data!");
    }

    xSemaphoreGive(sem_publish_done);
    mqttClient.disconnect();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    vTaskDelete(NULL);
}

// =============================================================
//  Core 1: Sensors, Logic, Actuation
// =============================================================

void readSensors() {
    Serial.println("[Core1] Reading sensors...");
    sensoreEsterno->sns->begin();
    sensoreInterno->sns->begin();
    delay(2000);

    for (int i = 0; i < 3; i++) {
        float h = sensoreEsterno->getHumidity();
        if (!isnan(h) && h >= 0 && h <= 100) {
            g_humid_ext = h;
            g_temp_ext  = sensoreEsterno->getTemperature();
            Serial.printf("[Core1] Ext: T=%.1f H=%.1f\n", g_temp_ext, g_humid_ext);
            break;
        }
        delay(sensoreEsterno->delayus / 1000);
    }

    for (int i = 0; i < 3; i++) {
        float h = sensoreInterno->getHumidity();
        if (!isnan(h) && h >= 0 && h <= 100) {
            g_humid_int = h;
            g_temp_int  = sensoreInterno->getTemperature();
            Serial.printf("[Core1] Int: T=%.1f H=%.1f\n", g_temp_int, g_humid_int);
            break;
        }
        delay(sensoreInterno->delayus / 1000);
    }
}

void processLockTimers() {
    for (int i = 0; i < ACT_COUNT; i++) {
        if (rtc_lock_remaining_s[i] > 0) {
            rtc_lock_remaining_s[i] -= DEEP_SLEEP_TIME_S;
            if (rtc_lock_remaining_s[i] <= 0) {
                rtc_lock_remaining_s[i] = 0;
                Serial.printf("[Core1] Lock expired: %s\n", actuatorNames[i]);
            }
        }
    }
    // Set locked flags on relay objects
    rValvEsterno->locked         = (rtc_lock_remaining_s[ACT_VALVE_EXT]    > 0);
    rValvDeumidificatore->locked = (rtc_lock_remaining_s[ACT_VALVE_DEUM]   > 0);
    rVentolaDeum->locked         = (rtc_lock_remaining_s[ACT_FAN]          > 0);
    rDeumidificatore->locked     = (rtc_lock_remaining_s[ACT_DEHUMIDIFIER] > 0);
}

void runVMCLogic() {
    Serial.println("[Core1] Evaluating rules...");
    processLockTimers();

    // Evaluate all rules
    int8_t desired[ACT_COUNT];
    evaluateAllRules(desired);

    // Apply desired states (locked actuators are skipped by tryXxx)
    for (int i = 0; i < ACT_COUNT; i++) {
        if (desired[i] >= 0) {
            trySetActuatorState(i, desired[i] == 1);
        } else {
            // No rule set this actuator → keep previous state, sync g_ from rtc_
            switch (i) {
                case ACT_VALVE_EXT:    g_valve_ext_open  = rtc_valve_ext_open;  break;
                case ACT_VALVE_DEUM:   g_valve_deum_open = rtc_valve_deum_open; break;
                case ACT_FAN:          g_fan_on          = rtc_fan_on;          break;
                case ACT_DEHUMIDIFIER: g_dehumidifier_on = rtc_dehumidifier_on; break;
            }
        }
    }

    Serial.printf("[Core1] → VE=%s%s VD=%s%s Fan=%s%s Deum=%s%s\n",
        g_valve_ext_open  ? "O" : "C", rValvEsterno->locked ? "(L)" : "",
        g_valve_deum_open ? "O" : "C", rValvDeumidificatore->locked ? "(L)" : "",
        g_fan_on          ? "1" : "0", rVentolaDeum->locked ? "(L)" : "",
        g_dehumidifier_on ? "1" : "0", rDeumidificatore->locked ? "(L)" : "");
}

void enterDeepSleep() {
    Serial.printf("[Core1] Sleep %ds...\n", DEEP_SLEEP_TIME_S);
    Serial.flush();
    esp_sleep_enable_timer_wakeup((uint64_t)DEEP_SLEEP_TIME_S * 1000000ULL);
    esp_deep_sleep_start();
}

// =============================================================
//  setup() — Core 1
// =============================================================

void setup() {
    Serial.begin(115200);
    rtc_boot_count++;
    Serial.printf("\n\n=== VMC Wake #%u ===\n", rtc_boot_count);

    // Load default rules on first cold boot
    if (!rtc_rules_initialized) {
        loadDefaultRules();
    }

    // Semaphores
    sem_sensors_done   = xSemaphoreCreateBinary();
    sem_wifi_ready     = xSemaphoreCreateBinary();
    sem_actuation_done = xSemaphoreCreateBinary();
    sem_publish_done   = xSemaphoreCreateBinary();

    // Hardware init
    sensoreEsterno = new Sensor(SENSORE_ESTERNO_PIN);
    sensoreInterno = new Sensor(SENSORE_INTERNO_PIN);

    rValvEsterno         = new RelayMotore(EXTERN_VALVE_OPEN_PIN, EXTERN_VALVE_CLOSE_PIN, MOTOR_TIME, false, true);
    rValvDeumidificatore = new RelayMotore(DEUM_VALVE_OPEN_PIN, DEUM_VALVE_CLOSE_PIN, MOTOR_TIME, false, true);
    rVentolaDeum         = new SingleRelay(FAN_PIN, 0, true);
    rDeumidificatore     = new SingleRelay(DEHUMIDIFIER_PIN, 0, true);

    digitalWrite(FAN_PIN, rtc_fan_on ? LOW : HIGH);
    digitalWrite(DEHUMIDIFIER_PIN, rtc_dehumidifier_on ? LOW : HIGH);

    rValvEsterno->is_open         = rtc_valve_ext_open;
    rValvDeumidificatore->is_open = rtc_valve_deum_open;
    rVentolaDeum->is_on           = rtc_fan_on;
    rDeumidificatore->is_on       = rtc_dehumidifier_on;

    // Launch Core 0 network task
    xTaskCreatePinnedToCore(networkTask, "Net", 16384, NULL, 1, NULL, 0);

    // Core 1: sense → evaluate → actuate
    readSensors();
    xSemaphoreGive(sem_sensors_done);
    runVMCLogic();
    xSemaphoreGive(sem_actuation_done);

    // Wait for Core 0 to finish
    Serial.println("[Core1] Waiting for network...");
    if (xSemaphoreTake(sem_publish_done, pdMS_TO_TICKS((COMMAND_LISTEN_S + 30) * 1000)) == pdTRUE) {
        Serial.println("[Core1] Network cycle done.");
    } else {
        Serial.println("[Core1] Network timeout.");
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
    }

    delay(100);
    enterDeepSleep();
}

void loop() {
    // Never reached — deep sleep restarts from setup()
}
