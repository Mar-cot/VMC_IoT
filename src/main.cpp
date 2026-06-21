#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Relay.h>
#include <MySensor.h>
#include "config.h"

// =============================================================
//  VMC IoT — Deep Sleep + MQTT + Dual-Core Architecture
//
//  Flow:  Wake → Sense (Core 1) + Connect (Core 0) → Publish → Act → Sleep
//
//  Core 1 (APP_CPU): setup/loop — sensor reads, VMC logic, actuator control
//  Core 0 (PRO_CPU): WiFi connection + MQTT publish (runs as one-shot task)
// =============================================================

// --- RTC Memory: persists across deep sleep ---
// First-boot defaults: ext valve open, deum valve closed, fan off, dehumidifier off
RTC_DATA_ATTR bool     rtc_valve_ext_open   = true;
RTC_DATA_ATTR bool     rtc_valve_deum_open  = false;
RTC_DATA_ATTR bool     rtc_fan_on           = false;
RTC_DATA_ATTR bool     rtc_dehumidifier_on  = false;
RTC_DATA_ATTR uint32_t rtc_boot_count       = 0;

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
    mqttClient.setBufferSize(512);
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

void publishJSON() {
    char topic[64];
    snprintf(topic, sizeof(topic), "%s/status", MQTT_TOPIC_PREFIX);

    // Build JSON payload
    char payload[512];
    snprintf(payload, sizeof(payload),
        "{\"boot\":%u,"
        "\"sensors\":{\"temp_ext\":%.2f,\"humid_ext\":%.2f,\"temp_int\":%.2f,\"humid_int\":%.2f},"
        "\"actuators\":{\"valve_ext\":\"%s\",\"valve_deum\":\"%s\",\"dehumidifier\":\"%s\",\"fan\":\"%s\"}}",
        rtc_boot_count,
        g_temp_ext, g_humid_ext, g_temp_int, g_humid_int,
        g_valve_ext_open  ? "APERTA" : "CHIUSA",
        g_valve_deum_open ? "APERTA" : "CHIUSA",
        g_dehumidifier_on ? "ON"     : "OFF",
        g_fan_on          ? "ON"     : "OFF"
    );

    mqttClient.publish(topic, payload, true);  // retained
    Serial.printf("[Core0] Published: %s\n", payload);
}

void networkTask(void* param) {
    // PHASE 1: Connect WiFi + MQTT while Core 1 reads sensors
    bool network_ok = false;
    if (connectWiFi()) {
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

void runVMCLogic() {
    Serial.println("[Core1] Running VMC logic...");

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
        bool ext_ok = (g_humid_ext <= SOGLIA_UMIDITA) &&
                      (g_temp_ext  >= TEMP_MIN_ESTERNO) &&
                      (g_temp_ext  <= TEMP_MAX_ESTERNO);

        if (ext_ok) {
            want_valve_ext_open = true;
            if (g_humid_int >= (SOGLIA_UMIDITA - 10)) {
                want_valve_deum_open = true;
                want_fan_on          = true;
                want_dehumidifier_on = true;
            } else {
                want_valve_deum_open = false;
                // Fan and dehumidifier: keep off if humidity is fine
            }
        } else {
            want_valve_ext_open  = false;
            want_valve_deum_open = false;  // Close deum valve when ext is closed
            if (g_humid_int != CANC_NUM && g_humid_int >= (SOGLIA_UMIDITA - 10)) {
                want_fan_on          = true;
                want_dehumidifier_on = true;
            } else {
                want_dehumidifier_on = false;
                // Fan stays in its current state unless dehumidifier changes
            }
        }
    }

    // --- Apply actuator changes ---
    // Only pulse the motorised valves if the desired state differs from current

    // External valve
    if (want_valve_ext_open && !rtc_valve_ext_open) {
        Serial.println("[Core1] Opening external valve...");
        rValvEsterno->is_open = false;  // Force state so openValve() will act
        rValvEsterno->openValve();
    } else if (!want_valve_ext_open && rtc_valve_ext_open) {
        Serial.println("[Core1] Closing external valve...");
        rValvEsterno->is_open = true;
        rValvEsterno->closeValve();
    }

    // Dehumidifier valve
    if (want_valve_deum_open && !rtc_valve_deum_open) {
        Serial.println("[Core1] Opening deum valve...");
        rValvDeumidificatore->is_open = false;
        rValvDeumidificatore->openValve();
    } else if (!want_valve_deum_open && rtc_valve_deum_open) {
        Serial.println("[Core1] Closing deum valve...");
        rValvDeumidificatore->is_open = true;
        rValvDeumidificatore->closeValve();
    }

    // Fan
    if (want_fan_on && !rtc_fan_on) {
        rVentolaDeum->is_on = false;
        rVentolaDeum->turnOn();
    } else if (!want_fan_on && rtc_fan_on) {
        rVentolaDeum->is_on = true;
        rVentolaDeum->turnOff();
    }

    // Dehumidifier
    if (want_dehumidifier_on && !rtc_dehumidifier_on) {
        rDeumidificatore->is_on = false;
        rDeumidificatore->turnOn();
    } else if (!want_dehumidifier_on && rtc_dehumidifier_on) {
        rDeumidificatore->is_on = true;
        rDeumidificatore->turnOff();
    }

    // Update shared state for MQTT publishing
    g_valve_ext_open  = want_valve_ext_open;
    g_valve_deum_open = want_valve_deum_open;
    g_fan_on          = want_fan_on;
    g_dehumidifier_on = want_dehumidifier_on;

    // Persist to RTC for next wake cycle
    rtc_valve_ext_open  = want_valve_ext_open;
    rtc_valve_deum_open = want_valve_deum_open;
    rtc_fan_on          = want_fan_on;
    rtc_dehumidifier_on = want_dehumidifier_on;

    Serial.printf("[Core1] State: ValvExt=%s ValvDeum=%s Fan=%s Deum=%s\n",
                  want_valve_ext_open  ? "OPEN"  : "CLOSED",
                  want_valve_deum_open ? "OPEN"  : "CLOSED",
                  want_fan_on          ? "ON"    : "OFF",
                  want_dehumidifier_on ? "ON"    : "OFF");
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

    // --- Wait for Core 0 to finish publishing ---
    Serial.println("[Core1] Waiting for MQTT publish to complete...");
    if (xSemaphoreTake(sem_publish_done, pdMS_TO_TICKS(30000)) == pdTRUE) {
        Serial.println("[Core1] MQTT publish complete.");
    } else {
        Serial.println("[Core1] Timeout waiting for MQTT publish, proceeding to sleep.");
        // Force WiFi cleanup
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
    }

    // --- PHASE 3: Deep sleep ---
    // Small delay to let serial flush
    delay(100);
    enterDeepSleep();
}

void loop() {
    // Never reached — deep sleep restarts from setup()
}
