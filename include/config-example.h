#pragma once
#ifndef CONFIG_H
#define CONFIG_H

// =============================================================
//  VMC IoT — Configuration Template
//  Copy this file to config.h and fill in your real values.
//  config.h is git-ignored, this file is tracked.
// =============================================================

// --- WiFi ---
#define WIFI_SSID          "YourSSID"
#define WIFI_PASSWORD      "YourPassword"
#define WIFI_TIMEOUT_MS    10000       // Max ms to wait for WiFi connection

// --- MQTT ---
#define MQTT_SERVER        "192.168.1.100"   // Your MQTT broker IP or hostname
#define MQTT_PORT          1883
#define MQTT_USER          ""                // Leave empty if no auth
#define MQTT_PASSWORD      ""
#define MQTT_CLIENT_ID     "vmc-esp32"
#define MQTT_TOPIC_PREFIX  "vmc"             // Topics: vmc/sensor/temp_ext, etc.

// --- Deep Sleep ---
#define DEEP_SLEEP_TIME_S  210               // Seconds between wake cycles
#define COMMAND_LISTEN_S   10                // Seconds to listen for MQTT commands after publish
#define ACTUATOR_LOCK_S    1800              // Default lock duration for manual actuator override (30 min)

// --- Pin Assignments ---
#define DEUM_VALVE_OPEN_PIN    19
#define DEUM_VALVE_CLOSE_PIN   18
#define EXTERN_VALVE_OPEN_PIN  14
#define EXTERN_VALVE_CLOSE_PIN 27
#define MOTOR_TIME             15000         // ms for motorised valve travel
#define FAN_PIN                25
#define DEHUMIDIFIER_PIN       26
#define SENSORE_ESTERNO_PIN    33
#define SENSORE_INTERNO_PIN    32

// --- VMC Thresholds ---
#define SOGLIA_UMIDITA     59.00f
#define TEMP_MIN_ESTERNO   12.00f
#define TEMP_MAX_ESTERNO   27.00f

#endif // CONFIG_H
