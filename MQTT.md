# VMC IoT — MQTT Protocol Specification

This document describes the MQTT topics, published payloads, and command payloads for the VMC (Ventilazione Meccanica Controllata) IoT system. Use this as the definitive reference for building a frontend dashboard.

## System Overview

The ESP32 operates in a **deep-sleep cycle**: it wakes every ~210 seconds, reads sensors, evaluates logic rules, actuates relays/valves, publishes status via MQTT, listens for commands for ~10 seconds, then goes back to sleep. The MQTT broker address and topic prefix are configurable; the default prefix is `domotica/vmc`.

### MQTT Broker Connection

| Parameter | Default |
|-----------|---------|
| Client ID | `vmc-esp32` |
| Topic Prefix | `domotica/vmc` |
| QoS | 0 |
| Retain | `true` (for status and rules) |

---

## Published Topics

### 1. `domotica/vmc/status` — System Status (Retained)

Published once per wake cycle after sensor reads and actuation. This is the primary data source for a dashboard.

#### Full Payload Example

```json
{
  "timestamp": "2026-06-21 18:00:00",
  "boot": 42,
  "sensors": {
    "temp_ext": 22.50,
    "humid_ext": 55.00,
    "temp_int": 21.30,
    "humid_int": 48.00
  },
  "actuators": {
    "valve_ext": {
      "state": "APERTA",
      "locked": false,
      "lock_s": 0
    },
    "valve_deum": {
      "state": "CHIUSA",
      "locked": true,
      "lock_s": 1590
    },
    "fan": {
      "state": "OFF",
      "locked": false,
      "lock_s": 0
    },
    "dehumidifier": {
      "state": "OFF",
      "locked": false,
      "lock_s": 0
    }
  },
  "thresholds": {
    "soglia_umidita": 59.00,
    "temp_min_ext": 12.00,
    "temp_max_ext": 27.00
  },
  "active_rules": 8
}
```

#### Field Reference

| Field | Type | Description |
|-------|------|-------------|
| `timestamp` | `string` | NTP timestamp in `YYYY-MM-DD HH:MM:SS` format (Europe/Rome timezone, UTC+1/+2). Empty string `""` if NTP sync failed. |
| `boot` | `integer` | Boot count since last full power loss. Increments by 1 each wake cycle. Useful to detect restarts. |

##### `sensors` Object

| Field | Type | Unit | Range | Description |
|-------|------|------|-------|-------------|
| `temp_ext` | `float` | °C | -40 to 80 | External temperature (DHT22). Value is `999.00` if sensor read failed. |
| `humid_ext` | `float` | % | 0 to 100 | External relative humidity (DHT22). Value is `999.00` if sensor read failed. |
| `temp_int` | `float` | °C | -40 to 80 | Internal temperature (DHT22). Value is `999.00` if sensor read failed. |
| `humid_int` | `float` | % | 0 to 100 | Internal relative humidity (DHT22). Value is `999.00` if sensor read failed. |

> **Important**: The value `999.00` is a sentinel indicating a sensor read failure. The dashboard should display this as "Sensor Error" or "N/A", not as an actual reading.

##### `actuators` Object

Contains one sub-object per actuator. The keys are the actuator identifiers.

| Actuator Key | Type | Description |
|-------------|------|-------------|
| `valve_ext` | Motorized valve | External air intake valve. Controls outdoor airflow. |
| `valve_deum` | Motorized valve | Dehumidifier circuit valve. Routes air through dehumidifier. |
| `fan` | Relay | Dehumidifier recirculation fan. |
| `dehumidifier` | Relay | Dehumidifier compressor unit. |

Each actuator sub-object has:

| Field | Type | Values | Description |
|-------|------|--------|-------------|
| `state` | `string` | `"APERTA"` / `"CHIUSA"` (valves) or `"ON"` / `"OFF"` (relays) | Current actuator state. `APERTA` = open, `CHIUSA` = closed. |
| `locked` | `boolean` | `true` / `false` | Whether the actuator is in manual override mode. When locked, automatic logic rules cannot change its state. |
| `lock_s` | `integer` | ≥ 0 | Remaining lock duration in seconds. `0` when not locked. Decreases by ~210s each wake cycle. |

##### `thresholds` Object

Current runtime thresholds used by the default logic rules. These can be updated via the `set_thresholds` command.

| Field | Type | Unit | Default | Description |
|-------|------|------|---------|-------------|
| `soglia_umidita` | `float` | % | 59.0 | Humidity threshold. External air is considered acceptable if humidity ≤ this value. Internal dehumidification activates at `soglia_umidita - 10`. |
| `temp_min_ext` | `float` | °C | 12.0 | Minimum acceptable external temperature for ventilation. |
| `temp_max_ext` | `float` | °C | 27.0 | Maximum acceptable external temperature for ventilation. |

| Field | Type | Description |
|-------|------|-------------|
| `active_rules` | `integer` | Number of active logic rules (out of 12 max slots). |

---

### 2. `domotica/vmc/rules/{N}` — Rule Details (Retained)

Published in response to a `get_rules` command. One message per rule slot, where `{N}` is `0` through `11`.

#### Example: Active Rule

```json
{
  "idx": 0,
  "active": true,
  "terms": [
    {"var": "humid_ext", "op": "<=", "val": 59.0},
    {"var": "temp_ext", "op": ">=", "val": 12.0},
    {"var": "temp_ext", "op": "<=", "val": 27.0},
    {"var": "humid_int", "op": ">=", "val": 49.0}
  ],
  "effects": [
    {"act": "valve_ext", "do": "on"},
    {"act": "valve_deum", "do": "on"},
    {"act": "fan", "do": "on"},
    {"act": "dehumidifier", "do": "on"}
  ]
}
```

#### Example: Empty/Inactive Rule Slot

```json
{
  "idx": 9,
  "active": false,
  "terms": [],
  "effects": []
}
```

#### Field Reference

| Field | Type | Description |
|-------|------|-------------|
| `idx` | `integer` | Rule slot index (0–11). |
| `active` | `boolean` | Whether this rule is evaluated. Inactive/empty rules are skipped. |

##### `terms` Array (max 5 elements)

Each term is a condition. ALL terms must be true (AND logic) for the rule to match.

| Field | Type | Values | Description |
|-------|------|--------|-------------|
| `var` | `string` | `"temp_ext"`, `"humid_ext"`, `"temp_int"`, `"humid_int"` | The sensor variable to evaluate. |
| `op` | `string` | `">"`, `">="`, `"<"`, `"<="`, `"=="` | Comparison operator. |
| `val` | `float` | any | Threshold value to compare against. |

If a sensor variable has an invalid reading (999.00 / NaN), any term referencing it evaluates to `false`, causing the entire rule to not match. This is a safety mechanism — actuators keep their previous state when sensors fail.

##### `effects` Array (max 4 elements)

Actions to apply when all terms evaluate to `true`.

| Field | Type | Values | Description |
|-------|------|--------|-------------|
| `act` | `string` | `"valve_ext"`, `"valve_deum"`, `"fan"`, `"dehumidifier"` | Target actuator. |
| `do` | `string` | `"on"` / `"open"`, `"off"` / `"close"` | Desired state. `on`/`open` are equivalent (open valve / turn on relay). `off`/`close` are equivalent. |

#### Rule Evaluation Behavior

- All 12 rule slots are evaluated in order (0 → 11) on every wake cycle.
- **All matching rules apply**. If multiple rules match and set the same actuator, the **last matching rule wins** for that actuator.
- Actuators not set by any matching rule **keep their previous state**.
- Locked actuators (manual override) are **not affected** by rule evaluation.
- Rules 0–7 are populated with default VMC logic on first boot. Rules 8–11 are empty and available for custom rules.

---

## Command Topic

### `domotica/vmc/commands` — Incoming Commands

All commands are JSON messages published to this topic. The ESP32 listens for commands for a short window (~10 seconds) after publishing its status each cycle.

> **Important timing**: The ESP32 is only awake for ~15–30 seconds per cycle (depending on sensor read time and valve actuation). Commands are only processed during the listen window. Commands published while the ESP32 is asleep will be processed on the next wake cycle **only if the MQTT broker does not have a retained message on this topic** — it is recommended to publish commands with `retain: false`.

---

## Command Reference

### Actuator Control

#### `set_actuator` — Force Actuator State with Lock

Forces an actuator to a specific state and locks it for a duration, preventing automatic rules from changing it.

```json
{
  "cmd": "set_actuator",
  "target": "valve_ext",
  "state": "open",
  "lock_s": 3600
}
```

| Field | Type | Required | Values | Description |
|-------|------|----------|--------|-------------|
| `cmd` | `string` | ✅ | `"set_actuator"` | Command identifier. |
| `target` | `string` | ✅ | `"valve_ext"`, `"valve_deum"`, `"fan"`, `"dehumidifier"` | Actuator to control. |
| `state` | `string` | ✅ | `"open"` / `"on"`, `"close"` / `"off"` | Desired state. For valves use open/close, for relays use on/off (both forms accepted for all actuators). |
| `lock_s` | `integer` | ❌ | > 0 | Lock duration in seconds. **Default: 1800** (30 minutes). The lock countdown decreases by ~210s each wake cycle. |

**Effect**: The actuator is immediately driven to the requested state. Automatic rule evaluation will skip this actuator until the lock expires. The lock status is visible in the `actuators` section of the status payload.

---

#### `unlock` — Release Actuator Lock

Releases the lock on a single actuator, allowing automatic rules to control it again on the next cycle.

```json
{
  "cmd": "unlock",
  "target": "valve_ext"
}
```

| Field | Type | Required | Values | Description |
|-------|------|----------|--------|-------------|
| `cmd` | `string` | ✅ | `"unlock"` | Command identifier. |
| `target` | `string` | ✅ | `"valve_ext"`, `"valve_deum"`, `"fan"`, `"dehumidifier"` | Actuator to unlock. |

---

#### `unlock_all` — Release All Locks

Releases locks on all actuators at once.

```json
{
  "cmd": "unlock_all"
}
```

---

### Rule Management

#### `set_rule` — Create or Replace a Rule

Sets a rule at a specific slot index. Completely replaces any existing rule at that index.

```json
{
  "cmd": "set_rule",
  "idx": 8,
  "terms": [
    {"var": "temp_ext", "op": ">", "val": 30.0},
    {"var": "humid_int", "op": ">=", "val": 60.0}
  ],
  "effects": [
    {"act": "fan", "do": "on"},
    {"act": "dehumidifier", "do": "on"}
  ]
}
```

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `cmd` | `string` | ✅ | `"set_rule"` |
| `idx` | `integer` | ✅ | Rule slot index (0–11). Overwrites existing rule at this slot. |
| `terms` | `array` | ✅ | Array of condition objects (max 5). All must be true for the rule to match (AND logic). |
| `effects` | `array` | ✅ | Array of effect objects (max 4). Applied when the rule matches. |

**Term object fields:**

| Field | Type | Required | Values |
|-------|------|----------|--------|
| `var` | `string` | ✅ | `"temp_ext"`, `"humid_ext"`, `"temp_int"`, `"humid_int"` |
| `op` | `string` | ✅ | `">"`, `">="`, `"<"`, `"<="`, `"=="` |
| `val` | `float` | ✅ | Comparison threshold value |

**Effect object fields:**

| Field | Type | Required | Values |
|-------|------|----------|--------|
| `act` | `string` | ✅ | `"valve_ext"`, `"valve_deum"`, `"fan"`, `"dehumidifier"` |
| `do` | `string` | ✅ | `"on"` / `"open"`, `"off"` / `"close"` |

> **Tip**: Default rules occupy slots 0–7. Use slots 8–11 for custom rules that layer on top of the defaults. Since later matching rules take priority per actuator, a custom rule in slot 10 will override a default rule in slot 2 if both match.

---

#### `clear_rule` — Deactivate a Rule Slot

Clears a rule slot, making it inactive.

```json
{
  "cmd": "clear_rule",
  "idx": 8
}
```

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `cmd` | `string` | ✅ | `"clear_rule"` |
| `idx` | `integer` | ✅ | Rule slot index to clear (0–11). |

---

#### `reset_rules` — Reset All Rules to Defaults

Clears all 12 rule slots and reloads the 8 default VMC rules (using current threshold values).

```json
{
  "cmd": "reset_rules"
}
```

---

#### `get_rules` — Query All Rules

Triggers the ESP32 to publish all 12 rule slots as individual retained messages to `domotica/vmc/rules/0` through `domotica/vmc/rules/11`.

```json
{
  "cmd": "get_rules"
}
```

**Response**: 12 messages published to `domotica/vmc/rules/{0..11}` (see the Rules topic format above).

---

### Threshold Management

#### `set_thresholds` — Update Thresholds and Reload Default Rules

Updates one or more threshold values and **reloads the default rules** (slots 0–7) using the new threshold values. Any custom rules in slots 8–11 are preserved.

> **Important**: This command rebuilds the default rule table. If you have manually modified rules in slots 0–7 via `set_rule`, those modifications will be overwritten.

```json
{
  "cmd": "set_thresholds",
  "soglia_umidita": 55.0,
  "temp_min_ext": 10.0,
  "temp_max_ext": 30.0
}
```

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| `cmd` | `string` | ✅ | | `"set_thresholds"` |
| `soglia_umidita` | `float` | ❌ | 59.0 | Humidity threshold (%). |
| `temp_min_ext` | `float` | ❌ | 12.0 | Minimum external temperature (°C). |
| `temp_max_ext` | `float` | ❌ | 27.0 | Maximum external temperature (°C). |

Any subset of fields can be provided; only provided fields are updated.

---

#### `reset_thresholds` — Reset Thresholds and Rules to Compile-time Defaults

Resets all three thresholds to their compile-time default values and reloads the default rules.

```json
{
  "cmd": "reset_thresholds"
}
```

---

### System

#### `reboot` — Restart the ESP32

Triggers an immediate restart (not deep sleep). The ESP32 will reconnect and resume normal operation. All RTC-persisted state (rules, thresholds, locks, actuator states) is **preserved** across reboots but **lost on full power loss**.

```json
{
  "cmd": "reboot"
}
```

---

## Default Logic Rules (Slots 0–7)

These rules encode the standard VMC ventilation logic. They are loaded on first boot and can be reloaded via `reset_rules` or `set_thresholds`.

Let `S` = `soglia_umidita` (default 59.0), `Tmin` = `temp_min_ext` (default 12.0), `Tmax` = `temp_max_ext` (default 27.0), `Si` = `S - 10` (default 49.0).

| Slot | Conditions (all AND) | Effects |
|------|---------------------|---------|
| 0 | `humid_ext ≤ S` AND `temp_ext ≥ Tmin` AND `temp_ext ≤ Tmax` AND `humid_int ≥ Si` | valve_ext=OPEN, valve_deum=OPEN, fan=ON, dehumidifier=ON |
| 1 | `humid_ext ≤ S` AND `temp_ext ≥ Tmin` AND `temp_ext ≤ Tmax` AND `humid_int < Si` | valve_ext=OPEN, valve_deum=CLOSE |
| 2 | `humid_ext > S` AND `humid_int ≥ Si` | valve_ext=CLOSE, valve_deum=CLOSE, fan=ON, dehumidifier=ON |
| 3 | `humid_ext > S` AND `humid_int < Si` | valve_ext=CLOSE, valve_deum=CLOSE, dehumidifier=OFF |
| 4 | `temp_ext < Tmin` AND `humid_int ≥ Si` | valve_ext=CLOSE, valve_deum=CLOSE, fan=ON, dehumidifier=ON |
| 5 | `temp_ext < Tmin` AND `humid_int < Si` | valve_ext=CLOSE, valve_deum=CLOSE, dehumidifier=OFF |
| 6 | `temp_ext > Tmax` AND `humid_int ≥ Si` | valve_ext=CLOSE, valve_deum=CLOSE, fan=ON, dehumidifier=ON |
| 7 | `temp_ext > Tmax` AND `humid_int < Si` | valve_ext=CLOSE, valve_deum=CLOSE, dehumidifier=OFF |

Slots 8–11 are empty on first boot and available for custom rules.

---

## Data Types Summary

| Identifier | Type | Possible Values | Context |
|-----------|------|-----------------|---------|
| Sensor Variable | `string` | `temp_ext`, `humid_ext`, `temp_int`, `humid_int` | Rule terms, status payload |
| Operator | `string` | `>`, `>=`, `<`, `<=`, `==` | Rule terms |
| Actuator Name | `string` | `valve_ext`, `valve_deum`, `fan`, `dehumidifier` | Commands, status payload, rule effects |
| Valve State | `string` | `APERTA` (open), `CHIUSA` (closed) | Status payload |
| Relay State | `string` | `ON`, `OFF` | Status payload |
| Action | `string` | `on` / `open`, `off` / `close` | Rule effects, set_actuator command |
| Sensor Error | `float` | `999.00` | Sentinel value for failed sensor reads |

---

## Timing Constraints

| Parameter | Default Value | Description |
|-----------|---------------|-------------|
| Wake Cycle | 210 seconds | Time between consecutive wake-sense-publish-sleep cycles |
| Command Listen Window | 10 seconds | Duration the ESP32 listens for MQTT commands after publishing status |
| Default Actuator Lock | 1800 seconds | Default lock duration when `lock_s` is not specified in `set_actuator` |
| WiFi Timeout | 10 seconds | Max time to wait for WiFi connection before skipping the network phase |
| Total Awake Time | ~15–40 seconds | Depends on sensor read retries, valve actuation (15s per motorized valve), and listen window |

---

## Topic Summary

| Topic | Direction | Retained | Description |
|-------|-----------|----------|-------------|
| `domotica/vmc/status` | ESP32 → Broker | ✅ | System status (sensors, actuators, thresholds) |
| `domotica/vmc/rules/{0-11}` | ESP32 → Broker | ✅ | Individual rule details (published on `get_rules`) |
| `domotica/vmc/commands` | Dashboard → ESP32 | ❌ | Command input |
