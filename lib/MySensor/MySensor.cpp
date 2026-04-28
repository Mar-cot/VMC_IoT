#include "MySensor.h"
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>


Sensor::Sensor(int pin){
    sns = new DHT_Unified(pin, DHT22);
    sensor_t infoSensor;
    sns->temperature().getSensor(&infoSensor);
    sns->humidity().getSensor(&infoSensor);
    delayus = infoSensor.min_delay;
}

Sensor::Sensor(int pin, const uint8_t type){
    sns = new DHT_Unified(pin, type);
}

float Sensor::getTemperature(){
    sensors_event_t event;
    sns->temperature().getEvent(&event);
    if (isnan(event.temperature)) {
        Serial.println(F("Error reading temperature!"));
        return CANC_NUM;
    }
    return event.temperature;
}

float Sensor::getHumidity(){
    sensors_event_t event;
    sns->humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
        Serial.println(F("Error reading humidity!"));
        return CANC_NUM;
    }
    return event.relative_humidity;
}