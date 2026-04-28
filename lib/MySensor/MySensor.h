#pragma once
#ifndef SENSOR_H
#define SENSOR_H

#include <DHT_U.h>

#define CANC_NUM 999.00f

class Sensor{
    public:
    DHT_Unified* sns;
    int pin;
    int delayus;
    Sensor(int pin);
    Sensor(int pin, const uint8_t type);

    //tim timer;
    float getTemperature();
    float getHumidity();

};


#endif