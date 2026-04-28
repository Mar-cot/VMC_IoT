#include "Relay.h"
#include <Arduino.h>

int relay_number = 0;

RelayMotore::RelayMotore(int open_pin, int close_pin, int motion_duration_ms, bool initially_open){
    this->open_pin = open_pin;
    this->close_pin = close_pin;
    this->motion_duration = motion_duration_ms;
    this->id = ++relay_number;
    pinMode(open_pin, OUTPUT);
    pinMode(close_pin, OUTPUT);
    digitalWrite(open_pin, HIGH);
    digitalWrite(close_pin, HIGH);
    if(initially_open){
        is_open = false;
        this->openValve();
    }
    else{
        is_open = true;
        this->closeValve();
    }
}

void RelayMotore::openValve(){
    if(is_open) return;
    is_open = true;
    digitalWrite(open_pin, LOW);
    delay(motion_duration);
    digitalWrite(open_pin, HIGH);
}

void RelayMotore::closeValve(){
    if(!is_open) return;
    is_open = false;
    digitalWrite(close_pin, LOW);
    delay(motion_duration);
    digitalWrite(close_pin, HIGH);
}

void RelayMotore::invertValve(){
    if(is_open) closeValve();
    else openValve();
}







SingleRelay::SingleRelay(int pin, int delay_ms = 0){
    this->pin = pin;
    this->delay_ms = delay_ms;
    this->id = ++relay_number;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
    this->is_on = false;
}

void SingleRelay::turnOn(){
    if(is_on) return;
    is_on = true;
    digitalWrite(pin, LOW);
    if(delay_ms)
        delay(delay_ms);

    return;
}

void SingleRelay::turnOff(){
    if(!is_on) return;
    is_on = false;
    digitalWrite(pin, HIGH);
    if(delay_ms)
        delay(delay_ms);

    return;
}

void SingleRelay::invertOnOff(){
    if(is_on) turnOff();
    else turnOn();
}



