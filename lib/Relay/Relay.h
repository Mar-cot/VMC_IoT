#pragma once
#ifndef RELAY_H
#define RELAY_H


class RelayMotore{
    public:
    int open_pin;
    int close_pin;
    int motion_duration;
    bool is_open;
    bool locked;       // When true, automatic logic cannot change state
    int id;

    RelayMotore(int open_pin, int close_pin, int motion_duration, bool initially_open = false, bool skip_init = false);
    void openValve();
    void closeValve();
    void invertValve();

    // Lock-aware: only actuate if not locked. Returns true if actuation happened.
    bool tryOpenValve();
    bool tryCloseValve();
};

class SingleRelay{
    public:
    int pin;
    bool is_on;
    bool locked;       // When true, automatic logic cannot change state
    int delay_ms;
    int id;

    SingleRelay(int pin, int delay_ms, bool skip_init = false);
    void turnOn();
    void turnOff();
    void invertOnOff();

    // Lock-aware: only actuate if not locked. Returns true if actuation happened.
    bool tryTurnOn();
    bool tryTurnOff();
};

#endif