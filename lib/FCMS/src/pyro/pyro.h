#ifndef PYRO_H
#define PYRO_H
// Pyro channel class for firing the motor / deploying the parachute
#include "Arduino.h"


enum PyroState{
    DISARMED=0,
    ARMED=1,
    FIRING=2,
    CONTINUITY=3
};

class Pyro {
  public:
    Pyro(int pin): _pin(pin) {
        pinMode(_pin, OUTPUT);
        digitalWrite(_pin, LOW);
        _state = DISARMED;
    };
    ~Pyro() = default;

    bool check_continuity();
    bool fire();
    bool arm();
    bool disarm();
    PyroState getState() { return _state; }
  private:
    int _pin;
    PyroState _state = DISARMED;
};

#endif