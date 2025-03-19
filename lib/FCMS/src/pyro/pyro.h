#include <Arduino.h>

class PyroDriver
{
private:
    uint8_t _pyro_pin;
    bool armed = false;
    bool fired = false;

public:
    PyroDriver(uint8_t pyro_pin) : _pyro_pin(pyro_pin), armed(false)
    {
        pinMode(_pyro_pin, OUTPUT);
        digitalWrite(_pyro_pin, LOW);
    }

    void fire()
    {
        if (armed)
        {
            digitalWrite(_pyro_pin, HIGH);
            fired = true;
        }
        else
        {
            Serial.println("Tried to fire pyro while disarmed");
        }
    }

    void arm()
    {
        digitalWrite(_pyro_pin, LOW);
        armed = true;
        Serial.println("Armed pyro");
    }

    void disarm()
    {
        digitalWrite(_pyro_pin, LOW);
        armed = false;
        fired = false;
        Serial.println("Disarmed pyro");
    }

    bool isArmed() { return armed; }
    bool isFired() { return fired; }
};