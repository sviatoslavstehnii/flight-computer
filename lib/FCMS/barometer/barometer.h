#pragma once

class Barometer {
public:
    virtual void setup() = 0;
    virtual float getAltitude() = 0;
    virtual void printAltitude() = 0;
    virtual ~Barometer() = default;
};