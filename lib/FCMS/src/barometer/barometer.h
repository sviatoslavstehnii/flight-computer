#ifndef BAROMETER_H
#define BAROMETER_H
class Barometer {
public:
    virtual void setup();
    virtual float getAltitude();
    virtual void printAltitude();
    virtual ~Barometer() = default;
};

#endif