#pragma once

class Barometer {
public:
    virtual bool setup() = 0;
    virtual float getAltitude() = 0;
    virtual void update() = 0;
    virtual ~Barometer() = default;
};