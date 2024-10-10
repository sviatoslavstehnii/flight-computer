#ifndef INA219LIB_H
#define INA219LIB_H

#include <Wire.h>

class INA219Lib {
  public:
    INA219Lib(uint8_t addr = 0x40);
    bool begin();
    float getBusVoltage();
    float getShuntVoltage();
    float getCurrent();
    float getPower();

  private:
    uint8_t _i2cAddr;
    int16_t read16(uint8_t reg);
};

#endif
