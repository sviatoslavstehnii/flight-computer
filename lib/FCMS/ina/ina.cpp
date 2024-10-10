#include "ina.h"

INA219Lib::INA219Lib(uint8_t addr) {
  _i2cAddr = addr;
}

bool INA219Lib::begin() {
  Wire.begin();
  
  uint16_t calibration = read16(0x05);
  return calibration != 0xFFFF; 
}

float INA219Lib::getBusVoltage() {
  int16_t raw = read16(0x02);
  return raw * 0.001;
}

float INA219Lib::getShuntVoltage() {
  int16_t raw = read16(0x01);
  return raw * 0.01;
}

float INA219Lib::getCurrent() {
  int16_t raw = read16(0x04);
  return raw * 0.1;
}

float INA219Lib::getPower() {
  int16_t raw = read16(0x03);
  return raw * 0.01;
}

int16_t INA219Lib::read16(uint8_t reg) {
  Wire.beginTransmission(_i2cAddr);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(_i2cAddr, (uint8_t)2);
  if (Wire.available() == 2) {
    int16_t value = Wire.read() << 8 | Wire.read();
    return value;
  }
  return 0xFFFF;
}
