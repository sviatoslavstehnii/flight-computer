#ifndef BMP280_H
#define BMP280_H
#include <Adafruit_BMP280.h>

#include "barometer.h"


class BMP280 {
  private:
    Adafruit_BMP280 bmp_;
    float altitude_ = 0.0;
    float altitudeCalibration_ = 0.0;

    void calibrate();

  
  public:
    BMP280() = default;
    ~BMP280() = default;

    BMP280(const BMP280&) = delete;
    BMP280& operator=(const BMP280&) = delete;

    void setup() ;

    void printAltitude();
    float getAltitude();
  
};

#endif