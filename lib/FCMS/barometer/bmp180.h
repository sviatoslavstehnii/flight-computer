#include <Adafruit_BMP085.h>

#include "barometer.h"



class BMP180: Barometer {
  private:
    Adafruit_BMP085 bmp_;
    float altitude_ = 0.0;
    float altitudeCalibration_ = 0.0;

    void calibrate();

  
  public:
    BMP180() = default;
    ~BMP180() = default;

    BMP180(const BMP180&) = delete;
    BMP180& operator=(const BMP180&) = delete;

    void setup() override;

    void printAltitude() override;
    float getAltitude() override;
  
};