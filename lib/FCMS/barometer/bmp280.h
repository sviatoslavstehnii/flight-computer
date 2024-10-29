#include <Adafruit_BMP280.h>
#include <cfloat>

#include "barometer.h"


class BMP280: Barometer {
  private:
    Adafruit_BMP280 bmp_;
    float altitude_ = 0.0;
    float altitudeCalibration_ = 0.0;
    
    float maxapogee = -FLT_MAX;

    void calibrate();

  
  public:
    BMP280() = default;
    ~BMP280() = default;

    BMP280(const BMP280&) = delete;
    BMP280& operator=(const BMP280&) = delete;

    void setup() override;

    void update();

    void printAltitude() override;
    float getAltitude() override;
    void apogeeDetection();


};