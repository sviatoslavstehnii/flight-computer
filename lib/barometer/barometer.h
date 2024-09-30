#include <Adafruit_BMP280.h>


class Barometer {
  private:
    Adafruit_BMP280 bmp_;
    float altitude_ = 0.0;
    float altitudeCalibration_ = 0.0;

    void calibrate();

  
  public:
    Barometer() = default;
    ~Barometer() = default;

    Barometer(const Barometer&) = delete;
    Barometer& operator=(const Barometer&) = delete;

    void setup();

    void printAltitide();
    float getAltitude();
  
};