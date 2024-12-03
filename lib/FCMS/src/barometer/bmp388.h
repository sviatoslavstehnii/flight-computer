#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <cfloat>


#define SEALEVELPRESSURE_HPA (1013.25)



#include "barometer.h"


class BMP388: Barometer {
  private:
    Adafruit_BMP3XX bmp_;
    float altitude_ = 0.0;
    float altitudeCalibration_ = 0.0;
    float maxapogee = -FLT_MAX;

    bool apogeeDetected = false;

    void calibrate();
    void detectApogee();

  public:
    BMP388() = default;
    ~BMP388() = default;

    BMP388(const BMP388&) = delete;
    BMP388& operator=(const BMP388&) = delete;

    void setup() override;

    void update();

    void printAltitude() override;
    float getAltitude() override;

    bool getApogeeDetected() { return apogeeDetected;}
    float getMaxApogee() { return maxapogee; }
};