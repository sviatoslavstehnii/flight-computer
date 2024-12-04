#include <BMP388_DEV.h> 

#include <cfloat>


#include "barometer.h"


class BMP388: Barometer {
  private:
    BMP388_DEV bmp_;
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