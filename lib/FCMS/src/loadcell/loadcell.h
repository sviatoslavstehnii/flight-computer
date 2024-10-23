#ifndef LOADCELL_H
#define LOADCELL_H


#include "HX711_ADC.h"

class LoadCell {
  private:
    HX711_ADC loadCell_;
    float weight_ = 0;
  public:
    LoadCell(int HX711_DOUT=2, int HW711_SCK=1): loadCell_(HX711_DOUT, HW711_SCK) {};
    ~LoadCell() = default;

    void setup(float calibration_factor=8.25f);
    float getWeight();
    void tare();
    void calibrate(float known_mass);
    void setCalFactor(float cal_factor);
};

#endif