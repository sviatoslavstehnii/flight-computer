#include "loadcell.h"

void LoadCell::setup(float calibration_factor=8.25f)
{
  Serial.println("Begin Setup LoadCell");
  Serial.println("Begin LoadCell Warmup");

  loadCell_.begin(128);
  //LoadCell.setReverseOutput(); //uncomment to turn a negative output value to positive
  unsigned long stabilizingtime = 5000;
  bool _tare = true;
  Serial.println("Taring LoadCell");
  loadCell_.start(stabilizingtime, _tare);
  Serial.println("Taring LoadCell complete");
  if (loadCell_.getTareTimeoutFlag() || loadCell_.getSignalTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    Serial.print("Calibration factor: ");
    Serial.println(calibration_factor);
    loadCell_.setCalFactor(calibration_factor);
    Serial.println("LoadCell Warmup is complete");
  }
  while (!loadCell_.update()){
    Serial.println("Waiting for loadcell update");
    delay(100);
  }
  Serial.println("Loadcell update received");

  for (int i = 0; i < 10; i++) {
    float weight = loadCell_.getData();
    Serial.print("Weight: ");
    Serial.println(weight);
    delay(100);
  }
  Serial.println("Loadcell setup complete");
  delay(100);
}

float LoadCell::getWeight()
{
  return loadCell_.getData();
}

void LoadCell::tare()
{
  Serial.println("Manually Taring LoadCell...");
  loadCell_.tare();
  Serial.println("Manual Taring LoadCell complete");
}

void LoadCell::calibrate(float known_mass)
{
  Serial.println("Calibrating LoadCell...");

  Serial.print("Known mass is: ");
  Serial.println(known_mass);

  float newCalibrationValue = loadCell_.getNewCalibration(known_mass);

  Serial.print("New LoadCell calibration value has been set to: ");
  Serial.println(newCalibrationValue);
  loadCell_.setCalFactor(newCalibrationValue);

  Serial.println("LoadCell Calibration complete");
}

void LoadCell::setCalFactor(float cal_factor)
{
  Serial.print("Setting LoadCell calibration factor to: ");
  Serial.println(cal_factor);
  loadCell_.setCalFactor(cal_factor);
  for (int i = 0; i < 10; i++) {
    float weight = loadCell_.getData();
    Serial.print("Weight: ");
    Serial.println(weight);
    delay(100);
  }
}