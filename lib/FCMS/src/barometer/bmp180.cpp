#include "bmp180.h"

void BMP180::calibrate()
{
  Serial.print("Calibrating BMP180...");

  // might take a bit long time
  for (size_t i = 0; i < 500; ++i) {
    altitudeCalibration_ += bmp_.readAltitude();
    delay(1);
  }


  altitudeCalibration_ /= 500;

  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
}

void BMP180::setup()
{
  if (!bmp_.begin()) {
    Serial.println("Could not find a valid BMP180 sensor, check wiring!");
    while (1) {}
  }

  calibrate();
}

void BMP180::printAltitude()
{
  altitude_ = bmp_.readAltitude() - altitudeCalibration_;
  Serial.print("Altitude = ");
  Serial.print(altitude_);
  Serial.println(" meters");
}

float BMP180::getAltitude()
{
  altitude_ = bmp_.readAltitude() - altitudeCalibration_;
  return altitude_;
}
