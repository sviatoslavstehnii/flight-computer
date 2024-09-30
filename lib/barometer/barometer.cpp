#include "barometer.h"

void Barometer::setup()
{
  if (!bmp_.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                "try a different address!"));
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp_.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void Barometer::calibrate()
{
  Serial.print("Calibrating barometer...");
  size_t n = 0;
  for (size_t i = 0; i < 2000; ++i) {
    if (bmp_.takeForcedMeasurement()) {
      altitudeCalibration_ += bmp_.readAltitude(1013.25);
      ++n;
    }
    delay(1);
  }

  altitudeCalibration_ /= n;
}

void Barometer::printAltitide()
{
  if (bmp_.takeForcedMeasurement()) {
    altitude_ = bmp_.readAltitude(1013.25) - altitudeCalibration_;
    Serial.print(F("Approx altitude = "));
    Serial.print(altitude_);
    Serial.println(" m\n");
  }



}

float Barometer::getAltitude()
{
  return altitude_;
}
