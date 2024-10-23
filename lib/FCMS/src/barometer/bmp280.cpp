#include "bmp280.h"

void BMP280::setup()
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

  calibrate();
}

void BMP280::calibrate()
{
  Serial.print("Calibrating BMP280...");
  size_t n = 0;
  for (size_t i = 0; i < 500; ++i) {
    if (bmp_.takeForcedMeasurement()) {
      altitudeCalibration_ += bmp_.readAltitude();
      ++n;
    }
    delay(1);
  }

  altitudeCalibration_ /= n;

  Serial.print("clibr: ");
  Serial.println(altitudeCalibration_);
}

void BMP280::printAltitude()
{
  if (bmp_.takeForcedMeasurement()) {
    altitude_ = bmp_.readAltitude() - altitudeCalibration_;
    Serial.print(F("Approx altitude = "));
    Serial.print(altitude_);
    Serial.println(" m\n");
  }



}

float BMP280::getAltitude()
{
  if (bmp_.takeForcedMeasurement()) {
    altitude_ = bmp_.readAltitude() - altitudeCalibration_;
  }
  return altitude_;
}
