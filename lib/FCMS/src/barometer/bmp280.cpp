#include "bmp280.h"

void BMP280::setup()
{
  if (!bmp_.begin(0x76))
  {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                     "try a different address!"));
    // while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp_.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                   Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                   Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                   Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                   Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  calibrate();
}

void BMP280::update()
{
  if (bmp_.takeForcedMeasurement())
  {
    getAltitude();
    detectApogee();
  }
}

void BMP280::calibrate()
{
  Serial.print("Calibrating BMP280...");
  size_t n = 0;
  float sum = 0;
  float minValue = FLT_MAX;
  float maxValue = -FLT_MAX;

  for (size_t i = 0; i < 500; ++i)
  {
    if (bmp_.takeForcedMeasurement())
    {
      float altitude = bmp_.readAltitude();
      sum += altitude;
      n++;
      if (altitude < minValue)
        minValue = altitude;
      if (altitude > maxValue)
        maxValue = altitude;
    }
    delay(10);
  }

  if (n > 0)
  {
    altitudeCalibration_ = sum / n;
    Serial.print("Calibrated altitude: ");
    Serial.println(altitudeCalibration_);
  }
  else
  {
    Serial.println("Calibration failed.");
  }
}

void BMP280::printAltitude()
{
  Serial.print(F("Approx altitude = "));
  Serial.print(altitude_);
  Serial.println(" m\n");
}

float BMP280::getAltitude()
{
  if (bmp_.takeForcedMeasurement())
  {
    altitude_ = bmp_.readAltitude() - altitudeCalibration_;
  }
#ifdef HITL_MODE
  if (hitl)
  {
    if (start_time == 0)
    {
      start_time = millis() + 1000;
    }
    time_now = millis();
    if (time_now > start_time)
    {
      double time = time_now - start_time;
      double decline_time = time - climb_duration;

      if (time <= climb_duration)
      {
        altitude_ = max_alt / sqrt(climb_duration / time);
      }
      else if (decline_time <= decline_duration)
      {
        altitude_ = max_alt * sqrt(decline_duration / decline_time);
      }
      else
      {
        altitude_ = 0;
      }
    }
  }
#endif

  return altitude_;
}

void BMP280::detectApogee()
{
  static float lastAltitude = 0;

  if (altitude_ > maxapogee)
  {
    maxapogee = altitude_;
  }
  if (altitude_ > minApogeeAlt && altitude_ < maxapogee - minDropThreshold && climbing)
  {
    apogeeDetected = true;
    Serial.println(maxapogee);
    climbing = false;
  }
  else if (altitude_ > lastAltitude + minDropThreshold)
  {
    climbing = true;
  }
  lastAltitude = altitude_;
}
