#include "bmp388.h"

void BMP388::setup()
{
  bmp_.begin();
  bmp_.setTimeStandby(TIME_STANDBY_1280MS);
  bmp_.startNormalConversion();

  calibrate();
}

void BMP388::update()
{
  bmp_.getAltitude(altitude_);
  detectApogee();
}

void BMP388::calibrate()
{
  Serial.print("Calibrating BMP388...");
  size_t n = 0;
  float sum = 0;
  float minValue = FLT_MAX;
  float maxValue = -FLT_MAX;

  for (size_t i = 0; i < 500; ++i)
  {
    if (bmp_.getAltitude(altitude_))
    {
      sum += altitude_;
      n++;
      if (altitude_ < minValue)
        minValue = altitude_;
      if (altitude_ > maxValue)
        maxValue = altitude_;
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

void BMP388::printAltitude()
{
  Serial.print(F("Approx altitude = "));
  Serial.print(altitude_);
  Serial.println(" m\n");
}

float BMP388::getAltitude()
{
  update();

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
  return altitude_;
#endif

  return altitude_ - altitudeCalibration_;
}

void BMP388::detectApogee()
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
