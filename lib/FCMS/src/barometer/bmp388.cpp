#include "bmp388.h"

bool BMP388::setup()
{
  bmp_.begin();
  bmp_.setTimeStandby(TIME_STANDBY_1280MS);
  bmp_.startNormalConversion();

  return calibrate();
}

void BMP388::update()
{
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
      double time = (time_now - start_time) / 1000.0;
      double decline_time = time - climb_duration;

      if (time <= climb_duration)
      {
        altitude_ = max_alt * (time / climb_duration) * (time / climb_duration);
      }
      else if (decline_time <= decline_duration)
      {
        double descent_progress = decline_time / decline_duration;
        altitude_ = max_alt * (1 - descent_progress * descent_progress);
      }
      else
      {
        altitude_ = 0;
      }

      Serial.print("Time: ");
      Serial.print(time);
      Serial.print(" | Altitude: ");
      Serial.println(altitude_);
    }
  }
  else
  {
    bmp_.getAltitude(altitude_);
    altitude_ = altitude_ - altitudeCalibration_;
  }
#else
  bmp_.getAltitude(altitude_);
  altitude_ = altitude_ - altitudeCalibration_;
#endif
  detectApogee();
}

bool BMP388::calibrate()
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
    Serial.print("Calibrated coefficient: ");
    Serial.println(altitudeCalibration_);
  }
  else
  {
    Serial.println("Calibration failed.");
    return false;
  }

  return altitudeCalibration_ > 100 && altitudeCalibration_ < 500;
}

void BMP388::printAltitude()
{
  Serial.print(F("Approx altitude = "));
  Serial.print(altitude_);
  Serial.println(" m\n");
}

float BMP388::getAltitude()
{
  return altitude_;
}

void BMP388::detectApogee()
{
  static float lastAltitude = 0;

  if (altitude_ > maxapogee)
  {
    maxapogee = altitude_;
  }
  if (altitude_ > minApogeeAlt && (altitude_ + minDropThreshold < maxapogee) && climbing)
  {
    apogeeDetected = true;
    Serial.printf("388 Apogee detected: %f\n", altitude_);
    Serial.printf("Max apogee: %f\n", maxapogee);
    Serial.printf("Last apogee: %f\n", lastAltitude);
    climbing = false;
  }
  // else if (altitude_ > lastAltitude + minDropThreshold)
  // {
  //   climbing = true;
  // }
  lastAltitude = altitude_;
}
