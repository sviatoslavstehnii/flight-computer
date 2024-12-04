#include "bmp388.h"


void BMP388::setup()
{
  bmp_.begin();                                 // Default initialisation, place the BMP388 into SLEEP_MODE 
  bmp_.setTimeStandby(TIME_STANDBY_1280MS);
  bmp_.startNormalConversion();                 // Start BMP388 continuous conversion in NORMAL_MODE  

  calibrate();
}


void BMP388::update()
{
  bmp_.getAltitude(altitude_);
}


void BMP388::calibrate()
{
  Serial.print("Calibrating BMP388...");
  size_t n = 0;
  float sum = 0;
  float minValue = FLT_MAX;
  float maxValue = -FLT_MAX;

  for (size_t i = 0; i < 500; ++i) {
    if(bmp_.getAltitude(altitude_)) {
    sum += altitude_;
    n++;
    if (altitude_ < minValue) minValue = altitude_;
    if (altitude_ > maxValue) maxValue = altitude_;

    }
    delay(10);
  }

  if (n > 0) {
    altitudeCalibration_ = sum / n;
    Serial.print("Calibrated altitude: ");
    Serial.println(altitudeCalibration_);
  }
  else {
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
  return altitude_ - altitudeCalibration_;
}

void BMP388::detectApogee()
{
    // static float lastAltitude = 0;
    static float maxAltitude = -FLT_MAX;
    static bool climbing = true;
    static unsigned long lastDetectionTime = 0;
    const unsigned long detectionCooldown = 2000;
    const float minDropThreshold = 0.5;

    // if (bmp_.takeForcedMeasurement()) {
    //     altitude_ = bmp_.readAltitude(SEALEVELPRESSURE_HPA) - altitudeCalibration_;
    //     Serial.print(F("Approx altitude = "));
    //     Serial.print(altitude_);
    //     Serial.println(" m\n");
    //     // altitudes[index] = altitude_;
    //     // index = (index + 1) % 100;
    //     // if (count < 100) count++;
    // }

    if (altitude_ > maxAltitude) {
        maxAltitude = altitude_;
    }
    if (altitude_ < maxAltitude - minDropThreshold && climbing) {
        if (millis() - lastDetectionTime > detectionCooldown) {
            apogeeDetected = true;
            Serial.println(maxAltitude);
            lastDetectionTime = millis();
            climbing = false;
            maxapogee = maxAltitude;
        }
    // } else if (altitude_ > lastAltitude) {
    //     climbing = true;
    //     if (maxAltitude > maxapogee) {
    //       maxapogee = maxAltitude;
    //     }
        // maxAltitude = altitude_;
    }

    // lastAltitude = altitude_;
}

