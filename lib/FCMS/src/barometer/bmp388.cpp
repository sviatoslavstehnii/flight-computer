#include "bmp388.h"


void BMP388::setup()
{
  if (!bmp_.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
  //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
  //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp_.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp_.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp_.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp_.setOutputDataRate(BMP3_ODR_50_HZ);

  calibrate();
}


void BMP388::update()
{
  bmp_.performReading();
}


void BMP388::calibrate()
{
  Serial.print("Calibrating BMP388...");
  size_t n = 0;
  float sum = 0;
  float minValue = FLT_MAX;
  float maxValue = -FLT_MAX;

  for (size_t i = 0; i < 500; ++i) {
    if(bmp_.performReading()){

      float altitude = bmp_.readAltitude(SEALEVELPRESSURE_HPA);
      sum += altitude;
      n++;
      if (altitude < minValue) minValue = altitude;
      if (altitude > maxValue) maxValue = altitude;
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
  altitude_ = bmp_.readAltitude(SEALEVELPRESSURE_HPA) - altitudeCalibration_;
  return altitude_;
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

