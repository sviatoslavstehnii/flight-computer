// open-source libs
#include <Arduino.h>
#include <Wire.h>

// custom libs
#include "imu.h"
#include "barometer.h"

IMU imu{};
Barometer baro{};

// ============================================================

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial)
    delay(10);
  // imu.setup();
  baro.setup();
}

void loop() {
  // put your main code here, to run repeatedly:
  // imu.updateRates();
  // imu.printRates();

  baro.printAltitide();
  delay(500);
}

// ============================================================
