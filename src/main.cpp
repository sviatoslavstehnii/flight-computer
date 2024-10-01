// open-source libs
#include <Arduino.h>
#include <Wire.h>

// main lib
#include "FCMS.h"

IMU imu{};
BMP280 baro{};

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

  baro.printAltitude();
  delay(500);
}

// ============================================================
