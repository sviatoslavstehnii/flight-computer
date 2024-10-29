
#include <Arduino.h>
#include <FCMS.h>

// FCMS fcms{};
// BMP280 baro{};
IMU imu{};

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Starting...");
  Wire.begin();

  delay(200);
  // fcms.setup();
  // setup baro 280
  // baro.setup();
  imu.setup();
  delay(1000);
}

uint32_t loopTimer = 0;
uint32_t commitTimer = 0; 

void loop() {
    // baro.update(); // Update BMP280 data and check for apogee
    // baro.printAltitude(); // Print the current altitude
    imu.update(); // Update IMU data and check for takeoff/landing
    // imu.printGyroData();
    // imu.printAccelData();
    delay(100); // Adjust the delay as necessary
}

