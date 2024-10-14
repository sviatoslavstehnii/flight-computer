
#include <Arduino.h>
// #include <FCMS.h>
#include <StaticFireStand.h>

// FCMS fcms{};
StaticFireStand sfs{};

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Starting...");
  Wire.begin();

  delay(200);

  sfs.setup();
  delay(1000);
}

uint32_t loopTimer = 0;
uint32_t commitTimer = 0; 

void loop() {
  sfs.monitor();

  if (millis() - commitTimer >= 1000) {
    // sfs.commit();
    commitTimer = millis();
  }

  // while (micros() - loopTimer < 4000);
  // loopTimer = micros();
}

