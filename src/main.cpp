
#include <Arduino.h>
#include <FCMS.h>

FCMS fcms{};

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Starting...");
  Wire.begin();

  delay(200);

  fcms.setup();
  delay(1000);
}

uint32_t loopTimer = 0;
uint32_t commitTimer = 0; 

void loop() {
  fcms.navigate();

  if (millis() - commitTimer >= 1000) {
    fcms.commit();
    commitTimer = millis();
  }

  while (micros() - loopTimer < 4000);
  loopTimer = micros();
}

