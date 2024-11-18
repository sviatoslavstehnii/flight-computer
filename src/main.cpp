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
}


void loop() {
  // fcms.updateData();
  // fcms.updateState();
  fcms.test_waypoint();
}

