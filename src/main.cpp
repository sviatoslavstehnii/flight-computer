#include <FCMS.h>

FCMS fcms;

void setup() {
  Serial.begin(115200);
  fcms.setup();
}

void loop() {
  fcms.step();
}