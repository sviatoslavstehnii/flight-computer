
#include "FCMS.h"

GPS gps(&Serial1);

void setup() {
  Serial.begin(115200);
  gps.setup();
}

void loop() {
  if (gps.readAndParse()) {
    Serial.print("Location: ");
    Serial.print(gps.getLatitude(), 4);
    Serial.print(", ");
    Serial.print(gps.getLongitude(), 4);
  }
}
