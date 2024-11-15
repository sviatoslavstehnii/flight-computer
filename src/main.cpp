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
  Serial.println("STAAART");
  for (int i = 0; i < 10000000; i++) {
    fcms.updateState();
  }
  fcms.commitSDMC();

  SDMC sdmc{};
  sdmc.setup();
  sdmc.read("dj.txt");
  sdmc.read("mej.txt");
  Serial.println("END");
}


void loop() {
}


// #include <Arduino.h>
// #include "FCMS.h"
// void setup() {
//   Serial.begin(115200);
//   while (!Serial);
//   Serial.println("Starting...");

//   SDMC sdmc{};
//   sdmc.setup();
    

//   Serial.println("dj.txt: ");
//   sdmc.read("dj.txt");
//   auto f = SD.open("mej.txt");
//   Serial.println(f.size());

//   Serial.println("mej.txt: ");
//   sdmc.read("mej.txt");

// }

// void loop() {}


