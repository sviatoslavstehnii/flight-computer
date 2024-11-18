#include <Arduino.h>
#include <FCMS.h>

FCMS fcms{};

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Starting...");
  #ifdef MASTER_MODE
    Serial.println("Running in MASTER mode");
  #elif defined(SLAVE_MODE)
    Serial.println("Running in SLAVE mode");
  #else
    Serial.println("No mode defined. Please set build flag!");
  #endif

  Wire.begin();
  delay(200);

  fcms.setup();
  Serial.println("STAAART");
}


void loop() {
  fcms.updateState();
}


// #include <Arduino.h>
// #include "FCMS.h"
// void setup() {
//   Serial.begin(115200);
//   while (!Serial);
//   Serial.println("Starting...");

//   Serial.println("STAAART");
//   for (int i = 0; i < 10000000; i++) {
//     fcms.updateState();
//   }

//   SDMC sdmc{};
//   sdmc.setup();
//   sdmc.read("dj.txt");
//   sdmc.read("mej.txt");
//   Serial.println("END");

// }

// void loop() {}


