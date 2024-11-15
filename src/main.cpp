// #include <Arduino.h>
// #include <FCMS.h>

// FCMS fcms{};

// void setup() {
//   Serial.begin(115200);
//   while (!Serial);
//   Serial.println("Starting...");
//   Wire.begin();
//   delay(200);

//   fcms.setup();
//   Serial.println("STAAART");
//   for (int i = 0; i < 20000; i++) {
//     fcms.updateState();
//     delay(1);
//   }
//   fcms.commitSDMC();

//   SDMC sdmc{};
//   sdmc.setup();
//   sdmc.read("mej.txt");
//   Serial.println("END");
// }


// void loop() {
// }


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

//   Serial.println("mej.txt: ");
//   sdmc.read("mej.txt");
// }

// void loop() {}


