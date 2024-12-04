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
  fcms.checkHealth();


  Serial.println("STAAART");
}


void loop() {
  fcms.step();
 
}

