#include <Arduino.h>
#include <FCMS.h>

#define PYRO_ONE_PIN 23 // Bottom, next to LORA
#define PYRO_TWO_PIN 21 // Top
//#define PYRO_THREE_PIN 22

FCMS fcms{};

void setup() {

  Wire.begin();
  Serial.begin(115200);
  Serial.println("START");

  Serial3.begin(115200);
  // "\r\n"
  byte bytes[128]{}; 
  Serial3.write(bytes, 128);
  size_t bytesLen = Serial3.readBytesUntil('\n', bytes, 128);


  pinMode(PYRO_ONE_PIN, OUTPUT);
  pinMode(PYRO_TWO_PIN, OUTPUT);
  Serial.println("Fire");
  digitalWrite(PYRO_ONE_PIN, LOW);
  digitalWrite(PYRO_TWO_PIN, LOW);


  fcms.setup();
}


void loop() {
  fcms.step();

  std::stringstream ss;
  // ss << "{";
  ss << std::fixed << std::setprecision(2);
  // ss << "}";
  Serial.println(ss.str().c_str());
  
  delay(50); 
}
