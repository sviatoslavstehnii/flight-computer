//#define GROUND_STATION

#ifdef GROUND_STATION
#include <GSMS.h>

GSMS gsms{};

void setup(){
  Serial.begin(9600);
  Serial.println("START");
  // Serial3.begin(115200);
  delay(100);
  gsms.setup();
}

void loop(){
  // print hex values of incoming data
  while(Serial3.available()){
    char c = Serial3.read();
    Serial.print("0x");
    Serial.print(c, HEX);
    Serial.print(" ");
  }
  Serial.println();
  delay(300);

  gsms.step();
}

#else
#include <FCMS.h>

#define PYRO_ONE_PIN 23 // Bottom, next to LORA
#define PYRO_TWO_PIN 21 // Top
//#define PYRO_THREE_PIN 22

FCMS fcms{};

void setup() {
  Serial.begin(9600);
  Serial.println("START");
  Wire.begin();

  pinMode(PYRO_ONE_PIN, OUTPUT);
  pinMode(PYRO_TWO_PIN, OUTPUT);
  Serial.println("Pyro Off");
  digitalWrite(PYRO_ONE_PIN, LOW);
  digitalWrite(PYRO_TWO_PIN, LOW);

  fcms.setup();
}


void loop() {
  fcms.step();
}
#endif