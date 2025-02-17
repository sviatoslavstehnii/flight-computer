#define GROUND_STATION
//#define ZEST_CHIP

#ifdef GROUND_STATION
#include <GSMS.h>

GSMS gsms{};

void setup(){
  Serial.begin(9600);
  Serial.println("START");
  // Serial3.begin(115200);
  gsms.setup();
}

void loop(){
  // print hex values of incoming data
  // while(Serial3.available()){
  //   char c = Serial3.read();
  //   // Serial.print("0x");
  //   Serial.print((uint8_t)c, HEX);
  //   Serial.print(" ");
  //   // Serial.print(c);
  // }
  // // Serial.println();
  delay(200);

  gsms.step();
}

#else
#include <FCMS.h>

#ifdef ZEST_CHIP
#define PYRO_ONE_PIN 23
#define PYRO_TWO_PIN 22
#define PYRO_THREE_PIN 21
#else
#define PYRO_ONE_PIN 23 // Bottom, next to LORA
#define PYRO_TWO_PIN 21 // Top
#define PYRO_THREE_PIN 22 // Unavailable
#endif

FCMS fcms{};

void setup() {
  Serial.begin(9600);
  Serial.println("START");
  Wire.begin();

  pinMode(PYRO_ONE_PIN, OUTPUT);
  pinMode(PYRO_TWO_PIN, OUTPUT);
  pinMode(PYRO_THREE_PIN, OUTPUT);
  // Serial.println("Pyro ON");
  // digitalWrite(PYRO_ONE_PIN, HIGH);
  // digitalWrite(PYRO_TWO_PIN, HIGH);
  // digitalWrite(PYRO_THREE_PIN, HIGH);
  // delay(10000);
  Serial.println("Pyro OFF");
  digitalWrite(PYRO_ONE_PIN, LOW);
  digitalWrite(PYRO_TWO_PIN, LOW);
  digitalWrite(PYRO_THREE_PIN, LOW);

  fcms.setup();
}


void loop() {
  fcms.step();
  // while(Serial3.available()){
  //       char c = Serial3.read();
  //       // Serial.print("0x");
  //       Serial.print((uint8_t)c, HEX);
  //       Serial.print(" ");
  //       // Serial.print(c);
  //       // Serial.print("> ");
  //     }
}
#endif