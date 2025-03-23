// #define GROUND_STATION
//  #define ZEST_CHIP

#ifdef GROUND_STATION
#include <GSMS.h>

GSMS gsms{};

void setup()
{
  Serial.begin(9600);
  Serial.println("START");
  Serial3.begin(115200);
  gsms.setup();
}

void loop()
{
  // print hex values of incoming data
  // while (Serial3.available())
  // {
  // char c = Serial3.read();
  // Serial.print("0x");
  // Serial.print((uint8_t)c, HEX);
  // Serial.print(" ");
  // Serial.print(c);
  // }
  // Serial.println();

  gsms.step();
  delay(100);
}

#else
#include <FCMS.h>

FCMS fcms{};

void setup()
{
  Serial.begin(115200);
  Serial.println("START");
  Wire.begin();

  // for (int i = 0; i < 128; i++) {
  //   Wire.beginTransmission(i);
  //   if (Wire.endTransmission() == 0) {
  //     Serial.print("Found device at address: ");
  //     Serial.println(i, HEX);
  //   }
  // }

  Serial3.begin(115200);
  // delay(3000);
  // while(true){
  //   if(Serial3.available())
  //   {
  //     Serial.write(Serial3.read());
  //   }
  //   if(Serial.available()){
  //     Serial3.write(Serial.read());
  //   }
  //   delay(5);
  // }

  fcms.setup();
}

void loop()
{
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