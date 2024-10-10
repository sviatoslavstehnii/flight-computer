#include <SPI.h>
#include <SPIFlash.h>
#include <Wire.h>
#include "FCMS.h"

Flash mem{10};

void setup() {
  Serial.begin(115200);
  while(!Serial);

  mem.setup(16777216, true);

  // data should end with \n and be shorter than 100
  char data1[] = "123456";
  char data2[] = "aboba1bboj";
  char data3[] = "Lorem ipsum dolor sit amet, consectetuer adipiscing elit. Aenean commodo ligula eget dolor";

  char readData[1000] = "";
  mem.writeToCJ(data1, sizeof(data1));
  delay(100);
  mem.writeToCJ(data2, sizeof(data2));
  delay(100);
  mem.writeToCJ(data3, sizeof(data3));


  delay(100);

  mem.readCJ(readData, 1000);
  Serial.println(readData);
  Serial.println("end");

}


void loop() {
  // Your code here
}


