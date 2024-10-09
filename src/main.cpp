

#include <FCMS.h>



void setup() {
  Serial.begin(9600);
  while (!Serial);
  SDMC sd;

  sd.setup();

  delay(100);

  sd.write("test.txt", "hello");

  delay(100);

  sd.read("test.txt");

  delay(100);
  sd.remove("test.txt");

  delay(100);

  if (!sd.read("test.txt")) {
    Serial.println("file removed");
  } 
  
}

void loop() {
  // nothing happens after setup
}