
#include "FCMS.h"

Flash mem{};

void setup() {
  Serial.begin(115200);
 
  while (!Serial) ;
  delay(100);
  
  mem.setup(true);
  String data_from_mem;


  String data = "helloff";
  String data1 = "aaa15aaa10aaa15aaa20aaa15aaa10aaa15aaa40";
  String data2 = "aaa15aaa10aaa15aaa20a   10aaa15aaa40";

  mem.writeToCJ(data);
  delay(1000);
  mem.writeToCJ(data1);
  delay(1000);
  mem.writeToCJ(data2);
  delay(1000);
  String final_data_from_mem = "";
  mem.readCJ(final_data_from_mem);
  Serial.println(final_data_from_mem);

}
 
void loop() {
 
}



