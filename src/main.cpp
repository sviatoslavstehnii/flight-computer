#include <Arduino.h>
#include "FCMS.h"

INA219Lib ina219;

void setup() {
  Serial.begin(115200);
  
  if (!ina219.begin()) {
    Serial.println("INA219 initialization failed!");
    while (1);
  }
  
  Serial.println("INA219 initialized successfully!");
}
void loop() {
  float busVoltage = ina219.getBusVoltage(); 
  float shuntVoltage = ina219.getShuntVoltage();
  float current = ina219.getCurrent();
  float power = ina219.getPower();

  Serial.print("Bus Voltage: "); Serial.print(busVoltage); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(shuntVoltage); Serial.println(" mV");
  Serial.print("Current: "); Serial.print(current); Serial.println(" mA");
  Serial.print("Power: "); Serial.print(power); Serial.println(" mW");
  Serial.println();

  delay(2000);
}
