#include <SPI.h>
#include "LORA.h"

// Define LoRa module 1 (Ground) pins
#define LORA1_CS_PIN 10
#define LORA1_RESET_PIN 9
#define LORA1_DIO0_PIN 2

// Define LoRa module 2 (Vehicle) pins
#define LORA2_CS_PIN 11
#define LORA2_RESET_PIN 8
#define LORA2_DIO0_PIN 3

// Create LoRa instances
LORA loraGround(LORA1_CS_PIN, LORA1_RESET_PIN, LORA1_DIO0_PIN);
LORA loraVehicle(LORA2_CS_PIN, LORA2_RESET_PIN, LORA2_DIO0_PIN);

void setup() {
  // Serial monitors for debugging
  Serial.begin(9600);  // Monitor for user input/output
  while (!Serial);

  Serial.println("Initializing LoRa modules...");

  // Initialize LoRa modules
  loraGround.begin();
  loraGround.setFrequency(868E6);   // 868 MHz
  loraGround.setTxPower(17);        // Max TX power
  loraGround.setSpreadingFactor(7); // SF7

  loraVehicle.begin();
  loraVehicle.setFrequency(868E6);   // 868 MHz
  loraVehicle.setTxPower(17);        // Max TX power
  loraVehicle.setSpreadingFactor(7); // SF7

  Serial.println("LoRa modules initialized...");
}

void loop() {
  // Send data from module 1 (Ground) to module 2 (Vehicle)
  if (Serial.available()) {
      String input = Serial.readString(); // Input from Serial Monitor
      loraGround.sendPacket((uint8_t *)input.c_str(), input.length());
      Serial.print("Sent from module 1: ");
      Serial.println(input);
  }

  // Receive data on module 1 (Ground)
  uint8_t buffer1[256];
  uint8_t length1;
  if (loraGround.receivePacket(buffer1, length1)) {
      Serial.print("Received on module 1: ");
      for (uint8_t i = 0; i < length1; i++) {
          Serial.print((char)buffer1[i]);
      }
      Serial.println();
  }

  // Send data from module 2 (Vehicle) to module 1 (Ground)
  if (Serial2.available()) { // Simulating Serial2 input for module 2
      String input = Serial2.readString();
      loraVehicle.sendPacket((uint8_t *)input.c_str(), input.length());
      Serial.print("Sent from module 2: ");
      Serial.println(input);
  }

  // Receive data on module 2 (Vehicle)
  uint8_t buffer2[256];
  uint8_t length2;
  if (loraVehicle.receivePacket(buffer2, length2)) {
      Serial.print("Received on module 2: ");
      for (uint8_t i = 0; i < length2; i++) {
          Serial.print((char)buffer2[i]);
      }
      Serial.println();
  }

  delay(100); // Avoid flooding the serial and LoRa interfaces
}
