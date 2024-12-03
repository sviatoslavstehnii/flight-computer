// #include "LoRa_E32.h"

// LoRa_E32 loraGround(&Serial2, LORA_AUX_PIN);
// LoRa_E32 loraVehicle(&Serial5); // TX2 (GPIO17), RX2 (GPIO16)


// // Initialize LoRa modules
//   loraGround.begin();
//   loraVehicle.begin();
//   Serial.println("LoRa modules initialized...");
//   Serial2.println("DATA from M2.");
//   delay(500);
//   Serial5.println("DATA from M5.");

// // Send from module 1 to module 2
//   if (Serial.available()) {
//     String input = Serial.readString(); // Input from Serial Monitor
//     loraGround.sendMessage(input);        // Send the message via LoRa module 1
//     Serial.print("Sent: ");
//     Serial.println(input);
//   }

//   // Receive from module 2
//   if (loraVehicle.available() > 1) {
//     ResponseContainer rc = loraVehicle.receiveMessage(); // Receive message on LoRa module 2
//     digitalWrite(ONBOARD_LED,HIGH);
//     delay(100);
//     digitalWrite(ONBOARD_LED,LOW);

//     if (rc.status.code != 1) {
//       Serial.println(rc.status.getResponseDescription()); // Print error if something goes wrong
//     } else {
//       Serial.print("Received from module 2: ");
//       Serial.println(rc.data);  // Print the received data
//     }
//   }

//   // Send from module 2 to module 1
//   if (Serial2.available()) {
//     String input = Serial2.readString(); // Input from Serial2 (LoRa 2)
//     loraVehicle.sendMessage(input);         // Send the message via LoRa module 2
//     Serial.print("Sent: ");
//     Serial.println(input);
//   }

//   // Receive from module 1
//   if (loraGround.available() > 1) {
//     ResponseContainer rc = loraGround.receiveMessage(); // Receive message on LoRa module 1
//     digitalWrite(ONBOARD_LED,HIGH);
//     delay(100);
//     digitalWrite(ONBOARD_LED,LOW);

//     if (rc.status.code != 1) {
//       Serial.println(rc.status.getResponseDescription()); // Print error if something goes wrong
//     } else {
//       Serial.print("Received from module 1: ");
//       Serial.println(rc.data);  // Print the received data
//     }
//   }