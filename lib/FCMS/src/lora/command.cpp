#include "LoRa_E32.h"

#define START_BYTE 0x7E
#define SENDER 0x01
#define RECEIVER 0x00

#define LORA_AUX_PIN 4
LoRa_E32 loraFCMS(&Serial2, LORA_AUX_PIN);

#define PACKET_TYPE 0x04 // send telemetry gs > fs, fc
#define PACKET_TYPE_RESPONCE 0x05 // send responce fs, fc > gs

void receiveCommand(uint8_t *packet, size_t packetSize, bool responce_command = false) {
  if (packetSize < 58) { // Мінімальна довжина пакету
    Serial.println("Invalid packet size!");
    return;
  }

  // Розбір заголовку
  uint8_t startByte = packet[0];
  uint8_t packetType = packet[1];
  uint8_t sender = packet[2];
  uint8_t receiver = packet[3];
  uint32_t timestamp;
  memcpy(&timestamp, &packet[4], sizeof(uint32_t));

  Serial.println("Header:");
  Serial.print("Start Byte: "); Serial.println(startByte, HEX);
  Serial.print("Packet Type: "); Serial.println(packetType, HEX);
  Serial.print("Sender: "); Serial.println(sender);
  Serial.print("Receiver: "); Serial.println(receiver);
  Serial.print("Timestamp: "); Serial.println(timestamp);

  // Розбір тіла
  Serial.println("\nBody:");
  uint8_t commandNumber = packet[8];
  Serial.print("Command Number: "); Serial.println(commandNumber);

  if (responce_command) {
    uint8_t commandResponse = packet[9];
    Serial.print("Command Response: "); Serial.println(commandResponse);
  }
}


void sendCommand(bool responce_command = false) {
  /*
  Header 
  Byte	Part	Type	Description
  0	    bin	Start Byte
  1		  bin	Packet Type
  2		  bin	Sender
  3		  bin	Receiver
  4-7		uint32	Timestamp	
  */
  const uint8_t templateResponce[4] = {START_BYTE, PACKET_TYPE_RESPONCE, SENDER, RECEIVER};
  const uint8_t templateNormal[4] = {START_BYTE, PACKET_TYPE, SENDER, RECEIVER};

  uint8_t packet[58];

  if (responce_command) {
    memcpy(packet, templateResponce, 4);
  } else {
    memcpy(packet, templateNormal, 4);
  }

  uint32_t timestamp = millis();
  memcpy(&packet[4], &timestamp, sizeof(uint32_t));

  // check if size of packet is less than 58 bytes
  if (sizeof(packet) > MAX_SIZE_TX_PACKET) {
    Serial.println("Packet size is too large...");
    return;
  }

  /*
  Body
  8   bin	Command number
  */
  uint8_t commandNumber = 0x01;
  packet[8] = commandNumber;


  // additional responce
  /*
  Body
  9   bin	Command response
  */
  if (responce_command) {
    uint8_t commandResponse = 0x01;
    packet[9] = commandResponse;
  }

  ResponseStatus response = loraFCMS.sendMessage(packet, sizeof(packet));
  if (response.code != 1) {
    Serial.println(response.getResponseDescription());
  } else {
    Serial.println("Packet sent...");
  }
}
  

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);
  loraFCMS.begin();
  Serial.println("LoRa module initialized...");
}

void loop() {
  sendCommand();
  delay(1000);

  // receive telemetry
  ResponseContainer response = loraFCMS.receiveMessage();
  uint8_t* dataBuffer = (uint8_t*)response.data.c_str();
  size_t dataSize = response.data.length();

  receiveCommand(dataBuffer, dataSize);
}
