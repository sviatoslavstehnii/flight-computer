#include "LoRa_E32.h"
#include <vector>
#include <FCMS.h>


#define START_BYTE 0x7E
#define OWN_ADDRESS 0x00
#define RECEIVER 0x00


#define PACKET_TELEMETRY 0x01 // send 1 packet of telemetry fc > gs
#define PACKET_COMMAND 0x02   // send 1 packet of command gs > fc
#define PACKET_COMMAND_RESPONCE 0x05 // send responce fs, fc > gs

#define TELEMETRY_SIZE 210
#define COMMAND_SIZE 10

#define LORA_AUX_PIN 4
LoRa_E32 loraFCMS(&Serial2, LORA_AUX_PIN);



struct ImuData {
  uint16_t yaw;         // Курс
  uint16_t pitch;       // Тангаж
  uint16_t roll;        // Крен
  int16_t accel_x;      // Прискорення по X
  int16_t accel_y;      // Прискорення по Y
  int16_t accel_z;      // Прискорення по Z
  int16_t velocity_x;   // Швидкість по X
  int16_t velocity_y;   // Швидкість по Y
  int16_t velocity_z;   // Швидкість по Z
  int16_t position_x;   // Позиція по X
  int16_t position_y;   // Позиція по Y
  int16_t position_z;   // Позиція по Z
};


void receiveTelemetry(uint8_t* packet, size_t length) {
  // Parse header
  uint8_t startByte = packet[0];
  uint8_t packetType = packet[1];
  uint8_t sender = packet[2];
  uint8_t receiver = packet[3];
  uint32_t timestamp;
  memcpy(&timestamp, &packet[4], sizeof(uint32_t));

  Serial.printf("Header: StartByte=%02X, PacketType=%02X, Sender=%02X, Receiver=%02X, Timestamp=%u\n", 
                startByte, packetType, sender, receiver, timestamp);

  // Parse flight and additional states
  uint8_t states = packet[8];
  uint8_t flightState = states & 0x0F;
  uint8_t additionalState = (states >> 4) & 0x0F;

  Serial.printf("FlightState=%02X, AdditionalState=%02X\n", flightState, additionalState);

  // Parse barometric altitude
  uint16_t barometricAltitude;
  memcpy(&barometricAltitude, &packet[9], sizeof(uint16_t));
  Serial.printf("Barometric Altitude=%04X\n", barometricAltitude);

  // 4 bytes reserved

  // Parse IMU data
  ImuData imuData;
  memcpy(&imuData, &packet[16], sizeof(ImuData));
  Serial.printf("IMU Data: Yaw=%04X, Pitch=%04X, Roll=%04X, AccelX=%04X, AccelY=%04X, AccelZ=%04X, "
                "VelX=%04X, VelY=%04X, VelZ=%04X, PosX=%04X, PosY=%04X, PosZ=%04X\n",
                imuData.yaw, imuData.pitch, imuData.roll, 
                imuData.accel_x, imuData.accel_y, imuData.accel_z,
                imuData.velocity_x, imuData.velocity_y, imuData.velocity_z,
                imuData.position_x, imuData.position_y, imuData.position_z);

  // Parse fins
  uint8_t fin0 = packet[40];
  uint8_t fin1 = packet[41];
  uint8_t fin2 = packet[42];
  uint8_t fin3 = packet[43];
  Serial.printf("Fins: [%02X, %02X, %02X, %02X]\n", fin0, fin1, fin2, fin3);

  // 4 bytes reserved

  // Parse battery data
  uint16_t vBat, currBat;
  memcpy(&vBat, &packet[48], sizeof(uint16_t));
  memcpy(&currBat, &packet[50], sizeof(uint16_t));
  Serial.printf("Battery: Voltage=%04X, Current=%04X\n", vBat, currBat);

  uint16_t loadCell;
  memcpy(&loadCell, &packet[52], sizeof(uint16_t));
  Serial.printf("LoadCell=%04X\n", loadCell);


  // Parse temperature and data
  int8_t temp = packet[54];
  uint8_t melData = packet[55];
  uint8_t clData = packet[56];
  uint8_t datalogData = packet[57];
  Serial.printf("Temp=%02X, MEL=%02X, CL=%02X, Datalog=%02X\n", temp, melData, clData, datalogData);

  // Parse GPS data
  int32_t gpsLatitude, gpsLongitude;
  int16_t gpsAltitude;
  memcpy(&gpsLatitude, &packet[58], sizeof(int32_t));
  memcpy(&gpsLongitude, &packet[62], sizeof(int32_t));
  memcpy(&gpsAltitude, &packet[66], sizeof(int16_t));
  Serial.printf("GPS: Latitude=%08X, Longitude=%08X, Altitude=%04X\n", 
                gpsLatitude, gpsLongitude, gpsAltitude);

  // Parse detections
  uint8_t detections[3];
  memcpy(detections, &packet[68], sizeof(detections));
  Serial.printf("Detections: [%02X, %02X, %02X]\n", detections[0], detections[1], detections[2]);

  // Parse apogee
  uint16_t apogee;
  memcpy(&apogee, &packet[71], sizeof(uint16_t));
  Serial.printf("Apogee=%04X\n", apogee);


  // reserved up to 200 byte

  // Parse events data
  uint8_t events_data[16]; // Assuming this array is large enough for the extracted events
  int index = 0; // To track the index in events_data

  for (int i = 0; i < 4; i++) {
      // Unpack each event from the byte
      events_data[index++] = packet[200 + i] & 0x03;               // Extract Event 0 (bits 0-1)
      events_data[index++] = (packet[200 + i] >> 2) & 0x03;        // Extract Event 1 (bits 2-3)
      events_data[index++] = (packet[200 + i] >> 4) & 0x03;        // Extract Event 2 (bits 4-5)
      events_data[index++] = (packet[200 + i] >> 6) & 0x03;        // Extract Event 3 (bits 6-7)
  }
  for (int i = 0; i < 16; i++) {
      Serial.printf("Event %d: %02X\n", i, events_data[i]);
  }

  uint16_t event0Time, event1Time;
  memcpy(&event0Time, &packet[204], sizeof(uint16_t));
  memcpy(&event1Time, &packet[206], sizeof(uint16_t));
  Serial.printf("EventTimes: Event0=%04X, Event1=%04X\n", event0Time, event1Time);

  uint8_t pyroFlags = packet[208];
  uint8_t pyroFlags_ad = packet[209];
  // Parse pyro flags
  bool extractedPyroFlags[8];
  bool extractedPyroFlags_ad[8];

  // Extract flags from the packed bytes and store them in arrays
  for (int i = 0; i < 8; i++) {
      extractedPyroFlags[i] = (pyroFlags >> i) & 0x01;
      extractedPyroFlags_ad[i] = (pyroFlags_ad >> i) & 0x01;
  }

  // Printing the stored pyro flags as arrays
  Serial.print("PyroFlags: ");
  for (int i = 0; i < 8; i++) {
      Serial.print(extractedPyroFlags[i]);
      Serial.print(" ");
  }
  Serial.println();

  Serial.print("PyroFlags_ad: ");
  for (int i = 0; i < 8; i++) {
      Serial.print(extractedPyroFlags_ad[i]);
      Serial.print(" ");
  }
  Serial.println();

  Serial.println("Telemetry data parsed successfully.");
}



void sendTelemetry(std::vector<uint8_t>& packet_send) {
  /*
  Header 
  Byte	Part	  Type
  0	    bin	    Start Byte
  1		  bin	    Packet Type
  2		  bin	    Sender
  3		  bin   	Receiver
  4-7		uint32	Timestamp	
  */
  uint8_t packet[TELEMETRY_SIZE] = {START_BYTE, PACKET_TELEMETRY, OWN_ADDRESS, RECEIVER};

  // timestamp
  uint32_t timestamp = 12345678;
  memcpy(&packet[4], &timestamp, sizeof(uint32_t));

  // check if size of packet is less than 58 bytes
  if (sizeof(packet) > 255) {
    Serial.println("Packet size is too large...");
    return;
  }
  // body  
  uint8_t FLIGHT_STATE = 0x01;      // Політний стан (1 біт або більше)
  uint8_t ADDITIONAL_STATE = 0x02;  // Додатковий стан (1 біт або більше)

  // Записуємо обидва стани в один байт
  packet[8] = (FLIGHT_STATE & 0x0F) | ((ADDITIONAL_STATE & 0x0F) << 4);

  uint16_t BAROMETRIC_ALTITUDE = 0x1234;  // Барометрична висота
  memcpy(&packet[9], &BAROMETRIC_ALTITUDE, sizeof(uint16_t));

  // 4 bytes reserved

  uint16_t yaw = 0x5678;    // Курс
  uint16_t pitch = 0x9ABC;  // Тангаж
  uint16_t roll = 0xDEF0;   // Крен

  int16_t accel_x = 0x1234;  // Прискорення по X
  int16_t accel_y = 0x5678;  // Прискорення по Y
  int16_t accel_z = 0x3452;  // Прискорення по Z

  int16_t velocity_x = 0x1234;  // Швидкість по X
  int16_t velocity_y = 0x5678;  // Швидкість по Y
  int16_t velocity_z = 0x3452;  // Швидкість по Z

  int16_t position_x = 0x1234;  // Позиція по X
  int16_t position_y = 0x5678;  // Позиція по Y
  int16_t position_z = 0x3452;  // Позиція по Z

  ImuData imuData = {yaw, pitch, roll, accel_x, accel_y, accel_z, velocity_x, velocity_y, velocity_z, position_x, position_y, position_z};
  memcpy(&packet[16], &imuData, sizeof(ImuData));

  // fin for 1 byte
  uint8_t fin0 = 0x01;  // Фін 0
  uint8_t fin1 = 0x02;  // Фін 1
  uint8_t fin2 = 0x03;  // Фін 2
  uint8_t fin3 = 0x04;  // Фін 3

  packet[40] = fin0;
  packet[41] = fin1;
  packet[42] = fin2;
  packet[43] = fin3;


  // 4 bytes reserved

  uint16_t V_BAT = 0x1234;  // Напруга батареї
  uint16_t CURR_BAT = 0x5678;  // Струм батареї

  memcpy(&packet[48], &V_BAT, sizeof(uint16_t));
  memcpy(&packet[50], &CURR_BAT, sizeof(uint16_t));

  uint16_t load_cell = 0x9ABC;  // Дані load cell
  memcpy(&packet[52], &load_cell, sizeof(uint16_t));

  int8_t Temp = 0x12;  // Температура
  uint8_t MEL_Data = 0x34;  // Дані MEL
  uint8_t CL_Data = 0x56;  // Дані CL
  uint8_t Datalog_Data = 0x78;  // Дані Datalog

  packet[54] = Temp;
  packet[55] = MEL_Data;
  packet[56] = CL_Data;
  packet[57] = Datalog_Data;

  int32_t GPS_Latitude = 0x12345678;  // Широта GPS
  int32_t GPS_Longitude = 0x9ABCDEF0;  // Довгота GPS
  int16_t GPS_Altitude = 0x1234;  // Висота GPS

  memcpy(&packet[58], &GPS_Latitude, sizeof(int32_t));
  memcpy(&packet[62], &GPS_Longitude, sizeof(int32_t));
  memcpy(&packet[66], &GPS_Altitude, sizeof(int16_t));

  uint8_t detections[3] = {true, false, false};  // Виявлення злітання
  for (int i = 0; i < 3; i++) {
      packet[68 + i] = detections[i];
  }

  uint16_t Apogee = 0x1234;  // max height
  memcpy(&packet[71], &Apogee, sizeof(uint16_t));



  // reserved up to 200 byte

  // Масив із 16 значень по 2 біти
  uint8_t events_data[16] = {0x01, 0x02, 0x03, 0x00, 0x01, 0x02, 0x03, 0x00,
                      0x01, 0x02, 0x03, 0x00, 0x01, 0x02, 0x03, 0x00};


  // Записуємо по 4 значення в кожен байт
  for (int i = 0; i < 4; i++) {
    packet[200 + i] = (events_data[i * 4] & 0x03) |
      ((events_data[i * 4 + 1] & 0x03) << 2) |
      ((events_data[i * 4 + 2] & 0x03) << 4) |
      ((events_data[i * 4 + 3] & 0x03) << 6);
  }

  uint16_t event0_time = 0x1234;  // Час події 0
  uint16_t event1_time = 0x5678;  // Час події 1

  memcpy(&packet[204], &event0_time, sizeof(uint16_t));
  memcpy(&packet[206], &event1_time, sizeof(uint16_t));


  uint8_t pyroFlags = 0;  // Один байт для зберігання всіх Pyro Flags
  bool pyroFlagsArray[8] = {true, false, true, false, true, false, true, false};

  for (int i = 0; i < 8; i++) {
      pyroFlags |= (pyroFlagsArray[i] << i);
  }
  packet[208] = pyroFlags;


  uint8_t pyroFlags_ad = 0;
  bool pyroFlags_adArray[8] = {true, false, true, false, true, false, true, false};

  for (int i = 0; i < 8; i++) {
      pyroFlags_ad |= (pyroFlags_adArray[i] << i);
  }
  packet[209] = pyroFlags_ad;


  packet_send.clear();  // Clear the vector if necessary
  packet_send.insert(packet_send.end(), std::begin(packet), std::end(packet));

  // send packet
  // ResponseStatus response = loraFCMS.sendMessage(packet, sizeof(packet));
  // if (response.code != 1) {
  //   Serial.println(response.getResponseDescription());
  // } else {
  //   Serial.println("Packet sent...");
  // }
}

void receiveCommand(uint8_t *packet, size_t packetSize, bool responce_command = false) {
  if (packetSize != COMMAND_SIZE) { // Мінімальна довжина пакету
    Serial.println("Invalid packet size! 7");
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


void sendCommand(std::vector<uint8_t>& packet_send, bool responce_command = false) {
  /*
  Header 
  Byte	Part	Type	Description
  0	    bin	Start Byte
  1		  bin	Packet Type
  2		  bin	Sender
  3		  bin	Receiver
  4-7		uint32	Timestamp	
  */
  const uint8_t templateNormal[4] = {START_BYTE, PACKET_COMMAND, OWN_ADDRESS, RECEIVER};
  const uint8_t templateResponce[4] = {START_BYTE, PACKET_COMMAND_RESPONCE, OWN_ADDRESS, RECEIVER};

  uint8_t packet[COMMAND_SIZE];

  if (responce_command) {
    memcpy(packet, templateResponce, 4);
  } else {
    memcpy(packet, templateNormal, 4);
  }

  uint32_t timestamp = 12345678;
  memcpy(&packet[4], &timestamp, sizeof(uint32_t));

  // check if size of packet is less than 58 bytes
  if (sizeof(packet) > 255) {
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
    uint8_t commandResponse = 0x02;
    packet[9] = commandResponse;
  }


  packet_send.clear();  // Clear the vector if necessary
  packet_send.insert(packet_send.end(), std::begin(packet), std::end(packet));


  // ResponseStatus response = loraFCMS.sendMessage(packet, sizeof(packet));
  // if (response.code != 1) {
  //   Serial.println(response.getResponseDescription());
  // } else {
  //   Serial.println("Packet sent...");
  // }
}
  

void parseHeader(uint8_t* packet, size_t length) {
  if (length != COMMAND_SIZE && length != TELEMETRY_SIZE) {
    Serial.println("Invalid packet size! 1");
    return;
  }
  uint8_t startByte = packet[0];
  if (startByte != START_BYTE) {
    Serial.println("Invalid start byte! 2");
    return;
  }

  uint8_t sender = packet[2];
  uint8_t receiver = packet[3];
  if (receiver != OWN_ADDRESS ){
    Serial.println("Not my address!");
    return;
  }

  uint8_t packetType = packet[1];
  if (packetType == PACKET_COMMAND) {
    if (length != COMMAND_SIZE) {
      Serial.println("Invalid packet size! 3");
      return;
    }
    receiveCommand(packet, length);
  } else if (packetType == PACKET_COMMAND_RESPONCE) {
    if (length != COMMAND_SIZE) {
      Serial.println("Invalid packet size! 4");
      return;
    }
    receiveCommand(packet, length, true);
  } else if (packetType == PACKET_TELEMETRY) {
    if (length != TELEMETRY_SIZE) {
      Serial.println("Invalid packet size! 5");
      return;
    }
    receiveTelemetry(packet, length);
  } else {
    Serial.println("Invalid packet type! 6");
  }
}
  

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);
  loraFCMS.begin();
  Serial.println("LoRa module initialized...");
}

void loop() {
  std::vector<uint8_t> packet_end;
  sendTelemetry(packet_end);
  parseHeader(packet_end.data(), packet_end.size());

  // sendCommand(packet_end, true);
  // parseHeader(packet_end.data(), packet_end.size());
  Serial.println();
  
  delay(3000);

  // sendTelemetry();

  // // receive telemetry
  // ResponseContainer response = loraFCMS.receiveMessage();
  // uint8_t* dataBuffer = (uint8_t*)response.data.c_str();
  // size_t dataSize = response.data.length();

  // receiveTelemetry(dataBuffer, dataSize);
}
