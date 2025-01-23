#include "LORA.h"
#include <vector>
#include <FCMS.h>

#define START_BYTE 0xAA
#define RECEIVER 0x00


#define TELEMETRY_SIZE 83
#define COMMAND_SIZE 21
#define RESPONSE_SIZE 20

enum PACKET_TYPE{
  PACKET_TELEMETRY=0x01,
  PACKET_COMMAND=0x02,
  PACKET_RESPONSE=0x05
};

void sendPing(uint8_t receiverAddr){
  Serial3.println();
};

void LORA::setup(){
    Serial.print("Initializing LoRa... ");
    Serial3.begin(115200);
    Serial.println("Success");
}

TelemetryPacket LORA::receiveTelemetry(uint8_t* buffer, size_t length) {
  TelemetryPacket packet{};
  // Parse header
  packet.sender = buffer[2];
  packet.receiver = buffer[3];
  uint32_t timestamp;
  memcpy(&timestamp, &buffer[4], sizeof(uint32_t));
  packet.timestamp = timestamp;
  uint32_t seq_id;
  memcpy(&seq_id, &buffer[8], sizeof(uint32_t));
  packet.seq_id = seq_id;


  //Serial.printf("Header: StartByte=%02X, PacketType=%02X, Sender=%02X, Receiver=%02X, Timestamp=%u\n", 
  //             startByte, packetType, sender, receiver, timestamp);

  // Parse flight and additional states
  //uint8_t flightState = states & 0x0F;
  //uint8_t additionalState = (states >> 4) & 0x0F;
  packet.telemetry.flightState = buffer[13];

  //Serial.printf("FlightState=%02X, AdditionalState=%02X\n", flightState, additionalState);

  // Parse barometric altitude
  uint16_t barometricAltitude;
  memcpy(&barometricAltitude, &buffer[14], sizeof(uint16_t));
  packet.telemetry.barometricAlt = barometricAltitude;
  // Serial.printf("Barometric Altitude=%04X\n", barometricAltitude);

  // 4 bytes reserved

  // Parse IMU data
  ImuData imuData;
  memcpy(&imuData, &buffer[16], sizeof(ImuData));
  Serial.printf("IMU Data: Yaw=%04X, Pitch=%04X, Roll=%04X, AccelX=%04X, AccelY=%04X, AccelZ=%04X, "
                "VelX=%04X, VelY=%04X, VelZ=%04X, PosX=%04X, PosY=%04X, PosZ=%04X\n",
                imuData.yaw, imuData.pitch, imuData.roll, 
                imuData.accel_x, imuData.accel_y, imuData.accel_z,
                imuData.velocity_x, imuData.velocity_y, imuData.velocity_z,
                imuData.position_x, imuData.position_y, imuData.position_z);

  packet.telemetry.imuData = imuData;
  // Parse fins
  packet.telemetry.fin1 = buffer[40];
  packet.telemetry.fin2 = buffer[41];
  packet.telemetry.fin3 = buffer[42];
  packet.telemetry.fin4 = buffer[43];
  //Serial.printf("Fins: [%02X, %02X, %02X, %02X]\n", fin0, fin1, fin2, fin3);

  // 4 bytes reserved

  // Parse battery data
  uint16_t mvBat, maBat;
  memcpy(&mvBat, &buffer[48], sizeof(uint16_t));
  memcpy(&maBat, &buffer[50], sizeof(uint16_t));
  packet.telemetry.mVBat = mvBat;
  packet.telemetry.mABat = maBat;
  // Serial.printf("Battery: Voltage=%04X, Current=%04X\n", vBat, currBat);

  uint16_t loadCell;
  memcpy(&loadCell, &buffer[52], sizeof(uint16_t));
  //Serial.printf("LoadCell=%04X\n", loadCell);
  packet.telemetry.loadCell = loadCell;


  // Parse temperature and data
  packet.telemetry.temp = buffer[54];
  packet.telemetry.mejPercent = buffer[55];
  packet.telemetry.cjPercent = buffer[56];
  packet.telemetry.datajournalPercent = buffer[57];
  //Serial.printf("Temp=%02X, MEL=%02X, CL=%02X, Datalog=%02X\n", temp, melData, clData, datalogData);

  // Parse GPS data
  int32_t gpsLatitude, gpsLongitude;
  int16_t gpsAltitude;
  memcpy(&gpsLatitude, &buffer[58], sizeof(int32_t));
  memcpy(&gpsLongitude, &buffer[62], sizeof(int32_t));
  memcpy(&gpsAltitude, &buffer[66], sizeof(int16_t));
  //Serial.printf("GPS: Latitude=%08X, Longitude=%08X, Altitude=%04X\n", 
  //              gpsLatitude, gpsLongitude, gpsAltitude);
  packet.telemetry.gpsLat=gpsLatitude;
  packet.telemetry.gpsLon=gpsLongitude;
  packet.telemetry.gpsAlt=gpsAltitude;

  // Parse detections
  uint8_t detections[3];
  memcpy(detections, &buffer[68], sizeof(detections));
  Serial.printf("Detections: [%02X, %02X, %02X]\n", detections[0], detections[1], detections[2]);

  // Parse apogee
  uint16_t apogee;
  memcpy(&apogee, &buffer[71], sizeof(uint16_t));
  //Serial.printf("Apogee=%04X\n", apogee);
  packet.telemetry.apogee=apogee;

  // Parse events data
  // uint8_t events_data[16]; 
  // int index = 0; // To track the index in events_data

  // for (int i = 0; i < 4; i++) {
  //     // Unpack each event from the byte
  //     events_data[index++] = packet[200 + i] & 0x03;               // Extract Event 0 (bits 0-1)
  //     events_data[index++] = (packet[200 + i] >> 2) & 0x03;        // Extract Event 1 (bits 2-3)
  //     events_data[index++] = (packet[200 + i] >> 4) & 0x03;        // Extract Event 2 (bits 4-5)
  //     events_data[index++] = (packet[200 + i] >> 6) & 0x03;        // Extract Event 3 (bits 6-7)
  // }
  // for (int i = 0; i < 16; i++) {
  //     Serial.printf("Event %d: %02X\n", i, events_data[i]);
  // }

  // uint16_t event0Time, event1Time;
  // memcpy(&event0Time, &packet[204], sizeof(uint16_t));
  // memcpy(&event1Time, &packet[206], sizeof(uint16_t));
  // Serial.printf("EventTimes: Event0=%04X, Event1=%04X\n", event0Time, event1Time);

  uint8_t pyroFlags = buffer[81];
  uint8_t pyroFlags_ad = buffer[82];
  // Parse pyro flags
  bool extractedPyroFlags[8];
  //bool extractedPyroFlags_ad[8];
  int j=0;
  packet.telemetry.pyroFlags.pyro1_safe=(pyroFlags >> j++) & 0x01;
  packet.telemetry.pyroFlags.pyro1_cont=(pyroFlags >> j++) & 0x01;
  packet.telemetry.pyroFlags.pyro1_fire=(pyroFlags >> j++) & 0x01;
  packet.telemetry.pyroFlags.pyro2_safe=(pyroFlags >> j++) & 0x01;
  packet.telemetry.pyroFlags.pyro2_cont=(pyroFlags >> j++) & 0x01;
  packet.telemetry.pyroFlags.pyro2_fire=(pyroFlags >> j++) & 0x01;
  packet.telemetry.pyroFlags.pyro3_safe=(pyroFlags >> j++) & 0x01;
  packet.telemetry.pyroFlags.pyro3_cont=(pyroFlags >> j++) & 0x01;
  j=0;
  packet.telemetry.pyroFlags.pyro3_fire=(pyroFlags_ad >> j++) & 0x01;

  Serial.println("Telemetry data parsed successfully.");
  return packet;
}

void LORA::parseTraffic(){
    if (!Serial3.available()){
        return;
    }

    uint8_t packet[128]{}; 
    size_t length = Serial3.readBytesUntil('\n', packet, 128);
    if (length > 1) {
        --length; // \r skip
    }

    
    if (length != COMMAND_SIZE && length != TELEMETRY_SIZE && length != RESPONSE_SIZE) {
        Serial.print("Invalid packet size of ");
        Serial.println(length);
        return;
    }
    uint8_t startByte = packet[0];
    if (startByte != START_BYTE) {
        Serial.println("Invalid start byte! 2");
        return;
    }

    uint8_t sender = packet[2];
    uint8_t receiver = packet[3];
    if (receiver != myAddr){
        Serial.println("Not my address!");
        return;
    }

    uint8_t packetType = packet[1];
    if (packetType == PACKET_COMMAND) {
        if (length != COMMAND_SIZE) {
            Serial.printf("Invalid packet size! Expected COMMAND of %d, got %d\n", COMMAND_SIZE, length);
            return;
        }
        receiveCommand(packet, length);
    } else if (packetType == PACKET_RESPONSE) {
        if (length != RESPONSE_SIZE) {
            Serial.printf("Invalid packet size! Expected RESPONSE of %d, got %d\n", RESPONSE_SIZE, length);
            return;
        }
        receiveResponse(packet, length);
    } else if (packetType == PACKET_TELEMETRY) {
        if (length != TELEMETRY_SIZE) {
            Serial.printf("Invalid packet size! Expected TELEMETRY of %d, got %d\n", TELEMETRY_SIZE, length);
            return;
        }
        receiveTelemetry(packet, length);
    } else {
        Serial.printf("Invalid packet type! %d\n", packetType);
    }
}
