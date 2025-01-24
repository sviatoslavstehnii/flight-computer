#include "lora.h"


void LORA::setup(){
    Serial.print("Initializing LoRa... ");
    Serial3.begin(115200);
    Serial.println("Success");
}

uint32_t LORA::getMyAddress() const {
    return myAddr;
}

TelemetryPacket& LORA::receiveTelemetry(uint8_t* buffer, size_t length) {
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

  packet.telemetry.flightState = buffer[13];

  // Parse barometric altitude
  uint16_t barometricAltitude;
  memcpy(&barometricAltitude, &buffer[14], sizeof(uint16_t));
  packet.telemetry.barometricAlt = barometricAltitude;

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

  // 4 bytes reserved

  // Parse fins
  packet.telemetry.fin1 = buffer[44];
  packet.telemetry.fin2 = buffer[45];
  packet.telemetry.fin3 = buffer[46];
  packet.telemetry.fin4 = buffer[47];
  //Serial.printf("Fins: [%02X, %02X, %02X, %02X]\n", fin0, fin1, fin2, fin3);

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
  int16_t apogee;
  memcpy(&apogee, &buffer[71], sizeof(int16_t));
  //Serial.printf("Apogee=%04X\n", apogee);
  packet.telemetry.apogee=apogee;

  uint8_t pyroFlags = buffer[81];
  uint8_t pyroFlags_ad = buffer[82];
  // Parse pyro flags
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

std::optional<PacketType> LORA::parseTraffic(){
    if (!Serial3.available()){
        return;
    }

    //uint8_t packet[128]{}; 
    size_t length = Serial3.readBytesUntil('\n', packet_, 128);
    if (length > 1) {
        --length; // \r skip
    }

    
    if (length != COMMAND_SIZE && length != TELEMETRY_SIZE && length != RESPONSE_SIZE) {
        Serial.printf("Invalid packet size of %d\n", length);
        return;
    }
    uint8_t startByte = packet_[0];
    if (startByte != START_BYTE) {
        Serial.println("Invalid start byte!");
        return;
    }

    uint8_t sender = packet_[2];
    uint8_t receiver = packet_[3];
    if (receiver != myAddr){
        Serial.println("Not my address!");
        return;
    }

    if (sender == myAddr){
        Serial.println("Received my own packet (-_-)");
        return;
    }

    uint8_t packetType = packet_[1];
    if (packetType == PACKET_COMMAND) {
        if (length != COMMAND_SIZE) {
            Serial.printf("Invalid packet size! Expected COMMAND of %d, got %d\n", COMMAND_SIZE, length);
            return;
        }
        return PACKET_COMMAND;
    } else if (packetType == PACKET_RESPONSE) {
        if (length != RESPONSE_SIZE) {
            Serial.printf("Invalid packet size! Expected RESPONSE of %d, got %d\n", RESPONSE_SIZE, length);
            return;
        }
        return PACKET_RESPONSE;
    } else if (packetType == PACKET_TELEMETRY) {
        if (length != TELEMETRY_SIZE) {
            Serial.printf("Invalid packet size! Expected TELEMETRY of %d, got %d\n", TELEMETRY_SIZE, length);
            return;
        }
        return PACKET_TELEMETRY;
    } else {
        Serial.printf("Invalid packet type! %d\n", packetType);
    }
}

void receivePacket(PacketType packetType, uint8_t *packet){
//    if (packetType == TelemetryPacket){
        
  //  }
}

void sendTelemetry(TelemetryPacket &packet){
     uint8_t buffer[TELEMETRY_SIZE] = {};

    buffer[0] = packet.startByte;
    buffer[1] = packet.getPacketType();
    buffer[2] = packet.sender;
    buffer[3] = packet.receiver;

    // Timestamp (4 bytes)
    memcpy(&buffer[4], &packet.timestamp, sizeof(uint32_t));

    // Sequence ID (4 bytes)
    memcpy(&buffer[8], &packet.seq_id, sizeof(uint32_t));

    // Flight State (1 byte)
    buffer[13] = packet.telemetry.flightState;

    // Barometric altitude (2 bytes)
    memcpy(&buffer[14], &packet.telemetry.barometricAlt, sizeof(int16_t));

    // IMU Data (12 bytes)
    memcpy(&buffer[16], &packet.telemetry.imuData, sizeof(ImuData));

    // Fins (4 bytes)
    buffer[44] = packet.telemetry.fin1;
    buffer[45] = packet.telemetry.fin2;
    buffer[46] = packet.telemetry.fin3;
    buffer[47] = packet.telemetry.fin4;

    // Battery data (4 bytes)
    memcpy(&buffer[48], &packet.telemetry.mVBat, sizeof(uint16_t));  // mVBat
    memcpy(&buffer[50], &packet.telemetry.mABat, sizeof(uint16_t));  // mABat

    // Load cell (2 bytes)
    memcpy(&buffer[52], &packet.telemetry.loadCell, sizeof(uint16_t));

    // Temperature and additional data (4 bytes)
    buffer[54] = packet.telemetry.temp;
    buffer[55] = packet.telemetry.mejPercent;
    buffer[56] = packet.telemetry.cjPercent;
    buffer[57] = packet.telemetry.datajournalPercent;

    // GPS data (12 bytes)
    memcpy(&buffer[58], &packet.telemetry.gpsLat, sizeof(int32_t));
    memcpy(&buffer[62], &packet.telemetry.gpsLon, sizeof(int32_t));
    memcpy(&buffer[66], &packet.telemetry.gpsAlt, sizeof(int16_t));

    // Detections (3 bytes)
    buffer[68] = packet.telemetry.takeOffDetection;
    buffer[69] = packet.telemetry.apogeeDetection;
    buffer[70] = packet.telemetry.landingDetection;

    // Apogee (2 bytes)
    memcpy(&buffer[71], &packet.telemetry.apogee, sizeof(int16_t));

    // Pyro flags (2 bytes)
    uint8_t pyroFlags = 0;
    pyroFlags |= (packet.telemetry.pyroFlags.pyro1_safe << 0);
    pyroFlags |= (packet.telemetry.pyroFlags.pyro1_cont << 1);
    pyroFlags |= (packet.telemetry.pyroFlags.pyro1_fire << 2);
    pyroFlags |= (packet.telemetry.pyroFlags.pyro2_safe << 3);
    pyroFlags |= (packet.telemetry.pyroFlags.pyro2_cont << 4);
    pyroFlags |= (packet.telemetry.pyroFlags.pyro2_fire << 5);
    pyroFlags |= (packet.telemetry.pyroFlags.pyro3_safe << 6);
    pyroFlags |= (packet.telemetry.pyroFlags.pyro3_cont << 7);
    buffer[81] = pyroFlags;

    uint8_t pyroFlagsAd = 0;
    pyroFlagsAd |= (packet.telemetry.pyroFlags.pyro3_fire << 0);
    buffer[82] = pyroFlagsAd;

    buffer[83] = '\r';
    buffer[84] = '\n';

    Serial3.write(buffer, TELEMETRY_SIZE);
    Serial.println("Sent Telemetry Packet");
}