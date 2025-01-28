#include "lora.h"


void LORA::setup(){
    Serial.print("Initializing LoRa... ");
    Serial3.begin(115200);
    Serial.println("Success");
    Serial3.println("DEB");

    // int time_s = 10;
    // size_t start_time = millis();
    // while(millis() - start_time < time_s * 1000){
    //     if (Serial3.available()){
    //         char c = Serial3.read();
    //         Serial.print(c);
    //     }
    // }
}

uint32_t LORA::getMyAddress() const {
    return myAddr;
}

bool LORA::available() const
{
    return Serial3.available();
}

size_t LORA::read(uint8_t *buffer, size_t size) const
{
    Serial.println(Serial3.readStringUntil('\n'));
    return 0;
    //return Serial3.readBytesUntil('\n', buffer, size);
}

void LORA::sendResponse(uint8_t receiver, Response& response)
{
    Serial.println("Sending response");
    ResponsePacket packet;
    Serial.println("Creating response packet");
    packet.receiver = receiver;
    packet.sender = myAddr;
    packet.sequenceId = seqID_++;
    packet.timestamp = millis();
    packet.response = response;
    Serial.println("Created response packet");

    uint8_t buffer[RESPONSE_SIZE] = {};

    buffer[0] = packet.startByte;
    buffer[1] = packet.getPacketType();
    buffer[2] = packet.sender;
    buffer[3] = packet.receiver;

    // Timestamp (4 bytes)
    memcpy(&buffer[4], &packet.timestamp, sizeof(uint32_t));

    // Sequence ID (4 bytes)
    memcpy(&buffer[8], &packet.sequenceId, sizeof(uint32_t));

    memcpy(&buffer[12], &packet.response.commandSeqId, sizeof(uint32_t));
    memcpy(&buffer[16], &packet.response.data, sizeof(packet.response.data));

    buffer[20] = '\r';
    buffer[21] = '\n';

    Serial3.write(buffer, RESPONSE_SIZE);
}

TelemetryPacket LORA::receiveTelemetry(uint8_t *buffer)
{
    TelemetryPacket packet{};
    // Parse header
    packet.sender = buffer[2];
    packet.receiver = buffer[3];
    uint32_t timestamp;
    memcpy(&timestamp, &buffer[4], sizeof(uint32_t));
    packet.timestamp = timestamp;
    uint32_t seq_id;
    memcpy(&seq_id, &buffer[8], sizeof(uint32_t));
    packet.sequenceId = seq_id;

    packet.telemetry.flightState = buffer[13];

    // Parse barometric altitude
    uint16_t barometricAltitude;
    memcpy(&barometricAltitude, &buffer[14], sizeof(uint16_t));
    packet.telemetry.barometricAlt = barometricAltitude;

    // Parse IMU data
    ImuData imuData{};
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
    // Serial.printf("Fins: [%02X, %02X, %02X, %02X]\n", fin0, fin1, fin2, fin3);

    // Parse battery data
    uint16_t mvBat, maBat;
    memcpy(&mvBat, &buffer[48], sizeof(uint16_t));
    memcpy(&maBat, &buffer[50], sizeof(uint16_t));
    packet.telemetry.mVBat = mvBat;
    packet.telemetry.mABat = maBat;
    // Serial.printf("Battery: Voltage=%04X, Current=%04X\n", vBat, currBat);

    uint16_t loadCell;
    memcpy(&loadCell, &buffer[52], sizeof(uint16_t));
    // Serial.printf("LoadCell=%04X\n", loadCell);
    packet.telemetry.loadCell = loadCell;

    // Parse temperature and data
    packet.telemetry.temp = buffer[54];
    packet.telemetry.mejPercent = buffer[55];
    packet.telemetry.cjPercent = buffer[56];
    packet.telemetry.datajournalPercent = buffer[57];
    // Serial.printf("Temp=%02X, MEL=%02X, CL=%02X, Datalog=%02X\n", temp, melData, clData, datalogData);

    // Parse GPS data
    int32_t gpsLatitude, gpsLongitude;
    int16_t gpsAltitude;
    memcpy(&gpsLatitude, &buffer[58], sizeof(int32_t));
    memcpy(&gpsLongitude, &buffer[62], sizeof(int32_t));
    memcpy(&gpsAltitude, &buffer[66], sizeof(int16_t));
    // Serial.printf("GPS: Latitude=%08X, Longitude=%08X, Altitude=%04X\n",
    //               gpsLatitude, gpsLongitude, gpsAltitude);
    packet.telemetry.gpsLat = gpsLatitude;
    packet.telemetry.gpsLon = gpsLongitude;
    packet.telemetry.gpsAlt = gpsAltitude;

    // Parse detections
    uint8_t detections[3];
    memcpy(detections, &buffer[68], sizeof(detections));
    Serial.printf("Detections: [%02X, %02X, %02X]\n", detections[0], detections[1], detections[2]);

    // Parse apogee
    int16_t apogee;
    memcpy(&apogee, &buffer[71], sizeof(int16_t));
    // Serial.printf("Apogee=%04X\n", apogee);
    packet.telemetry.apogee = apogee;

    uint8_t pyroFlags = buffer[81];
    uint8_t pyroFlags_ad = buffer[82];
    // Parse pyro flags
    int j = 0;
    packet.telemetry.pyroFlags.pyro1_safe = (pyroFlags >> j++) & 0x01;
    packet.telemetry.pyroFlags.pyro1_cont = (pyroFlags >> j++) & 0x01;
    packet.telemetry.pyroFlags.pyro1_fire = (pyroFlags >> j++) & 0x01;
    packet.telemetry.pyroFlags.pyro2_safe = (pyroFlags >> j++) & 0x01;
    packet.telemetry.pyroFlags.pyro2_cont = (pyroFlags >> j++) & 0x01;
    packet.telemetry.pyroFlags.pyro2_fire = (pyroFlags >> j++) & 0x01;
    packet.telemetry.pyroFlags.pyro3_safe = (pyroFlags >> j++) & 0x01;
    packet.telemetry.pyroFlags.pyro3_cont = (pyroFlags >> j++) & 0x01;
    j = 0;
    packet.telemetry.pyroFlags.pyro3_fire = (pyroFlags_ad >> j++) & 0x01;

    Serial.println("Telemetry data parsed successfully.");
    return packet;
}

ResponsePacket LORA::receiveResponse(uint8_t *buffer)
{
    ResponsePacket packet{};
    // Parse header
    packet.sender = buffer[2];
    packet.receiver = buffer[3];
    uint32_t timestamp;
    memcpy(&timestamp, &buffer[4], sizeof(uint32_t));
    packet.timestamp = timestamp;
    uint32_t seq_id;
    memcpy(&seq_id, &buffer[8], sizeof(uint32_t));
    packet.sequenceId = seq_id;

    // Parse response
    uint32_t command_seq_id;
    memcpy(&command_seq_id, &buffer[12], sizeof(uint32_t));
    packet.response.commandSeqId = command_seq_id;
    memcpy(&packet.response.data, &buffer[16], sizeof(packet.response.data));

    Serial.println("Response data parsed successfully.");
    return packet;
}

std::optional<PacketType> LORA::parseHeader(uint8_t *buffer, size_t length){
    // if (length != COMMAND_SIZE && length != TELEMETRY_SIZE && length != RESPONSE_SIZE) {
    //     Serial.printf("Invalid packet size of %d\n", length);
    //     return;
    // }
    Serial.println("Parsing header");
    uint8_t startByte = buffer[0];
    if (startByte != START_BYTE) {
        Serial.println("Invalid start byte!");
        return;
    }

    uint8_t sender = buffer[2];
    uint8_t receiver = buffer[3];
    if (receiver != myAddr){
        Serial.println("Not my address!");
        return;
    }

    if (sender == myAddr){
        Serial.println("Received my own packet (-_-)");
        return;
    }

    uint8_t packetType = buffer[1];
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

CommandPacket LORA::receiveCommand(uint8_t *buffer)
{
    CommandPacket packet{};
    packet.sender = buffer[2];
    packet.receiver = buffer[3];
    uint32_t timestamp;
    memcpy(&timestamp, &buffer[4], sizeof(uint32_t));
    packet.timestamp = timestamp;
    uint32_t seq_id;
    memcpy(&seq_id, &buffer[8], sizeof(uint32_t));
    packet.sequenceId = seq_id;
    packet.command.commandId = static_cast<COMMAND_ID>(buffer[12]);
    memcpy(&packet.command.args, &buffer[13], sizeof(packet.command.args));
    return packet;
}

void LORA::sendCommand(uint8_t receiverAddr, Command& command)
{
    CommandPacket packet;
    packet.receiver = receiverAddr;
    packet.sender = myAddr;
    packet.sequenceId = seqID_++;
    packet.timestamp = millis();
    packet.command = command;

    uint8_t buffer[COMMAND_SIZE] = {};

    buffer[0] = packet.startByte;
    buffer[1] = packet.getPacketType();
    buffer[2] = packet.sender;
    buffer[3] = packet.receiver;

    // Timestamp (4 bytes)
    memcpy(&buffer[4], &packet.timestamp, sizeof(uint32_t));

    // Sequence ID (4 bytes)
    memcpy(&buffer[8], &packet.sequenceId, sizeof(uint32_t));

    buffer[12] = packet.command.commandId;
    memcpy(&buffer[13], &packet.command.args, sizeof(packet.command.args));
    buffer[21] = '\r';
    buffer[22] = '\n';

    Serial3.println("DEB");
    Serial3.write(buffer, COMMAND_SIZE);
    Serial.println("Sent Command Packet");
}

void LORA::sendTelemetry(TelemetryPacket& packet)
{
    uint8_t buffer[TELEMETRY_SIZE] = {};

    buffer[0] = packet.startByte;
    buffer[1] = packet.getPacketType();
    buffer[2] = packet.sender;
    buffer[3] = packet.receiver;

    for(int i=0; i<TELEMETRY_SIZE; i++){
        buffer[i] = 0;
    }


    // // Timestamp (4 bytes)
    // memcpy(&buffer[4], &packet.timestamp, sizeof(uint32_t));

    // // Sequence ID (4 bytes)
    // memcpy(&buffer[8], &packet.sequenceId, sizeof(uint32_t));

    // // Flight State (1 byte)
    // buffer[13] = packet.telemetry.flightState;

    // // Barometric altitude (2 bytes)
    // memcpy(&buffer[14], &packet.telemetry.barometricAlt, sizeof(int16_t));

    // // IMU Data (12 bytes)
    // memcpy(&buffer[16], &packet.telemetry.imuData, sizeof(ImuData));

    // // Fins (4 bytes)
    // buffer[44] = packet.telemetry.fin1;
    // buffer[45] = packet.telemetry.fin2;
    // buffer[46] = packet.telemetry.fin3;
    // buffer[47] = packet.telemetry.fin4;

    // // Battery data (4 bytes)
    // memcpy(&buffer[48], &packet.telemetry.mVBat, sizeof(uint16_t));  // mVBat
    // memcpy(&buffer[50], &packet.telemetry.mABat, sizeof(uint16_t));  // mABat

    // // Load cell (2 bytes)
    // memcpy(&buffer[52], &packet.telemetry.loadCell, sizeof(uint16_t));

    // // Temperature and additional data (4 bytes)
    // buffer[54] = packet.telemetry.temp;
    // buffer[55] = packet.telemetry.mejPercent;
    // buffer[56] = packet.telemetry.cjPercent;
    // buffer[57] = packet.telemetry.datajournalPercent;

    // // GPS data (12 bytes)
    // memcpy(&buffer[58], &packet.telemetry.gpsLat, sizeof(int32_t));
    // memcpy(&buffer[62], &packet.telemetry.gpsLon, sizeof(int32_t));
    // memcpy(&buffer[66], &packet.telemetry.gpsAlt, sizeof(int16_t));

    // // Detections (3 bytes)
    // buffer[68] = packet.telemetry.takeOffDetection;
    // buffer[69] = packet.telemetry.apogeeDetection;
    // buffer[70] = packet.telemetry.landingDetection;

    // // Apogee (2 bytes)
    // memcpy(&buffer[71], &packet.telemetry.apogee, sizeof(int16_t));

    // // Pyro flags (2 bytes)
    // uint8_t pyroFlags = 0;
    // pyroFlags |= (packet.telemetry.pyroFlags.pyro1_safe << 0);
    // pyroFlags |= (packet.telemetry.pyroFlags.pyro1_cont << 1);
    // pyroFlags |= (packet.telemetry.pyroFlags.pyro1_fire << 2);
    // pyroFlags |= (packet.telemetry.pyroFlags.pyro2_safe << 3);
    // pyroFlags |= (packet.telemetry.pyroFlags.pyro2_cont << 4);
    // pyroFlags |= (packet.telemetry.pyroFlags.pyro2_fire << 5);
    // pyroFlags |= (packet.telemetry.pyroFlags.pyro3_safe << 6);
    // pyroFlags |= (packet.telemetry.pyroFlags.pyro3_cont << 7);
    // buffer[81] = pyroFlags;

    // uint8_t pyroFlagsAd = 0;
    // pyroFlagsAd |= (packet.telemetry.pyroFlags.pyro3_fire << 0);
    // buffer[82] = pyroFlagsAd;

    buffer[TELEMETRY_SIZE-2] = '\r';
    buffer[TELEMETRY_SIZE-1] = '\n';

    Serial3.write(buffer, TELEMETRY_SIZE);
}