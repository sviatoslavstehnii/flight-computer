#include "ducc_driver.h"


void DUCCDriver::setup(){
    Serial.print("Initializing Comms... ");
    serial.begin(115200);
    Serial.println("Done");
    // serial.println("DEB");

    // int time_s = 10;
    // size_t start_time = millis();
    // while(millis() - start_time < time_s * 1000){
    //     if (serial.available()){
    //         char c = serial.read();
    //         Serial.print(c);
    //     }
    // }
}

uint32_t DUCCDriver::getMyAddress() const {
    return myAddr;
}

bool DUCCDriver::available() const
{
    return serial.available();
}

std::unique_ptr<BasePacketRx> DUCCDriver::read() {
    while (reset) {
        if (ringBuffer.isEmpty()) {
            reset = false;
        } else {
            auto peekByte = ringBuffer.peek();
            if (peekByte.has_value() && peekByte.value() == START_BYTE) {
                packetFound = true;
                reset = false;
            } else {
                ringBuffer.pop();
            }
        }
    }
    
    while (serial.available() && !ringBuffer.isFull()){
        uint8_t readByte = serial.read();
        // Serial.print(readByte, HEX);
        // Serial.print(" ");
        
        if (!packetFound && readByte == START_BYTE) {
            packetFound = true;
            ringBuffer.clear();
        }
        
        ringBuffer.push(readByte);
    }
    
    if (ringBuffer.size() >= DUCC_HEADER_SIZE && packetFound) {
        uint8_t header[DUCC_HEADER_SIZE];
        if (!ringBuffer.copy(header, DUCC_HEADER_SIZE)) {
            // reset = true;
            // ringBuffer.pop();
            // packetFound = false;
            return nullptr;
        }
        
        uint8_t length = header[1];
        // Serial.printf("\nLen %d\n", length);
        if (length > MAX_PAYLOAD_SIZE || length < 2) {
            reset = true;
            ringBuffer.pop();
            packetFound = false;
            return nullptr;
        }
        
        if (ringBuffer.size() >= length + DUCC_HEADER_SIZE) {
            uint8_t packet[DUCC_HEADER_SIZE+length];
            memcpy(packet, header, DUCC_HEADER_SIZE);
            ringBuffer.erase(DUCC_HEADER_SIZE);
            if (!ringBuffer.copy(&packet[DUCC_HEADER_SIZE], length)) {
                // reset = true;
                // ringBuffer.pop();
                // packetFound = false;
                return nullptr;
            }
            
            uint8_t crcPacket = packet[length+1];
            // Calculate CRC: start_byte + len + rssi + snr + payload + crc
            // len = 5 + payload
            // crc is calculated over rssi + snr + payload
            uint8_t calculatedCrc = calcCRC8(&packet[2], length-1);
            // Serial.printf("\nCRC: %d/%d\n", crcPacket, calculatedCrc);
            // for (int i = 0; i < length+3;++i){
            //     Serial.print(packet[i], HEX);
            //     Serial.print(" ");
            // }

            if (calculatedCrc == crcPacket) {
                ringBuffer.erase(DUCC_HEADER_SIZE + length);
                return parseHeader(header, &packet[DUCC_HEADER_SIZE]);
            } else {
                reset = true;
                ringBuffer.pop();
                packetFound = false;
            }
        }
    }
    return nullptr;
}

std::unique_ptr<BasePacketRx> DUCCDriver::parseHeader(uint8_t* header, uint8_t* data) {
    uint8_t length = header[1];
    if (length > MAX_PAYLOAD_SIZE) return nullptr;
    
    if (data[5] != getMyAddress()) {
        Serial.println("This packet is not for me");
        return nullptr;
    }

    uint8_t packetType = data[3];
    switch (packetType) {
        case PACKET_TELEMETRY:
            return std::make_unique<TelemetryPacketRx>(receiveTelemetry(data));
        case PACKET_RESPONSE:
            return std::make_unique<ResponsePacketRx>(receiveResponse(data));
        case PACKET_COMMAND:
            return std::make_unique<CommandPacketRx>(receiveCommand(data));
        default:
            return nullptr;
    }
}

void DUCCDriver::sendResponse(ResponsePacketTx& packet)
{
    Serial.println("Sending response");
    uint8_t sys_len = packet.getPayloadLen()+DUCC_HEADER_SIZE+AVDCP2_HEADER_SIZE+DUCC_CRC_SIZE;
    uint8_t buffer[sys_len] = {};

    packHeaders(buffer, packet);

    // Command Payload
    memcpy(&buffer[13], &packet.response.commandSeqId, sizeof(uint32_t));
    memcpy(&buffer[17], &packet.response.data, sizeof(packet.response.data));

    buffer[sys_len-1] = calcCRC8(&buffer[2], AVDCP2_HEADER_SIZE+packet.getPayloadLen());
    serial.write(buffer, sys_len);
}

TelemetryPacketRx DUCCDriver::receiveTelemetry(uint8_t *buffer)
{
    TelemetryPacketRx packet{};
    memcpy(&packet.rssi, &buffer[0], sizeof(int16_t));
    packet.snr = buffer[2];
    packet.sender = buffer[4];
    packet.receiver = buffer[5];
    uint32_t timestamp;
    memcpy(&timestamp, &buffer[6], sizeof(uint32_t));
    packet.timestamp = timestamp;
    uint32_t seq_id;
    memcpy(&seq_id, &buffer[10], sizeof(uint32_t));
    packet.sequenceId = seq_id;

    packet.telemetry.flightState = buffer[14];

    // Parse barometric altitude
    uint16_t barometricAltitude;
    memcpy(&barometricAltitude, &buffer[15], sizeof(uint16_t));
    packet.telemetry.barometricAlt = barometricAltitude;

    // Parse IMU data
    ImuData imuData{};
    memcpy(&imuData, &buffer[17], sizeof(ImuData));
    // Serial.printf("IMU Data: Yaw=%04X, Pitch=%04X, Roll=%04X, AccelX=%04X, AccelY=%04X, AccelZ=%04X, "
    //               "VelX=%04X, VelY=%04X, VelZ=%04X, PosX=%04X, PosY=%04X, PosZ=%04X\n",
    //               imuData.yaw, imuData.pitch, imuData.roll,
    //               imuData.accel_x, imuData.accel_y, imuData.accel_z,
    //               imuData.velocity_x, imuData.velocity_y, imuData.velocity_z,
    //               imuData.position_x, imuData.position_y, imuData.position_z);

    packet.telemetry.imuData = imuData;

    // 4 bytes reserved

    // Parse fins
    // packet.telemetry.fin1 = buffer[45];
    // packet.telemetry.fin2 = buffer[46];
    // packet.telemetry.fin3 = buffer[47];
    // packet.telemetry.fin4 = buffer[48];
    // // Serial.printf("Fins: [%02X, %02X, %02X, %02X]\n", fin0, fin1, fin2, fin3);

    // // Parse battery data
    // uint16_t mvBat, maBat;
    // memcpy(&mvBat, &buffer[49], sizeof(uint16_t));
    // memcpy(&maBat, &buffer[51], sizeof(uint16_t));
    // packet.telemetry.mVBat = mvBat;
    // packet.telemetry.mABat = maBat;
    // // Serial.printf("Battery: Voltage=%04X, Current=%04X\n", vBat, currBat);

    // uint16_t loadCell;
    // memcpy(&loadCell, &buffer[53], sizeof(uint16_t));
    // // Serial.printf("LoadCell=%04X\n", loadCell);
    // packet.telemetry.loadCell = loadCell;

    // // Parse temperature and data
    // packet.telemetry.temp = buffer[55];
    // packet.telemetry.mejPercent = buffer[56];
    // packet.telemetry.cjPercent = buffer[57];
    // packet.telemetry.datajournalPercent = buffer[58];
    // // Serial.printf("Temp=%02X, MEL=%02X, CL=%02X, Datalog=%02X\n", temp, melData, clData, datalogData);

    // // Parse GPS data
    // int32_t gpsLatitude, gpsLongitude;
    // int16_t gpsAltitude;
    // memcpy(&gpsLatitude, &buffer[59], sizeof(int32_t));
    // memcpy(&gpsLongitude, &buffer[63], sizeof(int32_t));
    // memcpy(&gpsAltitude, &buffer[67], sizeof(int16_t));
    // // Serial.printf("GPS: Latitude=%08X, Longitude=%08X, Altitude=%04X\n",
    // //               gpsLatitude, gpsLongitude, gpsAltitude);
    // packet.telemetry.gpsLat = gpsLatitude;
    // packet.telemetry.gpsLon = gpsLongitude;
    // packet.telemetry.gpsAlt = gpsAltitude;

    // // Parse detections
    // uint8_t detections[3];
    // memcpy(detections, &buffer[69], sizeof(detections));
    // Serial.printf("Detections: [%02X, %02X, %02X]\n", detections[0], detections[1], detections[2]);

    // // Parse apogee
    // int16_t apogee;
    // memcpy(&apogee, &buffer[72], sizeof(int16_t));
    // // Serial.printf("Apogee=%04X\n", apogee);
    // packet.telemetry.apogee = apogee;

    // uint8_t pyroFlags = buffer[82];
    // uint8_t pyroFlags_ad = buffer[83];
    // // Parse pyro flags
    // int j = 0;
    // packet.telemetry.pyroFlags.pyro1_safe = (pyroFlags >> j++) & 0x01;
    // packet.telemetry.pyroFlags.pyro1_cont = (pyroFlags >> j++) & 0x01;
    // packet.telemetry.pyroFlags.pyro1_fire = (pyroFlags >> j++) & 0x01;
    // packet.telemetry.pyroFlags.pyro2_safe = (pyroFlags >> j++) & 0x01;
    // packet.telemetry.pyroFlags.pyro2_cont = (pyroFlags >> j++) & 0x01;
    // packet.telemetry.pyroFlags.pyro2_fire = (pyroFlags >> j++) & 0x01;
    // packet.telemetry.pyroFlags.pyro3_safe = (pyroFlags >> j++) & 0x01;
    // packet.telemetry.pyroFlags.pyro3_cont = (pyroFlags >> j++) & 0x01;
    // j = 0;
    // packet.telemetry.pyroFlags.pyro3_fire = (pyroFlags_ad >> j++) & 0x01;

    Serial.println("Telemetry data parsed successfully.");
    return packet;
}

ResponsePacketRx DUCCDriver::receiveResponse(uint8_t *buffer)
{
    ResponsePacketRx packet{};
    // Parse header
    memcpy(&packet.rssi, &buffer[0], sizeof(int16_t));
    packet.snr = buffer[2];
    packet.sender = buffer[4];
    packet.receiver = buffer[5];
    uint32_t timestamp;
    memcpy(&timestamp, &buffer[6], sizeof(uint32_t));
    packet.timestamp = timestamp;
    uint32_t seq_id;
    memcpy(&seq_id, &buffer[10], sizeof(uint32_t));
    packet.sequenceId = seq_id;

    // Parse response
    uint32_t command_seq_id;
    memcpy(&command_seq_id, &buffer[14], sizeof(uint32_t));
    packet.response.commandSeqId = command_seq_id;
    memcpy(&packet.response.data, &buffer[18], sizeof(packet.response.data));

    Serial.println("Response data parsed successfully.");
    return packet;
}

CommandPacketRx DUCCDriver::receiveCommand(uint8_t *buffer)
{
    CommandPacketRx packet{};
    // Parse header
    memcpy(&packet.rssi, &buffer[0], sizeof(int16_t));
    packet.snr = buffer[2];
    packet.sender = buffer[4];
    packet.receiver = buffer[5];
    uint32_t timestamp;
    memcpy(&timestamp, &buffer[6], sizeof(uint32_t));
    packet.timestamp = timestamp;
    uint32_t seq_id;
    memcpy(&seq_id, &buffer[11], sizeof(uint32_t));
    packet.sequenceId = seq_id;
    packet.command.commandId = static_cast<COMMAND_ID>(buffer[12]);
    memcpy(&packet.command.args, &buffer[16], sizeof(packet.command.args));
    return packet;
}

void DUCCDriver::sendCommand(CommandPacketTx& packet)
{
    uint8_t sys_len = packet.getPayloadLen()+DUCC_HEADER_SIZE+AVDCP2_HEADER_SIZE+DUCC_CRC_SIZE;
    uint8_t buffer[sys_len] = {};

    packHeaders(buffer, packet);

    buffer[13] = packet.command.commandId;
    memcpy(&buffer[14], &packet.command.args, sizeof(packet.command.args));

    buffer[sys_len-1] = calcCRC8(&buffer[2], AVDCP2_HEADER_SIZE+packet.getPayloadLen());
    serial.write(buffer, sys_len);
    Serial.println("Sent Command Packet");
}

void DUCCDriver::packHeaders(uint8_t* buffer, BasePacket& packet)
{
    buffer[0] = START_BYTE;
    buffer[1] = packet.getPayloadLen()+AVDCP2_HEADER_SIZE+DUCC_CRC_SIZE;
    buffer[2] = packet.getPacketType();;
    buffer[3] = packet.sender;
    buffer[4] = packet.receiver;
    memcpy(&buffer[5], &packet.timestamp, sizeof(uint32_t));
    memcpy(&buffer[9], &packet.sequenceId, sizeof(uint32_t));
}

void DUCCDriver::sendTelemetry(TelemetryPacketTx& packet)
{
    uint8_t sys_len = packet.getPayloadLen()+DUCC_HEADER_SIZE+AVDCP2_HEADER_SIZE+DUCC_CRC_SIZE;
    uint8_t buffer[sys_len] = {};

    packHeaders(buffer, packet);

    // Payload:
    // Flight State (1 byte)
    buffer[13] = packet.telemetry.flightState;

    // Barometric altitude (2 bytes)
    memcpy(&buffer[14], &packet.telemetry.barometricAlt, sizeof(int16_t));

    // IMU Data (12 bytes)
    memcpy(&buffer[16], &packet.telemetry.imuData, sizeof(ImuData));

    // Fins (4 bytes)
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

    // Serial.println(":::");
    // for (int i = 0; i < TELEMETRY_SIZE+2; i++) {
    //     Serial.print(buffer[i], HEX);
    //     Serial.print(" ");
    // }
    // Serial.println();

    buffer[sys_len-1] = calcCRC8(&buffer[2], AVDCP2_HEADER_SIZE+packet.getPayloadLen());
    serial.write(buffer, sys_len);
}