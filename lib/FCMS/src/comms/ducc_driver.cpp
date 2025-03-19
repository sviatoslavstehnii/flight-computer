#include "ducc_driver.h"

void DUCCDriver::setup()
{
    Serial.print("Initializing Comms... ");
    Serial.println("Done");

    // int time_s = 10;
    // size_t start_time = millis();
    // while(millis() - start_time < time_s * 1000){
    //     if (serial.available()){
    //         char c = serial.read();
    //         Serial.print(c);
    //     }
    // }
}

uint32_t DUCCDriver::getMyAddress() const
{
    return myAddr;
}

bool DUCCDriver::available() const
{
    return serial.available();
}

std::unique_ptr<BasePacket> DUCCDriver::read()
{
    while (reset)
    {
        if (ringBuffer.isEmpty())
        {
            reset = false;
        }
        else
        {
            auto peekByte = ringBuffer.peek();
            if (peekByte.has_value() && peekByte.value() == START_BYTE)
            {
                packetFound = true;
                reset = false;
            }
            else
            {
                ringBuffer.pop();
            }
        }
    }

    while (serial.available() && !ringBuffer.isFull())
    {
        uint8_t readByte = serial.read();
#ifdef DEBUG_SERIAL
        // Serial.print("_");
        DEBUG_SERIAL.print((char)readByte);
        DEBUG_SERIAL.print(" ");
// Serial.print(readByte, HEX);
#endif

        if (!packetFound && readByte == START_BYTE)
        {
            packetFound = true;
            ringBuffer.clear();
        }

        ringBuffer.push(readByte);
    }

    if (ringBuffer.isFull() && !packetFound)
    {
        ringBuffer.clear();
    }

    if (ringBuffer.size() >= DUCC_HEADER_SIZE && packetFound)
    {
        uint8_t header[DUCC_HEADER_SIZE];
        if (!ringBuffer.copy(header, DUCC_HEADER_SIZE))
        {
            Serial.println("DUCC ERROR: This error should never happen");
            reset = true;
            ringBuffer.pop();
            packetFound = false;
            return nullptr;
        }

        uint8_t length = header[1];
#ifdef DEBUG_SERIAL
        DEBUG_SERIAL.printf("\nLen %d\n", length);
#endif
        if (length > MAX_PAYLOAD_SIZE || length < 2)
        {
            reset = true;
            ringBuffer.pop();
            packetFound = false;
            return nullptr;
        }

        if (ringBuffer.size() >= length + DUCC_HEADER_SIZE)
        {
            uint8_t packet[DUCC_HEADER_SIZE + length];
            memcpy(packet, header, DUCC_HEADER_SIZE);
            ringBuffer.erase(DUCC_HEADER_SIZE);
            if (!ringBuffer.copy(&packet[DUCC_HEADER_SIZE], length))
            {
                Serial.println("DUCC ERROR: This error should never happen");
                reset = true;
                ringBuffer.pop();
                packetFound = false;
                return nullptr;
            }

            uint8_t crcPacket = packet[length + 1];
            // Calculate CRC: start_byte + len + rssi + snr + payload + crc
            // len = 5 + payload
            // crc is calculated over rssi + snr + payload
            uint8_t calculatedCrc = calcCRC8(&packet[2], length - 1);
#ifdef DEBUG_SERIAL
            DEBUG_SERIAL.printf("\nCRC: %d/%d\n", crcPacket, calculatedCrc);
#endif
            if (calculatedCrc == crcPacket)
            {
                ringBuffer.erase(DUCC_HEADER_SIZE + length);
                return parseHeader(header, &packet[DUCC_HEADER_SIZE]);
            }
            else
            {
                reset = true;
                packetFound = false;
            }
        }
    }
    return nullptr;
}

std::unique_ptr<BasePacket> DUCCDriver::parseHeader(const uint8_t *header, const uint8_t *data)
{
    uint8_t length = header[1];
    if (length > MAX_PAYLOAD_SIZE)
        return nullptr;

    if (data[5] != getMyAddress())
    {
        Serial.println("This packet is not for me");
        return nullptr;
    }

    uint8_t packetType = data[3];
    switch (packetType)
    {
    case PACKET_TELEMETRY:
        return std::make_unique<TelemetryPacket>(receiveTelemetry(data));
    case PACKET_RESPONSE:
        return std::make_unique<ResponsePacket>(receiveResponse(data));
    case PACKET_COMMAND:
        return std::make_unique<CommandPacket>(receiveCommand(data));
    default:
        return nullptr;
    }
}

void DUCCDriver::sendResponse(const ResponsePacket &packet)
{
    Serial.println("Sending response");
    uint8_t sys_len = packet.getPayloadLen() + DUCC_HEADER_SIZE + DUCC_CRC_SIZE;
    uint8_t buffer[sys_len] = {};

    packHeaders(buffer, packet);

    // Command Payload
    memcpy(&buffer[16], &packet.response.commandSeqId, sizeof(uint32_t));
    memcpy(&buffer[20], &packet.response.data, sizeof(packet.response.data));

    buffer[sys_len - 1] = calcCRC8(&buffer[2], packet.getPayloadLen());
    serial.write(buffer, sys_len);
}

TelemetryPacket DUCCDriver::receiveTelemetry(const uint8_t *buffer)
{
    TelemetryPacket packet{};
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
    memcpy(&imuData.yaw, &buffer[17], sizeof(uint16_t));
    memcpy(&imuData.pitch, &buffer[19], sizeof(uint16_t));
    memcpy(&imuData.roll, &buffer[21], sizeof(uint16_t));
    memcpy(&imuData.accel_x, &buffer[23], sizeof(int16_t));
    memcpy(&imuData.accel_y, &buffer[25], sizeof(int16_t));
    memcpy(&imuData.accel_z, &buffer[27], sizeof(int16_t));
    memcpy(&imuData.velocity_x, &buffer[29], sizeof(int16_t));
    memcpy(&imuData.velocity_y, &buffer[31], sizeof(int16_t));
    memcpy(&imuData.velocity_z, &buffer[33], sizeof(int16_t));
    // Serial.printf("IMU Data: Yaw=%04X, Pitch=%04X, Roll=%04X, AccelX=%04X, AccelY=%04X, AccelZ=%04X, "
    //               "VelX=%04X, VelY=%04X, VelZ=%04X, PosX=%04X, PosY=%04X, PosZ=%04X\n",
    //               imuData.yaw, imuData.pitch, imuData.roll,
    //               imuData.accel_x, imuData.accel_y, imuData.accel_z,
    //               imuData.velocity_x, imuData.velocity_y, imuData.velocity_z,
    //               imuData.position_x, imuData.position_y, imuData.position_z);

    packet.telemetry.imuData = imuData;

    // Parse fins
    packet.telemetry.fin1 = buffer[35];
    packet.telemetry.fin2 = buffer[36];
    packet.telemetry.fin3 = buffer[37];
    packet.telemetry.fin4 = buffer[38];

    // Parse battery data
    uint16_t mvBat, maBat;
    memcpy(&mvBat, &buffer[39], sizeof(uint16_t));
    memcpy(&maBat, &buffer[41], sizeof(uint16_t));
    packet.telemetry.mVBat = mvBat;
    packet.telemetry.mABat = maBat;

    uint8_t pyroFlags = buffer[47];
    uint8_t pyroFlags_ad = buffer[48];
    // Parse pyro flags
    int j = 0;
    packet.telemetry.flags.pyro1_armed = (pyroFlags >> j++) & 0x01;
    packet.telemetry.flags.pyro1_cont = (pyroFlags >> j++) & 0x01;
    packet.telemetry.flags.pyro1_fire = (pyroFlags >> j++) & 0x01;
    packet.telemetry.flags.pyro2_armed = (pyroFlags >> j++) & 0x01;
    packet.telemetry.flags.pyro2_cont = (pyroFlags >> j++) & 0x01;
    packet.telemetry.flags.pyro2_fire = (pyroFlags >> j++) & 0x01;
    packet.telemetry.flags.pyro3_armed = (pyroFlags >> j++) & 0x01;
    packet.telemetry.flags.pyro3_cont = (pyroFlags >> j++) & 0x01;
    j = 0;
    packet.telemetry.flags.pyro3_fire = (pyroFlags_ad >> j++) & 0x01;
    packet.telemetry.flags.liftoff = (pyroFlags_ad >> j++) & 0x01;
    packet.telemetry.flags.apogee = (pyroFlags_ad >> j++) & 0x01;
    packet.telemetry.flags.landed = (pyroFlags_ad >> j++) & 0x01;
    packet.telemetry.flags.parachute_fired = (pyroFlags_ad >> j++) & 0x01;
    packet.telemetry.flags.logged_to_sd = (pyroFlags_ad >> j++) & 0x01;

    // Parse temperature and data
    // packet.telemetry.temp = buffer[55];
    // packet.telemetry.mejPercent = buffer[56];
    // packet.telemetry.cjPercent = buffer[57];
    // packet.telemetry.datajournalPercent = buffer[58];
    // // Serial.printf("Temp=%02X, MEL=%02X, CL=%02X, Datalog=%02X\n", temp, melData, clData, datalogData);

    // Parse apogee
    memcpy(&packet.telemetry.apogee, &buffer[49], sizeof(int16_t));

    // Parse GPS data
    memcpy(&packet.telemetry.gpsLat, &buffer[51], sizeof(int32_t));
    memcpy(&packet.telemetry.gpsLon, &buffer[55], sizeof(int32_t));

    // // Parse detections
    // uint8_t detections[3];
    // memcpy(detections, &buffer[69], sizeof(detections));
    // Serial.printf("Detections: [%02X, %02X, %02X]\n", detections[0], detections[1], detections[2]);

    Serial.println("Telemetry data parsed successfully.");
    return packet;
}

ResponsePacket DUCCDriver::receiveResponse(const uint8_t *buffer)
{
    ResponsePacket packet{};
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

CommandPacket DUCCDriver::receiveCommand(const uint8_t *buffer)
{
    CommandPacket packet{};
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
    packet.command.commandId = static_cast<COMMAND_ID>(buffer[14]);
    memcpy(&packet.command.args, &buffer[15], sizeof(packet.command.args));
    return packet;
}

void DUCCDriver::sendCommand(const CommandPacket &packet)
{
    uint8_t sys_len = packet.getPayloadLen() + DUCC_HEADER_SIZE + DUCC_CRC_SIZE;
    uint8_t buffer[sys_len] = {};

    packHeaders(buffer, packet);

    buffer[16] = packet.command.commandId;
    memcpy(&buffer[17], &packet.command.args, sizeof(packet.command.args));

    buffer[sys_len - 1] = calcCRC8(&buffer[2], packet.getPayloadLen());
    serial.write(buffer, sys_len);

    Serial.println("Sent Command Packet");
    Serial.print("SENT:");
    for (int i = 0; i < sys_len; i++)
    {
        Serial.print(buffer[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}

void DUCCDriver::packHeaders(uint8_t *buffer, const BasePacket &packet)
{
    buffer[0] = START_BYTE;
    buffer[1] = packet.getPayloadLen() + DUCC_CRC_SIZE;
    memcpy(&buffer[2], &packet.rssi, sizeof(int16_t));
    buffer[4] = packet.snr;
    buffer[5] = packet.getPacketType();
    buffer[6] = packet.sender;
    buffer[7] = packet.receiver;
    memcpy(&buffer[8], &packet.timestamp, sizeof(uint32_t));
    memcpy(&buffer[12], &packet.sequenceId, sizeof(uint32_t));
}

void DUCCDriver::sendTelemetry(const TelemetryPacket &packet)
{
    uint8_t sys_len = packet.getPayloadLen() + DUCC_HEADER_SIZE + DUCC_CRC_SIZE;
    uint8_t buffer[sys_len] = {};

    packHeaders(buffer, packet);

    // Payload:
    // Flight State (1 byte)
    buffer[16] = packet.telemetry.flightState;

    // Barometric altitude (2 bytes)
    memcpy(&buffer[17], &packet.telemetry.barometricAlt, sizeof(int16_t));

    // IMU Data (12 bytes)
    memcpy(&buffer[19], &packet.telemetry.imuData.yaw, sizeof(int16_t));
    memcpy(&buffer[21], &packet.telemetry.imuData.pitch, sizeof(int16_t));
    memcpy(&buffer[23], &packet.telemetry.imuData.roll, sizeof(int16_t));

    memcpy(&buffer[25], &packet.telemetry.imuData.accel_x, sizeof(int16_t));
    memcpy(&buffer[27], &packet.telemetry.imuData.accel_y, sizeof(int16_t));
    memcpy(&buffer[29], &packet.telemetry.imuData.accel_z, sizeof(int16_t));

    memcpy(&buffer[31], &packet.telemetry.imuData.velocity_x, sizeof(int16_t));
    memcpy(&buffer[33], &packet.telemetry.imuData.velocity_y, sizeof(int16_t));
    memcpy(&buffer[35], &packet.telemetry.imuData.velocity_z, sizeof(int16_t));

    // Fins (4 bytes)
    buffer[37] = packet.telemetry.fin1;
    buffer[38] = packet.telemetry.fin2;
    buffer[39] = packet.telemetry.fin3;
    buffer[40] = packet.telemetry.fin4;

    // // Battery data (4 bytes)
    memcpy(&buffer[41], &packet.telemetry.mVBat, sizeof(uint16_t)); // mVBat
    memcpy(&buffer[43], &packet.telemetry.mABat, sizeof(uint16_t)); // mABat

    memcpy(&buffer[47], &packet.telemetry.takeoffDetectedTime, sizeof(uint16_t));

    // // Temperature and additional data (4 bytes)
    // buffer[54] = packet.telemetry.temp;
    // buffer[55] = packet.telemetry.mejPercent;
    // buffer[56] = packet.telemetry.cjPercent;
    // buffer[57] = packet.telemetry.datajournalPercent;

    // Pyro flags (2 bytes)
    uint8_t pyroFlags = 0;
    pyroFlags |= (packet.telemetry.flags.pyro1_armed << 0);
    pyroFlags |= (packet.telemetry.flags.pyro1_cont << 1);
    pyroFlags |= (packet.telemetry.flags.pyro1_fire << 2);
    pyroFlags |= (packet.telemetry.flags.pyro2_armed << 3);
    pyroFlags |= (packet.telemetry.flags.pyro2_cont << 4);
    pyroFlags |= (packet.telemetry.flags.pyro2_fire << 5);
    pyroFlags |= (packet.telemetry.flags.pyro3_armed << 6);
    pyroFlags |= (packet.telemetry.flags.pyro3_cont << 7);
    buffer[49] = pyroFlags;

    uint8_t pyroFlagsAd = 0;
    pyroFlagsAd |= (packet.telemetry.flags.pyro3_fire << 0);
    pyroFlagsAd |= (packet.telemetry.flags.liftoff << 1);
    pyroFlagsAd |= (packet.telemetry.flags.apogee << 2);
    pyroFlagsAd |= (packet.telemetry.flags.landed << 3);
    pyroFlagsAd |= (packet.telemetry.flags.parachute_fired << 4);
    pyroFlagsAd |= (packet.telemetry.flags.logged_to_sd << 5);
    buffer[50] = pyroFlagsAd;

    // Apogee (2 bytes)
    memcpy(&buffer[51], &packet.telemetry.apogee, sizeof(int16_t));

    // GPS data (12 bytes)
    memcpy(&buffer[53], &packet.telemetry.gpsLat, sizeof(int32_t));
    memcpy(&buffer[57], &packet.telemetry.gpsLon, sizeof(int32_t));

    memcpy(&buffer[61], &packet.telemetry.yaw_setp, sizeof(int16_t));
    memcpy(&buffer[65], &packet.telemetry.pitch_setp, sizeof(int16_t));
    memcpy(&buffer[69], &packet.telemetry.roll_setp, sizeof(int16_t));
    memcpy(&buffer[71], &packet.telemetry.parachuteDeploymentTime, sizeof(uint16_t));
    memcpy(&buffer[73], &packet.telemetry.event2_time, sizeof(uint16_t));
    // memcpy(&buffer[66], &packet.telemetry.gpsAlt, sizeof(int16_t));

    // // Detections (3 bytes)
    // buffer[68] = packet.telemetry.takeOffDetection;
    // buffer[69] = packet.telemetry.apogeeDetection;
    // buffer[70] = packet.telemetry.landingDetection;

    buffer[sys_len - 1] = calcCRC8(&buffer[2], packet.getPayloadLen());

    // Serial.print("TEENSY-SENDING:::");
    // for (int i = 0; i < sys_len; i++)
    // {
    //     Serial.print(buffer[i], HEX);
    //     Serial.print(" ");
    // }
    // Serial.println();

    serial.write(buffer, sys_len);
}