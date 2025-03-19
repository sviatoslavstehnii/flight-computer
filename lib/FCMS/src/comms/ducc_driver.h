#ifndef LORA_H
#define LORA_H

#include <queue>
#include <optional>
#include <vector>
#include <cstring>
#include "packets.h"
#include <Arduino.h>
#include "ring_buffer.h"
#include <memory>
#include <SoftwareSerial.h>

#define START_BYTE 0xAA
#define CRC_POLY 0xD5 // CRC-8 Polynomial 0xD5

class DUCCDriver
{
public:
    DUCCDriver(Stream &serialPort, uint8_t myAddress)
        : serial(serialPort), myAddr(myAddress) {}

    void setup();
    void sendCommand(const CommandPacket &packet);
    void sendTelemetry(const TelemetryPacket &packet);
    void sendResponse(const ResponsePacket &packet);

    TelemetryPacket receiveTelemetry(const uint8_t *buffer);
    ResponsePacket receiveResponse(const uint8_t *buffer);
    CommandPacket receiveCommand(const uint8_t *buffer);
    bool available() const;

    std::unique_ptr<BasePacket> read();
    uint32_t getMyAddress() const;

    static uint8_t calcCRC8(const uint8_t *data, size_t length)
    {
        uint8_t crc = 0xFF;
        for (size_t i = 0; i < length; ++i)
        {
            crc ^= data[i];
            for (int j = 7; j >= 0; --j)
            {
                if (crc & 0x80)
                {
                    crc = (crc << 1) ^ CRC_POLY; // If MSB is 1, shift and XOR with polynomial
                }
                else
                {
                    crc <<= 1; // Otherwise just shift
                }
            }
        }
        return crc;
    }

private:
    Stream &serial;
    uint8_t myAddr;
    uint32_t seqID_{0};
    RingBuffer ringBuffer;
    bool packetFound = false;
    bool reset = false;

    std::unique_ptr<BasePacket> parseHeader(const uint8_t *header, const uint8_t *data);
    void packHeaders(uint8_t *buffer, const BasePacket &packet);
};

#endif
