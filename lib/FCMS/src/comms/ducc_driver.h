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

#define START_BYTE 0xAA
#define CRC_POLY 0xD5 // CRC-8 Polynomial 0xD5

class DUCCDriver {
public:
    DUCCDriver(HardwareSerial& serialPort, uint8_t myAddress) 
        : serial(serialPort), myAddr(myAddress) {}

    void setup();

    void sendCommand(CommandPacketTx& packet);
    void sendTelemetry(TelemetryPacketTx &packet);
    void sendResponse(ResponsePacketTx& packet);

    TelemetryPacketRx receiveTelemetry(uint8_t *buffer);
    ResponsePacketRx receiveResponse(uint8_t *buffer);
    CommandPacketRx receiveCommand(uint8_t *buffer);

    std::unique_ptr<BasePacketRx> parseHeader(uint8_t* header, uint8_t* data);
    void packHeaders(uint8_t *buffer, BasePacket &packet);

    uint32_t getMyAddress() const;

    bool available() const;
    std::unique_ptr<BasePacketRx> read();

    static uint8_t calcCRC8(const uint8_t* data, size_t length) {
        uint8_t crc = 0xFF;
        for (size_t i = 0; i < length; ++i) {
            crc ^= data[i];
            for (int j = 7; j >= 0; --j) {
                if (crc & 0x80) {
                    crc = (crc << 1) ^ CRC_POLY;  // If MSB is 1, shift and XOR with polynomial
                } else {
                    crc <<= 1;  // Otherwise just shift
                }
            }
        }
        return crc;
    }

private:
    HardwareSerial& serial;
    uint8_t myAddr;
    uint32_t seqID_{0};
    RingBuffer ringBuffer;
    bool packetFound = false;
    bool reset = false;
};

#endif
