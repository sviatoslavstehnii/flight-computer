#ifndef LORA_H
#define LORA_H

#include <queue>
#include <optional>
#include <vector>
#include <cstring>
#include "packets.h"
#include <Arduino.h>
#include "ring_buffer.h"
#include "my_queue_t.h"

#define START_BYTE 0xAA
#define CRC_POLY 0xD5 // CRC-8 Polynomial 0xD5

class LoRaDriver {
public:
    LoRaDriver(uint8_t myAddress): myAddr(myAddress) {};

    using ResponseCallback = std::function<void(const ResponsePacketRx&)>;
    void setResponseCallback(ResponseCallback cb) {
        responseCallback_ = std::move(cb);
    }

    void setup();

    void sendCommand(CommandPacketTx& packet);
    void sendTelemetry(TelemetryPacketTx &packet);
    void sendResponse(ResponsePacketTx& packet);

    TelemetryPacketRx receiveTelemetry(uint8_t *buffer);
    ResponsePacketRx receiveResponse(uint8_t *buffer);
    CommandPacketRx receiveCommand(uint8_t *buffer);

    bool parseHeader(uint8_t* header, uint8_t* data);
    void packHeaders(uint8_t *buffer, BasePacket &packet);

    uint32_t getMyAddress() const;

    bool available() const;
    bool update();

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

    bool hasTelemetry() { return !receivedTelemetry.empty(); }
    TelemetryPacketRx popTelemetry() { return receivedTelemetry.pop(); }
    bool hasCommand() { return !receivedCommands.empty(); }
    CommandPacketRx popCommand() { return receivedCommands.pop(); }
    bool hasResponse() { return !receivedResponses.empty(); }
    ResponsePacketRx popResponse() { return receivedResponses.pop(); }

private:
    uint8_t myAddr;
    uint32_t seqID_{0};
    RingBuffer ringBuffer;
    bool packetFound = false;
    bool reset = false;
    my_queue_t<TelemetryPacketRx> receivedTelemetry{10};
    my_queue_t<CommandPacketRx> receivedCommands {10};
    my_queue_t<ResponsePacketRx> receivedResponses {10};
    ResponseCallback responseCallback_;
};

#endif // LORA_H
