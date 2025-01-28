#ifndef LORA_H
#define LORA_H

#include <queue>
#include <optional>
#include <vector>
#include <cstring>
#include "packets.h"
#include <Arduino.h>


class LORA {
public:
    LORA(uint8_t myAddress): myAddr(myAddress) {};

    void setup();

    void sendCommand(uint8_t receiverAddr, Command& command);
    void sendTelemetry(TelemetryPacket& packet);
    void sendResponse(uint8_t receiver, Response& response);

    TelemetryPacket receiveTelemetry(uint8_t *buffer);
    ResponsePacket receiveResponse(uint8_t *buffer);
    CommandPacket receiveCommand(uint8_t *buffer);

    std::optional<PacketType> parseHeader(uint8_t *buffer, size_t length);

    uint32_t getMyAddress() const;

    bool available() const;
    size_t read(uint8_t *buffer, size_t size) const;
private:
    uint8_t myAddr;
    uint32_t seqID_{0};
};

#endif // LORA_H
