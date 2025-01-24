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

    void sendCommand(uint8_t receiverAddr, COMMAND_ID command_id);
    void sendTelemetry(TelemetryPacket &packet);
    void sendResponse(CommandPacket& command, Response response);

    TelemetryPacket& receiveTelemetry(uint8_t *buffer, size_t length);
    ResponsePacket& receiveResponse(uint8_t *buffer, size_t length);
    CommandPacket& receiveCommand(uint8_t *buffer, size_t length);

    std::optional<PacketType> parseTraffic();

    uint32_t getMyAddress() const;


private:
    uint8_t myAddr;
    uint8_t packet_[128]{};
};

#endif // LORA_H
