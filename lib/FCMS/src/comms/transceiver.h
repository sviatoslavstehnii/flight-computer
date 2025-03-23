#ifndef TRANSCEIVER_H
#define TRANSCEIVER_H

#include <iostream>
#include <iomanip>
#include <map>
#include <chrono>
#include <optional>
#include "ducc_driver.h"
#include "my_queue_t.h"

class Transceiver
{
public:
    Transceiver(Stream &serial, uint8_t myAddr, size_t tel_queue_size = 10, size_t com_queue_size = 10, size_t resp_queue_size = 10)
        : ducc_(serial, myAddr), receivedTelemetry(tel_queue_size), receivedCommands(com_queue_size), receivedResponses(resp_queue_size) {};

    void setup();

    void receive();
    void retryCommands();
    void sendCommand(uint8_t receiver, const Command &packet);
    void sendTelemetry(uint8_t receiver, const Telemetry &telemetry);
    void sendResponse(uint8_t receiver, const Response &response);

    uint32_t getMyAddress() const { return ducc_.getMyAddress(); }

    bool hasTelemetry() { return !receivedTelemetry.empty(); }
    TelemetryPacket popTelemetry() { return receivedTelemetry.pop(); }
    bool hasCommand() { return !receivedCommands.empty(); }
    CommandPacket popCommand() { return receivedCommands.pop(); }
    bool hasResponse() { return !receivedResponses.empty(); }
    ResponsePacket popResponse() { return receivedResponses.pop(); }

private:
    DUCCDriver ducc_;
    uint32_t _seqId{0};
    uint32_t newSeqId() { return _seqId++; }

    my_queue_t<TelemetryPacket> receivedTelemetry;
    my_queue_t<CommandPacket> receivedCommands;
    my_queue_t<ResponsePacket> receivedResponses;

    std::map<uint32_t, std::chrono::time_point<std::chrono::steady_clock>> sentCommands;
    std::map<uint32_t, CommandPacket> unrespondedCommands;
    std::map<uint32_t, uint32_t> commandRetries;

    static const uint32_t MAX_RETRIES = 100;
    static const uint32_t TIMEOUT_MS = 5000;
};

#endif