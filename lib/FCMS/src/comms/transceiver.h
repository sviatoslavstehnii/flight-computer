#include <iostream>
#include <iomanip>
#include <map>
#include <chrono>
#include <optional>
#include "ducc_driver.h"
#include "my_queue_t.h"


class Transceiver {
public:
    Transceiver(HardwareSerial &serial, uint8_t myAddr):
        ducc_(serial, myAddr) {};

    void setup();

    void receive();
    void retryCommands();
    void sendCommand(uint8_t receiver, Command packet);
    void sendTelemetry(uint8_t receiver, Telemetry telemetry);
    void sendResponse(uint8_t receiver, Response response);

    bool hasTelemetry() { return !receivedTelemetry.empty(); }
    TelemetryPacketRx popTelemetry() { return receivedTelemetry.pop(); }
    bool hasCommand() { return !receivedCommands.empty(); }
    CommandPacketRx popCommand() { return receivedCommands.pop(); }
    bool hasResponse() { return !receivedResponses.empty(); }
    ResponsePacketRx popResponse() { return receivedResponses.pop(); }

private:
    DUCCDriver ducc_;
    uint32_t _seqId{0};
    uint32_t newSeqId() { return _seqId++; }

    my_queue_t<TelemetryPacketRx> receivedTelemetry{10};
    my_queue_t<CommandPacketRx> receivedCommands {10};
    my_queue_t<ResponsePacketRx> receivedResponses {10};

    std::map<uint32_t, std::chrono::time_point<std::chrono::steady_clock>> sentCommands;
    std::map<uint32_t, CommandPacketTx> unrespondedCommands;
    std::map<uint32_t, uint32_t> commandRetries;

    static const uint32_t MAX_RETRIES = 100;
    static const uint32_t TIMEOUT_MS = 5000;
};