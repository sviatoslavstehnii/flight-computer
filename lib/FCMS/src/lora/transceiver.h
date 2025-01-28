#include <iostream>
#include <iomanip>
#include <map>
#include <chrono>
#include "lora.h"
#include <optional>
#include "my_queue_t.h"

class BaseTransceiver {
public:
    virtual void setup() = 0;
};

class Transceiver {
public:
    Transceiver(uint8_t myAddr): 
        lora_(myAddr), seqId_(0) {};
    
    void setup();


    void receive();
    void retryCommands(uint8_t receiver);
    void sendCommand(uint8_t receiver, Command packet);
    void sendTelemetry(uint8_t receiver, Telemetry telemetry);
    void sendResponse(uint8_t receiver, Response response);

    bool hasTelemetry() { return !receivedTelemetry.empty(); }
    TelemetryPacket popTelemetry();
    bool hasCommand() { return !receivedCommands.empty(); }
    CommandPacket popCommand();
    bool hasResponse() { return !receivedResponses.empty(); }
    ResponsePacket popResponse();

private:
    LORA lora_;
    uint32_t seqId_;
    my_queue_t<TelemetryPacket> receivedTelemetry{10};
    my_queue_t<CommandPacket> receivedCommands {10};
    my_queue_t<ResponsePacket> receivedResponses {10};

    std::map<uint32_t, std::chrono::time_point<std::chrono::steady_clock>> sentCommands;
    std::map<uint32_t, COMMAND_ID> unrespondedCommands;
    std::map<uint32_t, uint8_t> commandRetries;

    static const uint32_t MAX_RETRIES = 300000;
    static const uint32_t TIMEOUT_MS = 5000;
};