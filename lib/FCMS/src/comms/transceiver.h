#include <iostream>
#include <iomanip>
#include <map>
#include <chrono>
#include "lora_driver.h"
#include <optional>

class BaseTransceiver {
public:
    virtual void setup() = 0;
};

class Transceiver {
public:
    Transceiver(uint8_t myAddr): 
        lora_(myAddr), _seqId(0) {};

    void setup();

    void receive();
    void retryCommands();
    void handleResponse(const ResponsePacketRx &packet);
    void sendCommand(uint8_t receiver, Command packet);
    void sendTelemetry(uint8_t receiver, Telemetry telemetry);
    void sendResponse(uint8_t receiver, Response response);

    bool hasTelemetry() { return lora_.hasTelemetry(); }
    TelemetryPacketRx popTelemetry() { return lora_.popTelemetry(); }
    bool hasCommand() { return lora_.hasCommand(); }
    CommandPacketRx popCommand() { return lora_.popCommand(); }
    bool hasResponse() { return lora_.hasResponse(); }
    ResponsePacketRx popResponse() { return lora_.popResponse(); }

private:
    LoRaDriver lora_;
    uint32_t _seqId;
    uint32_t newSeqId() { return _seqId++; }
    

    std::map<uint32_t, std::chrono::time_point<std::chrono::steady_clock>> sentCommands;
    std::map<uint32_t, CommandPacketTx> unrespondedCommands;
    std::map<uint32_t, uint32_t> commandRetries;

    static const uint32_t MAX_RETRIES = 100;
    static const uint32_t TIMEOUT_MS = 5000;
};