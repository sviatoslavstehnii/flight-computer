#include <iostream>
#include <iomanip>
#include <queue>
#include "lora.h"
#include <optional>

class BaseTransceiver {
public:
    virtual void setup() = 0;
};

class FlightTransceiver : public BaseTransceiver{
public:
    FlightTransceiver(uint8_t myAddr, uint8_t groundAddr): 
        lora_(myAddr), groundAddr_(groundAddr), seqId_(0) {};
    
    void setup() override;
    COMMAND_ID receiveCommand();
    void sendTelemetry(const Telemetry& telemetry);

private:
    LORA lora_;
    uint8_t groundAddr_;
    uint32_t seqId_;

};

class GroundTransceiver : public BaseTransceiver{
public:
    GroundTransceiver(uint8_t myAddr, uint8_t vehicleAddr):
        lora_(myAddr), vehicleAddr_(vehicleAddr), seqId_(0){};

    void setup() override;
    void receivePacket();

private:
    LORA lora_;
    uint8_t vehicleAddr_;
    uint32_t seqId_;
    std::queue<CommandPacket> unrespondedCommands {};
};
