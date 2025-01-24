#include <iostream>
#include <iomanip>
#include <queue>
#include "lora.h"
#include <optional>

class BaseTransceiver {
public:
    virtual void setup() = 0;
    virtual void receivePackets() = 0;
    virtual void sendPackets() = 0;
};

class FlightTransceiver : public BaseTransceiver{
public:
    FlightTransceiver(uint8_t myAddr, uint8_t groundAddr): 
        lora_(myAddr), groundAddr_(groundAddr), seqId_(0) {};
    
    void setup() override;
    void receivePackets() override;
    void sendPackets() override;

private:
    std::queue<CommandPacket> unrespondedCommands {};
    LORA lora_;
    uint8_t groundAddr_;
    uint32_t seqId_;

};