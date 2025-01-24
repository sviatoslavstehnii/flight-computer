#include "transceiver.h"


void FlightTransceiver::setup()
{
    lora_.setup();
}

COMMAND_ID FlightTransceiver::receiveCommand()
{
    if (!lora_.available()){
        return;
    }

    uint8_t buffer[128]{}; 
    size_t length = lora_.read(buffer, 128);

    std::optional<PacketType> packetType = lora_.parseHeader(buffer, length);

    if (!packetType.has_value()){
        Serial.println("Transceiver: packet empty");
        return;
    }

    switch(packetType.value()){
        case PACKET_RESPONSE:
        case PACKET_TELEMETRY:
            break;
        case PACKET_COMMAND:
            auto& command = lora_.receiveCommand(buffer);
            // Read command, send reply
            Response response{};
            response.command_seq_id = command.seq_id;
            lora_.sendResponse(command, response);
            Serial.printf("Received command from %d\n", command.sender);
            Serial.printf("Command ID: %d\n", command.command_id);
            break;
        default:
            break;
    }
}

void FlightTransceiver::sendTelemetry(const Telemetry& telemetry)
{
    // send telemetry
    TelemetryPacket packet{};
    packet.sender = lora_.getMyAddress();
    packet.receiver = groundAddr_;
    packet.timestamp = millis();
    packet.seq_id = seqId_++;
    packet.telemetry = telemetry;

    lora_.sendTelemetry(packet);
    Serial.println("Sent Telemetry Packet");
}

void GroundTransceiver::setup()
{
    lora_.setup();
}

void GroundTransceiver::receivePacket()
{
    if (!lora_.available()){
        return;
    }

    uint8_t buffer[128]{}; 
    size_t length = lora_.read(buffer, 128);

    std::optional<PacketType> packetType = lora_.parseHeader(buffer, length);

    if (!packetType.has_value()){
        Serial.println("Transceiver: packet empty");
        return;
    }

    switch(packetType.value()){
        case PACKET_RESPONSE:
            auto& packet = lora_.receiveResponse(buffer);
            Serial.printf("Received response from %d\n", packet.sender);

            // register response
            break;
        case PACKET_TELEMETRY:
            auto& packet = lora_.receiveTelemetry(buffer);
            Serial.printf("Received telemetry from %d\n", packet.sender);
            // read telemetry
            break;
        case PACKET_COMMAND:
            break;
        default:
            break;
    }
}
