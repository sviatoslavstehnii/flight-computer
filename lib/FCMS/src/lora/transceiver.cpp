#include "transceiver.h"


void Transceiver::setup()
{
    lora_.setup();
}

void Transceiver::sendTelemetry(uint8_t receiver, Telemetry telemetry)
{
    // send telemetry
    TelemetryPacket packet;
    packet.sender = lora_.getMyAddress();
    packet.receiver = receiver;
    packet.timestamp = millis();
    packet.sequenceId = seqId_++;
    packet.telemetry = telemetry;
    lora_.sendTelemetry(packet);
}

void Transceiver::sendResponse(uint8_t receiver, Response response)
{
    lora_.sendResponse(receiver, response);
}

TelemetryPacket Transceiver::popTelemetry()
{
    return receivedTelemetry.pop();
}

CommandPacket Transceiver::popCommand()
{
    return receivedCommands.pop();
}

ResponsePacket Transceiver::popResponse()
{
    return receivedResponses.pop();
}

void Transceiver::retryCommands(uint8_t receiver){
    for (auto& [seqId, commandId] : unrespondedCommands){
        if (commandRetries[seqId] >= MAX_RETRIES){
            Serial.printf("Command %d retries exceeded\n", seqId);
            unrespondedCommands.erase(seqId);
            commandRetries.erase(seqId);
            continue;
        }

        if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - sentCommands[seqId]).count() > TIMEOUT_MS){
            Command command{};
            command.commandId = commandId;
            sendCommand(receiver, command);
            commandRetries[seqId]++;
            Serial.printf("Command %d retry %d\n", seqId, commandRetries[seqId]);
        }
    }
}

void Transceiver::receive()
{
    if (!lora_.available()){
        return;
    }

    uint8_t buffer[128]{}; 
    size_t length = lora_.read(buffer, 128);

    std::optional<PacketType> packetType = lora_.parseHeader(buffer, length);

    if (!packetType.has_value()){
        return;
    }

    switch (packetType.value()) {
        case PACKET_RESPONSE: {
            auto packet = lora_.receiveResponse(buffer);
            Serial.printf("Received response from %d\n", packet.sender);
            
            // Find command in unresponded
            if (unrespondedCommands.find(packet.sequenceId) != unrespondedCommands.end()) {
                unrespondedCommands.erase(packet.sequenceId);
                commandRetries.erase(packet.sequenceId);
                Serial.printf("Response to command %d received.\n", packet.sequenceId);
            } else {
                Serial.printf("Unexpected response for command %d\n", packet.sequenceId);
            }

        } break;
        case PACKET_TELEMETRY: {
            TelemetryPacket packet = lora_.receiveTelemetry(buffer);
            Serial.printf("Received telemetry from %d\n", packet.sender);
            // Add telemetry to queue
            receivedTelemetry.push(packet);
        } break;

        case PACKET_COMMAND: {
            auto packet = lora_.receiveCommand(buffer);
            Serial.printf("Received command from %d: %d\n", packet.sender, packet.command.commandId);
            // Process the command
            Response response{};
            response.commandSeqId = packet.sequenceId;
            sendResponse(packet.sender, response);
        } break;
        default:
            break;
    }
}

void Transceiver::sendCommand(uint8_t receiver, Command packet)
{
    seqId_++;
    uint32_t commandId = packet.commandId;

    lora_.sendCommand(receiver, packet);
    sentCommands[commandId] = std::chrono::steady_clock::now();
    unrespondedCommands[commandId] = packet.commandId;
    commandRetries[commandId] = 0;

    Serial.printf("Command %d sent to %d\n", commandId, receiver);
}