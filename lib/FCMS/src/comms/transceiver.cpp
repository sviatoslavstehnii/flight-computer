#include "transceiver.h"

void Transceiver::setup(HardwareSerial *relaySerial)
{
    lora_.setup();
    lora_.setup_relay(relaySerial);
}

void Transceiver::sendTelemetry(uint8_t receiver, Telemetry telemetry)
{
    // send telemetry
    TelemetryPacketTx packet;
    packet.sender = lora_.getMyAddress();
    packet.receiver = receiver;
    packet.timestamp = millis();
    packet.sequenceId = newSeqId();
    packet.telemetry = telemetry;
    lora_.sendTelemetry(packet);
}

void Transceiver::sendResponse(uint8_t receiver, Response response)
{
    ResponsePacketTx packet;
    packet.sender = lora_.getMyAddress();
    packet.receiver = receiver;
    packet.timestamp = millis();
    packet.sequenceId = newSeqId();
    packet.response = response;

    lora_.sendResponse(packet);
}

void Transceiver::retryCommands(){

    for (auto& [seqId, packet] : unrespondedCommands){
        if (commandRetries[seqId] >= MAX_RETRIES){
            Serial.printf("Command %d retries exceeded\n", seqId);
            unrespondedCommands.erase(seqId);
            commandRetries.erase(seqId);
            continue;
        }

        auto time = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(time - sentCommands[seqId]).count() > TIMEOUT_MS){
            packet.timestamp = millis();
            lora_.sendCommand(packet);
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

    auto packet_ptr = lora_.read();
    if (!packet_ptr) return;

    auto type = packet_ptr->getPacketType();
    switch (type) {
        case PACKET_TELEMETRY:
            receivedTelemetry.push(static_cast<TelemetryPacketRx&>(*packet_ptr));
            break;
        case PACKET_COMMAND:
            receivedCommands.push(static_cast<CommandPacketRx&>(*packet_ptr));
            break;
        case PACKET_RESPONSE:{
            auto packet = static_cast<ResponsePacketRx&>(*packet_ptr);
            receivedResponses.push(packet);

            if (unrespondedCommands.find(packet.sequenceId) != unrespondedCommands.end()) {
                unrespondedCommands.erase(packet.sequenceId);
                commandRetries.erase(packet.sequenceId);
                sentCommands.erase(packet.sequenceId);
            } else {
                Serial.printf("Unexpected response for command %d\n", packet.sequenceId);
            }
            break;
        }
        default:
            Serial.println("Unknown packet type received.");
            break;
    }
}

void Transceiver::sendCommand(uint8_t receiver, Command command)
{
    CommandPacketTx packet;
    packet.sender = lora_.getMyAddress();
    packet.receiver = receiver;
    packet.timestamp = millis();
    packet.sequenceId = newSeqId();
    packet.command = command;

    uint32_t seqId = packet.sequenceId;

    lora_.sendCommand(packet);
    sentCommands[seqId] = std::chrono::steady_clock::now();
    unrespondedCommands[seqId] = packet;
    commandRetries[seqId] = 0;

    Serial.printf("Command %d sent to %d\n", command.commandId, receiver);
}