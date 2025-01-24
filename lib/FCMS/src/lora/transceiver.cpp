#include "transceiver.h"


void FlightTransceiver::setup()
{
    lora_.setup();
}

void FlightTransceiver::receivePackets()
{
    std::optional<PacketType> packet = lora_.parseTraffic();

    if (!packet.has_value()){
        Serial.println("Transceiver: packet empty");
        return;
    }

    switch(packet.value()){
        case PACKET_TELEMETRY:

            break;
        case PACKET_COMMAND:

            break;
        case PACKET_RESPONSE:

            break;
        default:
            break;
    }
}

void FlightTransceiver::sendPackets()
{

}
