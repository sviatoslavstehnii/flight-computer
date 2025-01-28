#include "GSMS.h"

void GSMS::setup()
{
    Serial.begin(9600);
    Serial.println("START GS");
    transceiver.setup();
    Command command{};
    command.commandId=PING;
    transceiver.sendCommand(VEHICLE_ADDR, command);
}

void GSMS::step()
{
    delay(200);
    //transceiver.receive();
    // if (transceiver.hasTelemetry())
    // {
    //     TelemetryPacket packet = transceiver.popTelemetry();
    //     Serial.printf("Received telemetry from %d\n", packet.sender);
    //     // process telemetry
    // }
    transceiver.retryCommands(VEHICLE_ADDR);
}

STATE GSMS::getState()
{
    return STATE();
}

void GSMS::goToState(STATE state)
{
}
