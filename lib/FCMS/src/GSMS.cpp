#include "GSMS.h"

void GSMS::setup()
{
    Serial.begin(9600);
    Serial.println("START GS");
    // GroundControl.begin(115200);
    transceiver.setup();
    Command command{};
    command.commandId=PING;
    transceiver.sendCommand(VEHICLE_ADDR, command);
}

void GSMS::step()
{
    time = std::chrono::steady_clock::now();
    transceiver.receive();
    if (transceiver.hasTelemetry())
    {
        TelemetryPacketRx packet = transceiver.popTelemetry();
        Serial.printf("Received telemetry from %d\n", packet.sender);
        // process telemetry
    }
    // transceiver.retryCommands(VEHICLE_ADDR);
    if (std::chrono::duration_cast<std::chrono::milliseconds>(time - prevTime).count() > 5000){
        prevTime = time;
        transceiver.retryCommands();
    }
}

STATE GSMS::getState()
{
    return STATE();
}

void GSMS::goToState(STATE state)
{
}
