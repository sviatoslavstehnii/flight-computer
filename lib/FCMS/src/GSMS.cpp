#include "GSMS.h"

void GSMS::setup()
{
    Serial.begin(9600);
    Serial.println("START GS");
    Serial3.begin(9600);
    // GroundControl.begin(115200);
    lora.setup();
    server.setup();
    Command command{};
    command.commandId = PING;
    lora.sendCommand(VEHICLE_ADDR, command);
}

void GSMS::step()
{
    time = std::chrono::steady_clock::now();
    lora.receive();
    server.receive();
    if (lora.hasTelemetry())
    {
        auto packet = lora.popTelemetry();
        Serial.printf("ACCEL: %d,%d,%d\n ", packet.telemetry.imuData.accel_x, packet.telemetry.imuData.accel_y, packet.telemetry.imuData.accel_z);
        server.sendTelemetry(packet.receiver, packet.telemetry);
    }
    if (lora.hasResponse())
    {
        auto packet = lora.popResponse();
        Serial.printf("Relayed response %X->%X\n ", packet.sender, packet.receiver);
        server.sendResponse(packet.receiver, packet.response);
    }
    if (server.hasCommand())
    {
        auto packet = server.popCommand();
        Serial.printf("Relayed command %X->%X\n ", packet.sender, VEHICLE_ADDR);
        lora.sendCommand(VEHICLE_ADDR, packet.command);
    }
    const int retry_period = 2000;
    if (std::chrono::duration_cast<std::chrono::milliseconds>(time - prevTime).count() > retry_period)
    {
        prevTime = time;
        lora.retryCommands();
    }
}

STATE GSMS::getState()
{
    return STATE();
}

void GSMS::goToState(STATE state)
{
}
