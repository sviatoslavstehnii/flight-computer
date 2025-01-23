#ifndef LORA_H
#define LORA_H

#include <queue.h>


class BasePacket{
public:
    BasePacket(): sender(), receiver(), timestamp(), seq_id(){};

    uint8_t sender;           // Sender
    uint8_t receiver;         // Receiver
    uint32_t timestamp;       // Timestamp (milliseconds)
    uint32_t seq_id;          // Sequence Id
};

struct ImuData {
  uint16_t yaw;         // Курс
  uint16_t pitch;       // Тангаж
  uint16_t roll;        // Крен
  int16_t accel_x;      // Прискорення по X
  int16_t accel_y;      // Прискорення по Y
  int16_t accel_z;      // Прискорення по Z
  int16_t velocity_x;   // Швидкість по X
  int16_t velocity_y;   // Швидкість по Y
  int16_t velocity_z;   // Швидкість по Z
  int16_t position_x;   // Позиція по X
  int16_t position_y;   // Позиція по Y
  int16_t position_z;   // Позиція по Z
};

enum COMMAND_ID{

};

class CommandPacket : public BasePacket{
public:
    COMMAND_ID command_id;
};

struct PyroFlags{
    bool pyro1_safe=0;
    bool pyro1_cont=0;
    bool pyro1_fire=0;
    bool pyro2_safe=0;
    bool pyro2_cont=0;
    bool pyro2_fire=0;
    bool pyro3_safe=0;
    bool pyro3_cont=0;
    bool pyro3_fire=0;
};

struct Telemetry {
    uint8_t flightState;      // Flight State + Additional State (binary)
    int16_t barometricAlt;    // Barometric Altitude (meters)
    ImuData imuData; 
    uint8_t fin1;           // Fin deflection (0-180°)
    uint8_t fin2;           // Fin deflection (-90° - 90°)
    uint8_t fin3;           // Fin deflection (-90° - 90°)
    uint8_t fin4;           // Fin deflection (-90° - 90°)
    uint16_t mVBat;            // Battery Voltage (mV)
    uint16_t mABat;         // Battery Current (mA)
    uint16_t loadCell;        // Load Cell (grams)
    int8_t temp;              // Temperature (°C)
    uint8_t mejPercent;   // MEJ Data (%)
    uint8_t cjPercent;    // CJ Data (%)
    uint8_t datajournalPercent; // Datalog Data (%)
    int32_t gpsLat;           // GPS Latitude (multiplied by 1,000,000 for 6-digit precision)
    int32_t gpsLon;           // GPS Longitude (multiplied by 1,000,000 for 6-digit precision)
    int16_t gpsAlt;           // GPS Altitude (meters)
    uint8_t takeOffDetection; // Take-off detection (binary)
    uint8_t apogeeDetection;  // Apogee detection (binary)
    uint8_t landingDetection; // Landing detection (binary)
    uint16_t apogee;          // Apogee (meters)

    // Event data (up to 16 events, 4 groups of 4 binary events)
    //uint8_t events[16];     // Events 0-15 (binary)
    //uint16_t eventTime[2];    // Event Time for event 0 and event 1 (ms)
    
    // Pyro Flags (12 binary flags)
    PyroFlags pyroFlags;
    
    // Additional Flags (4 binary flags)
    //uint8_t additionalFlags[4]; // Additional Flag 0-3 (binary)
};

class TelemetryPacket : public BasePacket {
public:
    Telemetry telemetry;
};

class Response{

};

class ResponsePacket{
public:
    Response response;
};

class LORA {
public:
    LORA(uint8_t myAddress): myAddr(myAddress), seq_id(0) {};

    void setup();

    void sendCommand(uint8_t receiverAddr, COMMAND_ID command_id);
    void sendTelemetry(uint8_t receiverAddr, Telemetry telemetry);
    void sendResponse(CommandPacket& command, Response response);

    TelemetryPacket receiveTelemetry();
    ResponsePacket receiveResponse();
    CommandPacket receiveCommand();

    void parseTraffic();

private:
    uint8_t myAddr;
    uint32_t seq_id;
    std::queue<CommandPacket> unrespondedCommands {};
};

#endif // LORA_H
