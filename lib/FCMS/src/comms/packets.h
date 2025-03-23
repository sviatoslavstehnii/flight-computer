#include <sstream>

#define TELEMETRY_SIZE 67
#define COMMAND_SIZE 9
#define RESPONSE_SIZE 12
#define DUCC_HEADER_SIZE 2
#define DUCC_PREFIX_SIZE 3
#define AVDCP2_HEADER_SIZE 11
#define DUCC_CRC_SIZE 1
#define MAX_PAYLOAD_SIZE 120

enum PacketType
{
    PACKET_TELEMETRY = 0x01,
    PACKET_COMMAND = 0x02,
    PACKET_RESPONSE = 0x05
};

class BasePacket
{
public:
    BasePacket(uint8_t sender_, uint8_t receiver_, uint32_t timestamp_, uint32_t seq_id_)
        : sender(sender_), receiver(receiver_), timestamp(timestamp_), sequenceId(seq_id_), rssi(0), snr(0) {}

    BasePacket() : sender(0), receiver(0), timestamp(0), sequenceId(0), rssi(0), snr(0) {}

    virtual uint8_t getPacketType() const = 0;
    uint8_t sender;
    uint8_t receiver;
    uint32_t timestamp;
    uint32_t sequenceId;
    virtual uint8_t getPayloadLen() const = 0;

    int16_t rssi;
    int8_t snr;
};

struct ImuData
{
    uint16_t yaw;
    uint16_t pitch;
    uint16_t roll;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t velocity_x;
    int16_t velocity_y;
    int16_t velocity_z;
    int16_t position_x;
    int16_t position_y;
    int16_t position_z;
};

enum COMMAND_ID
{
    PING = 1,
    MASTER_ABORT = 6,
    VEHICLE_SAFE = 7,
    FIN_ALIGNMENT = 11,
    CLEAR_EVENTS = 12,
    SET_HOME = 13,
    TEST_FINS = 14,
    CALIBRATE_IMU = 15,
    CALIBRATE_BARO = 16,
    HITL = 18,
    VEHICLE_ARM = 20,
    ENABLE_TAKEOFF_DETECTION = 21,
    MANUAL_DEPLOY_PARACHUTE = 33,
    ENABLE_RECOVERY_BUZZING = 37,
    ENABLE_CAMERAS = 40,
    DUMP_MEMORY_TO_FLASH = 42,
    DUMP_FLASH_TO_SD_CARD = 43,
    DISABLE_CONTROL = 50,
    ABORT_FLIGHT_PROGRAM = 51,
    SEND_FLIGHT_PROGRAM = 52
};

struct Command
{
    COMMAND_ID commandId;
    uint8_t args[8];
};

class CommandPacket : public BasePacket
{
public:
    Command command;
    uint8_t getPacketType() const override
    {
        return PACKET_COMMAND;
    }
    uint8_t getPayloadLen() const override
    {
        return COMMAND_SIZE + DUCC_PREFIX_SIZE + AVDCP2_HEADER_SIZE;
    }
};

struct FlagDefs
{
    bool pyro1_armed = false;
    bool pyro1_cont = false;
    bool pyro1_fire = false;
    bool pyro2_armed = false;
    bool pyro2_cont = false;
    bool pyro2_fire = false;
    bool pyro3_armed = false;
    bool pyro3_cont = false;
    bool pyro3_fire = false;
    bool liftoff = false;
    bool apogee = false;
    bool landed = false;
    bool parachute_fired = false;
    bool logged_to_sd = false;
    bool critical = false;
};

struct Telemetry
{
    uint8_t flightState;   // Flight State + Additional State (binary)
    int16_t barometricAlt; // Barometric Altitude (meters)
    ImuData imuData;
    uint8_t fin1;               // Fin deflection (0-180°)
    uint8_t fin2;               // Fin deflection (-90° - 90°)
    uint8_t fin3;               // Fin deflection (-90° - 90°)
    uint8_t fin4;               // Fin deflection (-90° - 90°)
    uint16_t mVBat;             // Battery Voltage (mV)
    uint16_t mABat;             // Battery Current (mA)
    uint16_t loadCell;          // Load Cell (grams)
    int8_t temp;                // Temperature (°C)
    uint8_t mejPercent;         // MEJ Data (%)
    uint8_t cjPercent;          // CJ Data (%)
    uint8_t datajournalPercent; // Datalog Data (%)
    int32_t gpsLat;             // GPS Latitude (multiplied by 1,000,000 for 6-digit precision)
    int32_t gpsLon;             // GPS Longitude (multiplied by 1,000,000 for 6-digit precision)
    int16_t gpsAlt;             // GPS Altitude (meters)
    int16_t apogee;             // Apogee (meters)

    // Event data (up to 16 events, 4 groups of 4 binary events)
    // uint8_t events[16];     // Events 0-15 (binary)
    // uint16_t eventTime[2];    // Event Time for event 0 and event 1 (ms)
    uint16_t takeoffDetectedTime;
    uint16_t parachuteDeploymentTime;
    uint16_t event2_time;
    // Pyro Flags
    FlagDefs flags;

    int16_t yaw_setp;
    int16_t pitch_setp;
    int16_t roll_setp;
};

class TelemetryPacket : public BasePacket
{
public:
    Telemetry telemetry;
    uint8_t getPacketType() const override
    {
        return PACKET_TELEMETRY;
    }
    uint8_t getPayloadLen() const override
    {
        return TELEMETRY_SIZE + DUCC_PREFIX_SIZE + AVDCP2_HEADER_SIZE;
    }
};

struct Response
{
    uint32_t commandSeqId;
    uint8_t data[8];
};

class ResponsePacket : public BasePacket
{
public:
    Response response;
    uint8_t getPacketType() const override
    {
        return PACKET_RESPONSE;
    }
    uint8_t getPayloadLen() const override
    {
        return RESPONSE_SIZE + DUCC_PREFIX_SIZE + AVDCP2_HEADER_SIZE;
    }
};