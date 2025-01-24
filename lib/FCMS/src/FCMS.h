#include "barometer/bmp280.h"
#include "barometer/bmp180.h"
#include "barometer/bmp388.h"
#include "imu/imu.h"
#include "imu/imu_9dof.h"

#include "imu/kalman_filter.h"

#include "flash/flash.h"
#include "gps/gps.h"
#include "sdmc/sdmc.h"
#include "ina/ina.h"
#include "lora/transceiver.h"

#include <sstream>
#include <iomanip>
#include <iostream>
#include <queue>

#define VEHICLE_ADDR 0xEE
#define GROUND_ADDR 0xCC

#define EPSYLON 0.0001
#define BUZZER_PIN 28
#define HEALTH_CHECK_TIMEOUT 10000
#define COMMIT_TIMEOUT 5000

// FCMS lib
// Flight Computer Managment System 
enum STATE {
  SAFE=1,
  IDLE=2,
  LAUNCH=3,
  FLIGHT=4,
  NO_POWER=5,
  DESCENT=6,
  PARACHUTE_LANDING=7,
  LANDED=8,
  ABORT=0
};

enum CURRENT_MODE {
  LAUNCH_DIRECTION_HOLD=0,
  ABORT_EULER=1,
  WAYPOINT_GUIDANCE=2
};




class FCMS {
  private:
    IMU imu_{};
    IMU9DOF imu9dof_{};
    KalmanFilter kf_;
    KalmanFilter kf9dof_;
    BMP280 baro_{};
    BMP388 baro388_{};
    Flash flash_;
    SDMC sdmc_{};
    GPS gps_{};
    FlightTransceiver transceiver{VEHICLE_ADDR, GROUND_ADDR};
    

    int land = 0;

    STATE curr_state_;

    struct SensorData{
      float pitchRate1, rollRate1, yawRate1;
      float pitch2, roll2, yaw2;
      float alt1, alt2;
      float lon, lat;
    } sensor_data_;
    float altitude_ = 0;

    struct Fins{
      int fin1, fin2, fin3, fin4;
    } fins;


    uint32_t estimateAltitudeMillis = 0;
    uint32_t estimateAltitudeInterval = 200;

    uint32_t estimateGPSMillis = 0;
    uint32_t estimateGPSInterval = 300;

    uint32_t commitMillis = 0;
    uint32_t commitInterval = 100;

    uint32_t commsMillis = 0;
    uint32_t commsInterval = 100;

    uint32_t monitorMillis = 0;
    uint32_t monitorInterval = 400;

    uint32_t buzzerMillis = 0;
    uint32_t sequence_id = 0;
    

    bool firstlaunch = true;
    bool firstAbortLoop = true;
    bool dataLogingStarted = false;
    bool parachuteDeployed = false;
    bool pyroOn = false;
    bool dataWrittenToSD = false;


    unsigned long launchAbortTime = 0;
    unsigned long landingDetectTime = 0;
    unsigned long abortLoopTime = 0;

    std::queue<std::pair<char*, uint32_t>> major_events_q_;

  public:
    FCMS() : flash_(10), kf_(0.04), kf9dof_(0.04) {}

    ~FCMS() = default;

    void setup();
    void checkHealth();
    void step();
    void updateState();

    STATE getState();
    void goToState(STATE state);
    
    void estimateAttitude();
    void estimateAltitude();
    void estimateGPS();

    void commitFlash();
    void commitSDMC();

    void buzzMillis(uint32_t ms);

    Telemetry& mapTelemetry();
};