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
#include "lora/lora.h"

#include <sstream>
#include <iomanip>
#include <iostream>
#include <queue>

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
    IMU imu_;
    IMU9DOF imu9dof_;
    KalmanFilter kf_;
    KalmanFilter kf9dof_;
    BMP280 baro_;
    BMP388 baro388_;
    Flash flash_;
    SDMC sdmc_;
    GPS gps_;

    STATE curr_state_;

    struct SensorData{
      float pitch1, roll1, yaw1 = 0;
      float pitch2, roll2, yaw2 = 0;
      float alt1, alt2 = 0;
      float lon, lat = 0;
    } sensor_data_;
    float altitude_ = 0;


    uint32_t estimateAltitudeMillis = 0;
    uint32_t estimateAltitudeInterval = 200;

    uint32_t estimateGPSMillis = 0;
    uint32_t estimateGPSInterval = 300;

    uint32_t commitMillis = 0;
    uint32_t commitInterval = 100;

    uint32_t monitorMillis = 0;
    uint32_t monitorInterval = 400;

    

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

    
};