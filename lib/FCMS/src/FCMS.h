#include "barometer/bmp280.h"
#include "barometer/bmp180.h"
#include "imu/imu.h"
#include "imu/kalman_filter.h"
#include "flash/flash.h"
#include "gps/gps.h"
#include "sdmc/sdmc.h"
#include "ina/ina.h"

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


class FCMS {
  private:
    IMU imu_;
    KalmanFilter kf_;
    BMP280 baro_;
    Flash flash_;
    SDMC sdmc_;

    STATE curr_state_;

    float pitch_ = 0; float roll_ = 0; float yaw_ = 0;

    uint32_t estimateMillis = 0;
    uint32_t estimateInterval = 4;

    uint32_t commitMillis = 0;
    uint32_t commitInterval = 100; 
    

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
    FCMS(): flash_(10), kf_(0.04) {};
    ~FCMS() = default;

    void setup();
    void updateState();

    STATE getState();
    void goToState(STATE state);
    
    void navigateFilter();
    void commitFlash();
    void commitSDMC();



};