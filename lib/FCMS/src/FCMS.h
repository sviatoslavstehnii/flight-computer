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

    float pitch_ = 0; float roll_ = 0;

    uint32_t loopTimer = 0;
    uint32_t commitFlashTimer = 0; 
    uint32_t frequency = 1000;

    bool firstlaunch = true;
    bool firstAbortLoop = true;
    bool dataLogingStarted = false;
    bool parachuteDeployed = false;
    bool pyroOn = false;
    bool dataWrittenToSD = false;

    unsigned long launchAbortTime = 0;
    unsigned long landingDetectTime = 0;
    unsigned long abortLoopTime = 0;

  
  public:
    FCMS(): flash_(10), kf_(0.004) {};
    ~FCMS() = default;

    void setup();
    void updateData();
    void updateState();

    STATE getState();
    void goToState(STATE state);
    
    // updates some variables related to IMU, barometer and gps using kalman filter
    void navigateFilter();
    // writes data to flash memmory
    void commitFlash();


    
};