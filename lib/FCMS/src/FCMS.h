#ifndef FCMS_H
#define FCMS_H
#include "barometer/bmp280.h"
#include "imu/imu.h"
#include "imu/kalman_filter.h"
#include "flash/flash.h"
#include "gps/gps.h"
#include "sdmc/sdmc.h"
#include "pyro/pyro.h"
#include "loadcell/loadcell.h"

#include <sstream>
#include <iomanip>


// FCMS lib
// Flight Computer Managment System 
enum FC_STATE {
  FC_SAFE=0,
  FC_IDLE=1,
  FC_FLIGHT=2,
  FC_DESCENT=3,
  FC_LANDING=4,
  FC_LANDED=5
};


class FCMS {
  private:
    IMU imu_;
    KalmanFilter kf_;
    BMP280 baro_;
    Flash flash_;

    FC_STATE curr_state_;

    float pitch_ = 0; float roll_ = 0;
  public:
    FCMS(): flash_(10), kf_(0.004) {};
    ~FCMS() = default;

    void setup();

    void setState(FC_STATE FC_state);
    bool nextState();
    FC_STATE getState();


    // updates some variables related to IMU, barometer and gps
    void navigate();
    // writes data to flash memmory
    void commit();
    


};


#endif