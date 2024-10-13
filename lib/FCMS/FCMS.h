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



// FCMS lib
// Flight Computer Managment System 
enum STATE {
  SAFE=0,
  IDLE=1,
  FLIGHT=2,
  DESCENT=3,
  LANDING=4,
  LANDED=5
};


class FCMS {
  private:
    IMU imu_;
    KalmanFilter kf_;
    BMP280 baro_;
    Flash flash_;

    STATE curr_state_;

    float pitch_ = 0; float roll_ = 0;
  public:
    FCMS(): flash_(10), kf_(0.004) {};
    ~FCMS() = default;

    void setup();

    void setState(STATE state);
    bool nextState();
    STATE getState();

    // TODO: implement below methods

    // updates some variables related to IMU, barometer and gps
    void navigate();
    // writes data to flash memmory
    void commit();
    


};